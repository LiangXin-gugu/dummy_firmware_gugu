#!/usr/bin/env python3
"""
关节指令 / 反馈 / 电流 交互式实时监测脚本
功能:
  1. 线程A(交互): 使能机器人并定位到初始位姿后, 从终端实时读取用户输入的
     move_j 指令并逐条下发 —— 每交互一次即产生一个"阶跃"式的目标位姿变化
  2. 线程B(采样): 以固定频率(默认 50Hz, args 可配置)同时读取
     关节反馈角(GETJPOS) 与 6 电机 FOC 电流(GET_CURRENT), 共用同一时间戳
  3. 主线程: 实时绘制 6 关节的 指令/反馈角度(左轴, °) 与 电流(右副轴, A)

线程模型 (遵循 TkAgg 约束: GUI 事件循环必须在主线程):
  主线程    -> matplotlib FuncAnimation + plt.show(block=True)
  后台线程A -> 交互式读取 stdin, 逐条下发 move_j 指令
  后台线程B -> 定频采样关节角 + 电流, 写入线程安全的 MonitorRecorder

用法:
  python3 test_motor_current.py
  python3 test_motor_current.py --target 0 0 90 0 0 0 --sample_rate 50
  python3 test_motor_current.py --no_plot                 # 仅采样并保存, 不绘图
  python3 test_motor_current.py --no_current              # 关闭电流采样/绘图
  python3 test_motor_current.py --plot_log current_logs/xxx.npz   # 离线绘制

交互指令 (在终端输入, 回车下发):
  <j1> <j2> <j3> <j4> <j5> <j6>       下发 move_j 到该 6 关节角(度)
  <j1> ... <j6> <speed>               同上, 并指定本次速度
  speed <v>                           修改后续默认速度
  stop                                急停并清空队列
  start                               重新使能
  q / quit / exit                     结束监测并保存

记录数据说明 (保存为 current_logs/*.npz):
  cmd_time       : (M,)   指令下发时间戳 (秒, time.time())
  cmd_positions  : (M, 6) 下发的指令关节角 (度)
  cmd_free       : (M,)   入队应答中的剩余队列容量 free=N (无应答为 NaN)
  fbk_time       : (K,)   反馈采样时间戳 (秒, 收到响应时刻)
  fbk_positions  : (K, 6) 反馈的实时关节角 (度, GETJPOS)
  fbk_currents   : (K, 6) 与关节反馈同频采样的电机 FOC 电流 (安培, GET_CURRENT);
                   仅在未指定 --no_current 时记录, 否则为空数组
"""

import re
import sys
import time
import argparse
import threading
from pathlib import Path

import numpy as np

# ---------- 项目路径 ----------
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "robot_control_sdk" / "dummy"))

# 电流日志目录 (与轨迹日志同级, 独立存放)
LOG_DIR = Path(__file__).resolve().parent / "current_logs"

# 从入队应答 "ok queued free=N" 中提取剩余队列容量
_FREE_RE = re.compile(r"free=(\d+)")


# ======================================================================
# 通用工具
# ======================================================================
def _setup_cjk_font():
    """自动探测并配置可用的中文字体, 避免绘图中文标签显示为方框"""
    import matplotlib
    from matplotlib import font_manager

    candidates = ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "WenQuanYi Micro Hei",
                  "SimHei", "Source Han Sans SC"]
    available = {f.name for f in font_manager.fontManager.ttflist}
    found = [c for c in candidates if c in available]
    if not found:
        print("[绘图] 警告: 未检测到可用中文字体, 图中中文将显示为方框。"
              "请安装: sudo apt install fonts-wqy-zenhei "
              "并删除 matplotlib 缓存: rm -rf ~/.cache/matplotlib")
    matplotlib.rcParams["font.sans-serif"] = found + ["DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False


def _parse_free(resp) -> float:
    """从入队应答 'ok queued free=N' 提取剩余队列容量; 无应答/流式返回 NaN"""
    if resp:
        m = _FREE_RE.search(resp)
        if m:
            return float(m.group(1))
    return float("nan")


def _wait_motion_done(robot, stop_flag: threading.Event = None,
                      timeout: float = 30.0) -> bool:
    """等待固件运动完成的异步 'ok' 广播 (move_j 的 wait_ack 只等入队应答)。

    stop_flag 置位时提前返回, 避免用户退出后仍在此空等到超时。
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if stop_flag is not None and stop_flag.is_set():
            return False
        for line in robot.drain_async_lines():
            if line.strip() == "ok":
                return True
        time.sleep(0.02)
    return False


def _print_help(speed):
    """打印交互指令用法"""
    print("=" * 62)
    print("[交互] 在终端输入指令 (回车下发), 每条 move_j 形成一个阶跃:")
    print("  <j1> <j2> <j3> <j4> <j5> <j6>    下发到该 6 关节角(度)")
    print("  <j1> ... <j6> <speed>            同上, 并指定本次速度")
    print(f"  speed <v>                         修改后续默认速度 (当前 {speed:g})")
    print("  stop                              急停并清空队列")
    print("  start                             重新使能")
    print("  q / quit / exit                   结束监测并保存")
    print("=" * 62)


# ======================================================================
# 1. 数据记录 (线程安全): 指令 + 关节反馈 + 电流
# ======================================================================
class MonitorRecorder:
    """线程安全地记录: 下发指令(时间戳+目标角+free)、反馈关节角、电机电流。

    指令为稀疏事件 (每次交互一条), 反馈/电流为定频采样且共用同一时间戳。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.cmd_time: list[float] = []
        self.cmd_positions: list[list[float]] = []
        self.cmd_free: list[float] = []           # 入队应答中的剩余队列容量 free=N
        self.fbk_time: list[float] = []
        self.fbk_positions: list[list[float]] = []
        self.fbk_currents: list[list[float]] = []  # 与关节反馈同频的电机电流 (A)

    def log_command(self, t: float, q, free: float = float("nan")):
        with self._lock:
            self.cmd_time.append(t)
            self.cmd_positions.append(list(q))
            self.cmd_free.append(free)

    def log_feedback(self, t: float, q, currents=None):
        """记录一次反馈采样: 关节角 q 必填; currents 为同频采样的电机电流,
        传 None 表示本次未采电流 (关闭电流采样时), 此时不追加 fbk_currents。
        开启电流采样时逐点必传 (失败用 NaN 占位), 保证与 fbk_time 长度一致。"""
        with self._lock:
            self.fbk_time.append(t)
            self.fbk_positions.append(list(q))
            if currents is not None:
                self.fbk_currents.append(list(currents))

    def snapshot(self):
        """返回 (cmd_t, cmd_q, cmd_free, fbk_t, fbk_q, fbk_cur) 的 numpy 副本"""
        with self._lock:
            cmd_t = np.asarray(self.cmd_time, dtype=float)
            cmd_q = np.asarray(self.cmd_positions, dtype=float)
            cmd_free = np.asarray(self.cmd_free, dtype=float)
            fbk_t = np.asarray(self.fbk_time, dtype=float)
            fbk_q = np.asarray(self.fbk_positions, dtype=float)
            fbk_cur = np.asarray(self.fbk_currents, dtype=float)
        return cmd_t, cmd_q, cmd_free, fbk_t, fbk_q, fbk_cur

    def fbk_count(self) -> int:
        with self._lock:
            return len(self.fbk_time)

    def save(self, path):
        path = Path(path)
        cmd_t, cmd_q, cmd_free, fbk_t, fbk_q, fbk_cur = self.snapshot()
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(path,
                 cmd_time=cmd_t, cmd_positions=cmd_q, cmd_free=cmd_free,
                 fbk_time=fbk_t, fbk_positions=fbk_q, fbk_currents=fbk_cur)
        cur_txt = f", 电流 {fbk_cur.shape[0]} 点" if fbk_cur.size else ""
        print(f"[记录] 数据已保存: {path}")
        print(f"[记录]   指令 {len(cmd_t)} 点, 反馈 {len(fbk_t)} 点{cur_txt}")

        # 事后用时间戳差值校验实际采样率 (名义频率仅供参考)
        if fbk_t.size >= 2:
            dt = np.diff(fbk_t)
            dt = dt[dt > 0]
            if dt.size > 0:
                print(f"[记录]   实际采样率 均值 {1.0 / np.mean(dt):.1f}Hz / "
                      f"中位 {1.0 / np.median(dt):.1f}Hz")


# ======================================================================
# 2. 实时绘图: 每关节一个子图, 左轴指令/反馈(°) + 右副轴电流(A)
# ======================================================================
class RealtimeJointPlot:
    """实时绘图窗口: 6 个子图 (3x2) 分别显示各关节:
      - 左轴: 指令角度(蓝, 阶跃 step) 与 反馈角度(红), 共用同一刻度(°)
      - 右副轴(twinx): 同关节电流(绿), 独立刻度(A), 与关节角共用横轴(时间)
    """

    def __init__(self, recorder: MonitorRecorder, stop_flag: threading.Event,
                 n_joints: int = 6, window_s: float = 10.0,
                 interval_ms: int = 100, sample_current: bool = True,
                 sample_rate: float = 50.0):
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.gridspec import GridSpec

        _setup_cjk_font()

        self.recorder = recorder
        self.stop_flag = stop_flag
        self.window_s = window_s
        self.sample_current = sample_current
        self._closed = False

        self.fig = plt.figure(figsize=(12, 8))
        gs = GridSpec(3, 2, figure=self.fig)
        axes = [self.fig.add_subplot(gs[r, c]) for r in range(3) for c in range(2)]
        self.axes = axes[:n_joints]
        self.cmd_lines, self.fbk_lines, self.cur_lines = [], [], []
        self.axes_cur = []          # 各关节电流的右侧副轴 (twinx), 与主轴共用横轴
        for k, ax in enumerate(self.axes):
            # 指令为稀疏阶跃: where="post" 保持上一目标直到下一条指令
            (l_cmd,) = ax.step([], [], "b-", lw=1.2, where="post",
                               marker="o", ms=3, label="指令")
            (l_fbk,) = ax.plot([], [], "r-", lw=1.0, label="反馈")
            self.cmd_lines.append(l_cmd)
            self.fbk_lines.append(l_fbk)
            ax.set_title(f"关节 {k + 1}")
            ax.set_xlabel("t (s)")
            ax.set_ylabel("角度 (°)")
            ax.grid(True, alpha=0.3)
            l_cur = None
            if self.sample_current:
                # 右侧副轴画电流, 独立纵轴刻度, 与关节角共用横轴 (时间)
                ax2 = ax.twinx()
                (l_cur,) = ax2.plot([], [], "g-", lw=1.0, alpha=0.7, label="电流")
                ax2.set_ylabel("电流 (A)", color="g")
                ax2.tick_params(axis="y", labelcolor="g")
                self.axes_cur.append(ax2)
                self.cur_lines.append(l_cur)
            if k == 0:
                handles = [l_cmd, l_fbk] + ([l_cur] if l_cur is not None else [])
                ax.legend(handles, [h.get_label() for h in handles],
                          loc="upper right", fontsize="small")

        cur_txt = " + 电流" if self.sample_current else ""
        self.fig.suptitle(f"关节指令/反馈{cur_txt} 实时监测   "
                          f"采样={sample_rate:g}Hz")

        # rect 给 suptitle 留出顶部空间
        self.fig.tight_layout(rect=[0, 0, 1, 0.96])

        # 持有引用防止被 GC; GUI 事件循环必须在主线程运行 (TkAgg 要求)
        self.anim = FuncAnimation(self.fig, self._update,
                                  interval=interval_ms, cache_frame_data=False)

    def show_blocking(self):
        """在当前(主)线程阻塞运行 GUI 事件循环, 直到窗口关闭"""
        import matplotlib.pyplot as plt
        print("[监测] 实时绘图窗口已启动 (关闭窗口或在终端输入 q 退出)")
        plt.show(block=True)

    def _update(self, _frame):
        import matplotlib.pyplot as plt

        # 停止信号 (窗口关闭 / 终端 q / Ctrl+C) -> 主线程安全地关闭窗口
        if self.stop_flag.is_set() and not self._closed:
            self._closed = True
            plt.close(self.fig)
            return

        cmd_t, cmd_q, _cmd_free, fbk_t, fbk_q, fbk_cur = self.recorder.snapshot()
        if len(cmd_t) == 0 and len(fbk_t) == 0:
            return
        t_ref = max(cmd_t[-1] if len(cmd_t) else 0.0,
                    fbk_t[-1] if len(fbk_t) else 0.0)
        t_min = t_ref - self.window_s
        # 电流与关节反馈共用 fbk_time (逐点对齐), 存在且为二维时才绘制
        has_cur = self.sample_current and fbk_cur.ndim == 2 and fbk_cur.size > 0

        for k, ax in enumerate(self.axes):
            y_shown = []
            if len(cmd_t) and cmd_q.ndim == 2 and cmd_q.shape[1] > k:
                m = cmd_t >= t_min
                self.cmd_lines[k].set_data(cmd_t[m], cmd_q[m, k])
                y_shown.append(cmd_q[m, k])
            if len(fbk_t) and fbk_q.ndim == 2 and fbk_q.shape[1] > k:
                m = fbk_t >= t_min
                self.fbk_lines[k].set_data(fbk_t[m], fbk_q[m, k])
                y_shown.append(fbk_q[m, k])
            ax.set_xlim(t_min, t_ref if t_ref > t_min else t_min + 1.0)
            if y_shown:
                y = np.concatenate(y_shown)
                if y.size > 0:
                    pad = max(1.0, 0.05 * float(np.ptp(y)))
                    ax.set_ylim(float(y.min()) - pad, float(y.max()) + pad)
            # 右侧副轴: 电流 (共用横轴/时间窗, 纵轴按可见电流独立自适应)
            if has_cur and fbk_cur.shape[1] > k and len(fbk_t):
                mc = fbk_t >= t_min
                self.cur_lines[k].set_data(fbk_t[mc], fbk_cur[mc, k])
                c = fbk_cur[mc, k]
                c = c[~np.isnan(c)]
                if c.size > 0:
                    pad_c = max(0.05, 0.05 * float(np.ptp(c)))
                    self.axes_cur[k].set_ylim(float(c.min()) - pad_c,
                                              float(c.max()) + pad_c)


# ======================================================================
# 3. 后台线程: 关节/电流采样 + 交互式指令下发
# ======================================================================
def _feedback_sampler(robot, recorder: MonitorRecorder,
                      stop_flag: threading.Event, period: float,
                      sample_current: bool = True):
    """后台线程: 以固定频率轮询 GETJPOS(+GET_CURRENT), 记录关节反馈角、
    电机电流与共用时间戳 (新 SDK 内部线程安全)。

    period        : 两次采样的目标间隔 (秒) = 1 / sample_rate
    sample_current: True 时在读取关节角后紧接着读取 GET_CURRENT, 与关节反馈
                    同频、共用同一时间戳; 单次电流读取失败记 NaN 占位, 以保持
                    与 fbk_time/fbk_positions 逐点对齐 (不打断关节采样)
    采用 perf_counter 绝对节拍, 避免 time.sleep 累积漂移。
    """
    next_t = time.perf_counter()
    fail_count = 0
    cur_fail_count = 0
    while not stop_flag.is_set():
        try:
            q = robot.get_joint_pos()
            t = time.time()          # 收到响应的时刻
            fail_count = 0
        except Exception as e:       # 含 SDKError / SerialTimeoutException
            fail_count += 1
            if fail_count <= 3:
                print(f"\n[采样] GETJPOS 失败 ({fail_count}): {e}")
            stop_flag.wait(0.05)
            continue

        currents = None
        if sample_current:
            # 紧随关节角读取电流, 保持与 joints 相同的采样频率与时间戳
            try:
                currents = robot.get_motor_currents()
                cur_fail_count = 0
            except Exception as e:
                cur_fail_count += 1
                if cur_fail_count <= 3:
                    print(f"\n[采样] GET_CURRENT 失败 ({cur_fail_count}): {e}")
                currents = [float("nan")] * len(q)   # 对齐占位
        recorder.log_feedback(t, q, currents)

        next_t += period
        delay = next_t - time.perf_counter()
        if delay > 0:
            stop_flag.wait(delay)        # 可被 stop_flag 提前唤醒
        else:
            next_t = time.perf_counter()  # 落后于节拍则重置, 不追赶
    print(f"\n[采样] 采样结束, 共 {recorder.fbk_count()} 点")


def _interactive_sender(robot, recorder: MonitorRecorder,
                        stop_flag: threading.Event, init_target, speed,
                        speed_factor: float, acc_percent: float, acc_base):
    """后台线程: 使能 + 设置参数 + 定位到初始位姿, 随后进入交互循环,
    从 stdin 实时读取用户输入的 move_j 指令并逐条下发 (每条形成一个阶跃)。

    连接对象 robot 由主流程创建并复用 (不在此重复开关串口)。
    """
    cur_speed = float(speed)
    try:
        print("[交互] 使能机器人 ...")
        robot.start()

        print(f"[交互] 设置速度系数 speed_factor={speed_factor} ...")
        robot.set_speed_factor(speed_factor)
        print(f"[交互] 设置加速度百分比 acc_percent={acc_percent} ...")
        robot.set_acc_percent(acc_percent)
        print(f"[交互] 设置加速度基值 acc_base={acc_base} ...")
        robot.set_acc_base(acc_base)

        # 定位到初始位姿 (作为交互前的第一个阶跃指令)
        j0 = list(init_target)
        print(f"[交互] 定位到初始位姿 {j0} speed={cur_speed:g} ...")
        t_cmd = time.time()
        resp = robot.move_j(j0, speed=cur_speed, wait_ack=True)
        recorder.log_command(t_cmd, j0, _parse_free(resp))
        if _wait_motion_done(robot, stop_flag):
            print("[交互] 已到达初始位姿")
        elif not stop_flag.is_set():
            print("[交互] 警告: 等待初始定位完成超时")

        _print_help(cur_speed)

        # ---------- 交互循环 ----------
        while not stop_flag.is_set():
            try:
                line = input("[movej] > ")
            except EOFError:
                print("\n[交互] 输入结束 (EOF), 退出")
                break
            line = line.strip()
            if not line:
                continue
            low = line.lower()

            if low in ("q", "quit", "exit"):
                print("[交互] 收到退出指令")
                stop_flag.set()
                break
            if low == "stop":
                try:
                    robot.stop()
                    print("[交互] 已急停并清空队列")
                except Exception as e:
                    print(f"[交互] 急停失败: {e!r}")
                continue
            if low == "start":
                try:
                    robot.start()
                    print("[交互] 已重新使能")
                except Exception as e:
                    print(f"[交互] 使能失败: {e!r}")
                continue
            if low.startswith("speed"):
                parts = line.split()
                if len(parts) == 2:
                    try:
                        cur_speed = float(parts[1])
                        print(f"[交互] 默认速度已设为 {cur_speed:g}")
                    except ValueError:
                        print("[交互] speed 需跟一个数值, 如: speed 100")
                else:
                    print("[交互] 用法: speed <数值>")
                continue
            if low in ("h", "help", "?"):
                _print_help(cur_speed)
                continue

            # 解析 6 个关节角, 或 6 关节角 + 本次速度 (共 7 个数值)
            parts = line.split()
            try:
                vals = [float(x) for x in parts]
            except ValueError:
                print("[交互] 无法解析, 请输入 6 个关节角(度), 或输入 help 查看用法")
                continue
            if len(vals) == 6:
                j_target, spd = vals, cur_speed
            elif len(vals) == 7:
                j_target, spd = vals[:6], vals[6]
            else:
                print(f"[交互] 需要 6 个关节角(可选第 7 个为速度), 当前 {len(vals)} 个")
                continue

            try:
                t_cmd = time.time()
                resp = robot.move_j(j_target, speed=spd, wait_ack=True)
                recorder.log_command(t_cmd, j_target, _parse_free(resp))
                qs = " ".join(f"J{k + 1}={j_target[k]:7.2f}" for k in range(6))
                print(f"[交互] 已下发 {qs}  speed={spd:g}  应答: {resp}")
            except Exception as e:
                print(f"[交互] 下发失败: {e!r}")
    except Exception as e:
        print(f"\n[交互] 异常终止: {e!r}")
        stop_flag.set()


# ======================================================================
# 4. 主编排: 交互下发 + 关节/电流采样 + 实时绘图
# ======================================================================
def monitor_interactive(
    init_target=(0.0, 0.0, 90.0, 0.0, 0.0, 0.0),
    port: str = "/dev/ttyACM0",
    sample_rate: float = 50.0,
    speed: int = 100,
    speed_factor: float = 0.2,
    acc_percent: float = 100.0,
    acc_base=None,
    sample_current: bool = True,
    realtime_plot: bool = True,
    window_s: float = 10.0,
    save_log: bool = True,
):
    """
    交互式下发 move_j 指令, 同时定频采样关节反馈角与电机电流并实时绘图。

    Parameters
    ----------
    init_target  : 交互前定位到的 6 关节初始角 (度), 默认 [0,0,90,0,0,0]
    port         : 串口端口
    sample_rate  : 关节/电流采样频率 (Hz), 默认 50
    speed        : move_j 默认速度参数 (交互中可用 'speed <v>' 修改)
    speed_factor : 速度换算系数, 使能后下发, 固件夹取 [0.01, 1.0]
    acc_percent  : 加速度百分比, 使能后下发, 固件夹取 [0, 100]
    acc_base     : 6 关节加速度基值 (r/s²), 固件逐个夹取 [0, 200]
    sample_current: True (默认) 时在采样线程内紧随 GETJPOS 读取 GET_CURRENT,
                   与关节反馈同频记录并在各关节子图右侧副轴绘制电流
    realtime_plot: True 时在主线程打开实时绘图窗口
    window_s     : 实时绘图滑动时间窗 (秒)
    save_log     : True 时退出前保存采样数据到 current_logs/*.npz
    """
    from dummy_robot_sdk import RobotArmSDK

    if acc_base is None:
        acc_base = [150.0, 100.0, 200.0, 200.0, 200.0, 200.0]
    init_target = list(init_target)
    if len(init_target) != 6:
        raise ValueError(f"init_target 须包含 6 个关节值, 当前 {len(init_target)}")

    period = 1.0 / sample_rate if sample_rate > 0 else 0.02

    robot = RobotArmSDK(port)
    print(f"[监测] 已连接机械臂: {port}")

    recorder = MonitorRecorder()
    stop_flag = threading.Event()    # 统一停止信号 (窗口关闭 / 终端 q / Ctrl+C)

    plot = (RealtimeJointPlot(recorder, stop_flag, window_s=window_s,
                              sample_current=sample_current,
                              sample_rate=sample_rate)
            if realtime_plot else None)

    sampler = threading.Thread(
        target=_feedback_sampler,
        args=(robot, recorder, stop_flag, period, sample_current),
        name="FeedbackSampler", daemon=True)
    sender = threading.Thread(
        target=_interactive_sender,
        args=(robot, recorder, stop_flag, init_target, speed,
              speed_factor, acc_percent, acc_base),
        name="InteractiveSender", daemon=True)

    try:
        # 采样线程先起, 便于捕获使能/初始定位过程中的电流瞬态
        sampler.start()
        sender.start()

        if plot is not None:
            # 关闭绘图窗口 -> 置位 stop_flag -> 停止采样/交互
            plot.fig.canvas.mpl_connect("close_event", lambda _e: stop_flag.set())
            try:
                plot.show_blocking()
            except KeyboardInterrupt:
                print("\n[监测] 用户中断")
            stop_flag.set()
        else:
            print("[监测] 无绘图模式, 在终端交互输入指令 (q 退出) ...")
            try:
                while not stop_flag.is_set():
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n[监测] 用户中断")
                stop_flag.set()
    finally:
        stop_flag.set()
        sampler.join(timeout=2.0)
        # 交互线程可能阻塞在 input(), daemon 线程不强制等待其退出
        sender.join(timeout=1.0)
        if save_log:
            log_path = LOG_DIR / f"current_log_{time.strftime('%Y%m%d_%H%M%S')}.npz"
            recorder.save(log_path)
        print("[监测] 断开连接 ...")
        try:
            robot.close()
        except Exception:
            pass


# ======================================================================
# 5. 离线绘制已保存的日志
# ======================================================================
def plot_saved_log(log_path: str):
    """绘制 MonitorRecorder 保存的 npz 日志, 用于事后分析指令跟随与电流"""
    import matplotlib.pyplot as plt

    _setup_cjk_font()

    data = np.load(log_path)
    cmd_t = data["cmd_time"] if "cmd_time" in data.files else np.zeros(0)
    cmd_q = (data["cmd_positions"] if "cmd_positions" in data.files
             else np.zeros((0, 6)))
    fbk_t = data["fbk_time"] if "fbk_time" in data.files else np.zeros(0)
    fbk_q = (data["fbk_positions"] if "fbk_positions" in data.files
             else np.zeros((0, 6)))
    fbk_cur = data["fbk_currents"] if "fbk_currents" in data.files else np.zeros(0)
    has_cur = (fbk_cur.ndim == 2 and fbk_cur.size > 0
               and fbk_cur.shape[0] == len(fbk_t))
    if len(cmd_t) == 0 and len(fbk_t) == 0:
        print(f"[离线绘图] 日志为空或格式不符: {log_path}")
        return

    t0 = float(cmd_t[0]) if len(cmd_t) else float(fbk_t[0])
    n_joints = int((cmd_q if len(cmd_t) else fbk_q).shape[1])

    fig = plt.figure(figsize=(12, 8))
    gs = fig.add_gridspec(3, 2)
    axes = [fig.add_subplot(gs[r, c]) for r in range(3) for c in range(2)]
    for k in range(min(n_joints, len(axes))):
        ax = axes[k]
        handles = []
        if len(cmd_t) and cmd_q.ndim == 2 and cmd_q.shape[1] > k:
            (l_cmd,) = ax.step(cmd_t - t0, cmd_q[:, k], "b-", lw=1.2,
                               where="post", marker="o", ms=3, label="指令")
            handles.append(l_cmd)
        if len(fbk_t) and fbk_q.ndim == 2 and fbk_q.shape[1] > k:
            (l_fbk,) = ax.plot(fbk_t - t0, fbk_q[:, k], "r-", lw=1.0, label="反馈")
            handles.append(l_fbk)
        ax.set_title(f"关节 {k + 1}")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("角度 (°)")
        ax.grid(True, alpha=0.3)
        # 右侧副轴: 电流 (与关节角共用横轴, 纵轴独立尺度)
        if has_cur and fbk_cur.shape[1] > k:
            ax2 = ax.twinx()
            (l_cur,) = ax2.plot(fbk_t - t0, fbk_cur[:, k], "g-", lw=1.0,
                                alpha=0.7, label="电流")
            ax2.set_ylabel("电流 (A)", color="g")
            ax2.tick_params(axis="y", labelcolor="g")
            handles.append(l_cur)
        if k == 0 and handles:
            ax.legend(handles, [h.get_label() for h in handles],
                      loc="upper right", fontsize="small")

    fig.suptitle(f"关节指令/反馈 + 电流: {Path(log_path).name}")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


# ======================================================================
# main
# ======================================================================
def main():
    parser = argparse.ArgumentParser(
        description="关节指令/反馈/电流 交互式实时监测 "
                    "(交互下发 move_j + 定频采样关节角与电流 + 实时绘图)")
    parser.add_argument("--target", type=float, nargs=6,
                        default=[0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
                        help="交互前定位到的初始关节角 (度), 6 个值, "
                             "默认 0 0 90 0 0 0")
    parser.add_argument("--sample_rate", type=float, default=50.0,
                        help="关节/电流采样频率 (Hz), 默认 50")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0",
                        help="串口端口, 默认 /dev/ttyACM0")
    parser.add_argument("--speed", type=int, default=100,
                        help="move_j 默认速度参数 (交互中可用 'speed <v>' 修改), 默认 100")
    parser.add_argument("--speed_factor", type=float, default=0.2,
                        help="速度单位->电机轴 r/s 换算系数, 固件夹取[0.01,1.0], 默认 0.2")
    parser.add_argument("--acc_percent", type=float, default=100.0,
                        help="加速度百分比, 固件夹取[0,100], 默认 100")
    parser.add_argument("--acc_base", type=float, nargs=6,
                        default=[150.0, 100.0, 200.0, 200.0, 200.0, 200.0],
                        help="6 关节加速度基值(r/s²), 固件逐个夹取[0,200], "
                             "默认 150 100 200 200 200 200")
    parser.add_argument("--window", type=float, default=10.0,
                        help="实时绘图滑动时间窗 (秒), 默认 10")
    parser.add_argument("--no_current", action="store_true",
                        help="禁用电流采样 (默认与关节反馈同频采样 GET_CURRENT, "
                             "并记录/在各关节子图右侧副轴绘制电流)")
    parser.add_argument("--no_plot", action="store_true",
                        help="不打开实时绘图窗口 (仍会采样并保存数据)")
    parser.add_argument("--no_save", action="store_true",
                        help="退出时不保存采样数据到 current_logs/")
    parser.add_argument("--plot_log", type=str, default=None,
                        help="离线绘制已保存的 npz 日志路径, 不连接机械臂")
    args = parser.parse_args()

    # ---------- 离线绘图模式 ----------
    if args.plot_log:
        plot_saved_log(args.plot_log)
        return

    # ---------- 实时交互监测模式 ----------
    monitor_interactive(
        init_target=args.target,
        port=args.port,
        sample_rate=args.sample_rate,
        speed=args.speed,
        speed_factor=args.speed_factor,
        acc_percent=args.acc_percent,
        acc_base=args.acc_base,
        sample_current=not args.no_current,
        realtime_plot=not args.no_plot,
        window_s=args.window,
        save_log=not args.no_save,
    )


if __name__ == "__main__":
    main()
