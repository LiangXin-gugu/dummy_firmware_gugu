#!/usr/bin/env python3
"""
正弦轨迹测试脚本
功能:
  1. 生成 6 关节正弦运动轨迹 (每关节可独立设置周期与幅值, 默认仅关节1运动)
  2. 使用 viser 可视化轨迹
  3. 通过 RobotArmSDK.move_j 下发轨迹给实体机械臂

用法:
  python test_sine_trajectory.py --Ts 2.0 --freq 50 --port /dev/ttyACM0
  python test_sine_trajectory.py --Ts 2 2 2 2 2 2 --amplitude 30 0 0 0 0 0   # 每关节独立指定
  python test_sine_trajectory.py --Ts 2.0 --freq 50 --vis_only          # 仅可视化
  python test_sine_trajectory.py --plot_log trajectory_logs/xxx.npz    # 离线绘制已保存日志

记录数据说明 (保存为 trajectory_logs/*.npz):
  cmd_time       : (M,)   指令下发时间戳 (秒, time.time())
  cmd_positions  : (M, 6) 下发的指令关节角 (度)
  cmd_free       : (M,)   入队应答中的剩余队列容量 free=N (流式/无应答为 NaN)
  fbk_time       : (K,)   反馈采样时间戳 (秒, 收到响应时刻)
  fbk_positions  : (K, 6) 反馈的实时关节角 (度, GETJPOS)
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
sys.path.insert(0, str(PROJECT_ROOT / "visualization_module"))
sys.path.insert(0, str(PROJECT_ROOT / "robot_control_sdk" / "dummy"))

URDF_PATH = PROJECT_ROOT / "robot_assets" / "dummy" / "urdf" / "dummy.urdf"


# ======================================================================
# 1. 轨迹生成
# ======================================================================
def _broadcast6(values, name: str) -> np.ndarray:
    """将标量/单元素序列广播为 6 维浮点数组; 已是 6 维则原样返回。

    用于 --Ts / --amplitude 等"每关节一个值"的参数: 允许只传 1 个值
    (应用到全部 6 个关节) 或恰好 6 个值 (逐个关节指定)。
    """
    arr = np.asarray(values, dtype=float).ravel()
    if arr.size == 1:
        arr = np.repeat(arr, 6)
    if arr.size != 6:
        raise ValueError(
            f"{name} 需为 1 个或 6 个值, 当前收到 {arr.size} 个: "
            f"{np.asarray(values).tolist()}")
    return arr


def generate_sine_trajectory(
    Ts,
    freq: float,
    amplitude_deg=(90.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    base_pose_deg: list[float] | None = None,
) -> dict:
    """
    生成 6 个关节的正弦运动轨迹, 每个关节可独立设置周期与幅值。
    正弦偏移叠加在基准位姿之上; 幅值为 0 的关节保持基准位姿不动。

    运动方程 (角度制, 第 j 个关节):
        qj(t)    =  base_j + Aj·sin(ωj·t)
        dqj(t)   =  Aj·ωj·cos(ωj·t)          (速度)
        ddqj(t)  = -Aj·ωj²·sin(ωj·t)         (加速度)

    其中各关节 1/4 周期 Ts_j → 完整周期 T_j = 4·Ts_j, ωj = 2π/T_j = π/(2·Ts_j)
    轨迹总时长取最慢关节的一个完整周期: T_total = max_j(T_j)

    Parameters
    ----------
    Ts            : float | sequence, 各关节 1/4 正弦周期 (秒);
                    传 1 个值则广播到 6 关节, 或传 6 个值分别指定
    freq          : float, 采样频率 f (Hz), delta_t = 1/f
    amplitude_deg : float | sequence, 各关节正弦幅值 (度);
                    传 1 个值则广播到 6 关节, 或传 6 个值分别指定,
                    默认 [90, 0, 0, 0, 0, 0] (仅关节1运动)
    base_pose_deg : list[float], 6个关节的基准位姿 (度),
                    默认 [0, -75, 180, 0, 0, 0]

    Returns
    -------
    dict with keys:
        time      : np.ndarray (N,)   时间序列 (秒)
        delta_t   : float             相邻两点时间间隔
        freq      : float             采样频率
        base_pose : np.ndarray (6,)   基准位姿 (度)
        Ts        : np.ndarray (6,)   各关节 1/4 周期 (秒)
        amplitude : np.ndarray (6,)   各关节幅值 (度)
        omega     : np.ndarray (6,)   各关节角频率 (rad/s)
        positions : np.ndarray (N, 6) 关节位置 (度)
        velocities: np.ndarray (N, 6) 关节速度 (度/秒)
        accelerations: np.ndarray (N, 6) 关节加速度 (度/秒²)
    """
    if base_pose_deg is None:
        base_pose_deg = [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]
    base_pose = np.asarray(base_pose_deg, dtype=float)
    assert len(base_pose) == 6, f"base_pose_deg 须包含 6 个关节值, 当前 {len(base_pose)}"

    Ts_arr = _broadcast6(Ts, "Ts")                       # (6,) 各关节 1/4 周期
    A_arr = _broadcast6(amplitude_deg, "amplitude_deg")  # (6,) 各关节幅值
    assert np.all(Ts_arr > 0), f"Ts 各分量须为正数, 当前 {Ts_arr.tolist()}"

    delta_t = 1.0 / freq
    T_j = 4.0 * Ts_arr                 # 各关节完整周期 (6,)
    omega = 2.0 * np.pi / T_j          # 各关节角频率 (rad/s) (6,)

    # 时间序列: 覆盖最慢关节的一个完整周期 [0, T_total]
    T_total = float(np.max(T_j))
    N = int(np.round(T_total * freq)) + 1
    t = np.linspace(0, T_total, N)     # (N,)

    # 各关节正弦偏移 (N, 6): 相位 = ωj·t, 按列广播幅值/角频率
    phase = np.outer(t, omega)                           # (N, 6)
    offsets = A_arr * np.sin(phase)                      # (N, 6) 度
    velocities = A_arr * omega * np.cos(phase)           # (N, 6) 度/秒
    accelerations = -A_arr * omega**2 * np.sin(phase)    # (N, 6) 度/秒²

    # 组装 6 关节数据: 基准位姿 + 各关节正弦偏移
    positions = np.tile(base_pose, (N, 1)) + offsets     # (N, 6)

    # 循环下发接缝检查: 各关节周期须整除 T_total, 首尾才能相接不跳变
    cycles = T_total / T_j
    if np.any(np.abs(cycles - np.round(cycles)) > 1e-6):
        print(f"[轨迹生成] 警告: 部分关节周期不能整除总时长 {T_total:.2f}s, "
              f"循环下发接缝处可能跳变 (各关节周期数={np.round(cycles, 3).tolist()})")

    print(f"[轨迹生成] freq={freq}Hz, delta_t={delta_t:.4f}s, "
          f"点数={N}, 总时长={T_total:.2f}s")
    print(f"[轨迹生成] Ts(1/4周期)={Ts_arr.tolist()}  幅值={A_arr.tolist()}")
    print(f"[轨迹生成] 基准位姿={base_pose.tolist()}")

    return {
        "time": t,
        "delta_t": delta_t,
        "freq": freq,
        "base_pose": base_pose,
        "Ts": Ts_arr,
        "amplitude": A_arr,
        "omega": omega,
        "positions": positions,
        "velocities": velocities,
        "accelerations": accelerations,
    }


# ======================================================================
# 2. Viser 可视化
# ======================================================================
def visualize_trajectory(traj: dict, urdf_path: Path, vis_step: int = 1):
    """
    使用 ViserRobotViewer 逐帧可视化轨迹

    Parameters
    ----------
    traj     : generate_sine_trajectory 返回的字典
    urdf_path: URDF 文件路径
    vis_step : 每隔 vis_step 个采样点更新一次可视化
    """
    from viser_robot_viewer import ViserRobotViewer

    viewer = ViserRobotViewer(urdf_path=urdf_path)
    print(f"[可视化] Viser 已启动, 打开浏览器访问: {viewer.get_url()}")
    print(f"[可视化] 关节名: {viewer.joint_names}")
    print(f"[可视化] 按 vis_step={vis_step} 播放轨迹, 共 {len(traj['time'])} 帧 ...")

    # 添加世界坐标系
    viewer.add_frame("/world", position=(0, 0, 0), axes_length=0.3)

    positions = traj["positions"]
    delta_t = traj["delta_t"]
    n_frames = len(traj["time"])

    viewer.set_joint_angles_degrees(positions[0])
    # import ipdb; ipdb.set_trace()

    for i in range(0, n_frames, vis_step):
        j_deg = positions[i]
        # 取前 len(joint_names) 个关节 (URDF 中可能只有 5~6 个可驱动关节)
        n_joints = len(viewer.joint_names)
        viewer.set_joint_angles_degrees(j_deg[:n_joints])

        t_now = traj["time"][i]
        qs = " ".join(f"J{k+1}={j_deg[k]:7.2f}" for k in range(len(j_deg)))
        print(f"\r  帧 {i:4d}/{n_frames}  t={t_now:6.3f}s  {qs}", end="")

        time.sleep(delta_t * vis_step)

    print("\n[可视化] 轨迹播放完成")
    return viewer


# ======================================================================
# 3. 下发轨迹到实体机械臂 (含指令/反馈记录与实时绘图)
# ======================================================================
LOG_DIR = Path(__file__).resolve().parent / "trajectory_logs"

# 从入队应答 "ok queued free=N" 中提取剩余队列容量
_FREE_RE = re.compile(r"free=(\d+)")


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


class TrajectoryRecorder:
    """线程安全地记录下发指令与反馈关节角 (含时间戳)"""

    def __init__(self):
        self._lock = threading.Lock()
        self.cmd_time: list[float] = []
        self.cmd_positions: list[list[float]] = []
        self.cmd_free: list[float] = []    # 入队应答中的剩余队列容量 free=N
        self.fbk_time: list[float] = []
        self.fbk_positions: list[list[float]] = []

    def log_command(self, t: float, q, free: float = float("nan")):
        with self._lock:
            self.cmd_time.append(t)
            self.cmd_positions.append(list(q))
            self.cmd_free.append(free)

    def log_feedback(self, t: float, q):
        with self._lock:
            self.fbk_time.append(t)
            self.fbk_positions.append(list(q))

    def snapshot(self):
        """返回 (cmd_t, cmd_q, cmd_free, fbk_t, fbk_q) 的 numpy 数组副本"""
        with self._lock:
            cmd_t = np.asarray(self.cmd_time, dtype=float)
            cmd_q = np.asarray(self.cmd_positions, dtype=float)
            cmd_free = np.asarray(self.cmd_free, dtype=float)
            fbk_t = np.asarray(self.fbk_time, dtype=float)
            fbk_q = np.asarray(self.fbk_positions, dtype=float)
        return cmd_t, cmd_q, cmd_free, fbk_t, fbk_q

    def save(self, path: Path):
        cmd_t, cmd_q, cmd_free, fbk_t, fbk_q = self.snapshot()
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(path,
                 cmd_time=cmd_t, cmd_positions=cmd_q, cmd_free=cmd_free,
                 fbk_time=fbk_t, fbk_positions=fbk_q)
        print(f"[记录] 数据已保存: {path}")
        print(f"[记录]   指令 {len(cmd_t)} 点, 反馈 {len(fbk_t)} 点")


class RealtimeJointPlot:
    """实时绘图窗口: 每个关节一个子图, 对比指令角度与反馈角度 (滑动时间窗)"""

    def __init__(self, recorder: TrajectoryRecorder,
                 n_joints: int = 6, window_s: float = 10.0,
                 interval_ms: int = 100):
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.gridspec import GridSpec

        _setup_cjk_font()

        self.recorder = recorder
        self.window_s = window_s

        # 4x2 网格: 前 3 行放关节子图, 底行整行放队列剩余容量 free
        self.fig = plt.figure(figsize=(12, 10))
        gs = GridSpec(4, 2, figure=self.fig)
        axes = [self.fig.add_subplot(gs[r, c]) for r in range(3) for c in range(2)]
        self.axes = axes[:n_joints]
        self.cmd_lines, self.fbk_lines = [], []
        for k, ax in enumerate(self.axes):
            (l_cmd,) = ax.plot([], [], "b-", lw=1.2, label="指令")
            (l_fbk,) = ax.plot([], [], "r-", lw=1.0, label="反馈")
            self.cmd_lines.append(l_cmd)
            self.fbk_lines.append(l_fbk)
            ax.set_title(f"关节 {k + 1}")
            ax.set_xlabel("t (s)")
            ax.set_ylabel("角度 (°)")
            ax.grid(True, alpha=0.3)
            if k == 0:
                ax.legend(loc="upper right")

        # 底部子图: 固件队列剩余容量 free 随时间变化
        # 占单个子图位置 (与关节图同尺寸, 便于肉眼对齐时间轴), 右侧留白
        self.ax_free = self.fig.add_subplot(gs[3, 0])
        (self.free_line,) = self.ax_free.plot([], [], "g-", lw=1.0,
                                              label="free (剩余队列容量)")
        self.ax_free.set_title("固件队列剩余容量 free")
        self.ax_free.set_xlabel("t (s)")
        self.ax_free.set_ylabel("free")
        self.ax_free.grid(True, alpha=0.3)
        self.ax_free.legend(loc="upper right")

        self.fig.tight_layout()

        # 持有引用防止被 GC; GUI 事件循环必须在主线程运行 (TkAgg 要求)
        self.anim = FuncAnimation(self.fig, self._update,
                                  interval=interval_ms, cache_frame_data=False)

    def show_blocking(self):
        """在当前(主)线程阻塞运行 GUI 事件循环, 直到窗口关闭"""
        import matplotlib.pyplot as plt
        print("[记录] 实时绘图窗口已启动 (关闭窗口或 Ctrl+C 退出)")
        plt.show(block=True)

    def _update(self, _frame):
        cmd_t, cmd_q, cmd_free, fbk_t, fbk_q = self.recorder.snapshot()
        if len(cmd_t) == 0 and len(fbk_t) == 0:
            return
        t_ref = max(cmd_t[-1] if len(cmd_t) else 0.0,
                    fbk_t[-1] if len(fbk_t) else 0.0)
        t_min = t_ref - self.window_s

        for k, ax in enumerate(self.axes):
            y_shown = []
            if len(cmd_t):
                m = cmd_t >= t_min
                self.cmd_lines[k].set_data(cmd_t[m], cmd_q[m, k])
                y_shown.append(cmd_q[m, k])
            if len(fbk_t):
                m = fbk_t >= t_min
                self.fbk_lines[k].set_data(fbk_t[m], fbk_q[m, k])
                y_shown.append(fbk_q[m, k])
            ax.set_xlim(t_min, t_ref if t_ref > t_min else t_min + 1.0)
            if y_shown:
                y = np.concatenate(y_shown)
                if y.size > 0:
                    pad = max(1.0, 0.05 * float(np.ptp(y)))
                    ax.set_ylim(float(y.min()) - pad, float(y.max()) + pad)

        # 底部子图: 队列剩余容量 free (与关节图共用滑动时间窗)
        if len(cmd_t):
            m = cmd_t >= t_min
            self.free_line.set_data(cmd_t[m], cmd_free[m])
            self.ax_free.set_xlim(t_min, t_ref if t_ref > t_min else t_min + 1.0)
            f = cmd_free[m]
            f = f[~np.isnan(f)]
            top = float(f.max()) + 1.0 if f.size > 0 else 16.0
            self.ax_free.set_ylim(-0.5, max(top, 2.0))


def _feedback_sampler(robot, recorder: TrajectoryRecorder,
                      stop_flag: threading.Event,
                      sample_interval: float = 0.02):
    """后台线程: 轮询 GETJPOS, 记录反馈关节角与时间戳 (新 SDK 内部线程安全)

    sample_interval: 两次采样之间的最小间隔 (秒), 防止固件被高频查询压垮
    """
    fail_count = 0
    while not stop_flag.is_set():
        try:
            q = robot.get_joint_pos()
            t = time.time()      # 收到响应的时刻
            fail_count = 0
        except Exception as e:
            fail_count += 1
            if fail_count <= 3:
                print(f"\n[记录] GETJPOS 失败 ({fail_count}): {e}")
            stop_flag.wait(0.05)
            continue
        recorder.log_feedback(t, q)
        stop_flag.wait(sample_interval)


def _parse_free(resp) -> float:
    """从入队应答 'ok queued free=N' 提取剩余队列容量; 无应答/流式返回 NaN"""
    if resp:
        m = _FREE_RE.search(resp)
        if m:
            return float(m.group(1))
    return float("nan")


def _wait_motion_done(robot, timeout: float = 30.0) -> bool:
    """等待固件运动完成的异步 'ok' 广播 (move_j 的 wait_ack 只等入队应答)"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        for line in robot.drain_async_lines():
            if line.strip() == "ok":
                return True
        time.sleep(0.02)
    return False


def send_trajectory_to_robot(
    traj: dict,
    port: str = "/dev/ttyACM0",
    speed: int | None = None,
    loop: bool = True,
    realtime_plot: bool = True,
    stream: bool = False,
    sample_interval: float = 0.02,
    sample_feedback: bool = True,
    speed_factor: float = 0.2,
    acc_percent: float = 100.0,
    acc_base: list[float] | None = None,
):
    """
    通过 RobotArmSDK.move_j 逐点下发轨迹, 同时记录:
      - 每条指令及其下发时间戳
      - 后台轮询 GETJPOS 得到的实时关节角及时间戳
    数据保存到 trajectory_logs/*.npz, 并可选实时绘图对比。
    RobotArmSDK 内部线程安全 (后台读线程 + 应答特征匹配), 无需外部串口锁。

    Parameters
    ----------
    traj          : generate_sine_trajectory 返回的字典
    port          : 串口端口
    speed         : move_j 的速度参数 (可选)
    loop          : True 时循环下发直到 Ctrl+C 中断
    realtime_plot : True 时打开实时绘图窗口
    stream        : True 时流式下发 (wait_ack=False, 发后即忘),
                    可达更高指令频率; 注意固件指令队列有限 (free=15),
                    若固件执行速率低于下发速率会溢出丢命令
    sample_interval: 反馈采样最小间隔 (秒), 默认 0.02,
                    防止高频 GETJPOS 压垮固件
    sample_feedback : False 时不启动 GETJPOS 反馈采样线程,
                    串口带宽全部留给 move_j 下发
    speed_factor  : 速度单位->电机轴 r/s 换算系数, 使能后下发,
                    固件夹取到 [0.01, 1.0], 默认 0.2
    acc_percent   : 加速度百分比, 使能后下发, 固件夹取到 [0, 100], 默认 100
    acc_base      : 6 关节加速度基值 (r/s²), 使能后下发, 固件逐个夹取到
                    [0, 200], 默认 [150, 100, 200, 200, 200, 200]
    """
    from dummy_robot_sdk import RobotArmSDK, SDKError

    if acc_base is None:
        acc_base = [150.0, 100.0, 200.0, 200.0, 200.0, 200.0]

    robot = RobotArmSDK(port)
    print(f"[下发] 已连接机械臂: {port}")

    # import ipdb;ipdb.set_trace()
    # robot.get_motor_currents()
    # robot.get_motor_temperatures()
    # import ipdb;ipdb.set_trace()

    recorder = TrajectoryRecorder()
    stop_flag = threading.Event()    # 停止反馈采样
    exit_flag = threading.Event()    # 绘图窗口关闭 → 请求停止下发
    sampler = None

    plot = RealtimeJointPlot(recorder) if realtime_plot else None

    # 阻塞式: 等待入队应答 (可感知 queue full); 流式: 发后即忘, 靠固件队列缓冲
    send_kw = dict(wait_ack=False) if stream else dict(wait_ack=True)
    if stream:
        print("[下发] 流式模式: 发后即忘, 注意固件队列溢出风险")

    def dispatch():
        """使能 + 定位 + 循环下发, 开启实时绘图时在后台线程中运行"""
        nonlocal sampler
        try:
            # 使能 & 设置连续轨迹模式
            print("[下发] 使能机器人 ...")
            robot.start()

            # 使能后设置速度/加速度参数 (固件侧各自夹取到有效范围)
            print(f"[下发] 设置速度系数 speed_factor={speed_factor} ...")
            robot.set_speed_factor(speed_factor)
            print(f"[下发] 设置加速度百分比 acc_percent={acc_percent} ...")
            robot.set_acc_percent(acc_percent)
            print(f"[下发] 设置加速度基值 acc_base={acc_base} ...")
            robot.set_acc_base(acc_base)

            # print("[下发] 设置连续轨迹模式 (CMDMODE=3) ...")
            # robot.set_command_mode(3)

            # 启动反馈采样线程 (使能后即可读取关节角); --no_sample 时跳过
            if sample_feedback:
                sampler = threading.Thread(
                    target=_feedback_sampler,
                    args=(robot, recorder, stop_flag, sample_interval),
                    daemon=True)
                sampler.start()
            else:
                print("[下发] 已禁用反馈采样 (--no_sample), 仅下发指令")

            # move robot to traj start position
            # SDK 内部以 %.3f 格式化关节角, 无需手动取整
            j_start = list(traj["positions"][0])
            t_cmd = time.time()
            resp = robot.move_j(j_start, speed=100, wait_ack=True)
            recorder.log_command(t_cmd, j_start, _parse_free(resp))
            # wait_ack 只等入队应答, 需再等运动完成广播, 确保到达轨迹起点
            if not _wait_motion_done(robot):
                print("[下发] 警告: 等待起点定位运动完成超时")
            
            time.sleep(5.0)

            positions = traj["positions"]
            delta_t = traj["delta_t"]
            n_points = len(traj["time"])

            print(f"[下发] 开始下发轨迹, 共 {n_points} 个点, delta_t={delta_t:.4f}s, "
                  f"循环={'开' if loop else '关'} ...")

            try:
                round_i = 0
                while not exit_flag.is_set():
                    round_i += 1
                    if loop:
                        print(f"\n[下发] ===== 第 {round_i} 轮 =====")

                    t_start = time.time()
                    for i in range(n_points):
                        if exit_flag.is_set():
                            break
                        j = list(positions[i])
                        t_cmd = time.time()   # 指令下发时刻
                        try:
                            resp = robot.move_j(j, speed=speed, **send_kw)
                        except SDKError as e:
                            # 固件过载 (queue full / 串口写超时): 清队列缓一缓再续发
                            print(f"\n[下发] 点{i} 固件暂不可用({e}), 停发1s并清空队列 ...")
                            try:
                                robot.stop()
                            except Exception:
                                pass
                            stop_flag.wait(1.0)
                            continue
                        recorder.log_command(t_cmd, j, _parse_free(resp))

                        # 按频率节奏等待
                        expected_t = traj["time"][i]
                        elapsed = time.time() - t_start
                        wait_time = expected_t - elapsed
                        if wait_time > 0:
                            time.sleep(wait_time)

                        if i % 10 == 0:
                            qs = " ".join(f"J{k+1}={j[k]:7.2f}" for k in range(len(j)))
                            print(f"\r  点 {i:4d}/{n_points}  t={traj['time'][i]:6.3f}s  "
                                  f"{qs}", end="")

                    print(f"\n[下发] 第 {round_i} 轮下发完成, 总耗时 {time.time() - t_start:.2f}s")

                    if not loop:
                        break
            except KeyboardInterrupt:
                print(f"\n[下发] 用户中断, 停止下发 (共完成 {round_i} 轮)")

            if exit_flag.is_set():
                print("\n[下发] 绘图窗口已关闭, 停止下发")
        except Exception as e:
            print(f"\n[下发] 异常终止: {e!r}")
            raise

    try:
        if plot is not None:
            # TkAgg 要求 GUI 事件循环在主线程: 下发移到后台线程,
            # 主线程阻塞在 plt.show() 上, 关闭窗口即停止下发
            dispatch_thread = threading.Thread(target=dispatch, daemon=True)
            dispatch_thread.start()
            plot.fig.canvas.mpl_connect("close_event", lambda _e: exit_flag.set())
            try:
                plot.show_blocking()
            except KeyboardInterrupt:
                print("\n[下发] 用户中断")
            exit_flag.set()
            try:
                dispatch_thread.join(timeout=30.0)
            except KeyboardInterrupt:
                pass
        else:
            dispatch()

    finally:
        stop_flag.set()
        if sampler is not None:
            sampler.join(timeout=2.0)
        log_path = LOG_DIR / f"sine_log_{time.strftime('%Y%m%d_%H%M%S')}.npz"
        recorder.save(log_path)
        print("[下发] 回休息位 & 断开 ...")
        # robot.reset()
        # time.sleep(5)
        # robot.disable()
        robot.close()


# ======================================================================
# 4. 离线绘制已保存的日志
# ======================================================================
def plot_saved_log(log_path: str):
    """绘制 TrajectoryRecorder 保存的 npz 日志, 用于事后分析指令跟随情况"""
    import matplotlib.pyplot as plt

    _setup_cjk_font()

    data = np.load(log_path)
    cmd_t, cmd_q = data["cmd_time"], data["cmd_positions"]
    fbk_t, fbk_q = data["fbk_time"], data["fbk_positions"]
    if len(cmd_t) == 0 and len(fbk_t) == 0:
        print(f"[离线绘图] 日志为空: {log_path}")
        return

    t0 = cmd_t[0] if len(cmd_t) else fbk_t[0]
    n_joints = (cmd_q if len(cmd_t) else fbk_q).shape[1]

    fig = plt.figure(figsize=(12, 10))
    gs = fig.add_gridspec(4, 2)
    axes = [fig.add_subplot(gs[r, c]) for r in range(3) for c in range(2)]
    for k in range(n_joints):
        ax = axes[k]
        if len(cmd_t):
            ax.plot(cmd_t - t0, cmd_q[:, k], "b-", lw=1.2, label="指令")
        if len(fbk_t):
            ax.plot(fbk_t - t0, fbk_q[:, k], "r-", lw=1.0, label="反馈")
        ax.set_title(f"关节 {k + 1}")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("角度 (°)")
        ax.grid(True, alpha=0.3)
        if k == 0:
            ax.legend(loc="upper right")
    fig.suptitle(f"指令跟随分析: {Path(log_path).name}")

    # 底部子图: 入队应答中的剩余队列容量 free 变化 (新日志才含该字段)
    # 占单个子图位置 (与关节图同尺寸, 便于对齐时间轴), 右侧留白
    if "cmd_free" in data.files and len(cmd_t):
        cmd_free = data["cmd_free"]
        if np.any(~np.isnan(cmd_free)):
            ax_free = fig.add_subplot(gs[3, 0])
            ax_free.plot(cmd_t - t0, cmd_free, "g-", lw=1.0,
                         label="free (剩余队列容量)")
            ax_free.set_title("固件队列剩余容量 free")
            ax_free.set_xlabel("t (s)")
            ax_free.set_ylabel("free")
            ax_free.grid(True, alpha=0.3)
            ax_free.legend(loc="upper right")
            ax_free.set_ylim(-0.5, np.nanmax(cmd_free) + 1.0)

    fig.tight_layout()
    plt.show()


# ======================================================================
# main
# ======================================================================
def main():
    parser = argparse.ArgumentParser(description="正弦轨迹测试")
    parser.add_argument("--Ts", type=float, nargs="+", default=[2.0],
                        help="各关节 1/4 正弦周期 (秒), 传 1 个值广播到 6 关节, "
                             "或传 6 个值分别指定, 默认 2.0")
    parser.add_argument("--freq", type=float, default=50.0,
                        help="采样频率 f (Hz), 默认 50")
    parser.add_argument("--amplitude", type=float, nargs="+",
                        # default=[45.0, 45.0, 30.0, 45.0, 45.0, 180.0],
                        default=[45.0, 0, 45.0, 0, 0, 0],
                        help="各关节正弦幅值 (度), 传 1 个值广播到 6 关节, "
                             "或传 6 个值分别指定, 默认 90 0 0 0 0 0 (仅关节1运动)")
    parser.add_argument("--base_pose", type=float, nargs=6,
                        # default=[0.0, 0.0, 90.0, 0.0, 0.0, 0.0],
                        default=[0.0, -75.0, 90.0, 0.0, 0.0, 0.0],
                        help="基准关节位姿 (度), 6个值, 默认 0.0, -75.0, 180.0, 0.0, 0.0, 0.0")
    parser.add_argument("--vis_only", action="store_true",
                        help="仅可视化, 不下发机械臂")
    parser.add_argument("--send_only", action="store_true",
                        help="仅下发机械臂, 不启动 viser 可视化")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0",
                        help="串口端口, 默认 /dev/ttyACM0")
    parser.add_argument("--speed", type=int, default=100,
                        help="move_j 速度参数 (可选)")
    parser.add_argument("--speed_factor", type=float, default=0.1,
                        help="速度单位->电机轴 r/s 换算系数, 使能后下发, "
                             "固件夹取[0.01,1.0], 默认 0.2")
    parser.add_argument("--acc_percent", type=float, default=15.0,
                        help="加速度百分比, 使能后下发, 固件夹取[0,100], 默认 100")
    parser.add_argument("--acc_base", type=float, nargs=6,
                        default=[150.0, 100.0, 200.0, 200.0, 200.0, 200.0],
                        help="6关节加速度基值(r/s²), 使能后下发, 固件逐个夹取"
                             "[0,200], 默认 150 100 200 200 200 200")
    parser.add_argument("--vis_step", type=int, default=1,
                        help="可视化跳帧步长, 默认 1 (每帧)")
    parser.add_argument("--no_plot", action="store_true",
                        help="下发时不打开实时绘图窗口 (仍会保存数据)")
    parser.add_argument("--plot_log", type=str, default=None,
                        help="离线绘制已保存的 npz 日志路径, 不连接机械臂")
    parser.add_argument("--stream", action="store_true",
                        help="流式下发 (wait_ok=False), 指令频率更高, 但无逐点完成确认")
    parser.add_argument("--sample_interval", type=float, default=0.02,
                        help="反馈采样最小间隔 (秒), 默认 0.02 (上限约 50Hz)")
    parser.add_argument("--no_sample", action="store_true",
                        help="禁用 GETJPOS 反馈采样, 仅下发 move_j 指令")
    args = parser.parse_args()

    # ---------- 0. 离线绘图模式 ----------
    if args.plot_log:
        plot_saved_log(args.plot_log)
        return

    # ---------- 1. 生成轨迹 ----------
    traj = generate_sine_trajectory(
        Ts=args.Ts,
        freq=args.freq,
        amplitude_deg=args.amplitude,
        base_pose_deg=args.base_pose,
    )

    # 打印轨迹摘要 (逐关节)
    print("  各关节位置范围 / 最大速度 / 最大加速度:")
    for k in range(6):
        pk = traj["positions"][:, k]
        vk = np.abs(traj["velocities"][:, k]).max()
        ak = np.abs(traj["accelerations"][:, k]).max()
        print(f"    J{k+1}: [{pk.min():7.1f}, {pk.max():7.1f}]°  "
              f"vmax={vk:7.1f}°/s  amax={ak:8.1f}°/s²")

    # ---------- 2 & 3. 可视化 / 下发 ----------
    if args.vis_only:
        viewer = visualize_trajectory(traj, URDF_PATH, vis_step=args.vis_step)
        print("[完成] 按 Ctrl+C 退出可视化 ...")
        viewer.wait_for_close()

    elif args.send_only:
        send_trajectory_to_robot(traj, port=args.port, speed=args.speed,
                                 realtime_plot=not args.no_plot,
                                 stream=args.stream,
                                 sample_interval=args.sample_interval,
                                 sample_feedback=not args.no_sample,
                                 speed_factor=args.speed_factor,
                                 acc_percent=args.acc_percent,
                                 acc_base=args.acc_base)

    else:
        # 先可视化, 播放完成后再下发
        viewer = visualize_trajectory(traj, URDF_PATH, vis_step=args.vis_step)

        send_trajectory_to_robot(traj, port=args.port, speed=args.speed,
                                 realtime_plot=not args.no_plot,
                                 stream=args.stream,
                                 sample_interval=args.sample_interval,
                                 sample_feedback=not args.no_sample,
                                 speed_factor=args.speed_factor,
                                 acc_percent=args.acc_percent,
                                 acc_base=args.acc_base)
        print("[完成] 轨迹已下发, Ctrl+C 退出")
        viewer.wait_for_close()


if __name__ == "__main__":
    main()
