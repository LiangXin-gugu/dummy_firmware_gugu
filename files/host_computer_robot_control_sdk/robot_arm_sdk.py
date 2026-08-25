# -*- coding: utf-8 -*-
"""
dummy-ref-core-fw 六轴机械臂上位机控制 SDK
============================================

基于固件 ASCII 协议（USB-CDC 虚拟串口）封装，支持多线程并发调用。

协议背景（对应固件 Bsp/communication/ascii_processor.cpp 与
UserApp/protocols/ascii_protocol.cpp）：
    * 命令以行结束（'\\r' 或 '\\n'），单行最长 256 字节
    * '!' 开头：控制类（START/STOP/HOME/...），立即执行
    * '#' 开头：查询/参数类（GETJPOS/GETLPOS/SET_DCE_KP/...），立即执行
    * '>' '&' '@' 开头：运动类，固件先入 FIFO 后台执行

多线程安全设计要点：
    1. 发送原子性：所有命令在 _io_lock 保护下拼装成"一整行"后单次
       serial.write()，杜绝多线程 write 字节交织（固件端无法检测
       交织产生的错误命令行）。
    2. 应答匹配：固件对命令逐行串行处理，但应答行没有序号/回显，且
       运动类命令在运动完成后还会额外广播 "ok"（数量随 commandMode
       变化），因此不能用严格 FIFO 一问一答匹配。本 SDK 为每个等待
       应答的事务注册一个"应答特征正则"，后台读线程把每条应答行派
       发给第一个特征匹配的事务；无人匹配的异步行（运动完成广播、
       printf 调试输出等）进入旁路队列，可注册回调或主动读取。
    3. 注册顺序 == 发送顺序：事务注册与串口写入在同一把锁内完成，
       保证并发时等待队列的顺序与固件实际收到的命令顺序一致。

依赖：pip install pyserial

快速示例：
    from robot_arm_sdk import RobotArmSDK

    with RobotArmSDK("COM5") as arm:
        arm.start()
        arm.set_command_mode(1)
        arm.move_j([10, 20, 30, 40, 50, 60])
        print(arm.get_joint_pos())
        arm.stop()
"""

import argparse
import collections
import re
import threading
import time

try:
    import serial
except ImportError as _e:
    raise ImportError("本 SDK 依赖 pyserial，请先执行: pip install pyserial") from _e

__all__ = [
    "RobotArmSDK",
    "SDKError",
    "SDKTimeoutError",
    "SDKResponseError",
]


# ---------------------------------------------------------------------------
# 异常定义
# ---------------------------------------------------------------------------

class SDKError(Exception):
    """SDK 基础异常。"""


class SDKTimeoutError(SDKError):
    """等待应答超时。"""


class SDKResponseError(SDKError):
    """固件返回了 'error ...' 应答。"""

    def __init__(self, response):
        super().__init__(response)
        self.response = response


# ---------------------------------------------------------------------------
# 应答特征（与固件 Respond 的格式逐条对应）
# ---------------------------------------------------------------------------

# 运动类入队应答："ok queued free=N" 或 "error queue full"
_RESP_QUEUED = re.compile(r"^(ok queued free=\d+|error queue full)$")
# 关节/末端位姿查询："ok x.xx x.xx x.xx x.xx x.xx x.xx"（固件 %.2f 格式）
_RESP_6FLOAT = re.compile(r"^ok -?\d+\.\d+( -?\d+\.\d+){5}$")
# DCE 参数设置："ok/error SET MOTOR [n] DCE_KP [v]"
_RESP_SET_DCE = re.compile(r"^(ok|error) SET MOTOR \[\d+\] DCE_K[PID] \[\d+\]( is wrong)?$")
# 电机重启："ok/error REBOOT MOTOR [n]"
_RESP_REBOOT = re.compile(r"^(ok|error) REBOOT MOTOR \[\d+\]$")
# 命令模式切换："ok Set command mode to [n]"
_RESP_CMDMODE = re.compile(r"^ok Set command mode to \[\d+\]$")


class _PendingTx:
    """一个正在等待应答的事务。"""

    __slots__ = ("pattern", "event", "line")

    def __init__(self, pattern):
        self.pattern = pattern        # 应答特征正则（已编译）
        self.event = threading.Event()
        self.line = None              # 命中的应答行


# ---------------------------------------------------------------------------
# SDK 主类
# ---------------------------------------------------------------------------

class RobotArmSDK:
    """六轴机械臂 USB ASCII 协议控制 SDK（线程安全）。

    参数:
        port:        串口名，如 Windows "COM5" / Linux "/dev/ttyACM0"
        baudrate:    波特率（USB-CDC 实际忽略，保持默认即可）
        timeout:     默认应答等待超时（秒）
        async_queue_size: 异步应答行旁路队列的最大长度
    """

    LINE_TERMINATOR = "\n"      # 固件 '\r'/'\n' 均可作为行结束符
    MAX_LINE_LENGTH = 256       # 固件 MAX_LINE_LENGTH

    def __init__(self, port, baudrate=115200, timeout=2.0, async_queue_size=256):
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout

        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.05,       # 读线程小超时轮询，保证可及时退出
            write_timeout=1.0,
        )

        self._io_lock = threading.Lock()        # 保护"事务注册 + 串口写入"的原子性
        self._match_lock = threading.Lock()     # 保护等待队列与异步队列的派发
        self._pending = collections.deque()     # 等待应答的事务（FIFO）
        self._async_lines = collections.deque(maxlen=async_queue_size)
        self._line_callbacks = []               # 异步行回调列表

        self._closed = False
        self._reader = threading.Thread(target=self._reader_loop,
                                        name="RobotSDK-Reader", daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------
    # 生命周期管理
    # ------------------------------------------------------------------

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        """关闭串口并停止读线程（幂等，可多线程调用）。"""
        if self._closed:
            return
        self._closed = True
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        try:
            self._serial.close()
        except Exception:
            pass

    @property
    def is_open(self):
        return not self._closed and self._serial.is_open

    # ------------------------------------------------------------------
    # 底层收发核心
    # ------------------------------------------------------------------

    def _reader_loop(self):
        """后台读线程：拼行 -> 按特征派发给等待事务，否则进入异步旁路。

        注意：必须用 read(in_waiting) 取"已到字节"立即返回，不能用
        read(n) —— 后者会凑满 n 字节或耗满超时才返回，会给每条应答
        引入固定延迟（实测 50ms/条）。
        """
        buf = b""
        while not self._closed:
            try:
                waiting = self._serial.in_waiting
                data = self._serial.read(waiting if waiting > 0 else 1)
            except (serial.SerialException, OSError):
                if self._closed:
                    break
                time.sleep(0.05)
                continue
            if not data:
                continue
            # 固件应答行以 '\r\n' 结尾；'\r'/'\n' 统一当作行分隔符
            buf += data
            buf = buf.replace(b"\r", b"\n")
            parts = buf.split(b"\n")
            buf = parts[-1]                     # 最后一段可能是不完整的行
            for raw in parts[:-1]:
                if not raw:
                    continue
                try:
                    line = raw.decode("ascii")
                except UnicodeDecodeError:
                    continue
                self._dispatch_line(line)

    def _dispatch_line(self, line):
        """把一条应答行派发给特征匹配的等待事务，否则归入异步旁路。"""
        with self._match_lock:
            for tx in self._pending:
                if tx.pattern.match(line):
                    self._pending.remove(tx)
                    tx.line = line
                    tx.event.set()
                    return
            # 无人认领：运动完成广播 "ok"、printf 调试输出等
            self._async_lines.append(line)
            callbacks = list(self._line_callbacks)
        for cb in callbacks:
            try:
                cb(line)
            except Exception:
                pass

    def send_command(self, cmd, wait=True, pattern=None, timeout=None,
                     raise_on_error=True):
        """发送一条原始命令（线程安全）。

        参数:
            cmd:            命令行文本，不含行结束符，如 ">10,20,30,40,50,60"
            wait:           True 等待应答；False 发后即忘（适合高频流式下发）
            pattern:        wait=True 时必须提供应答特征（已编译正则）
            timeout:        等待超时（秒），缺省用构造函数的 timeout
            raise_on_error: 应答以 "error" 开头时是否抛出 SDKResponseError

        返回:
            wait=True  -> 应答行文本
            wait=False -> None
        """
        if self._closed:
            raise SDKError("SDK 已关闭")
        if len(cmd) > self.MAX_LINE_LENGTH:
            raise SDKError("命令超过固件单行上限 %d 字节" % self.MAX_LINE_LENGTH)

        payload = (cmd + self.LINE_TERMINATOR).encode("ascii")

        tx = None
        if wait:
            if pattern is None:
                raise SDKError("wait=True 时必须提供应答特征 pattern")
            tx = _PendingTx(pattern)

        # 注册事务与写入串口必须原子：保证等待顺序 == 固件收到命令的顺序，
        # 且整行单次 write，避免多线程字节交织。
        with self._io_lock:
            if tx is not None:
                self._pending.append(tx)
            try:
                self._serial.write(payload)
            except Exception:
                if tx is not None:
                    with self._match_lock:
                        try:
                            self._pending.remove(tx)
                        except ValueError:
                            pass
                raise

        if tx is None:
            return None

        if not tx.event.wait(timeout if timeout is not None else self._timeout):
            with self._match_lock:
                try:
                    self._pending.remove(tx)
                except ValueError:
                    pass    # 超时瞬间应答刚好到达，视为成功路径继续处理
            if tx.line is None:
                raise SDKTimeoutError("等待应答超时: %r" % cmd)

        if raise_on_error and tx.line.startswith("error"):
            raise SDKResponseError(tx.line)
        return tx.line

    # ------------------------------------------------------------------
    # 异步应答行（运动完成广播 / printf 调试输出等）
    # ------------------------------------------------------------------

    def register_line_callback(self, callback):
        """注册异步行回调：callback(line)。回调在读线程中执行，勿阻塞。"""
        with self._match_lock:
            self._line_callbacks.append(callback)

    def unregister_line_callback(self, callback):
        with self._match_lock:
            try:
                self._line_callbacks.remove(callback)
            except ValueError:
                pass

    def drain_async_lines(self):
        """取走并清空异步旁路队列，返回 [line, ...]。"""
        with self._match_lock:
            lines = list(self._async_lines)
            self._async_lines.clear()
        return lines

    # ------------------------------------------------------------------
    # 控制类命令（'!'，立即执行）
    # ------------------------------------------------------------------

    def start(self, timeout=None):
        """!START 使能电机。"""
        return self.send_command("!START", pattern=re.compile(r"^Started ok$"),
                                 timeout=timeout)

    def stop(self, timeout=None):
        """!STOP 急停并清空运动队列。"""
        return self.send_command("!STOP", pattern=re.compile(r"^Stopped ok$"),
                                 timeout=timeout)

    def home(self, timeout=None):
        """!HOME 回零。注意固件侧回零耗时较长，默认超时可能不够。"""
        return self.send_command("!HOME", pattern=re.compile(r"^Started ok$"),
                                 timeout=timeout)

    def calibrate_home_offset(self, timeout=None):
        """!CALIBRATION 校准 home 偏置。"""
        return self.send_command("!CALIBRATION",
                                 pattern=re.compile(r"^calibration ok$"),
                                 timeout=timeout)

    def reset(self, timeout=None):
        """!RESET 复位到休息位。"""
        return self.send_command("!RESET", pattern=re.compile(r"^Started ok$"),
                                 timeout=timeout)

    def disable(self, timeout=None):
        """!DISABLE 去使能。"""
        return self.send_command("!DISABLE", pattern=re.compile(r"^Disabled ok$"),
                                 timeout=timeout)

    # ------------------------------------------------------------------
    # 查询类命令（'#'）
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_6_floats(line):
        return [float(x) for x in line.split()[1:7]]

    def get_joint_pos(self, timeout=None):
        """#GETJPOS 读取当前 6 个关节角（度），返回 [j1..j6]。"""
        line = self.send_command("#GETJPOS", pattern=_RESP_6FLOAT, timeout=timeout)
        return self._parse_6_floats(line)

    def get_cartesian_pos(self, timeout=None):
        """#GETLPOS 读取末端位姿，返回 [X, Y, Z, A, B, C]。"""
        line = self.send_command("#GETLPOS", pattern=_RESP_6FLOAT, timeout=timeout)
        return self._parse_6_floats(line)

    # ------------------------------------------------------------------
    # 参数类命令（'#'）
    # ------------------------------------------------------------------

    def _check_node(self, node):
        if not 1 <= node <= 6:
            raise SDKError("电机节点号必须在 1~6，当前: %r" % node)

    def set_dce_kp(self, node, kp, timeout=None):
        """#SET_DCE_KP node kp 设置关节位置环 Kp。"""
        self._check_node(node)
        return self.send_command("#SET_DCE_KP %d %d" % (node, kp),
                                 pattern=_RESP_SET_DCE, timeout=timeout)

    def set_dce_ki(self, node, ki, timeout=None):
        """#SET_DCE_KI node ki 设置关节位置环 Ki。"""
        self._check_node(node)
        return self.send_command("#SET_DCE_KI %d %d" % (node, ki),
                                 pattern=_RESP_SET_DCE, timeout=timeout)

    def set_dce_kd(self, node, kd, timeout=None):
        """#SET_DCE_KD node kd 设置关节位置环 Kd。"""
        self._check_node(node)
        return self.send_command("#SET_DCE_KD %d %d" % (node, kd),
                                 pattern=_RESP_SET_DCE, timeout=timeout)

    def reboot_motor(self, node, timeout=None):
        """#REBOOT node 重启指定电机。"""
        self._check_node(node)
        return self.send_command("#REBOOT %d" % node,
                                 pattern=_RESP_REBOOT, timeout=timeout)

    def set_command_mode(self, mode, timeout=None):
        """#CMDMODE mode 切换运动命令模式（决定 '>' '@' 的解析与应答行为）。"""
        return self.send_command("#CMDMODE %d" % mode,
                                 pattern=_RESP_CMDMODE, timeout=timeout)

    # ------------------------------------------------------------------
    # 运动类命令（'>' '&' '@'，固件先入 FIFO 后台执行）
    # ------------------------------------------------------------------

    @staticmethod
    def _format_points(points):
        if len(points) != 6:
            raise SDKError("必须提供 6 个坐标值，当前 %d 个" % len(points))
        return ",".join("%.3f" % float(v) for v in points)

    def move_j(self, joints, speed=None, wait_ack=True, timeout=None):
        """'>j1,j2,...,j6[,speed]' 关节空间点到点运动。

        wait_ack=True  等待入队应答（ok queued free=N / error queue full）；
                       注意固件在运动完成后还会异步广播 "ok"，可用
                       drain_async_lines()/register_line_callback() 获取。
        wait_ack=False 发后即忘，适合高频流式下发（如 50Hz 轨迹流）。
        """
        cmd = ">" + self._format_points(joints)
        if speed is not None:
            cmd += ",%.3f" % float(speed)
        if not wait_ack:
            self.send_command(cmd, wait=False)
            return None
        return self.send_command(cmd, pattern=_RESP_QUEUED, timeout=timeout)

    def move_j_seq(self, joints, speed=None, wait_ack=True, timeout=None):
        """'&...' 与 '>' 固件侧解析相同，作为等价的关节空间命令前缀提供。"""
        cmd = "&" + self._format_points(joints)
        if speed is not None:
            cmd += ",%.3f" % float(speed)
        if not wait_ack:
            self.send_command(cmd, wait=False)
            return None
        return self.send_command(cmd, pattern=_RESP_QUEUED, timeout=timeout)

    def move_l(self, pose, speed=None, wait_ack=True, timeout=None):
        """'@x,y,z,a,b,c[,speed]' 末端位姿直线运动（固件内部逆解）。"""
        cmd = "@" + self._format_points(pose)
        if speed is not None:
            cmd += ",%.3f" % float(speed)
        if not wait_ack:
            self.send_command(cmd, wait=False)
            return None
        return self.send_command(cmd, pattern=_RESP_QUEUED, timeout=timeout)


# ---------------------------------------------------------------------------
# 多线程演示：两个线程各以 50Hz 并发下发 运动命令 与 查询命令
# 运行: python robot_arm_sdk.py --port COM5
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="机械臂 SDK 多线程并发演示")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="串口名，如 COM5")
    parser.add_argument("--duration", type=float, default=5.0, help="演示时长（秒）")
    args = parser.parse_args()

    PERIOD = 1.0 / 50.0

    with RobotArmSDK(args.port) as arm:
        arm.start()
        arm.set_command_mode(1)

        stats = {"motion_ok": 0, "motion_err": 0, "pose_ok": 0, "pose_err": 0}
        stop_flag = threading.Event()

        def motion_worker():
            """线程 1：50Hz 下发 '>10,20,30,40,50,60'。"""
            next_t = time.perf_counter()
            while not stop_flag.is_set():
                try:
                    arm.move_j([10, 20, 30, 40, 50, 60])
                    stats["motion_ok"] += 1
                except SDKError:
                    stats["motion_err"] += 1
                next_t += PERIOD
                delay = next_t - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)

        def query_worker():
            """线程 2：50Hz 下发 '#GETJPOS'。"""
            next_t = time.perf_counter()
            while not stop_flag.is_set():
                try:
                    arm.get_joint_pos()
                    stats["pose_ok"] += 1
                except SDKError:
                    stats["pose_err"] += 1
                next_t += PERIOD
                delay = next_t - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)

        t1 = threading.Thread(target=motion_worker)
        t2 = threading.Thread(target=query_worker)
        t1.start()
        t2.start()
        time.sleep(args.duration)
        stop_flag.set()
        t1.join()
        t2.join()

        arm.stop()
        print("运动命令: 成功 %d / 失败 %d" % (stats["motion_ok"], stats["motion_err"]))
        print("姿态查询: 成功 %d / 失败 %d" % (stats["pose_ok"], stats["pose_err"]))
        print("异步行示例(运动完成广播等): %r" % arm.drain_async_lines()[:10])
