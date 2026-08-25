# 上位机机械臂控制 SDK 设计文档

> 配套代码：[`robot_arm_sdk.py`](./robot_arm_sdk.py)
> 目标固件：`dummy-ref-core-fw`（STM32F405 + FreeRTOS 六轴机械臂参考固件）
> 通信方式：USB-CDC 虚拟串口，ASCII 行协议

本文档总结该 SDK 的编写过程：先分析固件侧的命令处理行为，再推导出上位机
多线程并发调用时的全部风险点，最后给出 SDK 的对应设计方案。

---

## 1. 需求背景

### 1.1 典型使用场景

上位机存在多个业务线程，各以 50Hz 频率通过同一个 USB 串口下发指令，例如：

- 线程 A：持续下发运动指令 `>10,20,30,40,50,60`
- 线程 B：持续下发查询指令 `#GETJPOSE`

SDK 必须保证：**任意多个线程同时调用相同或不同的接口，命令不串行错乱、
应答不张冠李戴、字节流不交织损坏**。

### 1.2 固件支持的命令分类

参考 `project_threads_and_command_pipeline.md`，命令按首字符分三类
（以 USB 通道 `OnUsbAsciiCmd` 为例）：

| 首字符 | 命令类型 | 执行方式 | 示例 |
| --- | --- | --- | --- |
| `!` | 控制类 | 立即执行 | `!START`, `!STOP`, `!HOME`, `!CALIBRATION`, `!RESET`, `!DISABLE` |
| `#` | 查询/参数 | 立即执行 | `#GETJPOS`, `#GETLPOS`, `#SET_DCE_KP`, `#REBOOT`, `#CMDMODE` |
| `>` `&` `@` | 运动类 | 先入队 FIFO，后台解析执行 | `>j1..j6[,spd]`, `@x,y,z,a,b,c[,spd]` |

---

## 2. 固件侧行为分析（SDK 设计的依据）

编写 SDK 前，先逐层确认了固件的命令接收与应答行为。这些结论直接决定了
SDK 的架构选择。

### 2.1 USB 接收：单包串行流水线，天然无并发冲突

关键代码：`Bsp/communication/interface_usb.cpp`

```
CDC_Receive_FS()            ← USB 中断上下文
  └─ usb_rx_process_packet(): 记录 rx_buf/rx_len，置 data_pending，释放信号量
       └─ UsbServerTask():  处理当前包 → 才重新武装 USBD_CDC_ReceivePacket()
```

- 端点在同一时刻**只允许一包在途**：上一包没处理完，端点会 NAK 后续数据
  （USB 硬件流控），数据不丢、不覆盖，只是延迟；
- 所有命令最终由**同一个 `UsbServerTask` 线程**逐行串行解析执行。

**结论 1**：即使上位机两个线程的包"同时到达"，固件侧也严格串行处理，
不存在固件内部冲突。处理顺序 = USB 总线上的到达顺序。

### 2.2 解析：静态行缓冲，上位机字节交织无法被检测

关键代码：`Bsp/communication/ascii_processor.cpp`

- `ASCII_protocol_parse_stream()` 用 `static parse_buffer` 按 `\r`/`\n` 切行，
  支持**跨 USB 包拼行**；
- 固件只认"完整的一行"，**无法识别**字节流在行内被交织损坏的情况。

**结论 2**：如果上位机两个线程各自直接 `write()` 串口，且某次写入被操作系统
拆成多个 USB 传输并与另一线程交织，固件会收到形如 `>10,2#GETJPOS0,30...`
的"合法行"并错误执行。**防交织必须是上位机的责任。**

### 2.3 应答格式：逐条枚举

| 命令 | 固件应答（`Respond`，以 `\r\n` 结尾） |
| --- | --- |
| `!STOP` | `Stopped ok` |
| `!START` / `!HOME` / `!RESET` | `Started ok` |
| `!CALIBRATION` | `calibration ok` |
| `!DISABLE` | `Disabled ok` |
| `#GETJPOS` / `#GETLPOS` | `ok %.2f %.2f %.2f %.2f %.2f %.2f` |
| `#SET_DCE_KP/KI/KD n v` | `ok SET MOTOR [n] DCE_KP [v]` 或 `error ... is wrong` |
| `#REBOOT n` | `ok REBOOT MOTOR [n]` 或 `error REBOOT MOTOR [n]` |
| `#CMDMODE n` | `ok Set command mode to [n]` |
| `>` `&` `@`（入队） | `ok queued free=N` 或 `error queue full` |
| `!` + 未知关键字 | **无应答** |

### 2.4 关键发现：运动类命令的应答数量不固定

关键代码：`Robot/instances/dummy_robot.cpp` `CommandHandler::ParseCommand()`

运动命令（`>`/`&`/`@`）除 USB 任务立即返回的 `ok queued free=N` 外，
后台消费线程处理时还会**额外广播**应答，且随 `commandMode` 变化：

| commandMode | 额外广播 |
| --- | --- |
| SEQUENTIAL / CONTINUES | 运动完成后广播 1 条裸 `ok` |
| INTERRUPTABLE | 广播 `context->MoveJ succeeded` + `ok`（或 `... failed`） |

**结论 3**：固件协议**不是**"一问一答"。不能假设"发 1 条命令 = 收 1 条应答"，
否则后续应答会整体错位。这是 SDK 应答匹配方案的直接设计依据。

---

## 3. 多线程直接写串口的三大风险

在没有 SDK 封装、多个线程共享一个串口句柄时：

| # | 风险 | 后果 |
| --- | --- | --- |
| 1 | **写侧字节交织**：两线程 write 被 OS 拆分交织发送 | 固件解析出错误命令行且无法检测 |
| 2 | **读侧竞争**：多线程各自 `readline()` 抢同一个串口 | 应答字节被随机分给不同线程，行被撕碎 |
| 3 | **应答无序号**：固件应答不含命令回显/序号，运动命令还有额外广播 | 无法把应答与发起线程一一对应 |

SDK 的全部设计都围绕消除这三个风险展开。

---

## 4. SDK 总体架构

### 4.1 组件构成

```python
class RobotArmSDK:
    _io_lock          # 锁①：保护"事务注册 + 串口写入"原子性（发送方向）
    _match_lock       # 锁②：保护等待队列/异步队列的派发（接收方向）
    _pending          # deque：等待应答的事务（FIFO）
    _async_lines      # deque(maxlen)：无人认领的异步行旁路队列
    _line_callbacks   # 异步行回调列表
    _reader           # 后台读线程（串口的唯一读者）

class _PendingTx:     # 一次"发命令-等回复"事务
    pattern           # 应答特征正则（已编译）
    event             # threading.Event，阻塞/唤醒等待线程
    line              # 命中的应答文本
```

### 4.2 数据流

```
业务线程 A ─┐  send_command(">...")          ┌─→ 匹配 tx_motion.pattern → 唤醒线程 A
业务线程 B ─┤  send_command("#GETJPOS")      │
            │   │                            │
            │   ▼ [_io_lock 临界区]           │
            │   ①登记 _PendingTx 到 _pending  │
            │   ②整行单次 serial.write()      │
            │                                │
            ▼                                │
        USB 串口 ────────────────────────► 固件（串行处理，按序应答）
            ▲                                │
            │                          [_reader 读线程：唯一读者]
            └────────────────────────────────┤
                                        _dispatch_line():
                                          逐行用正则匹配 _pending
                                          ├─ 命中 → tx.line + event.set()
                                          └─ 未命中 → _async_lines + 回调
```

### 4.3 API 总览

| SDK 方法 | 固件命令 | 类型 |
| --- | --- | --- |
| `start() stop() home() calibrate_home_offset() reset() disable()` | `!START` `!STOP` `!HOME` `!CALIBRATION` `!RESET` `!DISABLE` | 控制类 |
| `get_joint_pos() get_cartesian_pos()` | `#GETJPOS` `#GETLPOS` | 查询类 |
| `set_dce_kp/ki/kd(node, v) reboot_motor(node) set_command_mode(mode)` | `#SET_DCE_*` `#REBOOT` `#CMDMODE` | 参数类 |
| `move_j(pts[,spd]) move_j_seq() move_l(pose[,spd])` | `>` `&` `@` | 运动类 |
| `send_command(raw, ...)` | 任意原始命令 | 底层接口 |
| `register_line_callback() drain_async_lines()` | — | 异步应答处理 |

---

## 5. 线程安全设计详解（核心）

### 5.1 发送原子性：整行预拼装 + `_io_lock` 单次 write

**对应风险 #1（字节交织）。**

```python
payload = (cmd + "\n").encode("ascii")      # 先拼成完整字节串
with self._io_lock:
    ...
    self._serial.write(payload)              # 一次 write，整行发出
```

两层保障：

1. 整行在写入前已是一个完整 `bytes` 对象；
2. `_io_lock` 保证任意时刻只有一个线程在写串口。

即使底层驱动拆分传输，也是"一行拆成的多个片段"顺序到达，固件跨包拼行后
仍是完整命令；**绝不会出现两条命令的字节互相穿插**。

### 5.2 登记顺序 == 固件接收顺序

**对应风险 #3 的排序基础。**

"把事务 append 进 `_pending`"与"write 串口"放在**同一个 `_io_lock` 临界区**
内。反例（若无锁）：

```
线程A append(tx_A) → 被切走
线程B append(tx_B) → write(B)          ← B 命令先到固件
线程A                 write(A)
```

固件先回 B，但 `tx_A` 排在队列前面。虽然特征匹配多数情况能纠错，但当
**两个并发事务应答特征相同**（如两线程同时 `#GETJPOS`）时，FIFO 顺序是唯一
裁决依据。原子化后，任何并发组合下登记顺序与固件接收顺序严格一致。

### 5.3 专职读线程：串口的唯一读者

**对应风险 #2（读侧竞争）。**

构造函数启动 daemon 线程 `_reader` 独占读串口，业务线程从不直接 read。
读线程切出完整行后调用 `_dispatch_line()` 派发，从根上消除多线程抢读。

### 5.4 应答特征匹配 + 异步旁路

**对应风险 #3（应答无序号）与 2.4 节发现（应答数量不固定）。**

固件应答无序号/回显，且运动命令会延迟广播额外应答，故**不能用严格
FIFO 一问一答**。方案：

- 每个等待事务携带**应答特征正则**（如 `#GETJPOS` 期望
  `^ok -?\d+\.\d+( -?\d+\.\d+){5}$`，运动命令期望
  `^(ok queued free=\d+|error queue full)$`）；
- 读线程对每条应答行，遍历 `_pending` 交给**第一个特征命中**的事务；
- 无人命中的行（运动完成裸 `ok`、printf 调试输出）进 `_async_lines`
  旁路队列并触发回调，**不阻塞、不误投**任何等待事务。

示例：运动完成后广播的裸 `ok` 到达时，若此时线程 B 正在等 `#GETJPOS`
（要求 6 浮点格式），裸 `ok` 不匹配 B 的特征，自动滑入旁路——避免了
FIFO 方案下必然发生的错位。

### 5.5 僵尸事务清理与超时竞态

事务必须从 `_pending` 可靠移除，否则会成为"僵尸"吞掉后续无关应答：

- **write 失败路径**：在异常处理中持 `_match_lock` 移除已登记事务；
- **超时路径**：`event.wait(timeout)` 失败后移除事务。

超时瞬间存在竞态窗口：

```
业务线程: event.wait() 刚返回 False（超时）
读线程:   同一瞬间匹配成功 → remove(tx) → tx.line = line → event.set()
```

因此 `remove` 抛 `ValueError` 不视为错误；最终判据是
`tx.line is None`——只有应答确实没到才抛 `SDKTimeoutError`，
否则"起死回生"走成功路径。

### 5.6 回调安全

`_dispatch_line()` 在锁内拷贝回调列表、**锁外**执行回调：
用户回调里即使再次调用 SDK（内部会取 `_io_lock`），也不会造成
读线程自锁或阻塞应答派发；单个回调异常被捕获，不影响其他回调。

### 5.7 锁约定与死锁预防

| 锁 | 职责 | 持有期间的耗时操作 |
| --- | --- | --- |
| `_io_lock` | 事务登记 + 串口 write | write（可能阻塞） |
| `_match_lock` | `_pending`/`_async_lines`/回调派发 | 无 |

全局唯一嵌套方向：`_io_lock → _match_lock`（仅 write 异常清理路径），
读线程只取 `_match_lock`、从不取 `_io_lock`，**不存在 AB-BA 死锁**。
两锁分离还带来吞吐收益：接收派发与命令发送互不阻塞。

### 5.8 风险—防护对照总表

| 风险 | 防护手段 |
| --- | --- |
| 多线程 write 字节交织 | 整行预拼装 + `_io_lock` 内单次 write |
| 等待顺序与接收顺序不一致 | 登记与写入同锁原子化 |
| 多线程抢读串口 | 唯一读线程 `_reader` |
| 共享队列并发读写 | `_match_lock` 全程保护 |
| 应答无序号/数量不固定 | 特征正则匹配 + 异步旁路队列 |
| write 失败/超时留下僵尸事务 | 异常与超时路径均 `remove(tx)` |
| 超时瞬间应答恰好到达 | 以 `tx.line is None` 为最终判据 |
| 用户回调阻塞/递归调用 SDK | 锁内拷列表、锁外执行 |
| 锁嵌套死锁 | 固定嵌套方向 `_io_lock → _match_lock` |

---

## 6. 使用示例

### 6.1 基本用法

```python
from robot_arm_sdk import RobotArmSDK

with RobotArmSDK("COM5") as arm:
    arm.start()                     # !START
    arm.set_command_mode(1)         # #CMDMODE 1
    arm.move_j([10, 20, 30, 40, 50, 60])
    print(arm.get_joint_pos())      # -> [j1..j6]
    arm.stop()                      # !STOP
```

### 6.2 多线程并发（50Hz 运动 + 50Hz 查询）

```python
import threading, time

stop_flag = threading.Event()

def motion_worker():
    while not stop_flag.is_set():
        arm.move_j([10, 20, 30, 40, 50, 60])   # 等 "ok queued free=N"
        time.sleep(1 / 50)

def query_worker():
    while not stop_flag.is_set():
        joints = arm.get_joint_pos()           # 等 "ok x.xx ..."
        time.sleep(1 / 50)

threading.Thread(target=motion_worker).start()
threading.Thread(target=query_worker).start()
```

`robot_arm_sdk.py` 的 `__main__` 内置了该场景的完整可运行演示：

```
python robot_arm_sdk.py --port COM5 --duration 5
```

### 6.3 高频流式下发（不等应答）

```python
arm.move_j(points, wait_ack=False)    # 发后即忘，无等待延迟
```

### 6.4 处理异步广播行

```python
arm.register_line_callback(lambda line: print("广播:", line))
# 或主动取走：
leftovers = arm.drain_async_lines()
```

### 6.5 异常处理

```python
from robot_arm_sdk import SDKTimeoutError, SDKResponseError

try:
    arm.move_j(target)
except SDKResponseError as e:     # 固件返回 "error ..."（如 queue full）
    print(e.response)
except SDKTimeoutError:           # 超时未收到匹配应答
    ...
```

---

## 7. 开发过程中遇到的问题与修正

### 7.1 应答延迟 50ms（读线程取数方式）

**现象**：每条应答固定延迟约 50ms 才被派发。
**原因**：`serial.read(256)` 的语义是"凑满 256 字节**或**耗满 timeout"，
应答只有十几个字节，每次都等满 `timeout=0.05s` 才返回。
**修正**：改为先查 `in_waiting`，有数据立即读空，无数据退化为 `read(1)`
阻塞等首字节：

```python
waiting = self._serial.in_waiting
data = self._serial.read(waiting if waiting > 0 else 1)
```

要点：`waiting > 0 else 1` 分支必不可少——`read(0)` 会立即返回 `b""`
造成忙轮询。`in_waiting` 快照竞态无害（唯一读者，缓冲区只增不减，
本次少读的字节下一轮取走）。

### 7.2 应答正则与固件格式逐条对齐

应答特征正则必须严格对应固件 `Respond` 的实际输出（如 `%.2f` 的浮点格式、
`SET MOTOR [%lu] DCE_KP [%lu]` 的字面结构），否则会误匹配或漏匹配。
固件改动应答文本时，SDK 正则需同步更新。

### 7.3 其他注意事项

- `!HOME` 回零耗时长，调用 `home(timeout=...)` 时建议显式传较大超时；
- `!` + 未知关键字固件**不应答**，SDK 高层 API 不使用此类命令；
- 依赖 `pyserial`（`pip install pyserial`）。

---

## 8. 已知局限与可扩展方向

| 局限 | 说明 |
| --- | --- |
| 特征匹配非绝对完美 | 两个并发事务特征完全相同时依赖 FIFO 顺序裁决（登记顺序已保证一致，正常可用） |
| 固件协议无序号 | 若固件未来在应答中加入命令回显/序号，可改为精确匹配，彻底消除歧义 |
| 未知 `!` 命令无应答 | `send_command` 原始接口发送此类命令时应使用 `wait=False` |
| 单串口单实例 | 一个 `RobotArmSDK` 实例绑定一个串口；多设备需创建多个实例（各自独立线程与锁，天然隔离） |
