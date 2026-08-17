# 上位机 USB ASCII 命令解析流程说明

本文档说明上位机通过 USB CDC 向 `dummy-ref-core-fw` 下位机发送 ASCII 命令后，下位机从收包、分行、命令分发、运动队列、后台解析到返回信息的完整流程。

主要涉及文件：

- `dummy-ref-core-fw/Bsp/communication/interface_usb.cpp`
- `dummy-ref-core-fw/Bsp/communication/ascii_processor.cpp`
- `dummy-ref-core-fw/Bsp/communication/ascii_processor.hpp`
- `dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp`
- `dummy-ref-core-fw/UserApp/main.cpp`
- `dummy-ref-core-fw/Robot/instances/dummy_robot.h`
- `dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`

## 1. 上位机发送命令格式

上位机通过 USB CDC 发送的是 ASCII 字符串。命令必须以 `\r` 或 `\n` 结尾，否则下位机会一直缓存，不能触发解析。

例如 Python 上位机中：

```python
cmd = f">{j1},{j2},{j3},{j4},{j5},{j6}\r\n"
serial.write(cmd.encode())
```

实际发出的内容类似：

```text
>10,20,30,40,50,60\r\n
```

注意：这里的 `f` 是 Python f-string 语法的一部分，不会作为字符发送。真正发送给下位机的首字符应该是 `>`。如果实际发送内容以字符 `f` 开头，下位机不会把它识别为运动命令。

## 2. USB 收包入口

USB CDC 数据到达后，底层回调会设置 `CDC_interface.data_pending`，然后释放 `sem_usb_rx` 信号量。

USB 服务线程 `UsbServerTask()` 被唤醒后，如果检测到 CDC 接口有数据，就调用：

```cpp
ASCII_protocol_parse_stream(CDC_interface.rx_buf, CDC_interface.rx_len, usb_stream_output);
```

定义位置：

- `dummy-ref-core-fw/Bsp/communication/interface_usb.cpp`

关键逻辑：

```cpp
if (CDC_interface.data_pending)
{
    CDC_interface.data_pending = false;

    ASCII_protocol_parse_stream(CDC_interface.rx_buf, CDC_interface.rx_len, usb_stream_output);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS, CDC_interface.out_ep);
}
```

这里的 `usb_stream_output` 是 USB 返回通道，后续 `Respond()` 会通过它把字符串发回上位机。

## 3. ASCII 流按行解析

`ASCII_protocol_parse_stream()` 逐字节扫描 USB 收到的数据，并用 `\r` 或 `\n` 判断一条命令结束。

定义位置：

- `dummy-ref-core-fw/Bsp/communication/ascii_processor.cpp`

核心逻辑：

```cpp
bool is_end_of_line = (c == '\r' || c == '\n');
if (is_end_of_line)
{
    if (read_active)
        ASCII_protocol_process_line(parse_buffer, parse_buffer_idx, response_channel);
    parse_buffer_idx = 0;
    read_active = true;
}
else
{
    if (read_active)
    {
        parse_buffer[parse_buffer_idx++] = c;
    }
}
```

因此：

- 收到普通字符时，先缓存到 `parse_buffer`。
- 收到 `\r` 或 `\n` 时，把缓存内容作为一条完整命令处理。
- 单条命令最大长度是 `MAX_LINE_LENGTH`，当前为 `256` 字节。

## 4. 根据通道分发到 USB 命令处理函数

一条完整命令形成后，`ASCII_protocol_process_line()` 会把数据复制到本地 `cmd` 数组，并补 `\0`，变成 C 字符串。

然后根据来源通道调用不同处理函数：

```cpp
if (response_channel.channelType == StreamSink::CHANNEL_TYPE_USB)
    OnUsbAsciiCmd(cmd, len, response_channel);
else if (response_channel.channelType == StreamSink::CHANNEL_TYPE_UART4)
    OnUart4AsciiCmd(cmd, len, response_channel);
else if (response_channel.channelType == StreamSink::CHANNEL_TYPE_UART5)
    OnUart5AsciiCmd(cmd, len, response_channel);
```

USB 上位机发送的命令会进入：

```cpp
OnUsbAsciiCmd(cmd, len, response_channel);
```

定义位置：

- `dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp`

## 5. USB ASCII 命令分类

`OnUsbAsciiCmd()` 按命令首字符分类：

- `!`：控制类命令，立即执行。
- `#`：查询或参数设置类命令，立即执行。
- `>`、`&`、`@`：运动类命令，先进入命令队列，再由后台线程解析执行。

### 5.1 `!` 控制类命令

这些命令在 `OnUsbAsciiCmd()` 中直接执行，并直接返回结果。

| 上位机命令 | 下位机动作 | USB 返回 |
| --- | --- | --- |
| `!STOP` | 急停，清空运动队列，失能机器人 | `Stopped ok\r\n` |
| `!START` | `dummy.SetEnable(true)`，使能机器人 | `Started ok\r\n` |
| `!HOME` | `dummy.Homing()`，执行回零 | `Started ok\r\n` |
| `!CALIBRATION` | `dummy.CalibrateHomeOffset()`，标定 Home 偏移 | `calibration ok\r\n` |
| `!RESET` | `dummy.Resting()`，回到休息位 | `Started ok\r\n` |
| `!DISABLE` | `dummy.SetEnable(false)`，失能机器人 | `Disabled ok\r\n` |

定义位置：

- `dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp`

示例代码：

```cpp
if (s.find("STOP") != std::string::npos)
{
    dummy.commandHandler.EmergencyStop();
    Respond(_responseChannel, "Stopped ok");
}
else if (s.find("START") != std::string::npos)
{
    dummy.SetEnable(true);
    Respond(_responseChannel, "Started ok");
}
```

### 5.2 `#` 查询和参数类命令

这些命令也在 `OnUsbAsciiCmd()` 中直接解析和返回。

| 上位机命令 | 下位机动作 | USB 返回 |
| --- | --- | --- |
| `#GETJPOS` | 读取当前 6 个关节角 | `ok j1 j2 j3 j4 j5 j6\r\n` |
| `#GETLPOS` | 更新并读取末端位姿 | `ok X Y Z A B C\r\n` |
| `#SET_DCE_KP node kp` | 设置指定电机 DCE_KP | 成功：`ok SET MOTOR [node] DCE_KP [kp]\r\n` |
| `#SET_DCE_KI node ki` | 设置指定电机 DCE_KI | 成功：`ok SET MOTOR [node] DCE_KI [ki]\r\n` |
| `#SET_DCE_KD node kd` | 设置指定电机 DCE_KD | 成功：`ok SET MOTOR [node] DCE_KD [kd]\r\n` |
| `#REBOOT node` | 重启指定电机 | 成功：`ok REBOOT MOTOR [node]\r\n` |
| `#CMDMODE mode` | 设置命令模式 | `ok Set command mode to [mode]\r\n` |
| 未识别的 `#...` | 不执行具体动作 | `ok\r\n` |

如果 `node` 不在 `1~6` 范围内，`#SET_DCE_KP`、`#SET_DCE_KI`、`#SET_DCE_KD`、`#REBOOT` 会返回 `error ... is wrong\r\n`。

示例代码：

```cpp
if (s.find("GETJPOS") != std::string::npos)
{
    Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
            dummy.currentJoints.a[0], dummy.currentJoints.a[1],
            dummy.currentJoints.a[2], dummy.currentJoints.a[3],
            dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
}
else if (s.find("CMDMODE") != std::string::npos)
{
    uint32_t mode;
    sscanf(_cmd, "#CMDMODE %lu", &mode);
    dummy.SetCommandMode(mode);
    Respond(_responseChannel, "ok Set command mode to [%lu]", mode);
}
```

### 5.3 `>`、`&`、`@` 运动类命令

运动类命令不会在 USB 接收线程中直接完成运动解析，而是先进入 `commandHandler` 队列。

| 命令首字符 | 含义 | 格式 |
| --- | --- | --- |
| `>` | 关节空间运动 | `>j1,j2,j3,j4,j5,j6` |
| `>` | 关节空间运动，附带速度 | `>j1,j2,j3,j4,j5,j6,speed` |
| `&` | 关节空间运动，解析逻辑与 `>` 基本相同 | `&j1,j2,j3,j4,j5,j6` |
| `&` | 关节空间运动，附带速度 | `&j1,j2,j3,j4,j5,j6,speed` |
| `@` | 末端位姿运动，内部通过 IK 转成关节角 | `@x,y,z,a,b,c` |
| `@` | 末端位姿运动，附带速度 | `@x,y,z,a,b,c,speed` |

USB 接收到运动命令后，先执行：

```cpp
uint32_t freeSize = dummy.commandHandler.Push(_cmd);
if (freeSize == 0xFF)
    Respond(_responseChannel, "error queue full");
else
    Respond(_responseChannel, "ok queued free=%lu", (unsigned long) freeSize);
```

定义位置：

- `dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp`

这里的返回值表示命令已经入队，并附带命令队列剩余空间。例如队列总深度为 16，如果压入后还剩 15 个空位，上位机会先收到：

```text
ok queued free=15\r\n
```

队列定义在：

- `dummy-ref-core-fw/Robot/instances/dummy_robot.h`

```cpp
commandFifo = osMessageQueueNew(16, 64, nullptr);
```

含义：

- 队列深度：`16`
- 单条命令最大存储长度：`64` 字节

## 6. 运动命令后台解析流程

系统初始化时会创建 `ThreadControlLoopUpdate` 线程。

定义位置：

- `dummy-ref-core-fw/UserApp/main.cpp`

线程逻辑：

```cpp
void ThreadControlLoopUpdate(void* argument)
{
    for (;;)
    {
        dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(osWaitForever));
    }
}
```

也就是说：

1. `Pop(osWaitForever)` 阻塞等待命令队列。
2. 一旦有 `>`、`&`、`@` 命令入队，就取出字符串。
3. 调用 `ParseCommand()` 做真正解析。

### 6.1 默认命令模式

机器人初始化时会设置默认命令模式：

```cpp
const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;
```

定义位置：

- `dummy-ref-core-fw/Robot/instances/dummy_robot.h`

因此默认情况下，运动命令会走 `COMMAND_TARGET_POINT_INTERRUPTABLE` 分支。

命令模式枚举：

```cpp
enum CommandMode
{
    COMMAND_TARGET_POINT_SEQUENTIAL = 1,
    COMMAND_TARGET_POINT_INTERRUPTABLE,
    COMMAND_CONTINUES_TRAJECTORY,
    COMMAND_MOTOR_TUNING
};
```

可以通过 USB 命令修改：

```text
#CMDMODE 1
#CMDMODE 2
#CMDMODE 3
#CMDMODE 4
```

## 7. `>` 和 `&` 关节运动解析

`ParseCommand()` 中对 `>` 和 `&` 使用 `sscanf()` 解析浮点数。

定义位置：

- `dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`

默认 `COMMAND_TARGET_POINT_INTERRUPTABLE` 模式下核心逻辑：

```cpp
if (_cmd[0] == '>')
    argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f",
                    joints, joints + 1, joints + 2,
                    joints + 3, joints + 4, joints + 5, &speed);

if (_cmd[0] == '&')
    argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f",
                    joints, joints + 1, joints + 2,
                    joints + 3, joints + 4, joints + 5, &speed);

if (argNum == 6)
{
    accepted = context->MoveJ(joints[0], joints[1], joints[2],
                              joints[3], joints[4], joints[5]);
}
else if (argNum == 7)
{
    context->SetJointSpeed(speed);
    accepted = context->MoveJ(joints[0], joints[1], joints[2],
                              joints[3], joints[4], joints[5]);
}
```

解析结果：

- 如果解析到 6 个数，调用 `MoveJ(j1, j2, j3, j4, j5, j6)`。
- 如果解析到 7 个数，先调用 `SetJointSpeed(speed)`，再调用 `MoveJ(...)`。
- 如果数量不是 6 或 7，不执行运动，也不会返回最终 `ok`。

### 7.1 `MoveJ()` 做了什么

`MoveJ()` 的主要工作：

1. 把 6 个目标角度组成 `targetJointsTmp`。
2. 检查每个关节是否超出对应电机角度限制。
3. 如果合法，根据当前角度和目标角度计算各关节动态速度。
4. 更新 `targetJoints`。
5. 返回 `true`。

如果角度超限，则返回 `false`。

定义位置：

- `dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`

角度限制来自机器人构造函数中各电机初始化参数，例如：

```cpp
motorJ[1] = new CtrlStepMotor(_hcan, 1, false, 50, -170, 170);
motorJ[2] = new CtrlStepMotor(_hcan, 2, true, 50, -75, 90);
motorJ[3] = new CtrlStepMotor(_hcan, 3, false, 50, 35, 180);
motorJ[4] = new CtrlStepMotor(_hcan, 4, true, 50, -180, 180);
motorJ[5] = new CtrlStepMotor(_hcan, 5, true, 50, -120, 120);
motorJ[6] = new CtrlStepMotor(_hcan, 6, true, 50, -720, 720);
```

## 8. `@` 末端位姿运动解析

`@` 命令格式为：

```text
@x,y,z,a,b,c
```

或：

```text
@x,y,z,a,b,c,speed
```

解析逻辑：

```cpp
argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f",
                pose, pose + 1, pose + 2,
                pose + 3, pose + 4, pose + 5, &speed);

if (argNum == 6)
{
    accepted = context->MoveL(pose[0], pose[1], pose[2],
                              pose[3], pose[4], pose[5]);
}
else if (argNum == 7)
{
    context->SetJointSpeed(speed);
    accepted = context->MoveL(pose[0], pose[1], pose[2],
                              pose[3], pose[4], pose[5]);
}
```

`MoveL()` 会调用 6 轴运动学求解器做 IK，把末端位姿转换成可行的关节角，再内部调用 `MoveJ()`。

## 9. 运动命令执行和电机下发

`ParseCommand()` 成功解析并接受运动命令后，只是更新目标状态。真正周期性下发电机控制命令的是固定周期控制线程 `ThreadControlLoopFixUpdate()`。

定义位置：

- `dummy-ref-core-fw/UserApp/main.cpp`

核心逻辑：

```cpp
if (dummy.IsEnabled())
{
    switch (dummy.commandMode)
    {
        case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
        case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
        case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
            dummy.MoveJoints(dummy.targetJoints);
            dummy.UpdateJointPose6D();
            break;
    }
}
```

`MoveJoints()` 会对 1 到 6 号电机逐个下发目标角度和速度限制：

```cpp
void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    for (int j = 1; j <= 6; j++)
    {
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                             dynamicJointSpeeds.a[j - 1]);
    }
}
```

定义位置：

- `dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`

## 10. 运动命令返回信息

运动命令通常会让上位机收到两类返回。

### 10.1 第一条返回：队列剩余空间

运动命令刚进入 `OnUsbAsciiCmd()` 时，会立即入队并返回队列剩余空间：

```cpp
uint32_t freeSize = dummy.commandHandler.Push(_cmd);
if (freeSize == 0xFF)
    Respond(_responseChannel, "error queue full");
else
    Respond(_responseChannel, "ok queued free=%lu", (unsigned long) freeSize);
```

例如：

```text
ok queued free=15\r\n
```

这表示命令成功放入队列后，队列还剩 15 个空位。

如果入队失败，`Push()` 返回 `0xFF`，上位机会收到更明确的错误信息：

```text
error queue full\r\n
```

`Push()` 定义位置：

- `dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`

```cpp
uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd)
{
    osStatus_t status = osMessageQueuePut(commandFifo, _cmd.c_str(), 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; // failed
}
```

### 10.2 第二条返回：最终接受结果 `ok`

后台线程 `ParseCommand()` 解析成功，并且 `MoveJ()` 或 `MoveL()` 返回 `true` 后，会返回：

```text
ok\r\n
```

默认 `COMMAND_TARGET_POINT_INTERRUPTABLE` 模式下：

```cpp
if (accepted)
{
    Respond(*usbStreamOutputPtr, "ok");
    Respond(*uart4StreamOutputPtr, "ok");
}
```

因此一条正常的关节运动指令：

```text
>10,20,60,0,0,0\r\n
```

上位机通常会收到：

```text
ok queued free=15\r\n
ok\r\n
```

如果命令格式错误、参数数量不对、角度超限、IK 无解，或者机器人当前命令模式不处理该命令，则通常只会收到第一条队列剩余空间，不会收到最终 `ok`。

## 11. 顺序模式和可中断模式的返回差异

`COMMAND_TARGET_POINT_SEQUENTIAL` 和 `COMMAND_CONTINUES_TRAJECTORY` 分支中，运动命令被接受后会立即触发一次 `MoveJoints()`，然后等待运动完成，再返回 `ok`：

```cpp
context->MoveJoints(context->targetJoints);

while (context->IsMoving() && context->IsEnabled())
    osDelay(5);
Respond(*usbStreamOutputPtr, "ok");
Respond(*uart4StreamOutputPtr, "ok");
```

`COMMAND_TARGET_POINT_INTERRUPTABLE` 分支中，接受命令后不等待运动完成，直接返回 `ok`：

```cpp
if (accepted)
{
    Respond(*usbStreamOutputPtr, "ok");
    Respond(*uart4StreamOutputPtr, "ok");
}
```

默认模式是 `COMMAND_TARGET_POINT_INTERRUPTABLE`，所以默认情况下 `ok` 表示命令已被解析并接受，不一定表示机械臂已经运动到目标位置。

## 12. 返回函数定义

所有文本返回最终都通过 `Respond()` 完成。

定义位置：

- `dummy-ref-core-fw/Bsp/communication/ascii_processor.hpp`

代码：

```cpp
template<typename ... TArgs>
void Respond(StreamSink &output , const char *fmt, TArgs &&... args)
{
    char response[64];
    size_t len = snprintf(response, sizeof(response), fmt, std::forward<TArgs>(args)...);
    output.process_bytes((uint8_t *) response, len, nullptr);
    output.process_bytes((const uint8_t *) "\r\n", 2, nullptr);
}
```

特点：

- 返回内容最长缓冲区为 `64` 字节。
- 格式化方式类似 `printf`。
- 每条返回末尾都会自动追加 `\r\n`。

## 13. 总体流程图

```text
上位机发送 ASCII 命令，例如：
>j1,j2,j3,j4,j5,j6\r\n
        |
        v
USB CDC 收包
interface_usb.cpp
        |
        v
UsbServerTask()
        |
        v
ASCII_protocol_parse_stream()
按 \r 或 \n 切分完整命令
        |
        v
ASCII_protocol_process_line()
根据通道类型分发
        |
        v
OnUsbAsciiCmd()
        |
        +-- 首字符为 !：
        |       立即执行控制命令
        |       立即 Respond()
        |
        +-- 首字符为 #：
        |       立即执行查询或参数命令
        |       立即 Respond()
        |
        +-- 首字符为 > / & / @：
                Push() 到 commandHandler 队列
                立即返回队列剩余空间
                |
                v
        ThreadControlLoopUpdate()
        Pop() 队列命令
                |
                v
        ParseCommand()
        sscanf() 解析参数
                |
                v
        MoveJ() 或 MoveL()
        检查限位 / IK / 更新 targetJoints
                |
                v
        若 accepted == true：
        Respond("ok")
                |
                v
        ThreadControlLoopFixUpdate()
        周期性 MoveJoints(targetJoints)
                |
                v
        通过 motorJ[1..6] 下发到各关节电机
```

## 14. 典型示例

### 示例 1：启动机器人

上位机发送：

```text
!START\r\n
```

下位机流程：

1. USB 收包。
2. ASCII 按行解析。
3. 进入 `OnUsbAsciiCmd()`。
4. 首字符为 `!`，匹配 `START`。
5. 执行 `dummy.SetEnable(true)`。
6. 返回 `Started ok\r\n`。

上位机收到：

```text
Started ok\r\n
```

### 示例 2：查询当前关节角

上位机发送：

```text
#GETJPOS\r\n
```

下位机流程：

1. USB 收包。
2. ASCII 按行解析。
3. 进入 `OnUsbAsciiCmd()`。
4. 首字符为 `#`，匹配 `GETJPOS`。
5. 读取 `dummy.currentJoints.a[0..5]`。
6. 返回 6 个关节角。

上位机收到类似：

```text
ok 0.00 -75.00 180.00 0.00 0.00 0.00\r\n
```

### 示例 3：发送关节运动命令

上位机发送：

```text
>10,20,60,0,0,0\r\n
```

下位机流程：

1. USB 收包。
2. ASCII 按行解析。
3. 进入 `OnUsbAsciiCmd()`。
4. 首字符为 `>`，命令进入 `commandHandler` 队列。
5. 立即返回队列剩余空间，例如 `ok queued free=15\r\n`。
6. `ThreadControlLoopUpdate()` 从队列取出命令。
7. `ParseCommand()` 使用 `sscanf()` 解析 6 个关节角。
8. 调用 `MoveJ(10, 20, 60, 0, 0, 0)`。
9. `MoveJ()` 检查角度限位并更新 `targetJoints`。
10. 如果接受成功，返回 `ok\r\n`。
11. 固定周期控制线程调用 `MoveJoints(targetJoints)`，向 6 个关节电机下发目标。

上位机通常收到：

```text
ok queued free=15\r\n
ok\r\n
```

## 15. 注意事项

1. 命令必须以 `\r` 或 `\n` 结尾。
2. 运动命令的第一条返回是入队结果和队列剩余空间，不代表运动成功。
3. 默认模式下，第二条 `ok` 代表命令被解析并接受，不代表已经到达目标位置。
4. 运动命令参数数量必须是 6 或 7。
5. 关节角必须在对应电机限位内，否则不会返回最终 `ok`。
6. `@` 命令需要 IK 求解成功，否则不会返回最终 `ok`。
7. 单条入队运动命令的队列存储长度是 64 字节，过长命令可能被截断或异常。
8. ASCII 流解析缓存最大长度是 256 字节，超过后会丢弃直到下一行。
