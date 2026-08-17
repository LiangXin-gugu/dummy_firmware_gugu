# dummy-ref-core-fw 工程启动流程、线程架构与指令执行 Pipeline 分析

本文档分析 `dummy-ref-core-fw` 主控固件的启动入口、线程结构和指令接收执行的完整 pipeline，并标注对应代码位置。

## 1. 工程概览

`dummy-ref-core-fw` 是 DummyRobot 六轴机械臂的主控固件，运行在 STM32F405RG 上，基于 FreeRTOS（CMSIS-RTOS v2）。主控通过 CAN 总线与 6 个关节电机（`dummy-42motor-fw` / `dummy-35motor-fw`）通信，通过 USB-CDC / UART4 / UART5 接收上位机指令，并通过 OLED 显示状态、IMU 采集姿态、RGB LED 提供视觉反馈。

## 2. 启动入口流程

### 2.1 硬件初始化与调度器启动

入口函数在 `Core/Src/main.c`：

```text
main()
  ├── HAL_Init()
  ├── SystemClock_Config()
  ├── 外设初始化 (GPIO, DMA, I2C, CAN, UART, SPI, ADC, TIM)
  ├── osKernelInitialize()
  ├── MX_FREERTOS_Init()
  └── osKernelStart()  ← 调度器接管，此后不会返回
```

定义位置：`dummy-ref-core-fw/Core/Src/main.c`

### 2.2 FreeRTOS 初始化

`MX_FREERTOS_Init()` 创建信号量、第一个系统线程：

```text
MX_FREERTOS_Init()
  ├── 创建信号量: sem_usb_irq, sem_uart4_dma, sem_uart5_dma, sem_usb_rx, sem_usb_tx, sem_can1_tx, sem_can2_tx
  ├── 创建线程: usbIrqTask  ← UsbDeferredInterruptTask, 优先级 AboveNormal, 栈 500
  └── 创建线程: defaultTask  ← StartDefaultTask, 优先级 Normal, 栈 2000
```

定义位置：`dummy-ref-core-fw/Core/Src/freertos.c`

### 2.3 defaultTask → C++ 入口 Main()

`defaultTask` 做了两件事：

```cpp
void StartDefaultTask(void *argument)
{
    MX_USB_DEVICE_Init();   // 初始化 USB Device 协议栈
    Main();                 // 调用 C++ 入口
    vTaskDelete(defaultTaskHandle);
}
```

定义位置：`dummy-ref-core-fw/Core/Src/freertos.c`

`Main()` 是 C++ 用户代码的真正入口，定义位置：`dummy-ref-core-fw/UserApp/main.cpp`

### 2.4 Main() 内部流程

```text
Main()
  ├── InitCommunication()          ← 启动通信子系统（含创建 commTask 线程）
  ├── dummy.Init()                 ← 初始化机器人实例
  ├── mpu6050.Init() / InitFilter  ← 初始化 IMU
  ├── oled.Init()                  ← 初始化 OLED
  ├── pwm.Start()                  ← 启动 PWM
  ├── 创建 4 个用户线程
  ├── timerCtrlLoop.SetCallback()  ← 设置定时器回调
  ├── timerCtrlLoop.Start()         ← 启动定时器（200Hz）
  └── pwm.SetDuty(CH_A1, 0.5)      ← 点亮指示灯
```

定义位置：`dummy-ref-core-fw/UserApp/main.cpp`

`InitCommunication()` 内部会阻塞等待 `endpointListValid` 信号，确保通信线程完成协议树注册后才继续。

```cpp
void InitCommunication(void)
{
    commTaskHandle = osThreadNew(CommunicationTask, nullptr, &commTask_attributes);
    while (!endpointListValid)
        osDelay(1);
}
```

定义位置：`dummy-ref-core-fw/Bsp/communication/communication.cpp`

## 3. 线程架构

整个系统共有约 8 个线程，分为系统层和用户层两组。

### 3.1 系统层线程

| 线程名 | 函数 | 优先级 | 栈大小 | 创建位置 | 职责 |
| --- | --- | --- | --- | --- | --- |
| `defaultTask` | `StartDefaultTask` | Normal | 2000 | freertos.c | 初始化 USB Device，调用 `Main()`，完成后自删除 |
| `usbIrqTask` | `UsbDeferredInterruptTask` | AboveNormal | 500 | freertos.c | 延迟处理 USB OTG 中断，避免在中断上下文中执行 `HAL_PCD_IRQHandler` |
| `commTask` | `CommunicationTask` | Normal | 45000 | communication.cpp | 注册协议树（`CommitProtocol`），启动 UART/USB/CAN 服务线程 |
| `UsbServerTask` | `UsbServerTask` | Normal | 2000 | interface_usb.cpp | 处理 USB CDC RX 数据，分发 ASCII 命令和 Native 协议包 |
| `UartServerTask` | `UartServerTask` | Normal | 2000 | interface_uart.cpp | 轮询 UART4/UART5 DMA 环形缓冲区，分发 ASCII 命令 |

注意：

- `commTask` 的栈高达 45000 字节，这是因为 `CommitProtocol()` 会用 `new(treeBuffer)` 在栈上构造整个协议对象树。
- CAN 总线没有专用接收线程，而是通过中断回调直接处理（见第 4.3 节）。

### 3.2 用户层线程

以下 4 个线程在 `Main()` 中创建：

| 线程名 | 函数 | 优先级 | 栈大小 | 触发方式 | 职责 |
| --- | --- | --- | --- | --- | --- |
| `ControlLoopFixUpdateTask` | `ThreadControlLoopFixUpdate` | Realtime | 2000 | Timer7 中断通知（200Hz） | 固定周期控制循环：向电机下发运动命令、更新关节状态 |
| `ControlLoopUpdateTask` | `ThreadControlLoopUpdate` | Normal | 2000 | 命令队列阻塞（`osWaitForever`） | 从命令队列取出运动命令并解析执行 |
| `OledTask` | `ThreadOledUpdate` | Normal | 2000 | 自旋循环（无阻塞等待） | 更新 IMU 数据，刷新 OLED 显示 |
| `RGBTask` | `ThreadRGBUpdate` | Normal | 2000 | `osDelay(30)` 循环 | RGB LED 动画刷新 |

定义位置：`dummy-ref-core-fw/UserApp/main.cpp`

### 3.3 线程协作关系

```text
                    ┌─────────────────────────────────────────────────────┐
                    │                   上位机 / CAN 总线                   │
                    └───────┬──────────────┬──────────────────┬────────────┘
                            │              │                  │
                     USB CDC 接收     UART DMA 轮询      CAN 中断回调
                            │              │                  │
                            v              v                  v
                    ┌─────────────┐ ┌──────────────┐  ┌──────────────────┐
                    │UsbServerTask │ │UartServerTask│  │HAL_CAN_RxFifo0  │
                    │(系统线程)     │ │(系统线程)     │  │MsgPendingCallback│
                    └──────┬──────┘ └──────┬───────┘  └────────┬─────────┘
                           │               │                   │
                           v               v                   v
                    ASCII_protocol_parse_stream           OnCanMessage()
                           │               │                   │
                           v               v                   v
                    OnUsbAsciiCmd()  OnUart4AsciiCmd()  can_protocol.cpp
                    OnUart5AsciiCmd()                    (更新电机角度/温度)
                           │               │
                    ┌──────┴───────────────┴──────┐
                    │  !和#命令：立即执行          │
                    │  > & @命令：Push到FIFO队列  │
                    └──────────────┬──────────────┘
                                   │
                                   v
                          ┌──────────────────┐
                          │ControlLoopUpdateTask│  ← 从队列Pop命令
                          │(用户线程)         │
                          └────────┬─────────┘
                                   │ ParseCommand()
                                   │ MoveJ() / MoveL()
                                   │ 更新 targetJoints
                                   v
                    ┌──────────────────────────────┐
                    │ControlLoopFixUpdateTask      │  ← Timer7 中断(200Hz)通知唤醒
                    │(用户线程, 优先级 Realtime)    │
                    │  MoveJoints(targetJoints)    │
                    │  → motorJ[1..6] CAN 下发      │
                    │  UpdateJointAngles / Pose6D  │
                    └──────────────────────────────┘
                                   │
                                   v
                          ┌──────────────┐
                          │OledTask      │  ← 自旋循环
                          │IMU + OLED刷新 │
                          └──────────────┘
                          ┌──────────────┐
                          │RGBTask       │  ← 30ms周期
                          │RGB LED动画    │
                          └──────────────┘
```

## 4. 指令接收与执行 Pipeline

### 4.1 Pipeline 总览

系统支持三种指令通道，每条通道有独立的接收路径，但最终汇聚到统一的命令处理逻辑：

| 通道 | 接收方式 | 处理路径 | 对应代码 |
| --- | --- | --- | --- |
| USB-CDC | CDC 回调 → 信号量 → 线程处理 | ASCII 流解析 → 命令分发 | `interface_usb.cpp` |
| UART4 / UART5 | DMA 环形缓冲 → 线程轮询 | ASCII 流解析 → 命令分发 | `interface_uart.cpp` |
| CAN | 中断回调直接处理 | 直接调用 `OnCanMessage()` | `interface_can.cpp` |
| USB-Native | CDC 回调 → 信号量 → 线程处理 | Fibre 二进制协议 | `interface_usb.cpp` |

### 4.2 USB / UART ASCII 命令 Pipeline

这是上位机控制机器人的主要路径。

#### 阶段 1：物理接收

**USB-CDC 接收：**

USB CDC 底层收到数据后调用 `usb_rx_process_packet()`，设置 `data_pending` 标志并释放 `sem_usb_rx` 信号量。`UsbServerTask` 被唤醒后调用 `ASCII_protocol_parse_stream()`。

定义位置：`dummy-ref-core-fw/Bsp/communication/interface_usb.cpp`

关键代码：

```cpp
void usb_rx_process_packet(uint8_t *buf, uint32_t len, uint8_t endpoint_pair)
{
    usb_iface->rx_buf = buf;
    usb_iface->rx_len = len;
    usb_iface->data_pending = true;
    osSemaphoreRelease(sem_usb_rx);
}
```

```cpp
// UsbServerTask
if (CDC_interface.data_pending)
{
    CDC_interface.data_pending = false;
    ASCII_protocol_parse_stream(CDC_interface.rx_buf, CDC_interface.rx_len, usb_stream_output);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS, CDC_interface.out_ep);
}
```

**UART 接收：**

UART4/UART5 使用 DMA 循环缓冲模式。`UartServerTask` 以 1ms 间隔轮询 DMA 的 `NDTR` 寄存器，计算新数据位置，处理环绕情况后调用 `ASCII_protocol_parse_stream()`。

定义位置：`dummy-ref-core-fw/Bsp/communication/interface_uart.cpp`

关键代码：

```cpp
uint32_t new_rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;

if (new_rcv_idx > dma_last_rcv_idx[0])
{
    ASCII_protocol_parse_stream(dma_rx_buffer[0] + dma_last_rcv_idx[0],
                                new_rcv_idx - dma_last_rcv_idx[0], uart4_stream_output);
    dma_last_rcv_idx[0] = new_rcv_idx;
}
```

#### 阶段 2：按行切分

`ASCII_protocol_parse_stream()` 逐字节扫描数据流，以 `\r` 或 `\n` 作为行结束符。完整的一行被送入 `ASCII_protocol_process_line()`。

定义位置：`dummy-ref-core-fw/Bsp/communication/ascii_processor.cpp`

关键代码：

```cpp
bool is_end_of_line = (c == '\r' || c == '\n');
if (is_end_of_line)
{
    if (read_active)
        ASCII_protocol_process_line(parse_buffer, parse_buffer_idx, response_channel);
    parse_buffer_idx = 0;
    read_active = true;
}
```

- 单行最大长度：`MAX_LINE_LENGTH = 256` 字节
- 超长行会被丢弃直到下一行

#### 阶段 3：按通道分发

`ASCII_protocol_process_line()` 根据来源通道的 `channelType` 分发到不同的命令处理函数。

定义位置：`dummy-ref-core-fw/Bsp/communication/ascii_processor.cpp`

```cpp
if (response_channel.channelType == StreamSink::CHANNEL_TYPE_USB)
    OnUsbAsciiCmd(cmd, len, response_channel);
else if (response_channel.channelType == StreamSink::CHANNEL_TYPE_UART4)
    OnUart4AsciiCmd(cmd, len, response_channel);
else if (response_channel.channelType == StreamSink::CHANNEL_TYPE_UART5)
    OnUart5AsciiCmd(cmd, len, response_channel);
```

#### 阶段 4：命令分类与分发

命令按首字符分三类处理（以 USB 通道 `OnUsbAsciiCmd` 为例）：

| 首字符 | 命令类型 | 执行方式 | 示例 |
| --- | --- | --- | --- |
| `!` | 控制类 | 立即执行 | `!START`, `!STOP`, `!HOME` |
| `#` | 查询/参数 | 立即执行 | `#GETJPOS`, `#CMDMODE` |
| `>` `&` `@` | 运动类 | 先入队 FIFO，再后台解析 | `>10,20,30,40,50,60` |

定义位置：`dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp`

立即命令直接调用 `dummy` 对应方法并通过 `Respond()` 返回结果。运动命令则调用 `dummy.commandHandler.Push()` 压入 FIFO 队列并立即返回队列剩余空间。

```cpp
// 运动命令入队
uint32_t freeSize = dummy.commandHandler.Push(_cmd);
if (freeSize == 0xFF)
    Respond(_responseChannel, "error queue full");
else
    Respond(_responseChannel, "ok queued free=%lu", (unsigned long) freeSize);
```

命令 FIFO 定义：

```cpp
commandFifo = osMessageQueueNew(16, 64, nullptr);
```

- 队列深度：16
- 单条命令最大存储：64 字节
- 定义位置：`dummy-ref-core-fw/Robot/instances/dummy_robot.h`

#### 阶段 5：后台命令解析线程

`ThreadControlLoopUpdate` 阻塞等待命令队列，取出命令后调用 `ParseCommand()` 解析。

定义位置：`dummy-ref-core-fw/UserApp/main.cpp`

```cpp
void ThreadControlLoopUpdate(void* argument)
{
    for (;;)
    {
        dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(osWaitForever));
    }
}
```

`ParseCommand()` 根据当前 `commandMode` 走不同分支：

| 命令模式 | 枚举值 | 行为 |
| --- | --- | --- |
| `COMMAND_TARGET_POINT_SEQUENTIAL` | 1 | 解析后立即 `MoveJoints()`，阻塞等待运动完成再返回 `ok` |
| `COMMAND_TARGET_POINT_INTERRUPTABLE` | 2（默认） | 解析后立即返回 `ok`，不等待运动完成 |
| `COMMAND_CONTINUES_TRAJECTORY` | 3 | 同顺序模式，阻塞等待完成 |
| `COMMAND_MOTOR_TUNING` | 4 | 不处理运动命令 |

定义位置：`dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`

`ParseCommand()` 内部使用 `sscanf()` 解析参数：

- `>` / `&`：解析 6 个关节角（+ 可选速度），调用 `MoveJ()`
- `@`：解析 6 维末端位姿（+ 可选速度），调用 `MoveL()`（内部做 IK 逆解）

`MoveJ()` 的职责：

1. 组装目标关节角 `targetJointsTmp`
2. 逐关节检查角度限位
3. 合法则计算各关节动态速度 `dynamicJointSpeeds`
4. 更新 `targetJoints`
5. 返回 `true` / `false`

定义位置：`dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`

#### 阶段 6：固定周期控制循环

`ThreadControlLoopFixUpdate` 是优先级最高的用户线程（`osPriorityRealtime`），由 Timer7 的 200Hz 中断通过 `vTaskNotifyGiveFromISR()` 唤醒。

定义位置：`dummy-ref-core-fw/UserApp/main.cpp`

定时器回调（中断上下文）：

```cpp
void OnTimer7Callback()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(TaskHandle_t(controlLoopFixUpdateHandle), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

线程主循环：

```cpp
void ThreadControlLoopFixUpdate(void* argument)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // 等待 Timer7 通知

        if (dummy.IsEnabled())
        {
            switch (dummy.commandMode)
            {
                case COMMAND_TARGET_POINT_SEQUENTIAL:
                case COMMAND_TARGET_POINT_INTERRUPTABLE:
                case COMMAND_CONTINUES_TRAJECTORY:
                    dummy.MoveJoints(dummy.targetJoints);  // 向 6 个电机下发电机控制命令
                    dummy.UpdateJointPose6D();              // 正运动学更新末端位姿
                    break;
                case COMMAND_MOTOR_TUNING:
                    dummy.tuningHelper.Tick(10);
                    dummy.UpdateJointPose6D();
                    break;
            }
        } else
        {
            dummy.UpdateJointAngles();   // 仅查询电机角度
            dummy.UpdateJointPose6D();
        }
    }
}
```

`MoveJoints()` 向每个关节电机通过 CAN 下发目标角度和速度限制：

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

定义位置：`dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`

### 4.3 CAN 命令 Pipeline

CAN 总线没有专用接收线程，而是通过 HAL 中断回调直接处理。

#### 接收流程

```text
CAN RX FIFO0 中断
  └── HAL_CAN_RxFifo0MsgPendingCallback()  ← interface_can.cpp
        ├── HAL_CAN_GetRxMessage()
        └── OnCanMessage()                 ← can_protocol.cpp
              ├── 解析 StdId: 高4位=nodeID, 低7位=cmd
              ├── 0x23: UpdateAngleCallback() → 更新 motorJ[id]->angle
              ├── 0x25: 更新 motorJ[id]->temperature
              └── UpdateJointAnglesCallback() → 刷新 currentJoints + jointsStateFlag
```

定义位置：`dummy-ref-core-fw/Bsp/communication/interface_can.cpp`、`dummy-ref-core-fw/UserApp/protocols/can_protocol.cpp`

关键代码：

```cpp
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &headerRx, data);
    OnCanMessage(ctx, &headerRx, data);
}
```

```cpp
void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data)
{
    uint8_t id = rxHeader->StdId >> 7;     // 4Bits ID
    uint8_t cmd = rxHeader->StdId & 0x7F;  // 7Bits Msg
    switch (cmd)
    {
        case 0x23:
            dummy.motorJ[id]->UpdateAngleCallback(*(float*)(data), data[4]);
            break;
        case 0x25:
            memcpy(&dummy.motorJ[id]->temperature, data, sizeof(uint32_t));
            break;
    }
    dummy.UpdateJointAnglesCallback();
}
```

#### 发送流程

主控向电机发送 CAN 命令时，通过 `CanSendMessage()` 获取发送信号量后调用 HAL 发送：

```cpp
void CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    osSemaphoreAcquire(sem_can1_tx, osWaitForever);  // 等待发送完成
    HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox);
}
```

发送完成后中断回调释放信号量：

```cpp
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{
    tx_complete_callback(hcan, 0);  // → osSemaphoreRelease(sem_can1_tx)
}
```

定义位置：`dummy-ref-core-fw/Bsp/communication/interface_can.cpp`

### 4.4 USB Native 协议 Pipeline

除 ASCII 协议外，系统还支持基于 Fibre 框架的二进制 Native 协议，通过 USB 的 ODrive 端点传输。

```text
USB ODrive 端点 RX
  └── usb_rx_process_packet()  ← interface_usb.cpp
        └── UsbServerTask
              └── usb_channel.process_packet()  ← Fibre 协议处理
```

协议树在 `CommunicationTask` 中通过 `CommitProtocol()` 注册，定义位置：

`dummy-ref-core-fw/UserApp/protocols/cmd_protocol.cpp`

```cpp
static inline auto MakeObjTree()
{
    return make_protocol_member_list(
        make_protocol_ro_property("serial_number", &serialNumber),
        make_protocol_function("get_temperature", ...),
        make_protocol_function("get_voltage", ...),
        make_protocol_object("robot", dummy.MakeProtocolDefinitions())
    );
}
```

`DummyRobot::MakeProtocolDefinitions()` 暴露了 `move_j`、`move_l`、`homing`、`set_enable` 等函数供 reftool 等上位机直接调用。

定义位置：`dummy-ref-core-fw/Robot/instances/dummy_robot.h`

## 5. 关键文件索引

| 模块 | 文件路径 | 职责 |
| --- | --- | --- |
| C++ 入口 | `UserApp/main.cpp` | 全局对象定义、用户线程创建、定时器启动 |
| 通用头文件 | `UserApp/common_inc.h` | 统一 include，声明 `Main()` |
| FreeRTOS 配置 | `UserApp/freertos_inc.h` | 信号量、线程句柄 extern 声明 |
| FreeRTOS 初始化 | `Core/Src/freertos.c` | 信号量创建、defaultTask、usbIrqTask |
| 硬件入口 | `Core/Src/main.c` | 时钟、外设初始化、调度器启动 |
| 通信总控 | `Bsp/communication/communication.cpp` | `InitCommunication()`、`CommunicationTask`、`UsbDeferredInterruptTask` |
| 通信总控头 | `Bsp/communication/communication.hpp` | 通信接口声明、`COMMIT_PROTOCOL` 宏 |
| USB 接口 | `Bsp/communication/interface_usb.cpp` | `UsbServerTask`、CDC RX 处理、Fibre Native 通道 |
| UART 接口 | `Bsp/communication/interface_uart.cpp` | `UartServerTask`、DMA 环形缓冲轮询 |
| CAN 接口 | `Bsp/communication/interface_can.cpp` | CAN 中断回调、`CanSendMessage()`、`StartCanServer()` |
| ASCII 流解析 | `Bsp/communication/ascii_processor.cpp` | 按行切分、通道分发 |
| ASCII 命令处理 | `UserApp/protocols/ascii_protocol.cpp` | `OnUsbAsciiCmd()` / `OnUart4AsciiCmd()` |
| CAN 协议 | `UserApp/protocols/can_protocol.cpp` | `OnCanMessage()` 电机角度/温度回调 |
| Fibre 协议树 | `UserApp/protocols/cmd_protocol.cpp` | `MakeObjTree()` 协议对象定义 |
| 机器人实例 | `Robot/instances/dummy_robot.cpp` | `MoveJ/MoveL/MoveJoints`、`CommandHandler` |
| 机器人实例头 | `Robot/instances/dummy_robot.h` | `DummyRobot` 类定义、`CommandMode` 枚举 |
| 关节电机 | `Robot/actuators/ctrl_step/ctrl_step.hpp` | `CtrlStepMotor` CAN 电机驱动 |
| 定时器 | `Bsp/utils/timer.hpp` | `Timer` 类，封装 HAL TIM |

## 6. 完整启动时序

```text
1. main.c: main()
   ├── HAL_Init, SystemClock_Config
   ├── 外设初始化 (GPIO/DMA/I2C/CAN/UART/SPI/ADC/TIM)
   ├── osKernelInitialize
   └── MX_FREERTOS_Init
       ├── 创建信号量 (sem_usb_irq, sem_uart4_dma, ...)
       ├── 创建 usbIrqTask 线程  ← UsbDeferredInterruptTask
       └── 创建 defaultTask 线程 ← StartDefaultTask
   └── osKernelStart  ← 调度器启动，以下在线程上下文中执行

2. freertos.c: StartDefaultTask()
   ├── MX_USB_DEVICE_Init()
   └── Main()

3. main.cpp: Main()
   ├── InitCommunication()
   │   └── communication.cpp: 创建 commTask 线程 ← CommunicationTask
   │       ├── CommitProtocol()  ← cmd_protocol.cpp: 注册 Fibre 协议树
   │       ├── endpointListValid = true  ← 通知 Main() 可以继续
   │       ├── StartUartServer()  ← interface_uart.cpp: 创建 UartServerTask
   │       ├── StartUsbServer()   ← interface_usb.cpp: 创建 UsbServerTask
   │       └── StartCanServer(CAN1), StartCanServer(CAN2)  ← interface_can.cpp
   │
   ├── dummy.Init()
   ├── mpu6050.Init() + InitFilter()
   ├── oled.Init(), pwm.Start()
   │
   ├── 创建用户线程:
   │   ├── ControlLoopFixUpdateTask  ← Realtime, Timer7 驱动(200Hz)
   │   ├── ControlLoopUpdateTask     ← Normal, 命令队列驱动
   │   ├── OledTask                  ← Normal, 自旋循环
   │   └── RGBTask                   ← Normal, 30ms 周期
   │
   ├── timerCtrlLoop.SetCallback(OnTimer7Callback)
   └── timerCtrlLoop.Start()  ← 开始 200Hz 中断, 驱动控制循环
```

## 7. 指令执行完整路径示例

以 USB 发送关节运动命令 `>10,20,60,0,0,0\r\n` 为例：

```text
上位机发送: >10,20,60,0,0,0\r\n
  │
  ▼ USB CDC RX
usb_rx_process_packet()               [interface_usb.cpp]
  │ 设置 data_pending, 释放 sem_usb_rx
  ▼
UsbServerTask()                       [interface_usb.cpp]
  │ ASCII_protocol_parse_stream()
  ▼
ascii_processor.cpp
  │ 按行切分 → ">10,20,60,0,0,0"
  │ ASCII_protocol_process_line() → 按 channelType 分发
  ▼
OnUsbAsciiCmd()                       [ascii_protocol.cpp]
  │ 首字符 '>' → 运动命令
  │ dummy.commandHandler.Push(">10,20,60,0,0,0")
  ▼
CommandHandler::Push()                [dummy_robot.cpp]
  │ osMessageQueuePut(commandFifo, ...)
  │ 返回队列剩余空间
  ▼
上位机收到第一条: ok queued free=15\r\n
  │
  ▼ ControlLoopUpdateTask 被唤醒
ThreadControlLoopUpdate()             [main.cpp]
  │ dummy.commandHandler.Pop(osWaitForever)
  │ dummy.commandHandler.ParseCommand(">10,20,60,0,0,0")
  ▼
ParseCommand()                        [dummy_robot.cpp]
  │ sscanf 解析 6 个关节角
  │ context->MoveJ(10, 20, 60, 0, 0, 0)
  ▼
MoveJ()                               [dummy_robot.cpp]
  │ 检查角度限位
  │ 计算 dynamicJointSpeeds
  │ 更新 targetJoints = {10, 20, 60, 0, 0, 0}
  │ 返回 true
  ▼
上位机收到第二条: ok\r\n (默认中断模式, 不等待运动完成)
  │
  ▼ Timer7 中断 (200Hz)
OnTimer7Callback()                    [main.cpp]
  │ vTaskNotifyGiveFromISR(controlLoopFixUpdateHandle)
  ▼
ThreadControlLoopFixUpdate()          [main.cpp]
  │ ulTaskNotifyTake() 被唤醒
  │ dummy.IsEnabled() == true
  ▼
MoveJoints(targetJoints)              [dummy_robot.cpp]
  │ for j=1..6:
  │   motorJ[j]->SetAngleWithVelocityLimit(angle, vel)
  ▼
CtrlStepMotor::SetAngleWithVelocityLimit()
  │ 组装 CAN 帧, StdId = nodeID<<7 | cmd
  │ CanSendMessage()
  ▼
interface_can.cpp: CanSendMessage()
  │ osSemaphoreAcquire(sem_can1_tx)
  │ HAL_CAN_AddTxMessage()
  ▼
CAN 总线 → 关节电机固件执行运动
  │
  ▼ 电机完成后通过 CAN 0x23 回复角度
HAL_CAN_RxFifo0MsgPendingCallback()   [interface_can.cpp]
  │ OnCanMessage()
  ▼
can_protocol.cpp: OnCanMessage()
  │ motorJ[id]->UpdateAngleCallback()
  │ UpdateJointAnglesCallback() → 更新 currentJoints + jointsStateFlag
  ▼
OledTask 刷新显示 currentJoints
```

## 8. 关键设计要点

1. **实时性保障**：`ControlLoopFixUpdateTask` 使用最高优先级 `osPriorityRealtime`，由硬件定时器驱动，确保控制周期稳定。

2. **中断延迟处理**：USB OTG 中断通过 `usbIrqTask` 延迟到线程上下文处理，避免长时间占用中断。CAN 接收则直接在中断中处理（`OnCanMessage`），因为 CAN 消息处理量小且要求低延迟。

3. **命令队列解耦**：运动命令通过 16 深度的消息队列解耦接收和执行，上位机可以快速连续发送命令，后台线程依次解析。

4. **双模式返回**：顺序模式（mode 1/3）阻塞等待运动完成后返回 `ok`，可中断模式（mode 2，默认）立即返回 `ok`，适合连续轨迹控制。

5. **协议树注册时机**：`CommitProtocol()` 在 `commTask` 线程中执行，`InitCommunication()` 阻塞等待 `endpointListValid` 确保 Fibre 协议树在 USB 服务启动前就绪。
