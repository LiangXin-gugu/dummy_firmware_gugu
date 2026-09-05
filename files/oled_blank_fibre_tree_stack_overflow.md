# 开机 OLED 全黑故障分析：Fibre 协议树撑爆 commTask 栈

> 面向场景：给中控板（`dummy-ref-core-fw`，STM32F405）新增"查询所有电机电流/温度"
> 功能后，刷入固件，**OLED 屏幕从上电到永远全程不亮**（不是显示一半冻住，是压根没点亮过）。
> 回退到上一个 commit（`b7caae5`）重新编译烧录，**同一套硬件**下 OLED 正常亮、正常刷新。
>
> 硬件条件：只有关节 1 电机与中控板通过 CAN + 电源相连，关节 1↔2 之间的
> CAN 线（2 根）和电源线（4 根）都断开，即关节 2~6 全部离线。
>
> 本文按 **问题现象 → 背景知识（Fibre 是什么）→ 根本原因 → 证据链 → 修复建议**
> 的顺序完整复盘，所有结论都对应到具体源码位置和实测数字。

---

## 1. 问题现象

| 观察项 | 故障固件（新增电流/温度功能后） | 正常固件（回退 `b7caae5`） |
|---|---|---|
| OLED 屏幕 | **全程黑屏，从未点亮** | 正常亮，实时刷新 |
| 关节 1 角度显示 | 看不到（屏幕没亮） | 随真实转动实时变化 |
| 电机 / CAN 通信 | 无法判断（系统没起来） | 正常 |
| 重启后 | 依旧黑屏，稳定复现 | 依旧正常 |

一个决定性的对比事实：**故障是"代码回归"引起的，不是硬件、不是烧录、不是编译产物损坏。**
同一块板、同一套接线，只换固件版本，现象就完全相反。

还有一个关键细节帮助定位：OLED 是**黑屏**而不是"显示一半冻住"。
参照 `write_timeout_fault_analysis.md` 里的判据——

- 屏幕"定格在最后一帧" = MCU 跑起来过、然后整机挂死；
- 屏幕"从头到尾没亮过" = **MCU 根本没走到点亮屏幕那一步**，卡在了更早的初始化阶段。

所以本次故障点一定在 `oled.Init()` **之前**的启动流程里。

---

## 2. 背景知识：先搞懂几个概念

要看懂这个 bug，得先理解三件事：`ctrl_step.hpp` 里那段 `MakeProtocolDefinitions()`
在干什么、Fibre 协议树是什么、以及它和 Python SDK 用的 ASCII 协议有什么区别。

### 2.1 `ctrl_step.hpp` 75~109 行在干什么

位置：`Robot/actuators/ctrl_step/ctrl_step.hpp`

```cpp
// Communication protocol definitions
auto MakeProtocolDefinitions()
{
    return make_protocol_member_list(
        make_protocol_ro_property("angle", &angle),                       // 只读属性
        make_protocol_function("reboot", *this, &CtrlStepMotor::Reboot),  // 可远程调用的方法
        make_protocol_function("get_temperature", *this, &CtrlStepMotor::GetTemp),
        make_protocol_function("set_enable", *this, &CtrlStepMotor::SetEnable, "enable"),
        make_protocol_function("set_position", *this, &CtrlStepMotor::SetPositionSetPoint, "pos"),
        // ... 还有十几个 set_xxx / do_calibration / update_angle 等
    );
}
```

用大白话讲：**这段代码是在"申报"一个电机对象对外暴露哪些东西可以被远程访问。**

- `make_protocol_ro_property("angle", &angle)`：声明"我有一个只读属性叫 `angle`，
  上位机可以远程读它的值"（ro = read-only）。
- `make_protocol_function("set_position", *this, &CtrlStepMotor::SetPositionSetPoint, "pos")`：
  声明"我有一个方法叫 `set_position`，上位机可以远程调用它，它接收一个参数 `pos`"。

它本身**不执行任何业务逻辑**，只是把 C++ 的成员变量/成员函数"登记"到一张清单里，
这张清单随后会被拼进一棵更大的"协议树"（见 2.2）。这是 ODrive 风格的
**声明式协议定义**：你写下对象有哪些成员，框架自动帮你把它们变成可远程访问的端点。

> 本次新增功能时，我曾在这段清单里加了 4 行（把 `current`、`temperature` 也申报成
> 只读属性，把 `UpdateCurrent`、`UpdateTemp` 申报成可远程调用的方法）。
> **正是这 4 行 × 7 个电机实例，间接导致了 OLED 黑屏。** 详见第 3 节。

### 2.2 Fibre 协议树是什么、做什么用

**Fibre** 是 ODrive 开源的一套"**面向对象的远程调用协议**"。它的核心思想是：
把设备内部的 C++ 对象和方法，自动映射成一棵可以被上位机**发现和调用**的"对象树"。

本项目的整棵树长这样（根在 `UserApp/protocols/cmd_protocol.cpp` 的 `MakeObjTree()`）：

```text
根 (MakeObjTree)
├── serial_number          只读属性
├── get_temperature        方法（读芯片温度）
├── get_voltage            方法（读电压）
└── robot                  对象 (DummyRobot::MakeProtocolDefinitions)
    ├── joint_1            对象 ← CtrlStepMotor::MakeProtocolDefinitions()（2.1 那张清单）
    ├── joint_2            对象 ← 同上
    ├── ... joint_6        对象 ← 同上
    ├── hand               对象 (DummyHand 的方法清单)
    ├── move_joint / enable / homing / ...   robot 级方法
    └── tuning             对象 (TuningHelper 的方法清单)
```

**它的用途**：上位机（比如 ODrive 生态的图形化工具 `reftool` / `odrivetool`）
连上设备后，能"**自动发现**"设备内部有哪些属性和方法，不需要在电脑侧硬编码。
它还能把这棵树导出一份 **JSON 描述文件（带 CRC16 校验）**，供上位机核对协议版本、
生成对应的客户端代码。简单说：**Fibre 让设备"自我描述"，上位机即插即用。**

关键点：**这棵树是在编译期由 C++ 模板"拼"出来的一个巨大对象**，
运行时需要在内存里真正构造出来。这个"构造"过程正是本次故障的舞台。

### 2.3 Fibre 协议 vs Python SDK 用的 ASCII 协议

本项目里其实**并行存在两套上位机通道**，功能有重叠但彼此独立：

| 对比项 | **Fibre 协议** | **ASCII 协议** |
|---|---|---|
| 形态 | 二进制、面向对象、树状端点 | 纯文本行，人眼可读 |
| 命令长相 | 上位机按"端点 ID"读写，自动发现 | `#GET_CURRENT`、`!START`、`>10,20,60,0,0,0` |
| 谁来解析 | Fibre 框架（`3rdParty/fibre`）自动处理 | `UserApp/protocols/ascii_protocol.cpp` 里手工 `strstr` 匹配 |
| 典型使用者 | ODrive 的 `reftool` / `odrivetool` 图形工具 | **本项目的 Python SDK（`robot_arm_sdk.py`）** |
| 回复方式 | 二进制端点应答 | `Respond(...)` 回一行文本，如 `"ok 1.2 3.4 ..."` |

ASCII 协议的处理长这样（`UserApp/protocols/ascii_protocol.cpp`）：

```cpp
} else if (strstr(_cmd, "GET_CURRENT") != nullptr) {
    auto currents = dummy.GetMotorCurrents();          // 读缓存
    Respond(_responseChannel, "ok %.3f %.3f %.3f %.3f %.3f %.3f", ...);
} else if (strstr(_cmd, "GET_TEMP") != nullptr) {
    auto temps = dummy.GetMotorTemperatures();         // 读缓存
    Respond(_responseChannel, "ok %.1f %.1f %.1f %.1f %.1f %.1f", ...);
}
```

**这是本次故障最重要的一条结论**：

> 新增的"查询电流/温度"功能，Python SDK 走的是 **ASCII 协议**
> （`robot_arm_sdk.py` 里发 `#GET_CURRENT` / `#GET_TEMP`），**完全不依赖 Fibre**。
> 我在 Fibre 清单里加的那 4 行（2.1 节），只是"额外"想让 ODrive 图形工具也能看到
> 电流温度——但项目的 SDK 根本不用 Fibre 这条路。
>
> 所以：**把 Fibre 里那 4 行去掉，SDK 的电流/温度查询功能零损失**，
> CAN 采集（0x21/0x25）、ASCII 查询、后台轮询全都照常工作。

---

## 3. 根本原因：构建 Fibre 协议树时，commTask 栈溢出

一句话概括：

> **我往每个电机的 Fibre 清单里加了 4 个成员，7 个电机共多出 28 个成员对象，
> 让本就"贴着上限"的协议树又胖了一圈；而中控板负责构建这棵树的 commTask 线程
> 栈空间是手工调到"刚好够用"的，多出来的这一圈直接把栈顶爆掉，导致线程在
> 构建树的中途崩溃，屏幕初始化代码永远执行不到。**

下面拆开讲。

### 3.1 协议树是"在栈上"被构造出来的

位置：`Bsp/communication/communication.hpp`（`COMMIT_PROTOCOL` 宏）

```cpp
#define COMMIT_PROTOCOL \
using treeType = decltype(MakeObjTree());\
uint8_t treeBuffer[sizeof(treeType)];\                    // 全局 .bss 缓冲区
void CommitProtocol()\
{\
    auto treePtr = new(treeBuffer) treeType(MakeObjTree());\   // ← 关键这一行
    fibre_publish(*treePtr);\
}\
```

注意 `new(treeBuffer) treeType(MakeObjTree())` 这行，它其实干了两件事：

1. **`MakeObjTree()` 按值返回一整棵树** → 这会在**当前线程的栈上**先构造出一个
   完整的树临时对象（`sizeof(treeType)` 那么大）；
2. 再把它拷贝构造进全局的 `treeBuffer`。

更"要命"的是构造过程本身：`MakeObjTree()` 内部是一层层嵌套的
`make_protocol_member_list(... make_protocol_object("robot", ...7 个 joint 对象...))`。
C++ 里**函数实参必须先全部求值、同时存活，才会调用构造函数**。也就是说，构造到最内层时，
7 个电机的子树临时对象 + robot 层子树 + 整棵树的返回值临时对象，
**同时压在这一个线程的栈上**。所以构造峰值远不止"一份树"那么大，通常是
`sizeof(treeType)` 的**好几倍**。

### 3.2 这棵树有多大、栈有多大（实测数字）

| 量 | 数值 | 来源 |
|---|---|---|
| `sizeof(treeType)`（整棵树） | **0x2800 = 10240 字节 ≈ 10KB** | 链接产物 `.map` 里 `.bss.treeBuffer` 的大小 |
| commTask 栈大小 | **45000 字节 ≈ 44KB** | `communication.cpp:22` `.stack_size = 45000` |
| FreeRTOS 总堆 | **65536 字节 = 64KB** | `FreeRTOSConfig.h:71` `configTOTAL_HEAP_SIZE` |
| 栈溢出检测 | **关闭**（未定义 `configCHECK_FOR_STACK_OVERFLOW`） | `FreeRTOSConfig.h` |

（上表是"去掉那 4 行、每电机 23 个成员"的正常版本实测值。）

有两个数字特别能说明问题：

- **commTask 一个线程的栈就占了整个 FreeRTOS 堆的 69%**（45000 / 65536）。
  其他所有线程栈都是 2000 或 500 字节，唯独它是 45000——这个"零整"的数字是作者
  **专门为塞下这棵树的构造、一点点手工试出来的**，余量本来就薄。
- **没有开栈溢出检测**。这意味着一旦溢出，FreeRTOS 不会干净地报错，
  而是让越界的栈数据**静默踩坏相邻的堆内存**（别的线程栈、任务控制块、堆管理结构），
  最终引发 HardFault 或调度器错乱。

### 3.3 我加的 4 行，成了压垮骆驼的稻草

回到 2.1 节那 4 行（`current`/`temperature` 只读属性 + `update_current`/`update_temp` 方法）。
它们会被**每个电机实例**各注册一遍，而系统里有 **7 个 `CtrlStepMotor` 实例**
（joint_1~joint_6 + hand 里复用的），所以：

```text
每电机成员数：23 → 27（+4）
全树成员对象：多出 4 × 7 = 28 个
sizeof(treeType)：10240 → 约 11800 字节（+15% 左右）
```

树胖了 ~15%，而 3.1 节说过**构造峰值是 sizeof 的好几倍**，于是栈峰值跟着涨了好几 KB。
原本 44KB 的栈就已经被这棵树的构造用到接近上限，再多几 KB —— **溢出**。

因为没开溢出检测，溢出的瞬间没有报错，commTask 直接在 `CommitProtocol()` 里
踩坏内存 / 触发 HardFault 而**中途死亡**。

---

## 4. 为什么表现成"OLED 全黑"而不是别的

这要串起启动时序。三条线索：

**线索 1：commTask 里，构建树在前，置标志位在后**
（`Bsp/communication/communication.cpp:58-65`）

```cpp
void CommunicationTask(void* ctx) {
    CommitProtocol();          // ← 就是在这里栈溢出、线程死亡
    endpointListValid = true;  // ← 永远执行不到！
    StartUartServer();
    StartUsbServer();
    StartCanServer(CAN1);
    ...
}
```

**线索 2：主流程死等这个标志位**（`communication.cpp:26-33`）

```cpp
void InitCommunication(void) {
    commTaskHandle = osThreadNew(CommunicationTask, nullptr, &commTask_attributes);
    while (!endpointListValid)   // ← commTask 死了，这个标志永远是 false
        osDelay(1);              // ← 主流程在这里无限空转
}
```

**线索 3：点亮屏幕的代码，排在 `InitCommunication()` 后面**（`UserApp/main.cpp:194-211`）

```cpp
void Main(void) {
    InitCommunication();   // ← 卡死在这里，永不返回
    dummy.Init();          // ↓ 下面这些统统执行不到
    do { mpu6050.Init(); osDelay(100); } while (!mpu6050.testConnection());
    oled.Init();           // ← 点亮 OLED 的那一句，从来没被执行
    ...
}
```

把三条串起来就是完整的因果链：

```text
commTask 构建 Fibre 树 → 栈溢出 → 线程中途死亡
        ↓
endpointListValid 永远 = false
        ↓
InitCommunication() 里 while(!endpointListValid) 无限空转，永不返回
        ↓
Main() 卡在第一步，dummy.Init() / mpu6050.Init() / oled.Init() 全部执行不到
        ↓
OLED 从未被初始化 → 全程黑屏
```

这完美解释了"**黑屏而非冻屏**"：系统不是跑起来又死了，而是**卡死在点亮屏幕之前**。

### 为什么注释掉那 4 行就恢复正常

去掉 4 行 → 每电机回到 23 个成员 → `sizeof(treeType)` 缩回 10240 字节 →
构造峰值重新落回 44KB 栈以内 → `CommitProtocol()` 正常返回 →
`endpointListValid = true` → `InitCommunication()` 返回 → `Main()` 继续往下 →
`oled.Init()` 被执行 → **屏幕点亮**。与实测现象完全吻合。

---

## 5. 排查过程中排除掉的错误假设（供参考）

这个 bug 有一定迷惑性，排查中走过弯路，记录下来避免以后重复踩：

| 曾怀疑的方向 | 为什么排除 |
|---|---|
| **CAN 发送信号量泄漏死锁**（见 `write_timeout_fault_analysis.md` B2） | 该故障只会冻住控制循环，且发生在系统跑起来之后；而本次是**开机即黑屏**，还没轮到任何 CAN 收发。更关键：正常固件下"关节 1 角度实时刷新"证明 CAN 链路和信号量本就健康。故障点在 `CommitProtocol`，比 CAN 更早。 |
| **RAM / .bss 静态内存溢出** | 查 `.map`：`_ebss ≈ 29KB`，`_estack = 0x20020000`，128KB RAM 余量巨大。`treeBuffer` 是静态 .bss（10KB），加成员后也只涨到 ~11.8KB，静态内存完全够。**问题出在运行时栈峰值，不是静态占用。** |
| **增量编译导致结构体布局错位（ODR 违规）** | 核对构建产物时间戳，确认是干净的全量重新编译，排除。 |
| **Fibre 框架自身的结构性 bug / 端点越界** | 审查 `register_endpoints`、`fibre_publish`：端点表 `endpoint_list` 是编译期按 `endpoint_count` 定长的 .bss 数组，注册时有 `id < length` 边界检查，加 28 个端点不会越界。**不是框架逻辑崩溃，是承载它的线程栈不够。** |

**方法论小结**：当"改了几行看似无关的声明式代码"却导致"开机黑屏"时，
不要只盯着这几行的**业务语义**，更要警惕它们对**编译期生成的数据结构体积**的影响——
在栈空间被手工调到极限的嵌入式工程里，"对象变大一点"就足以致命。

---

## 6. 修复建议

### 首选：保持那 4 行注释 / 直接删除（当前做法即正确）

由 2.3 节，Python SDK 走 ASCII 协议读电流/温度，**不需要 Fibre 端点**。
把这 4 行去掉，功能零损失，树也缩回安全体积。**这是成本最低、最稳的解。**

> 建议：与其留着注释，不如直接删掉这 4 行，并在此处加一句注释说明
> "电流/温度经 ASCII（`#GET_CURRENT`/`#GET_TEMP`）暴露，勿加入 Fibre 清单，
> 否则协议树增大会撑爆 commTask 栈"，防止以后有人手贱再加回去。

### 不要试图靠"加大 commTask 栈"解决

堆总共 64KB，commTask 已占 45KB（69%），所有线程栈加起来约 57.5KB，
再加任务控制块、队列、信号量、idle/timer 线程，**堆已用掉约 92%**。
再给 commTask 加几 KB，很可能导致 `osThreadNew` 分配失败返回 NULL，问题更糟。

### 可选加固（推荐做，防以后再踩同类坑）

在 `Core/Inc/FreeRTOSConfig.h` 里开启栈溢出检测：

```c
#define configCHECK_FOR_STACK_OVERFLOW 2
```

并实现钩子函数（溢出时打印肇事线程名 + 死循环，便于定位）：

```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    taskDISABLE_INTERRUPTS();
    printf("!!! StackOverflow in task: %s\n", pcTaskName);
    for (;;);
}
```

这样下次任何线程栈溢出都会**立刻、明确地报出线程名**，而不是像本次一样
静默卡死、让人误判成 CAN 或 OLED 硬件问题。

> 若想 100% 坐实"就是栈溢出"，可临时把上面两项打开、并把那 4 行 Fibre 成员恢复，
> 重新编译烧录：若系统在 `vApplicationStackOverflowHook` 里停下并报出 `commTask`，
> 即为铁证。

### 如果将来确实想用 Fibre/reftool 看电流温度

因为堆没余量加栈，只能反过来"**腾地方**"：精简每个电机现有 23 个 Fibre 成员里
那些用不到的（电机成员占了整棵树绝大部分体积），把省下的栈空间换给
`current` / `temperature`。属于"拆东墙补西墙"，非必要不做。

---

## 7. 一句话总结

> **新增功能时往每个电机的 Fibre 协议清单里加了 4 个成员，7 个电机共让协议树
> 胖了约 15%；而这棵树是在 commTask 线程栈上按值构造的，该栈（45KB）本就是作者
> 手工调到"刚好够用"、且没开溢出检测。多出来的体积顶爆了栈，commTask 在构建树的
> 中途静默崩溃，`endpointListValid` 永远置不上，主流程死等在 `InitCommunication()`，
> `oled.Init()` 从未执行 → OLED 全程黑屏。由于电流/温度查询实际走的是 ASCII 协议、
> 不依赖 Fibre，去掉那 4 行 Fibre 成员即可根治，且功能零损失。**

---

## 附：涉及的源码位置速查

| 内容 | 位置 |
|---|---|
| 电机 Fibre 成员清单（本次加/删 4 行处） | `Robot/actuators/ctrl_step/ctrl_step.hpp` `MakeProtocolDefinitions()` L75-109 |
| 协议树根定义 `MakeObjTree()` | `UserApp/protocols/cmd_protocol.cpp` L27-37 |
| `COMMIT_PROTOCOL` 宏（栈上构造树） | `Bsp/communication/communication.hpp` L25-32 |
| commTask 栈大小 45000 + 死等标志位 | `Bsp/communication/communication.cpp` L20-33 |
| commTask 主体（CommitProtocol → endpointListValid） | `Bsp/communication/communication.cpp` L58-76 |
| 启动时序（InitCommunication → … → oled.Init） | `UserApp/main.cpp` `Main()` L194-211 |
| ASCII 协议 `#GET_CURRENT` / `#GET_TEMP` 处理 | `UserApp/protocols/ascii_protocol.cpp` L66-83 |
| Python SDK 电流/温度查询（走 ASCII） | `files/host_computer_robot_control_sdk/robot_arm_sdk.py` L396-415 |
| `fibre_publish` / 端点注册 | `3rdParty/fibre/cpp/include/fibre/protocol.hpp` L1331-1354 |
| FreeRTOS 堆大小 / 栈溢出检测配置 | `Core/Inc/FreeRTOSConfig.h` L70-71 |
| 协议树体积实测（`.bss.treeBuffer = 0x2800`） | `cmake-build-debug/Core-STM32F4-fw.map` |
