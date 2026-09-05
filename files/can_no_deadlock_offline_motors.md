# 断线场景下 CAN 为何不死锁：电机 2~6 离线，程序照常运行

> 面向场景：关节 1↔2 之间的 CAN 线（2 根）和电源线（4 根）断开，电机 2~6 全部离线，
> 只有关节 1 与中控板（`dummy-ref-core-fw`）相连。此时中控板开机后仍会向 6 个电机
> 发各种 CAN 帧（`dummy_robot.cpp` 的 `ApplyJointAcceleration()` 发 6 帧、
> `EnableMotorTempWatch(true)` 发广播帧等）。
>
> 疑问：**发往已经断连的电机 2~6 的帧不会失败吗？为什么没有触发 CAN 信号量死锁、
> 把程序卡死？收不到 2~6 的信息，对程序运行到底有没有影响？**
>
> 本文用一句话先给结论，再逐层拆开讲清楚。所有论断都对应到具体源码位置。

---

## 0. 一句话结论

> **CAN 帧"发送成功"只要求总线上有任意一个活着的节点回 ACK，与帧的目标 ID 是谁无关。
> 关节 1 仍然连在总线上，且它的接收过滤器配成"收所有帧"，所以它会给中控板发出的
> 每一帧（包括那些"发给已经不存在的 2~6"的帧）回 ACK。于是中控板的每次 CAN 发送都
> 成功、信号量每次都归还、永不泄漏——也就永远不会死锁。至于 2~6，它们收不到帧、
> 不应答，只会让"2~6 的角度/电流/温度数据不更新"和"发给 2~6 的命令不生效"，
> 但这都是局部的、非阻塞的，绝不影响中控板自身继续运行。**

---

## 1. 先厘清一个最关键的误区：CAN 的"发送成功" ≠ "目标收到并执行"

绝大多数人 intuition 上会认为："我发一帧给 3 号电机，3 号电机不在了，这帧肯定发送失败。"
**这个直觉是错的。** CAN 总线上有两个完全不同层面的"成功"：

| 层面 | 名称 | 由谁决定 | 含义 |
|---|---|---|---|
| **链路层** | **ACK 应答** | 总线上**任意一个**正确收到帧的节点 | "这帧电平/CRC 没坏，总线上有人收到了" |
| **应用层** | **业务应答** | **目标节点**（ID 匹配的那个） | "3 号电机收到了查询，回一帧角度数据" |

这两件事**互相独立**：

- **ACK** 是 CAN 硬件在每一帧末尾的"ACK 槽"里自动完成的。协议规定：**任何**正确接收到
  该帧（没有位错误、格式错误、CRC 错误）的节点，都会在 ACK 槽把电平拉低（显性位），
  告诉发送方"有人收到了"。发送方只要检测到 ACK 槽被拉低，就判定**本帧发送成功**。
  **这个动作与帧的目标 ID 是谁、与接收节点的 ID 过滤器是否匹配，统统无关。**
- **业务应答** 才是"目标节点收到后，主动再发一帧数据回来"，这需要目标节点真的存在、
  真的收到了、且应用层愿意回复。

打个比方：

> 你在一个群里 @张三 发了条消息。
> - **ACK** = "消息成功发进群了"（只要群里还有**任何一个人**在线收到了，服务器就算送达）；
> - **业务应答** = "张三本人看到并回你了"。
>
> 现在张三退群了（电机 2~6 断连），但群里还有李四在线（关节 1）。
> 你 @张三 发消息——**消息依然"发送成功"**（李四所在的服务器收到了、确认了送达），
> 只是**张三不会回你**。发送方从"发送成功"这个信号里，**根本分辨不出**目标到底在不在。

CAN 就是这样：**"发出去了、有人 ACK 了" = 发送成功；"目标回没回数据" = 另一码事。**

### 1.1 本项目关节 1 的过滤器：干脆"照单全收"

位置：`dummy-42motor-fw/Core/Src/can.c` `MX_CAN_Init()` L65-76

```c
sFilterConfig.FilterMode       = CAN_FILTERMODE_IDMASK;
sFilterConfig.FilterIdHigh     = 0x0000;
sFilterConfig.FilterIdLow      = 0x0000;
sFilterConfig.FilterMaskIdHigh = 0x0000;   // ← 掩码全 0
sFilterConfig.FilterMaskIdLow  = 0x0000;   // ← 掩码全 0
sFilterConfig.FilterActivation = ENABLE;
```

IDMASK（掩码）模式下，**掩码位 = 0 表示"这一位我不关心"**。掩码全 0，就是
"**所有 ID 我都要**"——关节 1 会把总线上的**每一帧**都收进接收 FIFO，
不管那帧是发给 nodeID 0（广播）、nodeID 1（自己）还是 nodeID 2~6（别人）。

所以关节 1 是双重保险：

1. 作为 CAN 节点，它**天然会给每一帧正确接收的帧回 ACK**（协议硬件行为）；
2. 它的过滤器又配成**收所有帧**，所以发往 2~6 的帧它也照收不误（收下后再由应用层
   按 ID 判断：是自己的就执行，不是自己的就忽略）。

**结论：只要关节 1 通电、CAN 线接通，中控板发出的任何一帧都必然被 ACK，
中控板的 CAN 发送就必然"成功"。**

---

## 2. 信号量死锁的"真正触发条件"是什么

要理解为什么不死锁，得先看清楚死锁**到底需要什么前提**。

### 2.1 这把锁怎么拿、怎么还

位置：`dummy-ref-core-fw/Bsp/communication/interface_can.cpp`

```cpp
// 发帧前拿锁（无限期等待）
void CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader) {
    if (canCtx->handle->Instance == CAN1)
        semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);   // L241 拿锁
    ...
    if (semaphore_status == osOK)
        HAL_CAN_AddTxMessage(...);                                           // 把帧塞进发送邮箱
}

// 发送【成功】时还锁（由完成中断回调触发）
void tx_complete_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx) {
    if (hcan->Instance == CAN1)
        osSemaphoreRelease(sem_can1_tx);                                     // L123 还锁 ✓
}

// 发送【失败】时的错误回调 —— 空函数，不还锁
void tx_error(CAN_context* ctx, uint8_t mailbox_idx) {}                      // L135-137 ✗
```

`sem_can1_tx` 是一把**二值信号量**（全世界只有 1 把钥匙，见 `Core/Src/freertos.c`
`osSemaphoreNew(1, 1, ...)`）。规则是：

- 谁要发帧，先拿钥匙（`osSemaphoreAcquire(..., osWaitForever)`，**拿不到就永远等**）；
- 帧**发送成功** → 完成中断 → `tx_complete_callback` → 还钥匙；
- 帧**发送失败**（TERR）→ 错误中断 → `HAL_CAN_ErrorCallback` 走 TERR 分支 → 调空函数
  `tx_error()`、只清错误标志 → **不还钥匙**。

### 2.2 死锁的唯一前提：发送失败（TERR）

把上面串起来：**只有当某一帧"发送失败"时，钥匙才会永久丢失**，之后任何线程再调
`CanSendMessage` 都会在 `osWaitForever` 上永久阻塞 → 死锁。

那"发送失败"什么时候发生？由第 1 节可知——**当且仅当帧发出后，总线上没有任何一个
节点回 ACK**（ACK error）。典型情形：

- 总线上**一个活节点都没有**：所有电机都没上电 / 都没启动完；
- 断线把**所有**其他节点都甩出了总线（只剩中控板自己）；
- CAN_H/CAN_L 被短接、总线电平被破坏，帧本身损坏，没人能正确接收。

### 2.3 当前场景为什么不满足死锁条件

现在的硬件是：**关节 1 仍与中控板在同一段总线上、且通电工作**。

于是，中控板发出的**每一帧**——无论是：

- `EnableMotorTempWatch(true)` 发的广播帧（nodeID=0, cmd=0x7d），还是
- `ApplyJointAcceleration()` 发的 6 帧单播（nodeID=1~6, cmd=0x14），其中 5 帧"发给"已断连的 2~6，

——物理上都会到达关节 1，都会被关节 1 正确接收并**回 ACK**。因此：

```text
中控板发帧（哪怕目标是 2~6）
   → 关节 1 收到并 ACK
      → 中控板 CAN 外设判定 TXOK=1（发送成功）
         → 触发 tx_complete_callback
            → osSemaphoreRelease(sem_can1_tx)  钥匙归还 ✓
```

**钥匙每次都还回来了，信号量永远健康，`osSemaphoreAcquire` 永远不会卡住 → 不死锁。**

这就是为什么 `dummy_robot.cpp` L52-55 那两处（以及运行期每 5ms 的高频发帧）
在电机 2~6 断连时依然安然无恙。

---

## 3. 逐一回答三个问题

### Q1：为什么注释 4 行恢复正常后，程序不会因为 CAN 信号量死锁卡死在 `dummy_robot.cpp` 52-55？

```cpp
// Robot/instances/dummy_robot.cpp  DummyRobot::Init()
ApplyJointAcceleration();          // L52：发 6 帧单播 0x14 给 nodeID 1~6
// Motors force enableTempWatch=false on every boot ...
EnableMotorTempWatch(true);        // L55：发广播帧 0x7d（nodeID=0）
```

**因为这两处发出的帧全部被关节 1 ACK，发送全部成功，信号量全部归还**（见第 2.3 节）。
死锁需要"发送失败"，而只要关节 1 这个活邻居在，发送就不会失败。

> 补充：其实 `ApplyJointAcceleration()` 在旧版 `b7caae5` 里就一直存在、一直在发这 6 帧，
> 从来没死锁过——这本身就是"关节 1 会 ACK 所有帧"的活证据。`EnableMotorTempWatch(true)`
> 是新增的，但它发的是广播帧，同样被关节 1 ACK，性质完全一样。

### Q2：硬件上电机 2~6 的 CAN 已经断了，为什么程序还能正常运行？

因为**程序的运行不依赖"2~6 应答"**，只依赖"发送这一步不阻塞"。而发送这一步：

- **链路层**：关节 1 ACK 保证发送成功、信号量归还，`CanSendMessage` 立即返回，不阻塞；
- **应用层**：中控板发完查询帧后，是**异步**等回帧的——回帧来了就进
  `OnCanMessage` 更新数据，**回帧不来就什么都不发生**，中控板不会"站在原地等 2~6 回话"。

CAN 的收发是解耦的：发送方把帧塞进邮箱、拿到 ACK、就完事了；接收方什么时候回、
回不回，发送方并不阻塞等待。所以 2~6 离线，只是"少了几帧回帧"，主流程照常往前跑。

### Q3：收不到电机 2~6 的信息，对程序运行有影响吗？

**有影响，但都是局部的、非致命的、非阻塞的：**

| 受影响 | 具体表现 |
|---|---|
| 2~6 的角度 | `OnCanMessage` 收不到 2~6 的 0x23 回帧 → `motorJ[2..6]->angle` 保持上电初值/旧值不更新 → OLED 上这几个关节角度是"冻结"的 |
| 2~6 的电流/温度 | 收不到 0x21/0x25 回帧 → `current`/`temperature` 字段保持 0 或旧值 → ASCII `#GET_CURRENT`/`#GET_TEMP` 对这几个关节返回 0 |
| 发给 2~6 的命令 | 0x14（设加速度）、运动指令等，2~6 物理上收不到 → **不生效，这几个关节不会动** |

| **不受影响** | 原因 |
|---|---|
| 中控板自身运行 | 发送不阻塞、接收异步，主循环/各线程照常调度 |
| 关节 1 的控制与数据 | 关节 1 在线，0x23 等回帧正常收到，angle 实时刷新（这正是你观察到的"关节 1 角度随真实转动变化"） |
| OLED 显示刷新 | OLED 是独立任务，只要有初始化就会一直刷（本次黑屏是**另一个**问题，见第 4 节） |
| USB / ASCII 通信 | 与 CAN 完全无关，上位机照常收发 |
| 程序稳定性 | 不崩溃、不死锁、不阻塞 |

一句话：**2~6 离线 = "这几个关节失明+瘫痪"，但中控板这个"大脑"活得好好的。**

---

## 4. 重要澄清：这和"注释 4 行修复黑屏"是两个独立的问题

很容易把两件事混在一起，这里必须分清——它们发生在**完全不同的启动阶段**：

```text
Main()  (UserApp/main.cpp L194-211)
  │
  ├─ InitCommunication()          ← 【阶段一：启动/通信初始化】
  │     └─ commTask: CommitProtocol()  ← 在这里构建 Fibre 协议树
  │            · 故障固件：树太大 → commTask 栈溢出 → 线程死亡
  │            · endpointListValid 永不置位 → Main 死等在此 → 后面全执行不到
  │            · ★ 此时根本还没轮到 dummy.Init()，L52-55 的 CAN 发帧一次都没跑过 ★
  │
  ├─ dummy.Init()                 ← 【阶段二：机器人初始化】
  │     ├─ ApplyJointAcceleration()   (L52)  ← CAN 发帧在这里
  │     └─ EnableMotorTempWatch(true) (L55)  ← CAN 发帧在这里
  │
  └─ oled.Init()                  ← 点亮屏幕
```

- **黑屏问题**（`oled_blank_fibre_tree_stack_overflow.md` 记录的那个）发生在**阶段一**：
  Fibre 协议树把 commTask 栈撑爆，程序**卡死在 `InitCommunication()`**，
  **压根执行不到阶段二的 `dummy.Init()`**，更谈不上 L52-55 的 CAN 发帧。
- **注释掉那 4 行**修复的是阶段一的栈溢出。修好之后，程序才能顺利走过阶段一、
  进入阶段二执行 L52-55 的 CAN 发帧——而**这时**这些发帧因为有章节 2.3 说的
  关节 1 ACK 保证，也**不会**死锁。

> 所以：
> - 我**曾经**把黑屏错误地归因于"CAN 信号量死锁卡在 L52-55"，这是**错的**——
>   故障固件根本没跑到 L52-55；
> - 真相是黑屏 = 阶段一 commTask 栈溢出；而 L52-55 的 CAN 发帧**自始至终都不会死锁**
>   （只要关节 1 在）。两个问题互不相干。

---

## 5. 那么，CAN 信号量死锁到底什么时候会真的发生？

参见 `write_timeout_fault_analysis.md` 的 B2 节。死锁需要"**总线上一个能 ACK 的节点都没有**"，
例如：

- 开机瞬间所有电机都还没上电/没启动完，中控板就开始 200Hz 发帧 → 第一帧就没人 ACK；
- 断线把**所有**电机都甩出总线（不像现在这样还留着关节 1）；
- CAN_H/CAN_L 短接或严重畸变，帧本身损坏，无人能正确接收。

**当前场景恰恰不满足**：关节 1 一直在线兜底 ACK。这也是为什么同样的固件、
同样发往 2~6 的帧，在"关节 1 在线"时岁月静好，一旦"连关节 1 也掉线/断电"
就可能瞬间踩中 B2 死锁。**关节 1 是这套系统当前不自锁的唯一救命稻草。**

> 这也从反面说明 B2 那个"发送失败不还锁"的缺陷依然真实存在、只是暂时没被触发。
> 一旦哪天关节 1 也离线，死锁就会复现。根治办法见 `write_timeout_fault_analysis.md`
> 第 6 节（错误回调补还锁 / `CanSendMessage` 改有限超时 / 开 AutoBusOff / 加看门狗）。

---

## 6. 一句话总结

> **CAN 的"发送成功"只看有没有节点回 ACK，不看目标在不在。关节 1 在线且过滤器
> "照单全收"，会给中控板发出的每一帧（含发往已断连的 2~6 的帧）回 ACK，于是每次发送
> 都成功、信号量每次都归还、永不泄漏、永不死锁——`dummy_robot.cpp` L52-55 因此安全。
> 2~6 离线只造成"这几个关节数据冻结、命令不生效"，属于局部非阻塞影响，中控板自身、
> 关节 1、OLED、USB/ASCII 全部照常。而之前的开机黑屏是发生在更早阶段的
> "Fibre 协议树撑爆 commTask 栈"，与这里的 CAN 发送毫无关系。**

---

## 附：涉及的源码位置速查

| 内容 | 位置 |
|---|---|
| `DummyRobot::Init()` 里的 CAN 发帧（L52/L55） | `Robot/instances/dummy_robot.cpp` `ApplyJointAcceleration()` L251-255、`EnableMotorTempWatch()` L210-213 |
| 各命令帧的 mode 码与 StdId 拼装 | `Robot/actuators/ctrl_step/ctrl_step.cpp`：0x7d `SetEnableTemp` L38-50、0x14 `SetAcceleration` L187-199、0x21 `UpdateCurrent` L259-265、0x25 `UpdateTemp` L268-274、0x23 `UpdateAngle` L316-322 |
| `ALL = 0`（广播 nodeID） | `Robot/instances/dummy_robot.h` L7 |
| 拿锁 / 还锁 / 失败不还锁 | `Bsp/communication/interface_can.cpp`：`CanSendMessage` L237-249、`tx_complete_callback` L116-126、`tx_error`（空）L135-137、`HAL_CAN_ErrorCallback` L188-235 |
| 二值信号量创建（只有 1 把钥匙） | `Core/Src/freertos.c` `osSemaphoreNew(1, 1, ...)` |
| **关节 1 过滤器"收所有帧"（掩码全 0）** | `dummy-42motor-fw/Core/Src/can.c` `MX_CAN_Init()` L65-76 |
| 电机端 CAN 配置（NART / 不自动 bus-off） | `dummy-42motor-fw/Core/Src/can.c` L55-57 |
| 启动时序（阶段一 InitCommunication vs 阶段二 dummy.Init） | `UserApp/main.cpp` `Main()` L194-211 |
| 黑屏根因（另一问题，阶段一栈溢出） | `files/oled_blank_fibre_tree_stack_overflow.md` |
| CAN 信号量死锁的完整分析（B2） | `files/write_timeout_fault_analysis.md` |
