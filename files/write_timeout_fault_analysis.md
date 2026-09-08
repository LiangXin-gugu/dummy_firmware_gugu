# 上位机 "Write timeout" 故障原因全梳理

> 面向场景：上位机以 50Hz 下发 MoveJ 轨迹，运行 20 秒~几十分钟后，
> 机械臂停止响应，OLED 屏幕画面冻住，随后上位机出现
> `SerialTimeoutException('Write timeout')`，重启后恢复，跑一会儿又复发。
>
> 本文把所有可能原因（硬件、固件、上位机）汇总在一起，尽量用大白话解释。
>
> **关联文档**（本文多处结论与它们互相印证）：
> - `oled_blank_fibre_tree_stack_overflow.md`：**开机**就 OLED 全黑（从未点亮）的另一类故障——Fibre 协议树撑爆 commTask 栈，卡在启动阶段，与本文"运行中挂死"是不同阶段的问题。
> - `can_no_deadlock_offline_motors.md`：电机 2~6 断线离线时 CAN 为何**不会**因发帧失败而信号量死锁（关节 1 兜底 ACK），是本文 B2 触发边界的重要补充。

---

## ✅ 最新结论（已复现并修复确认，务必先读）

> **本节结论优先于下文的历史推断。** 下文第 2/3 节最初把"运行中 OLED 冻住 + Write timeout"
> 主要归因于 **A1/A2 硬件断线点火 + B1（CAN 收帧越界 HardFault）**。但在 **B1 与 J1~J2 线缆都已修复**后，
> 该现象仍然复现，据此定位到**真正的元凶是一条纯软件的并发堆竞态（记为 B6）**，与 CAN/硬件无关：
>
> - **决定性对照**：`--no_sample`（只发 move_j）**稳定不崩**；一旦并发开启 GETJPOS 采样线程
>   （move_j + GETJPOS 同时跑）**必然运行一段时间后 Write timeout + OLED 冻死**。
> - **根因**：GETJPOS 应答用 `snprintf("%.2f")` → newlib 浮点格式化 → `dtoa` → `malloc/free`；
>   move_j 在命令消费线程用 `sscanf("%f")` → 同样走 `dtoa/strtod` 的 `malloc/free`。
>   本工程 newlib 的 malloc **未加锁**（无 `__malloc_lock`）且所有线程**共用一个 `_reent`**
>   （`configUSE_NEWLIB_REENTRANT=0`）→ 两线程并发进堆 → **堆损坏 → HardFault → 整机定格**。
> - **修复**：把 GETJPOS（及 GETLPOS/GET_CURRENT/GET_TEMP）应答改为**纯整数定点格式化**
>   （`FormatFixedN`，只用 `%ld`，不走 dtoa/malloc），使 USB 任务彻底退出浮点堆 → 竞态消失。
>   已烧录验证：并发跑 GETJPOS + move_j 不再 Write timeout、OLED 不再冻死。
> - 详见下文 **B6** 节。A 类硬件与 B1~B4 CAN 缺陷仍值得修（纵深防御），但它们**不是本次现象的原因**。

---

## 1. 先搞懂：Write timeout 到底是什么意思

串口通信有两个方向，超时分两种，**性质完全不同**：

| 类型 | 含义 | 说明 |
|---|---|---|
| 应答超时 | 数据已经发出去了，设备没回话 | 设备还活着，只是"不干活" |
| **Write timeout** | `serial.write()` 本身失败，数据**塞都塞不进去** | **设备已经不再接收数据了** |

打个比方：

- 应答超时 = 你往邮筒里投了信，但对方一直不回信（邮筒还在收信）；
- Write timeout = 邮筒的口被堵死了，信根本投不进去。

### 为什么设备会"塞不进去"？

主控的 USB 接收是"**一包一包轮着来**"的模式：

```
收到一包 USB 数据
  → 任务处理完这一包
  → 才重新打开接收口，等下一包
```

只要"处理"这个环节卡住（或者整个 MCU 死了），接收口就永远不再打开，
上位机发的数据全部堆在电脑和 USB 控制器的缓冲区里，堆满之后
`write()` 就报 Write timeout。

**结论：Write timeout = 设备侧接收通道整体停摆，是比较严重的故障信号。**

---

## 2. 一个决定性的观察：故障时 OLED 也冻住了

OLED 屏幕是一个独立的任务在刷新的，它不依赖 USB、不依赖 CAN、不依赖电机，
只要系统还"活着"，屏幕就会一直刷新（右上角还有实时的 FPS 数字）。

所以故障时看屏幕，可以立刻区分故障等级：

| 故障时的屏幕表现 | 说明 |
|---|---|
| 屏幕还在刷新，FPS 还在变 | 系统活着，只是某个功能（如控制循环）卡死 |
| **屏幕画面定格在最后一帧** | **整个 MCU 挂死**：所有任务全部停摆（运行中跑飞） |
| **屏幕从上电起全程没亮过** | **MCU 卡在启动早期**：连点亮屏幕的初始化都没走到（如 Fibre 协议树撑爆 commTask 栈，见 `oled_blank_fibre_tree_stack_overflow.md`） |
| 屏幕熄灭后重新亮起/重新显示 | MCU 发生了复位（重启） |

本次故障 **OLED 冻住（定格在最后一帧）** → 不是 USB 单独挂了，而是**整块主控板进入了硬挂死**
（典型是程序跑飞进入 HardFault 死循环）。务必与"全程没亮过"区分：后者是**启动阶段**就卡死，
属于另一类问题。后面列原因时，凡是能导致"运行中整机挂死"的都是重点怀疑对象。

> ⚠️ **修正**："OLED 冻住 = 整机 HardFault" 这一步判断是对的，但**HardFault 的来源不止 CAN 越界（B1）一种**。
> 任何让程序跑飞进死循环的路径都会冻住 OLED——本次确认的正是**堆损坏（B6）**触发的 HardFault，
> 它**不依赖 CAN、不依赖硬件线缆**，判据是"只在并发 GETJPOS 时发作、`--no_sample` 完全不发作"。

---

## 3. 可能原因汇总

### A 类：硬件原因

#### A1. J1~J2 之间的连线断了（已发现的事实）

关节之间的线缆一般有 6 根：CAN 两根（CAN_H/CAN_L）、电源、地等。
断一根的危害取决于断的是哪根：

- **断的是 CAN 线**：
  - 断点后方的关节 2~6 与总线失联 → 上位机看到 J2/J3 角度永远显示
    上电初始值 -75/180（这是"从没收到过这些节点应答"的表现，不是编码器坏）；
  - 断点处的悬空线头像一根天线，还会造成信号反射，**污染整条总线**，
    让原本正常的帧也出错；
  - 线头随机械臂运动抖动、间歇性搭接 → 产生**畸变帧/垃圾帧**打到总线上。
- **断的是电源线**：关节 2~6 供电时断时续，板卡反复欠压、重启，
  它们的 CAN 芯片会往总线上喷毛刺；同时整机电源被拉低，
  **主控板也可能跟着欠压**。
- **断的是地线**：各板之间"电压基准"对不齐，CAN 信号判断出错，
  同样污染总线。

**为什么触发时间随机（第 4 轮才死、之前几轮没事）**：
断点只在抖动到某个姿态时才出事，而抖动是由关节 1 的运动引起的，
所以故障出现的时间完全随机，和指令内容无关。

#### A2. 接触不良 / 接插件氧化 / 压接不牢

和 A1 同类，只是程度轻一点：线没断但接触电阻大、时通时断。
发热后（跑 30 分钟）更容易发作——**这可以解释"最早线没断时也有类似问题"**。

#### A3. 整机供电不足 / 电源纹波大

6 个关节同时运动时电流冲击大，如果电源余量不足或滤波不好：

- 电压瞬间跌落 → 主控 MCU 触发欠压复位（表现为设备消失后重新出现）；
- 或者 MCU 没复位但内部状态错乱 → 跑飞挂死。

#### A4. CAN 总线缺终端电阻 / 走线过长

CAN 总线两端应各有一个 120Ω 电阻（用万用表测 CAN_H/CAN_L 之间应约 60Ω）。
缺电阻会导致反射、误码率升高，平时勉强能跑，遇到振动/干扰就出错。

#### A5. 上位机侧 USB 链路本身（可能性较小，但要排除）

- USB 线质量差、经过供电不足的 Hub；
- Linux 的 USB 自动休眠把设备挂起；
- pyserial 的 `write_timeout` 设得过短。

区分方法：故障时用 `dmesg -w` 观察有没有 `usb disconnect` / 重新枚举。
**如果 OLED 冻住，基本可以排除这一类**（电脑侧问题不会冻住设备的屏幕）。

---

### B 类：固件（软件）原因

硬件问题负责"点火"，下面这些固件缺陷负责"把小火苗变成整机死亡"。
**同样的硬件小故障，如果固件健壮，最多丢几帧；现在的固件则会直接挂死或永久卡死。**

#### B1. CAN 接收不做 ID 检查 → 越界访问 → 整机 HardFault（✅ 已修复，曾是★最可能的死因）

位置：`UserApp/protocols/can_protocol.cpp`

```cpp
uint8_t id = rxHeader->StdId >> 7;   // 取值范围 0~15，没有任何检查！
...
dummy.motorJ[id]->UpdateAngleCallback(...);  // motorJ 只有 7 个元素
```

- CAN 帧 ID 最大 0x7FF，右移 7 位后 `id` 可以到 **15**，
  而 `motorJ` 数组只有 7 个元素（下标 0~6）；
- 一旦总线上出现垃圾帧（A1~A4 都会制造垃圾帧），`id ≥ 8` 时
  程序会去访问数组之外的内存，拿到一个"野指针"，再拿野指针调用函数 →
  **HardFault（程序跑飞）**；
- 更致命的是：这段代码运行在 **CAN 接收中断** 里。中断里跑飞，
  整个系统永远回不来 → **所有任务瞬间定格：电机停、OLED 冻住、USB 不收包**。

这条路径曾完美解释"Write timeout + OLED 冻住 + 随机触发 + 重启才好"。

**当前状态：已加 ID 边界守卫，此路径已堵死**（`can_protocol.cpp` L26-45）：

```cpp
const bool validMotorId = (id >= 1 && id <= 6);   // 非法 id（含 hand=7、垃圾帧 ≥8）一律不解引用
switch (cmd) {
    case 0x21: if (validMotorId) memcpy(&dummy.motorJ[id]->current,     data, sizeof(float)); break;
    case 0x23: if (validMotorId) dummy.motorJ[id]->UpdateAngleCallback(*(float*)(data), data[4]); break;
    case 0x25: if (validMotorId) memcpy(&dummy.motorJ[id]->temperature, data, sizeof(float)); break;
}
```

> 影响：B1 这条"运行中整机 HardFault"死因**已被消除**。打补丁后若原故障不再复发，
> 即反证 B1 为元凶；若仍复发，则需在"B1 已排除"的前提下另寻整机挂死路径。

#### B2. CAN 发送信号量泄漏 → 控制循环永久卡死

位置：`Bsp/communication/interface_can.cpp`

- 每次发 CAN 帧前先拿一把"锁"（信号量 `sem_can1_tx`），
  发送成功后由完成回调把锁还回去；
- 但 CAN 配置成"只发一次、失败不重发"（NART）。如果哪一帧发送失败
  （比如总线上没人应答），程序走的是**错误回调**，而错误回调里
  **忘了把锁还回去**；
- 结果：一次发送失败 → 锁永久丢失 → 下一次发帧的线程永远等锁 →
  控制循环（每 5ms 给电机发指令的任务）当场卡死 → 机械臂停在原地、
  角度不再更新、OLED 状态位全显示 `_`。

注意：这个故障**只会冻住控制循环，USB 和 OLED 理论上还活着**。
它解释的是"机械臂停止响应 + 状态位全 `_`"，单独不足以造成 Write timeout。
但它和 B1 经常被同一次总线异常先后触发。

##### B2 附录：对照代码的逐步详解

> 下面按"锁是什么 → 怎么拿 → 正常怎么还 → 异常为什么没还 → 死了之后发生什么"
> 的顺序，把整条链路对照代码走一遍。

**第 0 步：这把"锁"是什么 —— 一个二值信号量**

位置：`Core/Src/freertos.c`（系统启动时创建）

```c
// Create a semaphore for CAN TX
osSemaphoreDef(sem_can1_tx);
sem_can1_tx = osSemaphoreNew(1, 1, osSemaphore(sem_can1_tx));
//                            ↑  ↑
//                    最大计数=1  初始计数=1
```

`osSemaphoreNew(1, 1, ...)` 创建的是**二值信号量**：全世界只有 **1 把钥匙**。
谁拿到钥匙谁才能用 CAN 发送邮箱，用完必须归还，否则别人永远拿不到。
关键点：这把钥匙**只在开机时创建这一次**，系统里没有任何"备用钥匙"或
"定期补发"机制——钥匙一旦弄丢，就是永久丢失。

**第 1 步：发帧前拿锁 —— `CanSendMessage`**

位置：`Bsp/communication/interface_can.cpp`

```cpp
void CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    osStatus semaphore_status;
    if (canCtx->handle->Instance == CAN1)
        semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever); // ← 拿锁，无限期等待！
    ...
    if (semaphore_status == osOK)
        HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData, ...);      // ← 把帧放进发送邮箱
}
```

所有要发 CAN 帧的代码都经过这里，而且第二个参数是 **`osWaitForever`（无限等）**——
只要锁丢了，调用者就永远停在这一行，没有任何超时退出。

谁在调用它？主要是两个高频路径（每 5ms 一次，200Hz）：

- 使能时：`ThreadControlLoopFixUpdate` → `MoveJoints` → 6 个关节各发一帧 0x07 位置指令；
- 未使能时：同线程 → `UpdateJointAngles` → 广播 0x23 角度查询。

此外命令线程的 `SetEnable`、`Reboot` 等也会调用。

**第 2 步：发送成功时，锁由中断归还**

帧发出成功后，STM32 的 CAN 外设产生"发送邮箱完成"中断，HAL 库检查状态位：

位置：`Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c`（中断服务程序）

```c
if ((tsrflags & CAN_TSR_RQCP0) != 0U)          // 邮箱0有了"结果"
{
    if ((tsrflags & CAN_TSR_TXOK0) != 0U)      // 结果是"成功"
    {
        HAL_CAN_TxMailbox0CompleteCallback(hcan);   // → 走【完成回调】
    }
    else                                        // 结果是"失败"
    {
        if (tsrflags & CAN_TSR_ALST0)  errorcode |= HAL_CAN_ERROR_TX_ALST0;  // 仲裁丢失
        else if (tsrflags & CAN_TSR_TERR0) errorcode |= HAL_CAN_ERROR_TX_TERR0; // 发送错误
        // ...最后统一调 HAL_CAN_ErrorCallback → 走【错误回调】
    }
}
```

**这里是整个问题的分水岭**：同一帧发送，成功走"完成回调"，失败走"错误回调"，
两条完全不同的路。成功那条路里，本项目归还了锁：

位置：`Bsp/communication/interface_can.cpp`

```cpp
void tx_complete_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    if (hcan->Instance == CAN1)
        osSemaphoreRelease(sem_can1_tx);   // ← 成功：还锁 ✓
    ...
}
```

**第 3 步：发送失败时，错误回调"忘了"还锁 —— 泄漏点**

先理解为什么会失败：主控 CAN 配置为
`AutoRetransmission = DISABLE`（即硬件 NART 位生效，"每帧只尝试发送一次"，
见 `Core/Src/can.c`）。总线上没人应答（ACK 错误）或信号畸变（位错误）时，
这一帧就被硬件放弃，`TXOK=0、TERR=1`，进入上面 HAL 代码的 else 分支，
最终调用项目的错误回调：

位置：`Bsp/communication/interface_can.cpp`

```cpp
void tx_error(CAN_context* ctx, uint8_t mailbox_idx)
{
}                              // ← 空函数！什么都没做！

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
    ...
    if (hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR0)
    {
        tx_error(ctx, 0);      // ← 调了个空函数
        hcan->ErrorCode &= ~HAL_CAN_ERROR_EWG;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_ACK;
        hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_TERR0;   // ← 只是清了错误标志
        // ★★★ 从头到尾没有 osSemaphoreRelease(sem_can1_tx)！★★★
    }
    // TERR1 / TERR2 两个邮箱的分支同样不还锁
    ...
}
```

对比一下就很清楚：

| 发送结果 | HAL 调用的回调 | 项目代码做了什么 | 锁的命运 |
|---|---|---|---|
| 成功（TXOK） | TxMailboxComplete | `osSemaphoreRelease(sem_can1_tx)` | 归还 ✓ |
| 失败（TERR） | ErrorCallback | 调空函数 `tx_error()`、清标志 | **永久丢失 ✗** |
| 仲裁丢失（ALST） | ErrorCallback | 手动置位重发该邮箱 | 锁先扣着等重发结果；重发成功能归还，重发再失败（转 TERR）同样泄漏 |
| 手动中止（Abort） | TxMailboxAbort | 只给计数器 +1 | **同样不还锁 ✗** |

也就是说：**总线上一帧都没失败时一切正常；只要失败一帧（TERR），
全世界唯一的那把钥匙就永远沉底了。**

**第 4 步：锁丢了之后，谁先死、谁还活着**

下一次任何线程调 `CanSendMessage`，都会在
`osSemaphoreAcquire(sem_can1_tx, osWaitForever)` 上永久阻塞。

- **最先死的是 Realtime 级的 `ThreadControlLoopFixUpdate` 线程**（它每 5ms
  必然发 CAN 帧）→ 控制循环停摆 → 电机收不到新目标，抱在当前位置，
  机械臂"停止响应"；不再发 0x23 查询 → 角度冻结、`jointsStateFlag` 停在
  清零状态 → OLED 状态位显示全 `_`；
- 之后凡是碰 CAN 的操作（命令线程执行 `SetEnable`/`Reboot` 等）也会跟着卡死；
- **不碰 CAN 的任务照活**：OLED 任务（纯显示，屏幕继续刷新、FPS 继续变）、
  USB 接收任务（`Push` 入队是非阻塞的，上位机还能写指令进来，命令线程
  在卡在 CAN 之前还能回"ok"）。

这正是 B2 与 B1 的区别：**B2 是"植物人"——部分系统还活着；
B1 是"脑死亡"——整机定格（OLED 冻住 + Write timeout）**。

**第 5 步：什么现实情况会触发"一帧发送失败"**

CAN 协议要求帧发出时总线上**至少有一个其他节点回 ACK**，否则算发送错误。

**关键澄清（极易误判）**：这里的 ACK 是**链路层硬件行为**，由**任意一个正确收到帧的节点**
提供，**与帧的目标 ID 是谁无关**。所以"发往一个已离线的节点"**并不等于**"发送失败"——
只要总线上还有别的活节点会 ACK，这帧就算发送成功。本项目电机侧过滤器配成"收所有帧"
（掩码全 0），关节 1 只要在线就会给中控板发出的每一帧（含发往离线 2~6 的帧）回 ACK。
详见 `can_no_deadlock_offline_motors.md`。

因此，真正触发 TERR（发送失败）的是"**总线上一个能 ACK 的活节点都没有**"或"**帧本身损坏**"：

- 所有电机都没上电 / 都没启动完（开机瞬间主控就开始 200Hz 查询，
  电机板还没准备好 → 第一帧就可能失败 → 还没动就死）；
- 线缆断路 / 接插件松脱，把**所有**节点（含关节 1）都甩出了总线；
- CAN_H/CAN_L 被短接、断线线头搭接 → 总线电平被破坏，位错误（帧损坏，无人能正确接收）；
- 强干扰造成的偶发误码（跑 30 分钟碰上一次也足够）。

结合本项目的实际硬件状态：**J1~J2 断线、关节 2~6 离线本身不会触发 TERR**（关节 1 仍兜底 ACK）；
真正的风险是**断点悬空线头随关节运动抖动、间歇搭接污染总线造成位错误**，或**关节 1 也意外掉线/断电**。
一旦满足其一，**一次 TERR 就足够让整个系统永久瘫痪**（因为 B2 的锁泄漏缺陷仍未修）。

**第 6 步：为什么"重启就好，跑一会儿又犯"**

锁丢了没有任何自愈路径（没有超时、没有看门狗、没有错误恢复），
只有断电重启才会重新执行 `osSemaphoreNew` 把钥匙发回来。
重启后硬件故障（断线）还在，于是又进入"正常跑 → 偶发一帧失败 → 永久死锁"
的循环，表现为随机时长后复发。

**第 7 步：修复方案（代码示意）**

最小修复——在错误回调的 TERR 分支归还锁（与完成回调对称）：

```cpp
} else if (hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR0)
{
    tx_error(ctx, 0);
    if (hcan->Instance == CAN1)
        osSemaphoreRelease(sem_can1_tx);   // ← 补上：失败也要还锁
    hcan->ErrorCode &= ~HAL_CAN_ERROR_EWG;
    ...
}
// TERR1 / TERR2 分支同理；Abort 回调也应补还锁
```

更稳妥的组合拳：

1. `CanSendMessage` 把 `osWaitForever` 改为有限超时（如 10ms），
   超时即放弃本帧并计数——任何泄漏都不再能把线程钉死；
2. 打开 `AutoBusOff = ENABLE`，bus-off 后硬件自动恢复（见 B3）；
3. 增加 IWDG 独立看门狗作为最后防线；
4. 用 `can1Ctx.unexpected_errors` / `ErrorCallbackCnt` 计数定位故障频率，
   验证硬件修复效果。

> 备注：`CanSendMessage` 中 `HAL_CAN_AddTxMessage` 的返回值也被忽略了——
> 若它返回失败（如邮箱忙），锁同样处于"拿了但没人还"的状态。
> 修复时应一并处理：返回非 HAL_OK 时立即 `osSemaphoreRelease` 回滚。

##### B2 附录（续）：为什么最小修复不够 + 组合拳分层详解

> 一句话概括：**最小修复只是"堵住了目前已知的一个漏洞"，
> 而组合拳是一套"纵深防御"——即使将来再出现没预料到的漏洞，
> 系统也不会再永久瘫痪**。

**为什么只做"TERR 分支还锁"不够**

1. **泄漏路径不止 TERR 这一条**：
   - ALST 分支（仲裁丢失）的处理是 `SET_BIT(...TXRQ)` 把帧重新排队，
     指望重发成功后走完成回调还锁——但如果重发又失败、
     或错误码组合没覆盖到（ALST 与 TERR 同时置位时的分支走向），锁照样丢；
   - Abort 分支（`tx_aborted_callback`）只加了计数器
     `TxMailboxAbortCallbackCnt++`，**没有还锁**。一旦有代码调用
     `HAL_CAN_AbortTxRequest`（比如将来加超时取消逻辑），锁就漏；
   - `HAL_CAN_AddTxMessage` 返回值被忽略：若返回非 `HAL_OK`
     （邮箱全忙、句柄状态不对），帧根本没进硬件，**任何回调都不会来**，
     锁拿了就没人还。
2. **它是"打补丁"，不是"改结构"**：`osWaitForever` 的等待逻辑没变，
   整个系统仍建立在"**每一条**还锁路径都必须永远正确"这个脆弱前提上。
   今天堵了 3 条，明天改代码引入第 4 条（新加错误分支、HAL 升级改变回调时序），
   故障就原样复发，且极难排查。
3. **它完全管不了 B1/B3/B4**：最小修复只针对 B2（信号量泄漏 → 控制循环卡死）。
   B1 的整机 HardFault（OLED 冻住）、B3 的 bus-off 不自愈、
   B4 的电机固件死循环，它一个都治不了。

**组合拳 = 四层保险，一层失效还有下一层兜底**

*第 1 层：有限超时等待 —— 结构性根治（最重要）*

```cpp
// 现在：拿不到锁就永远等
semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);

// 改成：最多等 10ms，等不到就放弃这一帧
semaphore_status = osSemaphoreAcquire(sem_can1_tx, 10);
if (semaphore_status != osOK)
{
    canCtx->tx_timeout_cnt++;   // 记一笔
    return;                     // 放弃本帧，线程照常返回
}
```

与最小修复的思路**完全不同**：最小修复是"保证锁一定会还"，
这条是"**就算锁永远不还，也没人能被困死**"。
改完后无论还有哪条没发现的泄漏路径、还是将来引入新泄漏，
最坏结果只是"CAN 发不出帧、计数上涨"，而不是 5ms 控制线程被永久钉死。
对 50Hz 指令流，偶尔丢一帧无关紧要（下一帧 20ms 后就来），
但线程卡死是灾难。**这条把"未知 bug"的杀伤力从致命降级为可忽略**。

*第 2 层：打开 AutoBusOff —— 让硬件自己爬起来*

把 `can.c` 里 CAN 初始化的 `AutoBusOff = DISABLE` 改成 `ENABLE`。
CAN 协议规定：连续发送失败会让错误计数器（TEC）累加，超过 255 进入
**bus-off**（控制器彻底离线，不收发任何帧）。当前配置下进入 bus-off 后
软件永远不会把它拉回来，只有断电重启（见 B3）。
打开 `AutoBusOff` 后，bxCAN 外设会在检测到总线空闲
（128 个 11 位 recessive）后**由硬件自动退出 bus-off**，无需软件干预。
本项目断线随关节运动抖动、总线时好时坏，这个特性正好匹配：
"坏的时候离线保护，线恢复接触后自动上线"。

*第 3 层：IWDG 独立看门狗 —— 最后防线*

启用 STM32 的独立看门狗（IWDG，由内部低速时钟驱动，
**和主程序、FreeRTOS 完全独立**），设定如 500ms 超时，
在某个低优先级任务里定期"喂狗"。
它专治前两层管不了的**整机挂死**（如 B1 的 HardFault：
程序跑飞进 `while(1)`，调度器都停了，软件层的保险全部失效）。
只要程序死了就没人喂狗，IWDG 超时后**强制复位整个 MCU**，
几十毫秒内系统重启恢复。效果区别：现在 = 停在原地直到手动断电重启；
加了 IWDG = 机械臂自动"昏睡半秒然后自己醒来"，
配合开机回安全位置的逻辑可大幅降低无人值守风险。
它不预防故障，但把"永久瘫痪"变成"短暂中断"。

*第 4 层：错误计数观测 —— 验证与定位*

利用 `CAN_context` 里现成的计数器（`unexpected_errors`、`ErrorCallbackCnt`、
`TxMailboxAbortCallbackCnt`，加上新增的超时计数），定期打印到 OLED 或上位机。
前三条都是"防死"，这一条是"**知道到底发生了什么、修没修好**"：

- **定位故障频率**：修好断线后计数器跑几小时纹丝不动 = 硬件真的干净了；
  仍在缓慢增长 = 总线上还有偶发干扰/接触不良没解决；
- **区分故障类型**：TERR 多 = 没人 ACK（断线/电机没上电）；
  bus-off 计数 = 总线被持续打爆；超时计数 = 锁曾经丢过；
- **验证修复效果**：故障本来就是偶发的，没有计数只能靠
  "跑了 30 分钟没死"碰运气验证，计数器给的是客观证据。

**各修复的分工总结**

| 修复 | 针对的故障 | 层次 | 单独用够吗 |
|---|---|---|---|
| 最小修复（TERR 还锁） | B2 已知泄漏点 | 补丁 | 不够：ALST / Abort / AddTxMessage 失败路径仍漏 |
| ① 有限超时 | **任何**锁泄漏（已知+未知） | 结构根治 | 治不了 HardFault 整机挂死（B1） |
| ② AutoBusOff | B3 bus-off 不自愈 | 硬件自恢复 | 治不了锁泄漏和挂死 |
| ③ IWDG | B1 HardFault、一切软件失效 | 最后防线 | 只负责"重启"，不负责"不出错" |
| ④ 错误计数 | 所有 | 观测验证 | 本身不修任何东西 |

简言之：**最小修复是"把发现的那个洞堵上"，组合拳是"让这艘船以后不管
哪里进水都沉不了"**。工程上两者都要做——先打补丁消除已知问题，
再上防御层兜住未知问题。

#### B3. CAN 控制器进入 bus-off 后不自愈

主控和电机两侧的 CAN 都配置了 `AutoBusOff = DISABLE`：
总线错误累计到一定程度，CAN 控制器会进入"bus-off"（彻底离线）状态，
**此后永远不会自己恢复**，只有重启才行。总线被垃圾帧持续攻击时，
这条会让故障从"偶发"变成"永久"。

#### B4. 电机固件：发送失败直接死循环

位置：`dummy-42motor-fw/Core/Src/can.c` 的 `CAN_Send()`

```cpp
if (HAL_CAN_AddTxMessage(...) != HAL_OK)
    Error_Handler();   // = while(1) 死循环
```

某个关节的电机板一旦踩中这个分支，这块电机板就永久死机：
不响应指令、不回复角度。表现是"个别关节彻底失联"。

#### B5. 历史问题：高频命令下的堆内存竞态（已修复，列出供参考）

项目曾经存在 50Hz 高频命令下两个线程同时使用 malloc 导致堆损坏、
`std::bad_alloc` 崩溃的问题，后来已用"高频路径零动态分配"方案修复。
如果将来改代码引入了新的堆分配，这类随机崩溃仍可能回来。

> ⚠️ **修正（重要）**：那次"零堆分配"改造**并不彻底**——它只清掉了命令链路里的 `std::string`
> 隐式 malloc，却**没管浮点 `printf/scanf` 内部（dtoa）的堆使用**（该文档 §4.4 当时已把它当作
> "次要问题"遗留下来）。正是这个残留口子，在"并发开 GETJPOS 采样线程"时被重新踩中，导致本文现象
> 在 **B1、硬件线缆都修好后仍然复现**。详见下方 **B6**（本次确认的真凶）。

#### B6. 并发浮点格式化踩坏 newlib 堆 → HardFault 整机挂死（✅ 已确认为本次真凶 + 已修复）

> **这是"B1 与硬件线缆都修好后、现象仍复现"最终定位到的根因**，与 CAN/硬件无关，是一条纯软件的
> **多线程堆竞态**，也是 B5 那次改造没覆盖到的残留口子。

**问题描述（决定性对照实验）**

同一脚本 `test_sine_trajectory.py --send_only`：

| 运行方式 | 现象 |
|---|---|
| 加 `--no_sample`（**只**发 move_j，不开 GETJPOS 采样线程） | **长时间稳定，不崩** |
| 不加 `--no_sample`（move_j 下发线程 + GETJPOS 采样线程**并发**） | 跑几轮~几十秒后必现 `Write timeout`，**且 OLED 冻死**（整机挂死） |

唯一变量就是"是否**并发**多开一个高频 GETJPOS 查询线程"。这直接排除了 CAN/硬件（线已修好，
且只发 move_j 时怎么跑都不崩），把矛头指向"两条命令路径并发时踩了共享资源"。

**原因（对照代码）**

两条**并发**执行、且都会进入 newlib 浮点转换（内部 `malloc/free`）的路径：

| 线程 | 命令 | 代码位置 | 堆行为 |
|---|---|---|---|
| `UsbServerTask`（收包+立即应答） | `#GETJPOS` | `ascii_protocol.cpp` `Respond("ok %.2f ...")` → `ascii_processor.hpp` `snprintf` | `%.2f` 浮点格式化 → `dtoa` → `_Balloc` → **malloc/free** |
| `ThreadControlLoopUpdate`（命令消费者） | `>` move_j | `main.cpp` → `ParseCommand` → `dummy_robot.cpp` `sscanf(">%f,%f,...")` | `%f` 浮点解析 → `strtod/dtoa` → `_Balloc` → **malloc/free** |

而本工程的 newlib **在多线程下不安全**（与 `high_freq_command_bad_alloc_fix.md` §2.5/§4.4 一致）：
① malloc/free **没加锁**（无 `__malloc_lock`/`__retarget_lock_acquire`，用 libgloss 空 weak 实现）；
② 所有线程**共用一个 `_reent`**（`configUSE_NEWLIB_REENTRANT` 默认 0）。

于是两个同为 Normal 优先级的线程在时间片轮转 + tick 抢占下，可能在对方 `malloc/free`/`dtoa` 执行到
一半时切进去，同时改同一张堆空闲链表和同一个 `_reent` → **堆损坏**。损坏是概率性的（"坏在 A 时刻、
炸在 B 时刻"），所以能跑几轮才死；一旦某次 `malloc` 返回野指针并被解引用 → **HardFault → `while(1)` →
所有任务定格：OLED 冻住、USB 不再收包 → 主机 1s 后 `Write timeout`**。

**为什么 `--no_sample` 就没事**：去掉 GETJPOS 后，`UsbServerTask` 只剩 move_j 的 `Push`（B5 已零堆分配）
+ 应答 `"ok queued free=%lu"`（`%lu` 是**整数**格式化，走栈上小缓冲、**不调用 dtoa、不 malloc**）。
这样全系统只剩 `ThreadControlLoopUpdate` **单线程**在用 newlib 浮点堆（`sscanf %f`）——单线程配对使用
malloc/free 是安全的。一旦开采样线程，GETJPOS 的 `snprintf("%.2f")` 把**第二个线程**也拉进同一片没加锁的
浮点堆 → 竞态成立 → 崩溃。（速率从 50Hz 翻到 ~100Hz 只是缩短了撞窗时间，不是本质。）

**解决方法（本次已实施并验证）**

对症、且符合工程既有"高频路径零堆分配"哲学：**把查询类应答的浮点格式化换成纯整数定点格式化**，
让 `UsbServerTask` 彻底退出 newlib 浮点堆。

- 在 `ascii_protocol.cpp` 增加 `FormatFixedN(value, out, outSize, decimals)`：先 `×10^decimals` 四舍五入成
  `long`，再用 `snprintf("%s%ld.%02ld")` 这类**只用整数转换**的格式拼出与 `%.Nf` **逐字节一致**的文本
  （整数 printf 不走 dtoa、不 malloc）；
- `GETJPOS`/`GETLPOS`（2 位小数）、`GET_CURRENT`（3 位）、`GET_TEMP`（1 位）应答全部改走 `FormatFixedN`；
- 输出文本不变 → 上位机 SDK 的 `_RESP_6FLOAT` 正则照常匹配，无需改上位机。

**验证结果**：烧录后并发跑 `--send_only`（带 GETJPOS 采样），**不再 Write timeout、OLED 不再冻死**。

**治本方案（可选，尚未做）**：若希望任意线程都能安全用 `printf/scanf/malloc`，应 ①
`configUSE_NEWLIB_REENTRANT=1`（每任务独立 `_reent`）+ ② 实现 `__retarget_lock_acquire/release`
（内部用 FreeRTOS 递归互斥量）给 newlib 堆加锁。本次先用"定点格式化"这条零成本路径消除了热路径竞态；
`GET_SPEED_CFG`/`GET_ACC_CFG` 等低频、非并发查询仍保留 `%f`（不在热路径，风险低）。

---

### C 类：各原因的"指纹"对照表

| 原因 | OLED 表现 | 上位机表现 | 重启后 |
|---|---|---|---|
| **B6 并发浮点 printf/scanf 堆竞态 → HardFault（本次真凶）【已修复】** | **整屏冻住** | Write timeout | 恢复，随机复发；**仅并发 GETJPOS 时发作，`--no_sample` 不复现** |
| B1 越界 HardFault（垃圾帧触发）**【已修复】** | **整屏冻住** | Write timeout | 恢复，随机复发 |
| B2 CAN 信号量泄漏（需关节 1 也掉线/总线损坏才触发） | 还在刷新，数值冻结 | 能写指令但无动作/无应答 | 恢复，随机复发 |
| B3 bus-off | 还在刷新，数值冻结 | 同 B2 | 恢复，随机复发 |
| B4 电机板死机 | 正常，个别关节不动/不更新 | 正常 | 恢复，随机复发 |
| A3 电源欠压复位 | 熄灭后重新亮起 | Write timeout，设备可能重新枚举 | 自动"恢复"（自己重启了） |
| A5 上位机 USB 问题 | **正常刷新** | Write timeout | 换线/换口即好 |
| **Fibre 协议树撑爆 commTask 栈**（见 `oled_blank_fibre_tree_stack_overflow.md`） | **全程没亮过** | 无响应（设备可能枚举但无输出） | **稳定复现**（与代码版本绑定，非随机） |

本次故障（OLED **冻住** + Write timeout + 随机复发）**最终确认的元凶是 B6**（并发浮点格式化踩坏
newlib 堆 → HardFault 整机挂死），**与 CAN/硬件无关**：决定性证据是"`--no_sample` 只发 move_j 稳定不崩、
一旦并发开 GETJPOS 采样必崩"，且此时 **B1 与 J1~J2 线缆都已修复**仍复现。B6 已用"定点整数格式化"修好并验证。

> 📌 **对本节历史推断的修正**：下文此前曾判断"最吻合 A1/A2 硬件点火 + B1 越界 HardFault"。该推断在
> "B1 未修、线缆断"的早期阶段有其合理性，但**不是本次（B1+线缆已修后）复发的原因**。A 类硬件与
> B1~B4 CAN 缺陷仍值得修（纵深防御），但它们解释不了"`--no_sample` 就完全不崩"这一现象，只有 B6 能。
> （若现象是**开机全程黑屏**而非"运行中冻住"，仍属另一类问题，直接查 Fibre 栈溢出文档。）

---

## 4. 排查流程（按顺序做，成本从低到高）

0. **⭐ 最省事、最能一刀切分软硬件的对照实验**：用 `--no_sample` 只发 move_j 跑一遍，再不带
   `--no_sample`（并发开 GETJPOS 采样）跑一遍：
   - **只发 move_j 稳、一并发 GETJPOS 就崩** → 基本坐实 **B6**（并发浮点堆竞态），与 CAN/硬件无关，
     直接查固件查询命令的浮点格式化路径；
   - 两种都崩 / 与是否并发无关 → 再往下走硬件与 CAN 排查（A 类、B1~B4）；
1. **故障复现时先看 OLED**（本次：冻住 → 整机挂死，排除 A5；但整机挂死既可能是 B1 也可能是 B6）；
2. **复现时另开终端跑 `dmesg -w`**：
   - 有 `usb disconnect` → 发生了复位，重点查电源（A3）；
   - 没有 → MCU 挂死未复位，重点查 **B6 / B1**（用第 0 步的并发对照结果区分）；
3. **修好 / 更换 J1~J2 线缆**，检查全链路接插件，复测同一脚本：
   - 不再复现 → 硬件点火 + 固件放大，因果链闭环；
   - 仍然复现 → 说明还有其他干扰源或纯软件触发（**本次即属此情况 → 最终定位到 B6**），继续第 4 步；
4. **用万用表测 CAN_H/CAN_L 电阻**（断电测，应约 60Ω），确认终端电阻；
5. **检查供电**：电源功率余量、大动作时电压是否跌落（示波器/带记录的万用表）。

---

## 5. 修复建议清单

### 固件侧（强烈建议，修完才能"抗造"）

0. ✅ **已完成（本次真凶 B6）：GETJPOS/GETLPOS/GET_CURRENT/GET_TEMP 应答改为定点整数格式化**
   ——`FormatFixedN` 只用 `%ld`，彻底不走 newlib 的 `%f`→dtoa→malloc 路径，消除"USB 任务浮点 printf"
   与"move_j 消费线程浮点 sscanf"的并发堆竞态（见 B6）；已烧录验证不再 Write timeout / OLED 冻死；
1. ✅ **已完成：`OnCanMessage` 增加 ID 边界检查**——已用 `validMotorId=(id>=1&&id<=6)`
   守卫堵死 B1 的 HardFault 路径（见 `can_protocol.cpp` L26-45）；
2. **加独立看门狗 IWDG**：无论将来因为什么挂死，都能自动复位，
   不再需要人工断电重启；
3. **修复 CAN 发送信号量泄漏（B2，仍未修）**：`HAL_CAN_ErrorCallback` 的 TX 错误分支里
   归还 `sem_can1_tx`（或把 `CanSendMessage` 改成带超时获取）；
4. **两侧 CAN 打开 `AutoBusOff = ENABLE`（当前仍为 DISABLE）**，bus-off 后自动恢复；
5. **电机固件 `CAN_Send` 失败改为丢帧**，不要调用 `Error_Handler()` 死循环；
6. **开启 `configCHECK_FOR_STACK_OVERFLOW = 2` + `vApplicationStackOverflowHook`**：
   把栈溢出从"静默崩溃"变成"报出任务名"，便于定位（详见 `oled_blank_fibre_tree_stack_overflow.md`）。

### 硬件侧（根因）

6. 更换/修复 J1~J2 线缆，全链路检查接插件与压接；
7. 确认 CAN 两端 120Ω 终端电阻；
8. 确认供电余量与滤波，必要时在电机供电入口加大电容。

---

## 6. 一句话总结

> **✅ 本次已确认结论（优先）：现象（并发跑 GETJPOS + move_j 时 Write timeout + OLED 冻死）的真凶是
> B6——GETJPOS 应答的 `snprintf("%.2f")` 与 move_j 消费线程的 `sscanf("%f")` 并发进入 newlib 的
> dtoa/malloc，而本工程 newlib 的堆未加锁、`_reent` 全局共享 → 堆损坏 → HardFault 整机定格。
> 判据是"`--no_sample` 只发 move_j 稳定不崩、一并发 GETJPOS 必崩"，且在 B1、J1~J2 线缆都已修好后仍复现。
> 修法：把 GETJPOS/GETLPOS/GET_CURRENT/GET_TEMP 应答改成纯整数定点格式化（`FormatFixedN`），让 USB 任务
> 退出浮点堆，已烧录验证不再复现。**
>
> **历史推断（已被上面修正，保留备查）：Write timeout 表示设备整机停止接收数据；结合 OLED
> "定格在最后一帧"，可判定是 MCU 运行中整机挂死而非 USB 单独故障。早期曾推断链条为
> 断线/接触不良产生 CAN 垃圾帧 → CAN 帧 ID 越界访问 → HardFault（B1）。B1 与硬件线缆修复后现象仍在，
> 说明那并非本次元凶。A 类硬件与 B1~B4 CAN 缺陷仍建议按"纵深防御"修（看门狗 + 信号量泄漏 +
> AutoBusOff + 栈溢出检测），但它们不是本次现象的原因。**
>
> 另注：若现象是**开机全程黑屏**（OLED 从未点亮）而非"运行中冻住"，则不属本文范畴，
> 那是启动阶段 Fibre 协议树撑爆 commTask 栈，见 `oled_blank_fibre_tree_stack_overflow.md`。
