# 不同指令下发频率下的机械臂响应分析（电机端速度规划视角）

> 关联源码：`dummy-42motor-fw/Ctrl/Motor/motion_planner.cpp`、`motor.cpp`、`UserApp/protocols/interface_can.cpp`；`dummy-ref-core-fw/Robot/instances/dummy_robot.cpp/.h`
> 前置文档：`dummy-42motor-fw/files/motion_planner_analysis.md`（规划器原理）、`high_freq_command_response_capability_analysis.md`（管线/吞吐）
> 测试对齐：`files/host_computer_robot_control_sdk/test_sine_trajectory.py`
> **讨论范围**：本文**只从电机端 `positionTracker` 速度规划的角度**分析下发频率对实际响应的影响，不涉及 USB 队列 / CAN 转发 / 管线延迟（那部分见前置文档）。

---

## 1. 结论速览

| 下发频率 | 目标点之间的状态 | `positionTracker` 的"到点即停"假设 | 实际响应 |
|---|---|---|---|
| **低频（如 1Hz）** | 目标点静止保持很久 | **成立** | 每点满速冲刺 + 精确停驻，采样点处零跟随误差；但轨迹是"走-停-走"的阶梯折线，不平滑 |
| **高频（≥50Hz）** | 目标点连续移动 | **失效** | 持续滞后一个"刹车距离" `e≈v²/2a`，表现为幅值衰减 + 相位滞后 + 过零点畸变 |
| **50 / 200 / 400Hz 之间** | 都是连续移动 | 都失效 | 滞后量几乎不变（`e` 由 `v²/a` 决定，与下发率无关）；下发率越高只是阶梯越细，对轨迹保真无实质增益 |

**一句话**：一旦下发频率高到足以描述这条正弦（8s 周期只需几 Hz，50Hz 已 400× 过采样），**响应快慢就完全由电机端规划器决定，而不是下发频率**。而当前 `positionTracker` 是"到点即停"型的，追踪运动目标时会稳定地落后一个刹车距离——这是**滞后（慢半拍）**，不是更快。

---

## 2. 前提：INTERRUPTABLE 走的是"到点即停"的 positionTracker

`COMMAND_TARGET_POINT_INTERRUPTABLE` 模式下，中控每拍把最新目标点经 CAN `0x07` 下发：

```text
中控 MoveJoints() --CAN 0x07--> 电机 interface_can.cpp case 0x07 --> MODE_COMMAND_POSITION
                                                                        --> positionTracker.CalcSoftGoal(goalPosition)
```

- `interface_can.cpp` `case 0x07`（L87）→ 切到 `MODE_COMMAND_POSITION`，第二个 float 是 `ratedVelocity`（**限速**，不是目标速度）。
- `motor.cpp` `CloseLoopControlTick`（L29-336）在该模式下只调用 `positionTracker.CalcSoftGoal(goalPosition)`（L216 附近）——**只喂位置，没有 `goalVelocity` 前馈**。

对照 `motion_planner_analysis.md` 的权威结论：

| 规划器 | 输入 | 行为 | 用途 |
|---|---|---|---|
| **positionTracker** | 仅 `goalPosition` | 到点自动减速锁定（**默认目标速度=0**） | 单轴点到点定位 |
| trajectoryTracker | `goalPosition` + `goalVelocity` | 速度前馈，跟随误差≈0 | 多轴连续轨迹 |

关键点：**流式下发不会重置规划器**。`NewTask()` 只在模式切换 / 使能-失能转换（`softNewCurve` 置位，`motor.cpp` L170-209）时调用一次；`SetPositionSetPoint` 更新 `goalPosition` 并不触发 `NewTask`。所以高频流式下发时，`positionTracker` 是在**持续追踪一个不断移动的目标**，而不是每次都从静止重新规划。

---

## 3. 核心机理：目标速度按 0 规划 → 稳定的"刹车距离"跟随误差

`positionTracker` 每拍用刹车距离判据决定是否减速（`motion_planner.cpp` `CalcSoftGoal` L190-330）：

```cpp
// 剩余距离 <= 当前速度的刹车距离 → 开始减速，保证到点速度=0
need_down_location = trackVelocity * trackVelocity * quickVelocityDownAcc;  // = v² / (2a)
```

它的设计前提是"**目标点最终会停在那里**"，于是提前一个刹车距离 `v²/2a` 开始减速，恰好到点速度归零。

但当目标点**本身在以速度 `v` 持续移动**（正弦过零点附近）时，这个前提不成立。规划器为了"到点即停"而始终保留一个刹车距离的余量，于是软目标稳定地落后硬目标：

```text
稳态跟随误差  e ≈ v² / (2a)
```

推导：追踪匀速移动目标时，自洽稳态是"软速度 `v_s = 目标速度 v`，且间距 `d = v²/2a`（正好等于刹车距离，处于'滑行'分支）"。此时软目标以相同速度前进、间距恒定 —— 这就是稳定滞后量。

性质：

- **`e ∝ v²`**：目标跑得越快，滞后越大（平方关系，很敏感）。
- **`e ∝ 1/a`**：电机加速度越大，滞后越小。
- **`e` 与下发频率无关**（只要采样够密）：50Hz→200Hz→400Hz **不会**减小这个滞后。
- 正弦上 `e` 随速度周期变化：**过零点速度最大 → 滞后最大；波峰速度为 0 → 滞后归零**。因此表现为**幅值衰减 + 波形畸变 + 视在相位滞后**，而不是整体平移。

---

## 4. 量化示例（对齐 `test_sine_trajectory.py` 默认参数）

取脚本默认：幅值 `A=45°`、`Ts=2.0s`→周期 `T=8s`、仅关节1 正弦、`speed=100`、INTERRUPTABLE（LOW 加速度）、关节1 减速比 `R=50`。

**① 峰值速度**

```text
ω = 2π/T = 0.785 rad/s
关节输出峰值速度 v_joint = A·ω = 45 × 0.785 = 35.3 °/s
电机轴峰值速度   v_motor = 35.3/360 × 50 = 4.90 r/s = 4.90 × 51200 = 2.51e5 counts/s
```

**② 限速是否卡脖子？——不卡**

```text
限速 = speed × JOINT_SPEED_UNIT_TO_RPS = 100 × 0.2 = 20 r/s = 1.024e6 counts/s
v_motor(4.90 r/s) << 20 r/s  →  速度上限远未触顶，真正的约束是加速度
```

**③ 加速度（LOW）**

```text
SetJointAcceleration(LOW=15) → joint1: 15/100 × base(150) = 22.5 r/s²
                             = 22.5 × 51200 = 1.152e6 counts/s²
```

**④ 跟随误差**

```text
e = v² / (2a) = (2.51e5)² / (2 × 1.152e6) ≈ 2.73e4 counts
  = 2.73e4 / 51200 = 0.534 圈(电机轴) = 0.534/50 × 360 ≈ 3.8° (关节)
```

**⑤ 直观换算**：过零点处滞后 `3.8°` 位置 ≈ 时间滞后 `Δt = e/v = 3.8/35.3 ≈ 109ms` ≈ 相位滞后 `109ms/8000ms × 360° ≈ 4.9°`。即机械臂在最快的位置**慢约 0.11 秒 / 落后约 3.8°**，占 45° 幅值约 **8.5%**。

**不同幅值 / 加速度下的跟随误差**（T=8s、joint1、R=50；`e∝A²`、`e∝1/a`）：

| 幅值 A | LOW 加速度 (22.5 r/s²) | HIGH 加速度 (150 r/s²) |
|---|---|---|
| 30° | ~1.7° | ~0.26° |
| **45°（默认）** | **~3.8°** | ~0.58° |
| 90° | ~15.4° | ~2.3° |

> 注：`e≈v²/2a` 是准稳态估算（正弦周期 8s ≫ 规划器 ~0.2s 的调整时间，近似成立）；实际动态滞后还叠加 DCE 闭环响应，但**平方律、过零点最大、波峰归零**这些定性特征不变。

---

## 5. 不同下发频率的实际区别

### 5.1 低频（如 1Hz）：目标点之间是静止的 → 假设成立

- 每 1s 才下发一个新点，点与点之间目标**静止保持**。
- `positionTracker` 的"到点即停"前提**正确**：满速冲刺到该点 → 提前刹车 → 精确停驻，**采样点处零跟随误差**。
- 代价：轨迹是**阶梯折线**（connect-the-dots），不是平滑正弦；点间距大、间隔长时有明显的"走-停-走"顿挫。
- 采样不足：1Hz 对 8s 正弦只有 8 个点/周期，**丢失正弦形状**（这是采样问题，不是规划问题）。

### 5.2 高频（≥50Hz）：目标连续移动 → 假设失效

- 目标点**持续移动**，"到点即停"前提**错误**。
- 规划器始终保留刹车距离余量 → **持续滞后 `e≈v²/2a`**（默认参数下 ~3.8°）。
- 表现为**幅值衰减 + 相位滞后 + 过零点畸变**；速度越大的地方滞后越明显。

### 5.3 50 / 200 / 400Hz 之间：滞后量几乎不变

- 8s 周期正弦只需几 Hz 就能描述，**50Hz 已是 400× 过采样**，阶梯细到不可感知。
- 从 50→200→400Hz，**跟随误差 `e` 不变**（`e` 由 `v²/a` 决定，与下发率无关）。
- 且中控 `TIM7@200Hz` 是采样转发上限，400Hz 对电机侧**零增益**（多出来的点被覆盖丢弃）。

---

## 6. "更慢还是更快"的直接回答

- **高频连续追踪 = 更"慢"（滞后 / 慢半拍），不是更快。**
  - 稳态下软速度其实**追平了**目标速度（`v_s = v`），但位置**永远落后一个刹车距离 `e`**。
  - 宏观上就是幅值被削、相位拖后、过零点跟不上。
- **低频 = 到点更快（满速冲刺 + 精确停驻）。**
  - 因为目标静止，规划器可以放开加速再精确刹停，采样点处零误差；但走的是折线，不是平滑正弦。
- **提高下发频率治不了这个滞后。**
  - 滞后由**轨迹速度 `v` 和电机加速度 `a`** 决定（`e=v²/2a`），不是由下发率决定。想减小滞后，得从 `a` 或规划器类型入手（见下节）。

---

## 7. 如何减小滞后

**① 治本：让电机走 `trajectoryTracker`（`MODE_COMMAND_Trajectory`）**

- 上位机周期性下发 `(位置, 速度)` 设定点对，规划器用 `goalVelocity` 做**速度前馈**，跟随误差 ≈ 0。
- 但当前 INTERRUPTABLE 链路走的是 `0x07 → positionTracker`，**没有速度前馈**。要用它需要同时改：上位机下发方式（带速度）+ 电机模式切到 `Trajectory`。

**② 治标：提高加速度**

- `e ∝ 1/a`。LOW(15%) → HIGH(100%) 可把默认参数下的滞后从 **~3.8° 降到 ~0.58°**（joint1, A=45°）。
- 但注意架构短板：**能流式下发的只有 INTERRUPTABLE（用 LOW 加速度）**；`CONTINUES_TRAJECTORY`（用 HIGH）会 `while(IsMoving()) osDelay(5)` 阻塞到到位，**不能高频流式下发**。
- 即"想流式就得忍受低加速度的大滞后"，这是当前模式设计的取舍点。

**③ 无效项：提高 `speed`（限速）**

- 默认参数下限速 20 r/s 远未触顶（峰值仅 4.9 r/s），**约束是加速度不是速度**，调大 `speed` 对滞后无帮助。

---

## 8. 关键代码位置索引

| 主题 | 文件 : 行 | 说明 |
|---|---|---|
| CAN 0x07 → 位置模式 | `dummy-42motor-fw/UserApp/protocols/interface_can.cpp` : L87 | 切 `MODE_COMMAND_POSITION`，第二 float 是限速 |
| 闭环主拍调用 positionTracker | `dummy-42motor-fw/Ctrl/Motor/motor.cpp` : L216 附近 | `positionTracker.CalcSoftGoal(goalPosition)`，仅位置 |
| NewTask 只在模式切换时触发 | `dummy-42motor-fw/Ctrl/Motor/motor.cpp` : L170-209 | `softNewCurve` 置位；设定点更新不重置规划器 |
| 刹车距离判据 v²/2a | `dummy-42motor-fw/Ctrl/Motor/motion_planner.cpp` : L190-330 | `CalcSoftGoal`，`need_down_location = v²·quickVelocityDownAcc` |
| 减速度参数换算 | `dummy-42motor-fw/Ctrl/Motor/motion_planner.cpp` : L173-178 | `SetVelocityAcc`：`quickVelocityDownAcc = 0.5/a` |
| 规划器选型对照 | `dummy-42motor-fw/files/motion_planner_analysis.md` : §1.4 / §2 | positionTracker vs trajectoryTracker |
| 加速度设置 | `dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` : L191-198 | `SetJointAcceleration`：`_acc/100 × BASE` |
| 速度/加速度常量 | `dummy-ref-core-fw/Robot/instances/dummy_robot.h` | `JOINT_SPEED_UNIT_TO_RPS=0.2`、`BASES={150,...}`、`LOW=15`、`HIGH=100` |
| 正弦测试默认参数 | `files/host_computer_robot_control_sdk/test_sine_trajectory.py` : L600-616 | `Ts=2.0`(T=8s)、`freq=50`、`amplitude=45`、`speed=100` |

---

## 9. 相关文档

- `dummy-42motor-fw/files/motion_planner_analysis.md` —— 规划器原理、刹车距离、模式选型
- `dummy-42motor-fw/files/close_loop_control_tick.md` —— 20kHz 闭环主流水线
- `high_freq_command_response_capability_analysis.md` —— 管线吞吐 / 端到端延迟（本文不重复）
- `project_threads_and_command_pipeline.md` —— 线程与命令管线全景
