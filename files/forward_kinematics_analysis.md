# 六轴机械臂正运动学（FK）分析

> 关联源码：`Robot/algorithms/kinematic/6dof_kinematic.cpp`、`6dof_kinematic.h`、
> `Robot/instances/dummy_robot.cpp`
> 前置文档：`robot_home_calibration_notes.md`、`project_threads_and_command_pipeline.md`

---

## 1. 如何进入正运动学计算（调用链）

FK 由主控控制循环周期性调用，输入是六个关节角（来自电机 CAN 回报），输出是末端位姿：

```text
ThreadControlLoopUpdate (UserApp/main.cpp, 10ms 级)
  └── dummy.UpdateJointAngles()        // CAN 广播查询 6 个关节电机角度
  └── dummy.UpdateJointAnglesCallback()// 0x23 回复经中断回调更新 motorJ[i]->angle
        └── currentJoints.a[i] = motorJ[i]->angle + initPose.a[i]   (度)
  └── dummy.UpdateJointPose6D()        [dummy_robot.cpp L287-293]
        ├── dof6Solver->SolveFK(currentJoints, currentPose6D)   ← FK 入口
        └── currentPose6D.X/Y/Z *= 1000;   // m → mm（供显示/上报）
```

关键约定：

| 项 | 单位/形式 |
|---|---|
| 输入 `Joint6D_t` | 关节角，**度**（SolveFK 内部第一步除以 `RAD_TO_DEG` 转弧度） |
| 输出 `Pose6D_t` | X/Y/Z **米**，A/B/C 欧拉角**度**，R[9] 为 3×3 旋转矩阵 |
| `hasR` 标志 | FK 算出的 Pose 自带旋转矩阵；IK 输入若手填 A/B/C 则需置 false 让 IK 内部重建矩阵 |

运动学对象在 `DummyRobot` 构造函数中创建，传入 6 个结构尺寸（米）：

```cpp
dof6Solver = new DOF6Kinematic(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f);
//                             L_BASE  D_BASE  L_ARM   L_FOREARM D_ELBOW L_WRIST
```

---

## 2. DH 参数的含义

### 2.1 标准 DH 四参数

每个关节 i 用四个参数描述相邻两个连杆坐标系的关系（Standard DH）：

| 参数 | 名称 | 含义 |
|---|---|---|
| θ | 关节角 | 绕 z_{i-1} 轴的旋转，**变量**（旋转关节的输入） |
| d | 连杆偏距 | 沿 z_{i-1} 轴的平移，常量（旋转关节时） |
| a | 连杆长度 | 沿 x_i 轴的平移，常量 |
| α | 连杆扭角 | 绕 x_i 轴的旋转，常量 |

### 2.2 本项目的 DH_matrix

头文件注释 `float DH_matrix[6][4]; // home,d,a,alpha`，列含义为
**[θ 零位偏置, d, a, α]**：

| 关节 | θ 偏置 (home) | d | a | α | 物理角色 |
|---|---|---|---|---|---|
| J1 | 0 | L_BASE (0.109) | D_BASE (0.035) | −90° | 底座回转（绕竖直轴） |
| J2 | −90° | 0 | L_ARM (0.146) | 0° | 肩俯仰（大臂） |
| J3 | +90° | D_ELBOW (0.052) | 0 | +90° | 肘俯仰（小臂） |
| J4 | 0 | L_FOREARM (0.115) | 0 | −90° | 腕部旋转 |
| J5 | 0 | 0 | 0 | +90° | 腕俯仰 |
| J6 | 0 | L_WRIST (0.072) | 0 | 0° | 末端法兰回转 |

要点：

- **θ 偏置（列 0）**：把 DH 约定的 θ=0 方向对齐到机械结构的零位。FK 中
  `q[i] = q_in[i] + DH_matrix[i][0]`——用户给的关节角先加上这个偏置才是真正的 DH θ。
  例如 J2 为 −90°：机械零位（大臂水平）对应 DH 角的 −90° 位置。
- **a、d**：由结构尺寸（构造参数）直接填入，是相邻坐标系原点的几何距离。
- **α = ±90°**：表示相邻关节轴正交（典型的肩-肘-腕结构）。

---

## 3. 相邻关节间的位姿转换关系

### 3.1 标准 DH 齐次变换（理论形式）

坐标系 {i-1} → {i} 的变换由四个基本变换复合：

\[
{}^{i-1}T_i = Rot_z(\theta_i)\cdot Trans_z(d_i)\cdot Trans_x(a_i)\cdot Rot_x(\alpha_i)
\]

展开为 4×4 齐次矩阵：

\[
{}^{i-1}T_i =
\begin{bmatrix}
c\theta & -s\theta c\alpha & s\theta s\alpha & a\,c\theta \\
s\theta & c\theta c\alpha & -c\theta s\alpha & a\,s\theta \\
0 & s\alpha & c\alpha & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

末端位姿由六个变换连乘得到：\( {}^{0}T_6 = {}^{0}T_1 \cdot {}^{1}T_2 \cdots {}^{5}T_6 \)。

### 3.2 本实现的拆分技巧（旋转与平移分开算）

代码**没有构造 4×4 齐次矩阵**，而是拆成两条线，省掉大量无效乘加（中间帧的平移多为 0）：

**① 旋转线**：每个关节只取 DH 矩阵的 3×3 旋转部分（即 \(Rot_z(\theta)\cdot Rot_x(\alpha)\)），
SolveFK L136-153 逐关节填充：

\[
R_i = \begin{bmatrix}
\cos q & -\cos\alpha\sin q & \sin\alpha\sin q \\
\sin q & \cos\alpha\cos q & -\sin\alpha\cos q \\
0 & \sin\alpha & \cos\alpha
\end{bmatrix}, \quad q = q_{in} + \theta_{home}
\]

然后链式连乘（L155-159）：

```text
R02 = R[0]·R[1]
R03 = R02·R[2]
R04 = R03·R[3]
R05 = R04·R[4]
R06 = R05·R[5]      ← 末端姿态（base 系下的旋转矩阵）
```

**② 平移线**：相邻坐标系原点的偏移被归并成 4 个常向量（构造函数 L98-105，
在各自所属坐标系中表示）：

| 向量 | 值 | 含义 |
|---|---|---|
| `L1_base` | {D_BASE, −L_BASE, 0} | 底座 → 肩 |
| `L2_arm` | {L_ARM, 0, 0} | 肩 → 肘（大臂） |
| `L3_elbow` | {−D_ELBOW, 0, L_FOREARM} | 肘 → 腕（含肘偏距与小臂长，跨 J3/J4 归并） |
| `L6_wrist` | {0, 0, L_WRIST} | 末端法兰延伸 |

计算时（L161-167）用对应累积旋转矩阵把每个向量旋到 base 坐标系再求和：

\[
P_{06} = R_{01}\,L_1 + R_{02}\,L_2 + R_{03}\,L_3 + R_{06}\,L_6
\]

这等价于齐次矩阵连乘的位置列，但因为 J4/J5 等中间变换是纯旋转，
相邻平移可以提前归并，**6 次 4×4 乘法降为 5 次 3×3 乘法 + 4 次矩阵-向量乘**。

---

## 4. SolveFK 逐步骤解读（L114-180）

```text
步骤 1  角度预处理      q_in[i] = 输入度 ÷ RAD_TO_DEG      (度 → 弧度)
步骤 2  逐关节建旋转    q[i] = q_in[i] + DH_matrix[i][0]   (加 home 偏置)
                       按 §3.2 公式填 R[i]（用 arm_cos_f32/arm_sin_f32）
步骤 3  旋转链乘        R06 = R01·R12·…·R56                (末端姿态)
步骤 4  位置合成        P06 = Σ R0i·Li                     (末端位置, 米)
步骤 5  姿态 → 欧拉角   RotMatToEulerAngle(R06, &P06[3])
步骤 6  输出打包        X/Y/Z = P06[0..2]
                       A/B/C = P06[3..5] × RAD_TO_DEG     (弧度 → 度)
                       memcpy(R06 → Pose.R)               (保留矩阵, hasR 语义)
```

### 欧拉角约定

`RotMatToEulerAngle` 按 **R = Rz(A)·Ry(B)·Rx(C)**（ZYX 顺序）分解旋转矩阵：

```cpp
B = atan2(−R[6], √(R[0]² + R[3]²));   // 俯仰
A = atan2(R[3]/cosB, R[0]/cosB);      // 偏航
C = atan2(R[7]/cosB, R[8]/cosB);      // 横滚
```

并处理了 `|R[6]| ≈ 1` 的**万向锁**奇异（B = ±90° 时退化为两轴，令 A=0 用 atan2 解出 C）。
注意函数按 `euler[0..2] = {C, B, A}` 的顺序返回，随后存入
`Pose.A = euler[0], Pose.B = euler[1], Pose.C = euler[2]`——
即 **Pose.A/B/C 字段实际存放的是 {绕x角, 绕y角, 绕z角}**。FK 与 IK 使用同一套
索引约定（`EulerAngleToRotMat` 为其逆变换），往返自洽，但与其他系统对接时需留意。

---

## 5. 实现特点小结

1. **CMSIS-DSP 加速**：`cosf/sinf` 重定向到 `arm_cos_f32/arm_sin_f32`
   （查表 + 插值），适配 STM32F405 无硬件双精度、追求控制周期实时性的场景。
2. **旋转/平移分离**：避免 4×4 齐次矩阵的冗余运算，中间纯旋转帧的平移被归并为常向量。
3. **home 偏置内置**：DH θ 与机械零位的对齐在算法层完成，上层直接用机械关节角。
4. **位置单位为米**：FK 输出米制，`UpdateJointPose6D` 再乘 1000 转 mm 供人机界面使用；
   IK 输入则先 ÷1000 转回米，两侧对称。
5. **FK/IK 共用同一套 DH_matrix 与连杆向量**，保证正反解一致；IK 的 8 组解、
   奇异处理（腕部共线时借用上一时刻关节角）见 `SolveIK`（L182-499）。

---

## 6. 位置计算分解详解（L161-167）

对应代码：

```cpp
MatMultiply(R[0], L1_base, L0_bs, 3, 3, 1);    // L0_bs = R01 · L1_base
MatMultiply(R02, L2_arm,  L0_se, 3, 3, 1);     // L0_se = R02 · L2_arm
MatMultiply(R03, L3_elbow, L0_ew, 3, 3, 1);    // L0_ew = R03 · L3_elbow
MatMultiply(R06, L6_wrist, L0_wt, 3, 3, 1);    // L0_wt = R06 · L6_wrist

for (int i = 0; i < 3; i++)
    P06[i] = L0_bs[i] + L0_se[i] + L0_ew[i] + L0_wt[i];   // ← L167
```

### 6.1 出发点：齐次矩阵连乘的位置列可以"逐段拆开"

设 \( {}^{i-1}T_i = \begin{bmatrix} R_i & p_i \\ 0 & 1 \end{bmatrix} \)，其中 \(p_i\) 是
坐标系 {i} 原点相对 {i−1} 原点的偏移（在 {i−1} 系中表示）。
把 \( {}^{0}T_6 = {}^{0}T_1\cdot{}^{1}T_2 \cdots {}^{5}T_6 \) 的位置列展开（逐层提取，telescoping）：

\[
p_{06} = p_1 + R_{01}\,p_2 + R_{02}\,p_3 + R_{03}\,p_4 + R_{04}\,p_5 + R_{05}\,p_6
\]

**几何意义**：从 base 原点走到末端原点，等价于把相邻坐标系之间一段段
"原点到原点"的位移向量**首尾相接**；每一段向量在自己的坐标系里是固定的，
求和前先要用该处累积的姿态矩阵把它旋转到 base 系方向。

### 6.2 关键化简：段向量用"后一坐标系"表示时是与 θ 无关的常量

标准 DH 中，第 i 段的偏移在前系表示时**含有 θ**：

\[
p_i^{(i-1)} = \begin{bmatrix} a_i\cos q_i \\ a_i\sin q_i \\ d_i \end{bmatrix}
\]

它随关节角变化，没法预计算。但换到**后一坐标系 {i}** 中表示就不同了：

\[
p_i^{(i)} = R_i^{\mathsf T}\,p_i^{(i-1)}
          = Rot_x(-\alpha_i)\,Rot_z(-q_i)\begin{bmatrix} a_i\cos q_i \\ a_i\sin q_i \\ d_i \end{bmatrix}
          = Rot_x(-\alpha_i)\begin{bmatrix} a_i \\ 0 \\ d_i \end{bmatrix}
          = \begin{bmatrix} a_i \\ d_i\sin\alpha_i \\ d_i\cos\alpha_i \end{bmatrix}
\]

\(Rot_z(-q_i)\) 先把 \(a\cos q / a\sin q\) 合回 \(x\) 轴（**θ 被消掉了**），
剩下的 \(Rot_x(-\alpha)\) 只含常量——**因此每段偏移在本坐标系中都是一个固定常向量**，
可以写死在构造函数里。这正是 §3.2 中四个常向量的来源，逐个验证：

| 段 | a | d | α | 公式 \([a,\, d\sin\alpha,\, d\cos\alpha]\) | 代码向量 |
|---|---|---|---|---|---|
| J1 底座→肩 | D_BASE | L_BASE | −90° | {D_BASE, −L_BASE, 0} | `L1_base` ✓ |
| J2 肩→肘 | L_ARM | 0 | 0° | {L_ARM, 0, 0} | `L2_arm` ✓ |
| J6 腕→工具 | 0 | L_WRIST | 0° | {0, 0, L_WRIST} | `L6_wrist` ✓ |

### 6.3 为什么只有 4 项：归并与零段

6 个关节的 DH 行里真正含平移的是 J1(d,a)、J2(a)、J3(d)、J4(d)、J6(d)，
代码把它们归并成 4 项：

1. **J5 是零段**：腕俯仰的 a=d=0（球腕设计，腕部三轴交于一点），
   只贡献旋转不贡献位移，所以不出现在求和里。
2. **J3+J4 归并为一项 `L3_elbow`**：两段都换算到 frame 3 中再相加：
   - J3 肘偏距：沿 frame 3 的 −x 轴，得 `{−D_ELBOW, 0, 0}`（方向取自作者对
     该帧坐标轴的布置，即肘轴相对腕轴的侧向偏置）；
   - J4 小臂长：本系表示为 `{0, −L_FOREARM, 0}`，再旋入 frame 3：
     \(R_{34}\{0,-L,0\}\)。由于该向量经 \(Rot_x(-90^\circ)\) 后落在
     \(z_3\) 轴上，而 \(Rot_z(q_4)\) 绕 \(z_3\) 旋转不改变 \(z_3\) 方向的分量，
     结果恒为 `{0, 0, L_FOREARM}`——**沿关节轴的平移不受该关节角影响**，
     仍可与 J3 段合并成一个常向量。

   于是 `L3_elbow = {−D_ELBOW, 0, L_FOREARM}`。

### 6.4 代码逐行对应

| 代码行 | 公式 | 几何含义 |
|---|---|---|
| `L0_bs = R[0]·L1_base` | \(R_{01}L_1\) | 肩偏移段旋到 base 系 |
| `L0_se = R02·L2_arm` | \(R_{02}L_2\) | 大臂段旋到 base 系 |
| `L0_ew = R03·L3_elbow` | \(R_{03}L_3\) | 肘偏距+小臂段旋到 base 系 |
| `L0_wt = R06·L6_wrist` | \(R_{06}L_6\) | 工具延伸段旋到 base 系 |
| L167 求和 | \(P_{06} = \sum\) | 四段向量首尾相接得到末端位置 |

变量名即几何段：`L0_bs`(base→shoulder)、`L0_se`(shoulder→elbow)、
`L0_ew`(elbow→wrist)、`L0_wt`(wrist→tool)，前缀 `L0_` 表示"已旋到 0 系"。

### 6.5 示意图

```mermaid
flowchart LR
    O0["base 原点"] -- "R01·L1_base<br/>{D_BASE, −L_BASE, 0}" --> O1["肩"]
    O1 -- "R02·L2_arm<br/>{L_ARM, 0, 0}" --> O2["肘"]
    O2 -- "R03·L3_elbow<br/>{−D_ELBOW, 0, L_FOREARM}" --> O3["腕心"]
    O3 -. "J4/J5 纯旋转<br/>无位移" .-> O6["末端系"]
    O6 -- "R06·L6_wrist<br/>{0, 0, L_WRIST}" --> OT["工具端 P06"]
```

### 6.6 一句话总结

> L167 不是经验拼凑，而是齐次变换连乘的位置列展开式：
> **末端位置 = 各相邻坐标系原点偏移段，分别用其累积姿态旋到 base 系后求和**；
> 而这些段向量在各自坐标系里恰好都是与关节角无关的常量（θ 被旋量公式消去），
> 因此能预计算、归并，把每拍 FK 的平移计算压缩为 4 次矩阵-向量乘 + 3 次向量加法。
