# DummyRobot 关节位置、编码器与 Home Offset 标定说明

本文整理了关于 `currentJoints`、MT6816 编码器、`encoderHomeOffset`、`_pos`、`REST_POSE` / `initPose` 以及 `CalibrateHomeOffset()` 标定流程的分析结论。

相关源码主要位于：

- 主控固件：`dummy-ref-core-fw`
- 42 电机固件：`dummy-42motor-fw`
- 35 电机固件：`dummy-35motor-fw`

## 1. 核心结论

1. MT6816 是单圈绝对磁编码器，只能直接给出电机轴当前一圈内的绝对角度。
2. MT6816 不具备断电保存多圈绝对位置的功能。
3. 电机固件运行中会用软件累加的方式维护多圈位置 `realPosition`，但这个多圈值断电后会丢失。
4. 电机上电后会根据当前单圈编码器角度和 EEPROM 中保存的 `encoderHomeOffset` 重新建立位置。
5. 主控端的 `currentJoints` 初值是 `REST_POSE`，但一旦收到电机端 `0x23` 位置回复，就会被电机反馈值刷新。
6. `CalibrateHomeOffset()` 的最终目的，是让机械臂处于 `REST_POSE` 时，电机端上报 `_pos ≈ 0`，从而主控读到 `currentJoints ≈ REST_POSE`。
7. 标定时手动摆放 L-Pose 的精度，会直接影响最终物理 REST_POSE 的真实精度。

## 2. `currentJoints` 的来源

主控端默认定义：

```cpp
// dummy-ref-core-fw/Robot/instances/dummy_robot.h
const DOF6Kinematic::Joint6D_t REST_POSE = {0, -75, 180, 0, 0, 0};

DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
DOF6Kinematic::Joint6D_t initPose = REST_POSE;
```

但运行过程中，`currentJoints` 会被电机反馈刷新：

```cpp
// dummy-ref-core-fw/Robot/instances/dummy_robot.cpp
void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];
    }
}
```

因此：

```text
currentJoints = 电机反馈角度 + initPose
```

上电后如果电机反馈角度接近 0，则 `currentJoints` 会接近 `initPose`。默认情况下 `initPose = REST_POSE`。

## 3. 主控如何查询电机位置

主控发送 `0x23` 查询电机位置：

```cpp
// dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp
void CtrlStepMotor::UpdateAngle()
{
    uint8_t mode = 0x23;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}
```

主控收到 `0x23` 回复后：

```cpp
// dummy-ref-core-fw/UserApp/protocols/can_protocol.cpp
case 0x23:
    dummy.motorJ[id]->UpdateAngleCallback(*(float*) (data), data[4]);
    break;
```

回调中把电机返回的 `_pos` 转为关节角：

```cpp
// dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp
void CtrlStepMotor::UpdateAngleCallback(float _pos, bool _isFinished)
{
    state = _isFinished ? FINISH : RUNNING;

    float tmp = _pos / (float) reduction * 360;
    angle = inverseDirection ? -tmp : tmp;
}
```

因此：

```text
motorJ[i]->angle = _pos / reduction * 360
```

其中各关节 `reduction` 在 `DummyRobot` 构造函数中定义，多数关节为 50。

## 4. 电机端 `_pos` 是怎么来的

电机端 CAN `0x23` 返回：

```cpp
// dummy-42motor-fw/UserApp/protocols/interface_can.cpp
case 0x23: // Get Position
{
    tmpF = motor.controller->GetPosition();
    ...
    CAN_Send(&txHeader, _data);
}
```

`GetPosition()` 的核心逻辑：

```cpp
// dummy-42motor-fw/Ctrl/Motor/motor.cpp
float Motor::Controller::GetPosition(bool _isLap)
{
    return _isLap ?
           (float) (realLapPosition - context->config.motionParams.encoderHomeOffset) /
           (float) (context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS)
                  :
           (float) (realPosition - context->config.motionParams.encoderHomeOffset) /
           (float) (context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
}
```

默认 `_isLap = false`，因此：

```text
_pos = (realPosition - encoderHomeOffset) / MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS
```

其中：

```text
MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = 200 * 256 = 51200
```

也就是说，`_pos` 的单位是电机轴转过的圈数。

## 5. MT6816 是单圈绝对编码器

代码中使用的是 `MT6816`：

```cpp
// dummy-42motor-fw/Port/mt6816_stm32.h
class MT6816 : public MT6816Base
{
public:
    explicit MT6816() : MT6816Base((uint16_t*) (0x08017C00))
    {}
};
```

读取角度时，每次通过 SPI 读 MT6816 的角度寄存器：

```cpp
// dummy-42motor-fw/Ctrl/Sensor/Encoder/mt6816_base.cpp
uint16_t MT6816Base::UpdateAngle()
{
    dataTx[0] = (0x80 | 0x03) << 8;
    dataTx[1] = (0x80 | 0x04) << 8;

    ...

    spiRawData.rawAngle = spiRawData.rawData >> 2;
    angleData.rawAngle = spiRawData.rawAngle;
    angleData.rectifiedAngle = quickCaliDataPtr[angleData.rawAngle];

    return angleData.rectifiedAngle;
}
```

这里的 `rawAngle` 是当前一圈内的绝对角度。`quickCaliDataPtr` 是校准表，用于把原始角度映射为电机细分角，不是断电位置存储。

因此 MT6816 的能力是：

- 能在上电后直接读出当前单圈内角度。
- 不需要先转动找零。
- 不能知道电机轴已经转过几圈。
- 不能在断电后恢复软件累加的多圈位置。

## 6. 多圈位置是软件运行中累加的

电机固件运行时通过单圈角度差累加 `realPosition`：

```cpp
// dummy-42motor-fw/Ctrl/Motor/motor.cpp
controller->realLapPositionLast = controller->realLapPosition;
controller->realLapPosition = encoder->angleData.rectifiedAngle;

deltaLapPosition = controller->realLapPosition - controller->realLapPositionLast;
if (deltaLapPosition > MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
    deltaLapPosition -= MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
else if (deltaLapPosition < -MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
    deltaLapPosition += MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

controller->realPositionLast = controller->realPosition;
controller->realPosition += deltaLapPosition;
```

这说明：

- `realLapPosition` 是单圈位置。
- `realPosition` 是软件累加得到的多圈位置。
- 断电后 `realPosition` 不会保存。

上电第一次控制循环会根据当前单圈角度初始化 `realPosition`：

```cpp
// dummy-42motor-fw/Ctrl/Motor/motor.cpp
if (isFirstCalled)
{
    ...
    controller->realLapPosition = angle;
    controller->realLapPositionLast = angle;
    controller->realPosition = angle;
    controller->realPositionLast = angle;
    ...
}
```

因此上电后多圈信息已经丢失，系统只保留当前单圈角度和 `encoderHomeOffset`。

## 7. `encoderHomeOffset` 的作用

`encoderHomeOffset` 保存于电机端 EEPROM：

```cpp
// dummy-42motor-fw/UserApp/configurations.h
typedef struct Config_t
{
    ...
    int32_t encoderHomeOffset;
    ...
} BoardConfig_t;
```

启动时从 EEPROM 读入：

```cpp
// dummy-42motor-fw/UserApp/main.cpp
eeprom.get(0, boardConfig);
...
motor.config.motionParams.encoderHomeOffset = boardConfig.encoderHomeOffset;
```

`encoderHomeOffset` 的意义是：

```text
当机械臂处于被定义为 home 的姿态时，当前电机单圈位置应被视为逻辑零点。
```

因此 `GetPosition()` 实际返回的是：

```text
当前位置相对于 home offset 的电机圈数
```

## 8. `ApplyPositionAsHome()` 做了什么

主控调用：

```cpp
// dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp
void CtrlStepMotor::ApplyPositionAsHome()
{
    uint8_t mode = 0x15;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}
```

电机端收到 `0x15`：

```cpp
// dummy-42motor-fw/UserApp/protocols/interface_can.cpp
case 0x15:  // Apply Home-Position and Store to EEPROM
    motor.controller->ApplyPosAsHomeOffset();
    boardConfig.encoderHomeOffset = motor.config.motionParams.encoderHomeOffset %
                                    motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    boardConfig.configStatus = CONFIG_COMMIT;
    break;
```

核心函数：

```cpp
// dummy-42motor-fw/Ctrl/Motor/motor.cpp
void Motor::Controller::ApplyPosAsHomeOffset()
{
    context->config.motionParams.encoderHomeOffset = realPosition %
                                                     context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}
```

因此：

```text
encoderHomeOffset = realPosition mod 51200
```

它不是让 MT6816 保存多圈位置，也不是直接保存当前 `realPosition`。它保存的是当前电机位置的单圈部分。

执行后，如果 reboot 且机械位置不变，则上电后：

```text
realPosition ≈ encoderHomeOffset
_pos = (realPosition - encoderHomeOffset) / 51200 ≈ 0
```

## 9. `CalibrateHomeOffset()` 的流程和目的

主控标定函数：

```cpp
// dummy-ref-core-fw/Robot/instances/dummy_robot.cpp
void DummyRobot::CalibrateHomeOffset()
{
    // Disable FixUpdate, but not disable motors
    isEnabled = false;
    motorJ[ALL]->SetEnable(true);

    // 1.Manually move joints to L-Pose [precisely]
    // ...
    motorJ[2]->SetCurrentLimit(0.5);
    motorJ[3]->SetCurrentLimit(0.5);
    osDelay(500);

    // 2.Apply Home-Offset the first time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);

    // 3.Go to Resting-Pose
    initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    Resting();
    osDelay(500);

    // 4.Apply Home-Offset the second time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);
    motorJ[2]->SetCurrentLimit(1);
    motorJ[3]->SetCurrentLimit(1);
    osDelay(500);

    Reboot();
}
```

这个流程可以理解为：

1. 手动把机械臂精确摆到 L-Pose。
2. 第一次 `ApplyPositionAsHome()`，把 L-Pose 临时定义为电机端零点。
3. 临时把主控 `initPose/currentJoints` 设置成 `{0, 0, 90, 0, 0, 0}`。
4. 调用 `Resting()`，让机械臂从 L-Pose 自动运动到 `REST_POSE`。
5. 第二次 `ApplyPositionAsHome()`，把最终 resting 姿态定义为电机端正式零点。
6. `Reboot()`，让标定过程中临时的 `initPose/currentJoints` 丢失，恢复默认 `REST_POSE` 体系。

## 10. 为什么要 Apply 两次

第一次 `ApplyPositionAsHome()` 的意义：

- 将手动摆好的 L-Pose 作为临时电机零点。
- 让后续程序能够从这个姿态出发，按软件定义的增量运动。

随后代码临时设定：

```cpp
initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
```

这表示：在标定过程中，软件暂时认为当前 L-Pose 对应关节角 `{0, 0, 90, 0, 0, 0}`。

`Resting()` 的目标是：

```cpp
REST_POSE = {0, -75, 180, 0, 0, 0}
```

运动命令中使用：

```cpp
// dummy-ref-core-fw/Robot/instances/dummy_robot.cpp
motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                     dynamicJointSpeeds.a[j - 1]);
```

因此从 L-Pose 到 REST_POSE 的增量为：

```text
REST_POSE - 临时 initPose
= {0, -75, 180, 0, 0, 0} - {0, 0, 90, 0, 0, 0}
= {0, -75, 90, 0, 0, 0}
```

第二次 `ApplyPositionAsHome()` 的意义：

- 此时机械臂已经到达程序认为的 REST_POSE。
- 将这个 resting 姿态设置为电机端正式 home offset。
- reboot 后，机械臂在 resting 姿态时 `_pos ≈ 0`。
- 主控默认 `initPose = REST_POSE`，因此 `currentJoints ≈ REST_POSE`。

如果只做第一次 Apply，那么最终 home 会停留在 L-Pose，不会和日常上电的 `REST_POSE` 对齐。

## 11. L-Pose 的精度会影响最终 REST_POSE

这套标定方案中，L-Pose 是手动摆放的机械基准。程序假设：

```text
手动摆好的物理 L-Pose = 软件定义的 {0, 0, 90, 0, 0, 0}
```

后续从 L-Pose 运动到 REST_POSE，是在这个假设基础上执行的。

因此：

- 如果 L-Pose 手动摆放准确，则最终物理 REST_POSE 更准确。
- 如果 L-Pose 有误差，则程序仍会认为当前姿态是 `{0, 0, 90, 0, 0, 0}`。
- 后续运动到 REST_POSE 会继承这个误差。
- 第二次 `ApplyPositionAsHome()` 会把带有该误差的 resting 姿态标定为正式零点。

也就是说：

```text
标定完成后，软件坐标系是自洽的；
但真实物理姿态是否等于设计中的 REST_POSE，取决于 L-Pose 手调精度。
```

## 12. `osDelay(500)` 的单位

主控使用 FreeRTOS/CMSIS-RTOS：

```cpp
// dummy-ref-core-fw/Core/Inc/FreeRTOSConfig.h
#define configTICK_RATE_HZ ((TickType_t)1000)
```

因此 1 tick = 1 ms。

```text
osDelay(500) = 500 ms = 0.5 s
```

这些延时主要用于等待 CAN 命令、限流设置或 EEPROM 写入状态生效，不是留给用户手动摆姿态的时间。

因此应当在调用 `CalibrateHomeOffset()` 前，先手动把机械臂摆到 L-Pose。

## 13. 标定期间 enable 后会不会被主循环拉走

`CalibrateHomeOffset()` 中：

```cpp
isEnabled = false;
motorJ[ALL]->SetEnable(true);
```

这里有两个概念：

- `isEnabled = false`：主控 FixUpdate 不发送运动命令。
- `motorJ[ALL]->SetEnable(true)`：通过 CAN 使能电机，让电机闭环保持。

主控循环：

```cpp
// dummy-ref-core-fw/UserApp/main.cpp
if (dummy.IsEnabled())
{
    dummy.MoveJoints(dummy.targetJoints);
    dummy.UpdateJointPose6D();
}
else
{
    dummy.UpdateJointAngles();
    dummy.UpdateJointPose6D();
}
```

因为标定期间 `isEnabled == false`，所以主循环不会走 `MoveJoints(dummy.targetJoints)` 分支，只会读取角度并更新正解。

因此第 203 行电机 enable 后，不会因为主控循环而自动移动到 `targetJoints`。真正主动运动发生在 `CalibrateHomeOffset()` 内部调用 `Resting()` 时。

注意：`motorJ[ALL]->SetEnable(true)` 可能让电机闭环保持当前位置。如果机械臂尚未精确摆到 L-Pose，就不应调用标定函数。

## 14. 常见问题整理

### 14.1 上电后为什么 `currentJoints` 不一定等于 `REST_POSE`

因为 `currentJoints` 初值虽然是 `REST_POSE`，但主控会周期性查询电机位置。收到 `0x23` 回复后：

```text
currentJoints = 电机反馈角度 + initPose
```

如果电机端 `_pos != 0`，则 `currentJoints != REST_POSE`。

### 14.2 `_pos` 为什么可能不为 0

因为：

```text
_pos = (realPosition - encoderHomeOffset) / 51200
```

如果当前机械姿态与标定时的 home 姿态不一致，或 `encoderHomeOffset` 没有正确保存，`_pos` 就不会是 0。

### 14.3 下电再上电后为什么读数回到 `initPose` 附近

因为 MT6816 只保留单圈绝对角，多圈位置断电后丢失。上电后电机固件用当前单圈角度和 EEPROM 中的 `encoderHomeOffset` 重新建立位置。

如果机械臂实际仍停在标定后的 REST 姿态附近，则 `_pos ≈ 0`，主控读到：

```text
currentJoints ≈ initPose
```

默认 `initPose = REST_POSE`，因此看起来回到了 `REST_POSE` 附近。

### 14.4 MT6816 是否会保存断电前角度

不会保存断电前的多圈位置。

它只能在上电后读出当前单圈内角度。若电机轴断电期间转动了整圈数，MT6816 本身无法分辨。

### 14.5 `ApplyPositionAsHome()` 是否把 `_pos` 设为 0

更准确地说，它保存：

```text
encoderHomeOffset = 当前 realPosition 的单圈余数
```

在 reboot 后，若机械位置不变，上电初始化出的 `realPosition` 会接近该 offset，因此 `_pos ≈ 0`。

### 14.6 L-Pose 是最终 home 吗

不是。

L-Pose 是标定过程中的手动基准和中间参考。最终 home 是第二次 `ApplyPositionAsHome()` 时的 resting 姿态。

## 15. 推荐标定操作顺序

1. 确认电机固件已完成编码器校准，MT6816 校准表有效。
2. 手动把机械臂精确摆到 L-Pose。
3. 调用主控命令 `calibrate_home_offset`。
4. 标定函数会：
   - 禁止主控 FixUpdate 发运动命令。
   - 使能所有电机。
   - 降低 J2/J3 电流限制。
   - 在 L-Pose 第一次 Apply Home。
   - 临时设置 `initPose/currentJoints = {0,0,90,0,0,0}`。
   - 自动运动到 `REST_POSE`。
   - 在 REST_POSE 第二次 Apply Home。
   - 恢复 J2/J3 电流限制。
   - reboot 主控和电机。
5. 重启后，在机械臂仍处于 REST 姿态时读取 `currentJoints`，应接近 `{0, -75, 180, 0, 0, 0}`。

## 16. 关键公式汇总

```text
MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = 200 * 256 = 51200
```

```text
_pos = (realPosition - encoderHomeOffset) / 51200
```

```text
motor.angle = _pos / reduction * 360
```

```text
currentJoints = motor.angle + initPose
```

```text
ApplyPositionAsHome:
encoderHomeOffset = realPosition mod 51200
```

```text
CalibrateHomeOffset 中的运动增量:
REST_POSE - 临时 initPose
= {0, -75, 180, 0, 0, 0} - {0, 0, 90, 0, 0, 0}
= {0, -75, 90, 0, 0, 0}
```

## 17. 整体数据流

```text
MT6816 单圈角度
    -> rectifiedAngle
    -> 电机固件 realLapPosition
    -> 软件累加 realPosition
    -> 减 encoderHomeOffset
    -> 得到 _pos
    -> CAN 0x23 返回主控
    -> 主控换算 motor.angle
    -> currentJoints = motor.angle + initPose
```

标定完成后的期望状态：

```text
机械臂处于 REST_POSE
    -> 电机端 _pos ≈ 0
    -> 主控 motor.angle ≈ 0
    -> currentJoints ≈ initPose
    -> initPose 默认等于 REST_POSE
    -> currentJoints ≈ REST_POSE
```

## 18. 这套方案的局限

1. 不能恢复断电期间的多圈运动。
2. L-Pose 手调误差会传递到最终物理 REST_POSE。
3. 标定完成后软件坐标系内部自洽，但不保证绝对物理位置与 CAD/设计模型完全一致。
4. 若需要更高绝对精度，需要额外基准，例如限位开关、机械定位治具、视觉/外部测量系统等。

