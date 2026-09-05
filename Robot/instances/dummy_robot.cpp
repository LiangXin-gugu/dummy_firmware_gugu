#include "communication.hpp"
#include "dummy_robot.h"

#include <cstring>

inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index)
{
    float max = -1;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (abs(_joints.a[i]) > max)
        {
            max = abs(_joints.a[i]);
            _index = i;
        }
    }

    return max;
}


DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);
    motorJ[1] = new CtrlStepMotor(_hcan, 1, false, 50, -170, 170);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, true, 50, -75, 90);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, false, 50, 0, 180);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, true, 50, -180, 180);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true, 50, -90, 90);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, false, 1, -360, 360);
    hand = new DummyHand(_hcan, 7);

    dof6Solver = new DOF6Kinematic(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f);
}


DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];

    delete hand;
    delete dof6Solver;
}


void DummyRobot::Init()
{
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeedPercent(DEFAULT_JOINT_SPEED);
    ApplyJointAcceleration();
}


void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();
    osDelay(500); // waiting for all joints done
    HAL_NVIC_SystemReset();
}

void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    for (int j = 1; j <= 6; j++)
    {
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                             dynamicJointSpeeds.a[j - 1]);
    }
}


void DummyRobot::MoveJointsTrajectory(DOF6Kinematic::Joint6D_t _joints, DOF6Kinematic::Joint6D_t _jointVels)
{
    // Feed-forward ONE (pos, vel) waypoint to each motor via CAN 0x08.
    // Position needs the initPose offset (like MoveJoints); velocity is a pure rate, no offset.
    for (int j = 1; j <= 6; j++)
    {
        motorJ[j]->SetAngleWithTrajectoryVelocity(_joints.a[j - 1] - initPose.a[j - 1],
                                                  _jointVels.a[j - 1]);
    }
}


bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    bool valid = true;

    for (int j = 1; j <= 6; j++)
    {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }

    if (valid)
    {
        DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;
        uint8_t index;
        float maxAngle = AbsMaxOf6(deltaJoints, index);
        float time = maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;
        for (int j = 1; j <= 6; j++)
        {
            dynamicJointSpeeds.a[j - 1] =
                abs(deltaJoints.a[j - 1] * (float) (motorJ[j]->reduction) / time * jointSpeedUnitToRps); // r/s on motor shaft
        }

        jointsStateFlag = 0;
        targetJoints = targetJointsTmp;

        return true;
    }

    return false;
}


bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};

    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);

    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;

        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                continue;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        return MoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5]);
    }

    return false;
}

void DummyRobot::UpdateJointAngles()
{
    motorJ[ALL]->UpdateAngle();
}


void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];

        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
}


void DummyRobot::SetJointSpeedPercent(float _speed)
{
    if (_speed < 0)_speed = 0;
    else if (_speed > 100) _speed = 100;

    jointSpeed = _speed * jointSpeedRatio;
}

void DummyRobot::SetJointSpeedFactor(float _unit)
{
    if (_unit < 0.01f) _unit = 0.01f;
    else if (_unit > 1.0f) _unit = 1.0f;

    jointSpeedUnitToRps = _unit;
}

void DummyRobot::SetJointAccelerationPercent(float _acc)
{
    if (_acc < 0)_acc = 0;
    else if (_acc > 100) _acc = 100;

    jointAccPercent = _acc;
}

void DummyRobot::SetJointAccelerationBases(float _b1, float _b2, float _b3, float _b4, float _b5, float _b6)
{
    float b[6] = {_b1, _b2, _b3, _b4, _b5, _b6};
    for (int i = 0; i < 6; i++)
    {
        if (b[i] < 0) b[i] = 0;
        else if (b[i] > 200) b[i] = 200;
        jointAccelerationBases.a[i] = b[i];
    }
}

void DummyRobot::ApplyJointAcceleration()
{
    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(jointAccPercent / 100 * jointAccelerationBases.a[i - 1]);
}

float DummyRobot::GetJointSpeedPercent() const
{
    return jointSpeed;
}

float DummyRobot::GetJointSpeedFactor() const
{
    return jointSpeedUnitToRps;
}

float DummyRobot::GetJointAccelerationPercent() const
{
    return jointAccPercent;
}

DOF6Kinematic::Joint6D_t DummyRobot::GetJointAccelerationBases() const
{
    return jointAccelerationBases;
}


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


void DummyRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeedPercent(10);

    MoveJ(0, 0, 120, 0, 0, 0);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeedPercent(lastSpeed);
}


void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeedPercent(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5]);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeedPercent(lastSpeed);
}


void DummyRobot::SetEnable(bool _enable)
{
    motorJ[ALL]->SetEnable(_enable);
    isEnabled = _enable;
}

void DummyRobot::SetRGBEnable(bool _enable)
{
    isRGBEnabled = _enable;
}

bool DummyRobot::GetRGBEnabled()
{
    return isRGBEnabled;
}

void DummyRobot::SetRGBMode(uint32_t mode)
{
    rgbMode = mode;
}

uint32_t DummyRobot::GetRGBMode()
{
    return rgbMode;
}

void DummyRobot::UpdateJointPose6D()
{
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    currentPose6D.X *= 1000; // m -> mm
    currentPose6D.Y *= 1000; // m -> mm
    currentPose6D.Z *= 1000; // m -> mm
}


bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;
}


bool DummyRobot::IsEnabled()
{
    return isEnabled;
}


void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_MOTOR_TUNING)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    // switch (commandMode)
    // {
    //     case COMMAND_TARGET_POINT_SEQUENTIAL:
    //     case COMMAND_TARGET_POINT_INTERRUPTABLE:
    //         jointSpeedRatio = 1;
    //         SetJointAccelerationPercent(DEFAULT_JOINT_ACCELERATION_LOW);
    //         break;
    //     case COMMAND_CONTINUES_TRAJECTORY:
    //         SetJointAccelerationPercent(DEFAULT_JOINT_ACCELERATION_HIGH);
    //         jointSpeedRatio = 0.3;
    //         break;
    //     case COMMAND_MOTOR_TUNING:
    //         break;
    // }
}


DummyHand::DummyHand(CAN_HandleTypeDef* _hcan, uint8_t
_id) :
    nodeID(_id), hcan(_hcan)
{
    txHeader =
        {
            .StdId = 0,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE
        };
}


void DummyHand::SetAngle(float _angle)
{
    if (_angle > 30)_angle = 30;
    if (_angle < 0)_angle = 0;

    uint8_t mode = 0x02;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_angle;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetMaxCurrent(float _val)
{
    if (_val > 1)_val = 1;
    if (_val < 0)_val = 0;

    uint8_t mode = 0x01;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetEnable(bool _enable)
{
    if (_enable)
        SetMaxCurrent(maxCurrent);
    else
        SetMaxCurrent(0);
}


uint32_t DummyRobot::CommandHandler::Push(const char* _cmd)
{
    // Copy into a fixed-size message so osMessageQueuePut (msg_size=CMD_MAX_LENGTH)
    // never reads past the caller's buffer. No heap allocation involved.
    char msg[CMD_MAX_LENGTH];
    strncpy(msg, _cmd, CMD_MAX_LENGTH - 1);
    msg[CMD_MAX_LENGTH - 1] = 0;

    osStatus_t status = osMessageQueuePut(commandFifo, msg, 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; // failed
}


void DummyRobot::CommandHandler::EmergencyStop()
{
    context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
                   context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    context->MoveJoints(context->targetJoints);
    context->isEnabled = false;
    ClearFifo();
}


bool DummyRobot::CommandHandler::Pop(char* _buffer, uint32_t timeout)
{
    return osMessageQueueGet(commandFifo, _buffer, nullptr, timeout) == osOK;
}


uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}


uint32_t DummyRobot::CommandHandler::ParseCommand(const char* _cmd)
{
    uint8_t argNum;

    bool accepted = false;

    switch (context->commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float joints[6];
                float speed;

                if (_cmd[0] == '>')
                    argNum = sscanf(_cmd, ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (_cmd[0] == '&')
                    argNum = sscanf(_cmd, "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    accepted = context->MoveJ(joints[0], joints[1], joints[2],
                                              joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeedPercent(speed);
                    accepted = context->MoveJ(joints[0], joints[1], joints[2],
                                              joints[3], joints[4], joints[5]);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                if (accepted)
                {
                    context->MoveJoints(context->targetJoints);

                    while (context->IsMoving() && context->IsEnabled())
                        osDelay(5);
                    Respond(*usbStreamOutputPtr, "ok");
                    // Respond(*uart4StreamOutputPtr, "ok");
                }
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd, "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    accepted = context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeedPercent(speed);
                    accepted = context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }

                if (accepted)
                {
                    context->MoveJoints(context->targetJoints);
                    while (context->IsMoving() && context->IsEnabled())
                        osDelay(5);
                    Respond(*usbStreamOutputPtr, "ok");
                    // Respond(*uart4StreamOutputPtr, "ok");
                }
            }

            break;

        case COMMAND_CONTINUES_TRAJECTORY:
            // Non-blocking & event-driven: each command carries ONE (pos, vel) waypoint and is
            // forwarded to the motors exactly once via CAN 0x08. The 200Hz FixUpdate loop must NOT
            // resend it -- the motor's trajectory tracker only reacts to a CHANGED goal, so
            // resending an identical setpoint does nothing but waste CAN bandwidth.
            // The host should stream progressive waypoints at a period < 200ms (the motor-side
            // trajectory timeout); otherwise the motor auto-decelerates to a safe stop.
            // Format: >p1,p2,p3,p4,p5,p6,v1,v2,v3,v4,v5,v6   (6x joint deg, then 6x joint deg/s)
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float joints[6];
                float vels[6];
                argNum = sscanf(_cmd + 1, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                                joints, joints + 1, joints + 2, joints + 3, joints + 4, joints + 5,
                                vels, vels + 1, vels + 2, vels + 3, vels + 4, vels + 5);
                if (argNum == 12)
                {
                    bool valid = true;
                    for (int j = 1; j <= 6; j++)
                    {
                        if (joints[j - 1] > context->motorJ[j]->angleLimitMax ||
                            joints[j - 1] < context->motorJ[j]->angleLimitMin)
                            valid = false;
                    }

                    if (valid)
                    {
                        for (int j = 0; j < 6; j++)
                        {
                            context->targetJoints.a[j] = joints[j];
                            context->targetJointVels.a[j] = vels[j];
                        }
                        context->MoveJointsTrajectory(context->targetJoints, context->targetJointVels);
                        Respond(*usbStreamOutputPtr, "ok");
                    } else
                    {
                        Respond(*usbStreamOutputPtr, "error trajectory joint limit exceeded");
                    }
                } else
                {
                    Respond(*usbStreamOutputPtr,
                            "error trajectory needs 12 args (6 pos + 6 vel), got %d", argNum);
                }
            }
            break;

        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float joints[6];
                float speed;

                if (_cmd[0] == '>')
                    argNum = sscanf(_cmd, ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (_cmd[0] == '&')
                    argNum = sscanf(_cmd, "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    accepted = context->MoveJ(joints[0], joints[1], joints[2],
                                              joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeedPercent(speed);
                    accepted = context->MoveJ(joints[0], joints[1], joints[2],
                                              joints[3], joints[4], joints[5]);
                }
                if (accepted)
                {
                    // Respond(*usbStreamOutputPtr, "context->MoveJ succeeded");
                    // Respond(*uart4StreamOutputPtr, "context->MoveJ succeeded");
                    Respond(*usbStreamOutputPtr, "ok");
                    // Respond(*uart4StreamOutputPtr, "ok");
                }
                else
                {
                    Respond(*usbStreamOutputPtr, "context->MoveJ failed, check whether joint limits are exceeded");
                    // Respond(*uart4StreamOutputPtr, "context->MoveJ failed, check whether joint limits are exceeded");
                }
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd, "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    accepted = context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeedPercent(speed);
                    accepted = context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                if (accepted)
                {
                    // Respond(*usbStreamOutputPtr, "context->MoveL succeeded");
                    // Respond(*uart4StreamOutputPtr, "context->MoveL succeeded");
                    Respond(*usbStreamOutputPtr, "ok");
                    // Respond(*uart4StreamOutputPtr, "ok");
                }
                else
                {
                    Respond(*usbStreamOutputPtr, "context->MoveL failed");
                    // Respond(*uart4StreamOutputPtr, "context->MoveL failed");
                }
            }
            break;

        case COMMAND_MOTOR_TUNING:
            break;
    }

    return osMessageQueueGetSpace(commandFifo);
}


void DummyRobot::CommandHandler::ClearFifo()
{
    osMessageQueueReset(commandFifo);
}


void DummyRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}


void DummyRobot::TuningHelper::Tick(uint32_t _timeMillis)
{
    time += PI * 2 * frequency * (float) _timeMillis / 1000.0f;
    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (tuningFlag & (1 << (i - 1)))
            context->motorJ[i]->SetAngle(delta);
}


void DummyRobot::TuningHelper::SetFreqAndAmp(float _freq, float _amp)
{
    if (_freq > 5)_freq = 5;
    else if (_freq < 0.1) _freq = 0.1;
    if (_amp > 50)_amp = 50;
    else if (_amp < 1) _amp = 1;

    frequency = _freq;
    amplitude = _amp;
}
