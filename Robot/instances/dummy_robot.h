#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include "string"
#define ALL 0

/*
  |   PARAMS   | `current_limit` | `acceleration` | `dce_kp` | `dce_kv` | `dce_ki` | `dce_kd` |
  | ---------- | --------------- | -------------- | -------- | -------- | -------- | -------- |
  | **Joint1** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint2** | 2               | 30             | 1000     | 80       | 200      | 200      |
  | **Joint3** | 2               | 30             | 1500     | 80       | 200      | 250      |
  | **Joint4** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint5** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint6** | 2               | 30             | 1000     | 80       | 200      | 250      |
 */


class DummyHand
{
public:
    uint8_t nodeID = 7;
    float maxCurrent = 1.0;


    DummyHand(CAN_HandleTypeDef* _hcan, uint8_t _id);


    void SetAngle(float _angle);
    void SetMaxCurrent(float _val);
    void SetEnable(bool _enable);


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("set_angle", *this, &DummyHand::SetAngle, "angle"),
            make_protocol_function("set_enable", *this, &DummyHand::SetEnable, "enable"),
            make_protocol_function("set_current_limit", *this, &DummyHand::SetMaxCurrent, "current")
        );
    }


private:
    CAN_HandleTypeDef* hcan;
    uint8_t canBuf[8];
    CAN_TxHeaderTypeDef txHeader;
    float minAngle = 0;
    float maxAngle = 45;
};


class DummyRobot
{
public:
    explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    ~DummyRobot();


    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        COMMAND_CONTINUES_TRAJECTORY,
        COMMAND_MOTOR_TUNING
    };


    class TuningHelper
    {
    public:
        explicit TuningHelper(DummyRobot* _context) : context(_context)
        {
        }

        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);


        // Communication protocol definitions
        auto MakeProtocolDefinitions()
        {
            return make_protocol_member_list(
                make_protocol_function("set_tuning_freq_amp", *this,
                                       &TuningHelper::SetFreqAndAmp, "freq", "amp"),
                make_protocol_function("set_tuning_flag", *this,
                                       &TuningHelper::SetTuningFlag, "flag")
            );
        }


    private:
        DummyRobot* context;
        float time = 0;
        uint8_t tuningFlag = 0;
        float frequency = 1;
        float amplitude = 1;
    };
    TuningHelper tuningHelper = TuningHelper(this);


    // This is the pose when power on.
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -75, 180, 0, 0, 0};
    // Unit of jointSpeed: 1 unit = JOINT_SPEED_UNIT_TO_RPS r/s on MOTOR shaft (motor side, not reducer side).
    // jointSpeed range 0~100 maps to motor velocity limit 0~(100*JOINT_SPEED_UNIT_TO_RPS) r/s. Tune this to change the cap.
    const float DEFAULT_JOINT_SPEED_UNIT_TO_RPS = 0.2f;
    const float DEFAULT_JOINT_SPEED = 50;  // jointSpeed percent, scope 0~100, maximum 100 means 100*JOINT_SPEED_UNIT_TO_RPS r/s on motor shaft
    const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {150, 100, 200, 200, 200, 200};
    // const float DEFAULT_JOINT_ACCELERATION_LOW = 15;    // 0~100
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;  // 0~100
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;


    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJointVels = {0, 0, 0, 0, 0, 0}; // joint-space velocity feed-forward (deg/s), for CONTINUES_TRAJECTORY
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;
    DOF6Kinematic::Pose6D_t currentPose6D = {};
    volatile uint8_t jointsStateFlag = 0b00000000;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    CtrlStepMotor* motorJ[7] = {nullptr};
    DummyHand* hand = {nullptr};


    void Init();
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    void MoveJointsTrajectory(DOF6Kinematic::Joint6D_t _joints, DOF6Kinematic::Joint6D_t _jointVels);
    void SetJointSpeedPercent(float _speed);
    void SetJointSpeedFactor(float _unit);
    void SetJointAccelerationPercent(float _acc);
    void SetJointAccelerationBases(float _b1, float _b2, float _b3, float _b4, float _b5, float _b6);
    void ApplyJointAcceleration();
    float GetJointSpeedPercent() const;
    float GetJointSpeedFactor() const;
    float GetJointAccelerationPercent() const;
    DOF6Kinematic::Joint6D_t GetJointAccelerationBases() const;
    // Snapshot getters for motor-side telemetry (values are refreshed asynchronously by
    // the CAN RX handler; the 200Hz FixUpdate loop drives the periodic broadcast).
    DOF6Kinematic::Joint6D_t GetMotorCurrents() const;
    DOF6Kinematic::Joint6D_t GetMotorTemperatures() const;
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
    // Broadcast 0x21 / 0x25 to all motors (nodeID==0). Non-blocking: responses arrive via
    // OnCanMessage() and refresh motorJ[i]->current / ->temperature.
    void UpdateAllCurrent();
    void UpdateAllTemp();
    // Broadcast 0x7d so motors start sampling chip temperature (they default to disabled on boot).
    void EnableMotorTempWatch(bool _enable);
    void UpdateJointPose6D();
    void Reboot();
    void SetEnable(bool _enable);
    void SetRGBEnable(bool _enable);
    bool GetRGBEnabled();
    void SetRGBMode(uint32_t mode);
    uint32_t GetRGBMode();
    void CalibrateHomeOffset();
    void Homing();
    void Resting();
    bool IsMoving();
    bool IsEnabled();
    void SetCommandMode(uint32_t _mode);
    // Synchronous query for Controller Status of a single motor
    // nodeId: 1..6, timeout ~50ms. Returns true on success.
    bool GetMotorControllerStatus(uint8_t nodeId, uint8_t* requestMode, 
                                   uint8_t* modeRunning, uint8_t* state);
    // Synchronous query for DCE Parameters of a single motor
    // nodeId: 1..6, timeout ~100ms (waiting for both 0x31 and 0x32). Returns true on success.
    bool GetMotorDceParameters(uint8_t nodeId, int32_t* kp, int32_t* kv, 
                                int32_t* ki, int32_t* kd);
    
    // Asynchronous telemetry getters (refreshed by UpdateSingleNodeTelemetry)
    // Similar pattern to GetMotorCurrents() / GetMotorTemperatures()
    struct MotorTelemetry {
        int32_t dceOutputKp, dceOutputKi;
        int32_t dceOutputKd, dceOutputTotal;
        int32_t realPosition, estPosition;
        int32_t estVelocity, softVelocity;
        int32_t softPosition;
    };
    MotorTelemetry GetMotorTelemetry(uint8_t nodeId) const;

    // Asynchronous telemetry update
    // queryNodeForTelemetry != 0 indicates the node to query in UpdateLoop
    uint8_t queryNodeForTelemetry = 0;
    void UpdateSingleNodeTelemetry();

    // current polling button
    bool enableCurrentPolling = true;


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("calibrate_home_offset", *this, &DummyRobot::CalibrateHomeOffset),
            make_protocol_function("homing", *this, &DummyRobot::Homing),
            make_protocol_function("resting", *this, &DummyRobot::Resting),
            make_protocol_object("joint_1", motorJ[1]->MakeProtocolDefinitions()),
            make_protocol_object("joint_2", motorJ[2]->MakeProtocolDefinitions()),
            make_protocol_object("joint_3", motorJ[3]->MakeProtocolDefinitions()),
            make_protocol_object("joint_4", motorJ[4]->MakeProtocolDefinitions()),
            make_protocol_object("joint_5", motorJ[5]->MakeProtocolDefinitions()),
            make_protocol_object("joint_6", motorJ[6]->MakeProtocolDefinitions()),
            make_protocol_object("joint_all", motorJ[ALL]->MakeProtocolDefinitions()),
            make_protocol_object("hand", hand->MakeProtocolDefinitions()),
            make_protocol_function("reboot", *this, &DummyRobot::Reboot),
            make_protocol_function("set_enable", *this, &DummyRobot::SetEnable, "enable"),
            make_protocol_function("set_rgb_enable", *this, &DummyRobot::SetRGBEnable, "enable"),
            make_protocol_function("set_rgb_mode", *this, &DummyRobot::SetRGBMode, "mode"),
            make_protocol_function("move_j", *this, &DummyRobot::MoveJ, "j1", "j2", "j3", "j4", "j5", "j6"),
            make_protocol_function("move_l", *this, &DummyRobot::MoveL, "x", "y", "z", "a", "b", "c"),
            make_protocol_function("set_joint_speed", *this, &DummyRobot::SetJointSpeedPercent, "speed"),
            make_protocol_function("set_joint_acc", *this, &DummyRobot::SetJointAccelerationPercent, "acc"),
            make_protocol_function("set_command_mode", *this, &DummyRobot::SetCommandMode, "mode"),
            make_protocol_object("tuning", tuningHelper.MakeProtocolDefinitions())
        );
    }


    class CommandHandler
    {
    public:
        // Fixed message size of the FIFO, avoids any per-command heap allocation
        static const uint32_t CMD_MAX_LENGTH = 64;

        explicit CommandHandler(DummyRobot* _context) : context(_context)
        {
            commandFifo = osMessageQueueNew(16, CMD_MAX_LENGTH, nullptr);
        }

        uint32_t Push(const char* _cmd);
        bool Pop(char* _buffer, uint32_t timeout);
        uint32_t ParseCommand(const char* _cmd);
        uint32_t GetSpace();
        void ClearFifo();
        void EmergencyStop();


    private:
        DummyRobot* context;
        osMessageQueueId_t commandFifo;
    };
    CommandHandler commandHandler = CommandHandler(this);


private:
    CAN_HandleTypeDef* hcan;
    float jointSpeed = DEFAULT_JOINT_SPEED;
    float jointSpeedUnitToRps = DEFAULT_JOINT_SPEED_UNIT_TO_RPS;
    float jointSpeedRatio = 1;
    DOF6Kinematic::Joint6D_t jointAccelerationBases = DEFAULT_JOINT_ACCELERATION_BASES;
    float jointAccPercent = DEFAULT_JOINT_ACCELERATION_HIGH;
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
    DOF6Kinematic* dof6Solver;
    bool isEnabled = false;
    bool isRGBEnabled = true;
    uint32_t rgbMode = 0;
};


#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
