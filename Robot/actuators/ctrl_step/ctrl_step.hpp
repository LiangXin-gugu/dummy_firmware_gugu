#ifndef DUMMY_CORE_FW_CTRL_STEP_HPP
#define DUMMY_CORE_FW_CTRL_STEP_HPP

#include "fibre/protocol.hpp"
#include "can.h"

class CtrlStepMotor
{
public:
    enum State
    {
        RUNNING,
        FINISH,
        STOP
    };


    const uint32_t CTRL_CIRCLE_COUNT = 200 * 256;

    CtrlStepMotor(CAN_HandleTypeDef* _hcan, uint8_t _id, bool _inverse = false, uint8_t _reduction = 1,
                  float _angleLimitMin = -180, float _angleLimitMax = 180);

    uint8_t nodeID;
    float angle = 0;
    float angleLimitMax;
    float angleLimitMin;
    // FOC current in amps; refreshed asynchronously by the CAN 0x21 RX handler.
    // Trigger a broadcast/single request via UpdateCurrent().
    float current = 0;
    // Motor chip temperature in deg C; refreshed asynchronously by the CAN 0x25 RX handler.
    // The motor side only samples ~1Hz and requires 0x7d (SetEnableTemp(true)) first,
    // otherwise this stays 0. Trigger a request via UpdateTemp().
    float temperature = 0;
    // Controller status bytes from CAN 0x30 ACK: [requestMode, modeRunning, state]
    // Synchronously queried on demand via GetControllerStatus()
    uint8_t statusRequestMode = 0;
    uint8_t statusModeRunning = 0;
    uint8_t statusState = 0;
    // Flag to indicate fresh 0x30 ACK received since last query
    volatile bool statusReceived = false;
    
    // DCE parameters from CAN 0x31/0x32 ACK: [kp, kv] + [ki, kd]
    // Synchronously queried on demand via GetDceParameters()
    int32_t dceKp = 0;
    int32_t dceKv = 0;
    int32_t dceKi = 0;
    int32_t dceKd = 0;
    // Flags to indicate fresh 0x31/0x32 ACK received since last query
    volatile bool dceParamsLowReceived = false;  // 0x31
    volatile bool dceParamsHighReceived = false;  // 0x32
    bool inverseDirection;
    uint8_t reduction;
    State state = STOP;

    void SetAngle(float _angle);
    void SetAngleWithVelocityLimit(float _angle, float _vel);
    void SetAngleWithTrajectoryVelocity(float _angle, float _angleVel); // joint deg + deg/s (signed feed-forward)
    // CAN Command
    void SetEnable(bool _enable);
    void SetEnableTemp(bool _enable);
    void DoCalibration();
    void SetCurrentSetPoint(float _val);
    void SetVelocitySetPoint(float _val);
    void SetPositionSetPoint(float _val);
    void SetPositionWithVelocityLimit(float _pos, float _vel);
    void SetTrajectorySetPoint(float _pos, float _vel); // 0x08: motor-circle pos + motor r/s vel (signed)
    void SetNodeID(uint32_t _id);
    void SetCurrentLimit(float _val);
    void SetVelocityLimit(float _val);
    void SetAcceleration(float _val);
    void SetDceKp(int32_t _val);
    void SetDceKv(int32_t _val);
    void SetDceKi(int32_t _val);
    void SetDceKd(int32_t _val);
    void ApplyPositionAsHome();
    void SetEnableOnBoot(bool _enable);
    void SetEnableStallProtect(bool _enable);
    void Reboot();
    // Pure getter: returns the cached temperature (see field comment for update semantics).
    // Retained for fibre protocol backwards compatibility; use UpdateTemp() to request a refresh.
    float GetTemp();
    void EraseConfigs();

    // Broadcast (nodeID==0) or single-node request; result is delivered asynchronously
    // via OnCanMessage() and stored into `current` / `temperature`.
    void UpdateCurrent();
    void UpdateTemp();
    void UpdateAngle();
    void UpdateAngleCallback(float _pos, bool _isFinished);
    // Synchronous query for Controller Status (CAN 0x30)
    // Returns true if successful, false on timeout/error
    bool GetControllerStatus();
    // Synchronous query for DCE Parameters (CAN 0x31/0x32)
    // Returns true if successful, false on timeout/error
    bool GetDceParameters();


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            // Note: Current/temperature data uses ASCII. Do not add it to the Fibre list, as this may enlarge the protocol tree and overflow the commTask stack.
            make_protocol_ro_property("angle", &angle),
            make_protocol_function("reboot", *this, &CtrlStepMotor::Reboot),
            make_protocol_function("get_temperature", *this, &CtrlStepMotor::GetTemp),
            make_protocol_function("set_enable_temperature", *this, &CtrlStepMotor::SetEnableTemp, "enable"),
            make_protocol_function("erase_configs", *this, &CtrlStepMotor::EraseConfigs),
            make_protocol_function("set_enable", *this, &CtrlStepMotor::SetEnable, "enable"),
            make_protocol_function("set_position_with_time", *this,
                                   &CtrlStepMotor::SetPositionWithVelocityLimit, "pos", "time"),
            make_protocol_function("set_position", *this, &CtrlStepMotor::SetPositionSetPoint, "pos"),
            make_protocol_function("set_velocity", *this, &CtrlStepMotor::SetVelocitySetPoint, "vel"),
            make_protocol_function("set_velocity_limit", *this, &CtrlStepMotor::SetVelocityLimit, "vel"),
            make_protocol_function("set_current", *this, &CtrlStepMotor::SetCurrentSetPoint, "current"),
            make_protocol_function("set_current_limit", *this, &CtrlStepMotor::SetCurrentLimit, "current"),
            make_protocol_function("set_node_id", *this, &CtrlStepMotor::SetNodeID, "id"),
            make_protocol_function("set_acceleration", *this, &CtrlStepMotor::SetAcceleration, "acc"),
            make_protocol_function("apply_home_offset", *this, &CtrlStepMotor::ApplyPositionAsHome),
            make_protocol_function("do_calibration", *this, &CtrlStepMotor::DoCalibration),
            make_protocol_function("set_enable_on_boot", *this, &CtrlStepMotor::SetEnableOnBoot, "enable"),
            make_protocol_function("set_dce_kp", *this, &CtrlStepMotor::SetDceKp, "vel"),
            make_protocol_function("set_dce_kv", *this, &CtrlStepMotor::SetDceKv, "vel"),
            make_protocol_function("set_dce_ki", *this, &CtrlStepMotor::SetDceKi, "vel"),
            make_protocol_function("set_dce_kd", *this, &CtrlStepMotor::SetDceKd, "vel"),
            make_protocol_function("set_enable_stall_protect", *this, &CtrlStepMotor::SetEnableStallProtect,
                                   "enable"),
            make_protocol_function("update_angle", *this, &CtrlStepMotor::UpdateAngle)
        );
    }


private:
    CAN_HandleTypeDef* hcan;
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = {};
};

#endif //DUMMY_CORE_FW_CTRL_STEP_HPP
