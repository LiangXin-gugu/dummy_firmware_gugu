#include "common_inc.h"
#include <cstring>

extern DummyRobot dummy;

// Format a float as fixed-point with `decimals` fractional digits (1, 2 or 3) using ONLY
// integer printf. This deliberately avoids newlib's "%f" path (dtoa -> _Balloc ->
// malloc/free), which is NOT thread-safe here (no __malloc_lock, single shared _reent):
// a high-rate GETJPOS/GETLPOS/GET_CURRENT/GET_TEMP Respond in the USB task races with the
// move_j sscanf("%f") consumer thread and corrupts the heap -> HardFault -> whole-MCU hang
// (OLED freeze + host "Write timeout"). Output is byte-identical to the matching "%.*f" so
// the host SDK regexes still match.
static void FormatFixedN(float value, char* out, size_t outSize, int decimals)
{
    int negative = (value < 0.0f);
    float magnitude = negative ? -value : value;
    const char* sign = negative ? "-" : "";
    switch (decimals)
    {
        case 1:
        {
            long scaled = (long) (magnitude * 10.0f + 0.5f);
            snprintf(out, outSize, "%s%ld.%ld", sign, scaled / 10, scaled % 10);
            break;
        }
        case 3:
        {
            long scaled = (long) (magnitude * 1000.0f + 0.5f);
            snprintf(out, outSize, "%s%ld.%03ld", sign, scaled / 1000, scaled % 1000);
            break;
        }
        case 2:
        default:
        {
            long scaled = (long) (magnitude * 100.0f + 0.5f);
            snprintf(out, outSize, "%s%ld.%02ld", sign, scaled / 100, scaled % 100);
            break;
        }
    }
}

void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    uint8_t  i;
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!' )
    {
        // NOTE: keep this hot path heap-free (no std::string): newlib's
        // malloc/free is not thread-safe here, and GETJPOS-like high-rate
        // commands would race with other tasks' heap usage and corrupt it.
        if (strstr(_cmd, "STOP") != nullptr)
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        } else if (strstr(_cmd, "START") != nullptr)
        {
            dummy.SetEnable(true);
            Respond(_responseChannel, "Started ok");
        } else if (strstr(_cmd, "HOME") != nullptr)
        {
            dummy.Homing();
            Respond(_responseChannel, "Started ok");
        } else if (strstr(_cmd, "CALIBRATION") != nullptr)
        {
            dummy.CalibrateHomeOffset();
            Respond(_responseChannel, "calibration ok");
        } else if (strstr(_cmd, "RESET") != nullptr)
        {
            dummy.Resting();
            Respond(_responseChannel, "Started ok");
        } else if (strstr(_cmd, "DISABLE") != nullptr)
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
        }
    } else if (_cmd[0] == '#')
    {
        if (strstr(_cmd, "GETJPOS") != nullptr)
        {
            // Fixed-point integer formatting (2 decimals), no dtoa/malloc -- see FormatFixedN.
            char s[6][16];
            for (int k = 0; k < 6; k++)
                FormatFixedN(dummy.currentJoints.a[k], s[k], sizeof(s[k]), 2);
            Respond(_responseChannel, "ok %s %s %s %s %s %s",
                    s[0], s[1], s[2], s[3], s[4], s[5]);
        } else if (strstr(_cmd, "GETLPOS") != nullptr)
        {
            dummy.UpdateJointPose6D();
            // Fixed-point integer formatting (2 decimals), no dtoa/malloc -- see FormatFixedN.
            const float pose[6] = {dummy.currentPose6D.X, dummy.currentPose6D.Y,
                                   dummy.currentPose6D.Z, dummy.currentPose6D.A,
                                   dummy.currentPose6D.B, dummy.currentPose6D.C};
            char s[6][16];
            for (int k = 0; k < 6; k++)
                FormatFixedN(pose[k], s[k], sizeof(s[k]), 2);
            Respond(_responseChannel, "ok %s %s %s %s %s %s",
                    s[0], s[1], s[2], s[3], s[4], s[5]);
        } else if (strstr(_cmd, "GET_SPEED_CFG") != nullptr)
        {
            Respond(_responseChannel, "ok %.2f %.3f",
                    dummy.GetJointSpeedPercent(), dummy.GetJointSpeedFactor());
        } else if (strstr(_cmd, "GET_ACC_CFG") != nullptr)
        {
            auto bases = dummy.GetJointAccelerationBases();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.GetJointAccelerationPercent(),
                    bases.a[0], bases.a[1], bases.a[2],
                    bases.a[3], bases.a[4], bases.a[5]);
        } else if (strstr(_cmd, "GET_CURRENT") != nullptr)
        {
            // Reads the cached motor currents (amps). Cache is refreshed at ~50Hz by
            // the FixUpdate thread broadcasting CAN 0x21; no blocking.
            // Fixed-point integer formatting (3 decimals), no dtoa/malloc -- see FormatFixedN.
            auto currents = dummy.GetMotorCurrents();
            char s[6][16];
            for (int k = 0; k < 6; k++)
                FormatFixedN(currents.a[k], s[k], sizeof(s[k]), 3);
            Respond(_responseChannel, "ok %s %s %s %s %s %s",
                    s[0], s[1], s[2], s[3], s[4], s[5]);
        } else if (strstr(_cmd, "GET_TEMP") != nullptr)
        {
            // Reads the cached motor chip temperatures (deg C). Cache is refreshed at ~1Hz
            // (matching the motor-side sampling rate) by the FixUpdate thread broadcasting
            // CAN 0x25. Requires enableTempWatch=true on motor side, which DummyRobot::Init
            // broadcasts via 0x7d at boot.
            // Fixed-point integer formatting (1 decimal), no dtoa/malloc -- see FormatFixedN.
            auto temps = dummy.GetMotorTemperatures();
            char s[6][16];
            for (int k = 0; k < 6; k++)
                FormatFixedN(temps.a[k], s[k], sizeof(s[k]), 1);
            Respond(_responseChannel, "ok %s %s %s %s %s %s",
                    s[0], s[1], s[2], s[3], s[4], s[5]);
        } else if (strstr(_cmd, "GET_STATUS") != nullptr)
        {
            // Synchronous query for Controller Status (CAN 0x30): requestMode | modeRunning | state
            // Example: #GET_STATUS 1 → ok 0|0|5 (node 1: STOP|STOP|NO_CALIB)
            uint32_t node;
            if (sscanf(_cmd, "#GET_STATUS %lu", &node) == 1 && node >= 1 && node <= 6) {
                uint8_t reqMode = 0, modeRun = 0, stat = 0;
                bool ok = dummy.GetMotorControllerStatus(node, &reqMode, &modeRun, &stat);
                if (ok)
                    Respond(_responseChannel, "ok MOTOR[%lu] MODE_REQ[%d]|MODE_RUN[%d]|STATE[%d]",
                            node, reqMode, modeRun, stat);
                else
                    Respond(_responseChannel, "error MOTOR[%lu] TIMEOUT", node);
            } else {
                Respond(_responseChannel, "error GET_STATUS requires nodeId 1..6");
            }
        } else if (strstr(_cmd, "GET_DCE_PARAMS") != nullptr)
        {
            // Synchronous query for DCE Parameters (CAN 0x31/0x32): kp, kv, ki, kd
            // Example: #GET_DCE_PARAMS 1 → ok kp=12345|kv=678|ki=90|kd=12
            uint32_t node;
            if (sscanf(_cmd, "#GET_DCE_PARAMS %lu", &node) == 1 && node >= 1 && node <= 6) {
                int32_t kp = 0, kv = 0, ki = 0, kd = 0;
                bool ok = dummy.GetMotorDceParameters(node, &kp, &kv, &ki, &kd);
                if (ok)
                    Respond(_responseChannel, "ok MOTOR[%lu] KP[%d]|KV[%d]|KI[%d]|KD[%d]",
                            node, kp, kv, ki, kd);
                else
                    Respond(_responseChannel, "error MOTOR[%lu] TIMEOUT", node);
            } else {
                Respond(_responseChannel, "error GET_DCE_PARAMS requires nodeId 1..6");
            }
        } else if (strstr(_cmd, "SET_DCE_KP") != nullptr)
        {
            uint32_t kp;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KP %lu %lu", &node, &kp);
            if (node >= 1 & node <= 6){
                dummy.motorJ[node]->SetDceKp(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KP [%lu]", node, kp);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KP [%lu] is wrong", node, kp);
            }
        } else if (strstr(_cmd, "SET_DCE_KV") != nullptr)
        {
            uint32_t kv;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KV %lu %lu", &node, &kv);
            if (node >= 1 & node <= 6){
                dummy.motorJ[node]->SetDceKv(kv);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KV [%lu]", node, kv);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KV [%lu] is wrong", node, kv);
            }
        } else if (strstr(_cmd, "SET_DCE_KI") != nullptr)
        {
            uint32_t kp;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KI %lu %lu", &node, &kp);
            if (node >= 1 & node <= 6){
                dummy.motorJ[node]->SetDceKi(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KI [%lu]", node, kp);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KI [%lu] is wrong", node, kp);
            }
        } else if (strstr(_cmd, "SET_DCE_KD") != nullptr)
        {
            uint32_t kp;
            uint32_t node;
            sscanf(_cmd, "#SET_DCE_KD %lu %lu", &node, &kp);
            if (node >= 1 & node <= 6){
                dummy.motorJ[node]->SetDceKd(kp);
                Respond(_responseChannel, "ok SET MOTOR [%lu] DCE_KD [%lu]", node, kp);
            }
            else {
                Respond(_responseChannel, "error SET MOTOR [%lu] DCE_KD [%lu] is wrong", node, kp);
            }
        } else if (strstr(_cmd, "REBOOT") != nullptr)
        {
            uint32_t node;
            sscanf(_cmd, "#REBOOT %lu", &node);
            if (node >= 1 & node <= 6){
                dummy.motorJ[node]->Reboot();
                Respond(_responseChannel, "ok REBOOT MOTOR [%lu]", node);
            }
            else {
                Respond(_responseChannel, "error REBOOT MOTOR [%lu] is wrong", node);
            }
        }else if (strstr(_cmd, "CMDMODE") != nullptr)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "ok Set command mode to [%lu]", mode);
        } else if (strstr(_cmd, "SET_SPEED_FACTOR") != nullptr)
        {
            float unit;
            sscanf(_cmd, "#SET_SPEED_FACTOR %f", &unit);
            dummy.SetJointSpeedFactor(unit);
            Respond(_responseChannel, "ok SET SPEED_UNIT_TO_RPS [%.3f]", unit);
        } else if (strstr(_cmd, "SET_ACC_Percent") != nullptr)
        {
            float factor;
            sscanf(_cmd, "#SET_ACC_Percent %f", &factor);
            dummy.SetJointAccelerationPercent(factor);
            dummy.ApplyJointAcceleration();
            Respond(_responseChannel, "ok SET ACC_Percent [%.2f]", factor);
        } else if (strstr(_cmd, "SET_ACC_BASE") != nullptr)
        {
            float b[6];
            int n = sscanf(_cmd, "#SET_ACC_BASE %f %f %f %f %f %f",
                           &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]);
            if (n == 6)
            {
                dummy.SetJointAccelerationBases(b[0], b[1], b[2], b[3], b[4], b[5]);
                dummy.ApplyJointAcceleration();
                Respond(_responseChannel, "ok SET ACC_BASE [%.2f %.2f %.2f %.2f %.2f %.2f]",
                        b[0], b[1], b[2], b[3], b[4], b[5]);
            } else
                Respond(_responseChannel, "error SET_ACC_BASE needs 6 args, got %d", n);
        } else
            Respond(_responseChannel, "ok");
    } else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&')
    {
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        if (freeSize == 0xFF)
            Respond(_responseChannel, "error queue full");
        else
            Respond(_responseChannel, "ok queued free=%lu", (unsigned long) freeSize);
    }

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!' || !dummy.IsEnabled())
    {
        // NOTE: keep this hot path heap-free (no std::string), see OnUsbAsciiCmd.
        if (strstr(_cmd, "STOP") != nullptr)
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        } else if (strstr(_cmd, "START") != nullptr)
        {
            dummy.SetEnable(true);
            Respond(_responseChannel, "Started ok");
        } else if (strstr(_cmd, "HOME") != nullptr)
        {
            dummy.Homing();
            Respond(_responseChannel, "Started ok");
        } else if (strstr(_cmd, "CALIBRATION") != nullptr)
        {
            dummy.CalibrateHomeOffset();
            Respond(_responseChannel, "calibration ok");
        } else if (strstr(_cmd, "RESET") != nullptr)
        {
            dummy.Resting();
            Respond(_responseChannel, "Started ok");
        } else if (strstr(_cmd, "DISABLE") != nullptr)
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
        }
    } else if (_cmd[0] == '#')
    {
        if (strstr(_cmd, "GETJPOS") != nullptr)
        {
            // Fixed-point integer formatting (2 decimals), no dtoa/malloc -- see FormatFixedN.
            char s[6][16];
            for (int k = 0; k < 6; k++)
                FormatFixedN(dummy.currentJoints.a[k], s[k], sizeof(s[k]), 2);
            Respond(_responseChannel, "ok %s %s %s %s %s %s",
                    s[0], s[1], s[2], s[3], s[4], s[5]);
        } else if (strstr(_cmd, "GET_STATUS") != nullptr)
        {
            uint32_t node;
            if (sscanf(_cmd, "#GET_STATUS %lu", &node) == 1 && node >= 1 && node <= 6) {
                uint8_t reqMode = 0, modeRun = 0, stat = 0;
                bool ok = dummy.GetMotorControllerStatus(node, &reqMode, &modeRun, &stat);
                if (ok)
                    Respond(_responseChannel, "ok MOTOR[%lu] MODE_REQ[%d]|MODE_RUN[%d]|STATE[%d]",
                            node, reqMode, modeRun, stat);
                else
                    Respond(_responseChannel, "error MOTOR[%lu] TIMEOUT", node);
            } else {
                Respond(_responseChannel, "error GET_STATUS requires nodeId 1..6");
            }
        } else if (strstr(_cmd, "GETLPOS") != nullptr)
        {
            dummy.UpdateJointPose6D();
            // Fixed-point integer formatting (2 decimals), no dtoa/malloc -- see FormatFixedN.
            const float pose[6] = {dummy.currentPose6D.X, dummy.currentPose6D.Y,
                                   dummy.currentPose6D.Z, dummy.currentPose6D.A,
                                   dummy.currentPose6D.B, dummy.currentPose6D.C};
            char s[6][16];
            for (int k = 0; k < 6; k++)
                FormatFixedN(pose[k], s[k], sizeof(s[k]), 2);
            Respond(_responseChannel, "ok %s %s %s %s %s %s",
                    s[0], s[1], s[2], s[3], s[4], s[5]);
        } else if (strstr(_cmd, "CMDMODE") != nullptr)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "Set command mode to [%lu]", mode);
        } else
            Respond(_responseChannel, "ok");
    } else if (_cmd[0] == '>' || _cmd[0] == '@' || _cmd[0] == '&')
    {
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        if (freeSize == 0xFF)
            Respond(_responseChannel, "error queue full");
        else
            Respond(_responseChannel, "ok queued free=%lu", (unsigned long) freeSize);
    }
/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}
