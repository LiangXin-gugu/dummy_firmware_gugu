#include "common_inc.h"
#include <cstring>

extern DummyRobot dummy;

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
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
        } else if (strstr(_cmd, "GETLPOS") != nullptr)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
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
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
        } else if (strstr(_cmd, "GETLPOS") != nullptr)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
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
