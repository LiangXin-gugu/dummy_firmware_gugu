#include "common_inc.h"

// Used for response CAN message.
static CAN_TxHeaderTypeDef txHeader =
    {
        .StdId = 0,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };

extern DummyRobot dummy;

void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data)
{
    // Common CAN message callback, uses ID 32~0x7FF.
    if (canCtx->handle->Instance == CAN1)
    {
        uint8_t id = rxHeader->StdId >> 7; // 4Bits ID & 7Bits Msg
        uint8_t cmd = rxHeader->StdId & 0x7F; // 4Bits ID & 7Bits Msg

        /*----------------------- ↓ Add Your CAN1 Packet Protocol Here ↓ ------------------------*/
        // Guard: motorJ[] is sized 7 (index 0..6, motorJ[0] is the broadcast pseudo-node).
        // Any id >= 7 (e.g. the hand at id=7) or unexpected frames must NOT dereference motorJ[id].
        const bool validMotorId = (id >= 1 && id <= 6);
        switch (cmd)
        {
            case 0x21: // Get Current ACK: [0..3]=float amps, [4]=finished flag
                if (validMotorId)
                    memcpy(&dummy.motorJ[id]->current, data, sizeof(float));
                break;
            case 0x23:
                if (validMotorId)
                    dummy.motorJ[id]->UpdateAngleCallback(*(float*) (data), data[4]);
                break;
            case 0x25: // Get Temperature ACK: [0..3]=float deg C
                if (validMotorId)
                    memcpy(&dummy.motorJ[id]->temperature, data, sizeof(float));
                break;
            case 0x30: // Get Controller Status ACK: [0]=requestMode, [1]=modeRunning, [2]=state
                if (validMotorId) {
                    {
                        Respond(*usbStreamOutputPtr, "[CAN-0x30] Motor[%lu] ID=0x%X [Data=%d,%d,%d]", 
                               id, rxHeader->StdId, data[0], data[1], data[2]);
                    }
                    
                    // Clear the flag first, then update data atomically
                    dummy.motorJ[id]->statusReceived = false;
                    dummy.motorJ[id]->statusRequestMode = data[0];
                    dummy.motorJ[id]->statusModeRunning = data[1];
                    dummy.motorJ[id]->statusState = data[2];
                    // Set flag to indicate fresh ACK received
                    dummy.motorJ[id]->statusReceived = true;
                }
                break;
            case 0x31: // Get DCE Parameters Low ACK: [0..3]=kp, [4..7]=kv
                if (validMotorId) {
                    int32_t kp = *(int32_t*)data;
                    int32_t kv = *(int32_t*)(data + 4);
                    {
                        Respond(*usbStreamOutputPtr, "[CAN-0x31] Motor[%lu] ID=0x%X [kp=%d, kv=%d]", 
                               id, rxHeader->StdId, kp, kv);
                    }
                    
                    // Clear flags first, then update data atomically
                    dummy.motorJ[id]->dceParamsLowReceived = false;
                    dummy.motorJ[id]->dceKp = kp;
                    dummy.motorJ[id]->dceKv = kv;
                    // Set flag to indicate fresh ACK received
                    dummy.motorJ[id]->dceParamsLowReceived = true;
                }
                break;
            case 0x32: // Get DCE Parameters High ACK: [0..3]=ki, [4..7]=kd
                if (validMotorId) {
                    int32_t ki = *(int32_t*)data;
                    int32_t kd = *(int32_t*)(data + 4);
                    {
                        Respond(*usbStreamOutputPtr, "[CAN-0x32] Motor[%lu] ID=0x%X [ki=%d, kd=%d]", 
                               id, rxHeader->StdId, ki, kd);
                    }
                    
                    // Clear flags first, then update data atomically
                    dummy.motorJ[id]->dceParamsHighReceived = false;
                    dummy.motorJ[id]->dceKi = ki;
                    dummy.motorJ[id]->dceKd = kd;
                    // Set flag to indicate fresh ACK received
                    dummy.motorJ[id]->dceParamsHighReceived = true;
                }
                break;
            default:
                break;
        }

        dummy.UpdateJointAnglesCallback();

    } else if (canCtx->handle->Instance == CAN2)
    {
        /*----------------------- ↓ Add Your CAN2 Packet Protocol Here ↓ ------------------------*/
    }
    /*----------------------- ↑ Add Your Packet Protocol Here ↑ ------------------------*/
}