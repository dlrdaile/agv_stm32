/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/4/12 下午9:32
* @version: 1.0
* @description: 
********************************************************************************/
#include "Motor2Can.h"
#include "ros.h"
#include "string"
#include "cmsis_os.h"
#include "UserConfig.h"
#include "SEGGER_RTT.h"
extern ros::NodeHandle nh;

static inline string chIDName(const uint32_t &ExtID) {
    string name;
    name.resize(50);
    switch (ExtID) {
        case setSpeed_msgID:
            name = "setSpeed";
            break;
        case sysReset_msgID:
            name = "sysReset";
            break;
        case mvDirection_msgID:
            name = "mvDirection";
            break;
        case battery_msgID:
            name = "battery";
            break;
        case oneMS_Encoder_msgID:
            name = "oneMS";
        case EncoderData_msgID:
            name = "encoder";
    }
    name += " is error because: ";
    return name;
}

HAL_StatusTypeDef Motor2Can::verifyReceive(const CanStatusTypeDef &transtatus, const uint32_t &ExtID) {
    if (transtatus == CAN_OK) {
        CanStatusTypeDef can_receive_result = this->mCan->CAN_ReadMsg(ExtID, this->CanRxBuffer);
        if (can_receive_result == CAN_OK) {
            return HAL_OK;
        } else {
            string name = chIDName(ExtID);
            switch (can_receive_result) {
                case CAN_GET_MESSAGE_ERROR:
                    name += "CAN_GET_MESSAGE_ERROR";
                    break;
                case CAN_RECEIVE_ERROR:
                    name += "CAN_RECEIVE_ERROR";
                    break;
                case CAN_NO_RECEIVE_ERROR:
                    name += "CAN_NO_RECEIVE_ERROR";
                    break;
                default:
                    name += "no known the reason";
                    break;
            }
            nh.logwarn(name.c_str());
        }

    } else {
        string name = chIDName(ExtID);
        name += "transmit data error";
        nh.logwarn(name.c_str());
    }
    return HAL_ERROR;
}

Motor2Can::Motor2Can(CAN_HandleTypeDef &hcan) {
    this->mCan = new Can(hcan);
}

Motor2Can::~Motor2Can() {
    if (this->mCan != NULL) {
        delete this->mCan;
        this->mCan = NULL;
    }
}

HAL_StatusTypeDef Motor2Can::setSpeed(int16_t v1, int16_t v2, int16_t v3, int16_t v4) {
    int16_t speed[4] = {v1, v2, v3, v4};
    speed[2] *= -1;
    speed[1] *= -1;
    result = this->mCan->CAN_SendMsg(setSpeed_msgID, (uint8_t *) speed, DLSEND_41);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the setSpeed msgID is:0x%08x\n", setSpeed_msgID);
#endif
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, setSpeed_msgID);
}

HAL_StatusTypeDef Motor2Can::motion_system_reset() {
    result = this->mCan->CAN_SendMsg(sysReset_msgID, nullptr, DLSEND_25);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the reset msgID is:0x%08x\n", sysReset_msgID);
#endif
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, sysReset_msgID);
}

HAL_StatusTypeDef Motor2Can::directe_motion(int16_t rotate_speed, int16_t speed_x, int16_t speed_y) {
    speed_y *= -1;
    int16_t raw_data[4] = {speed_y, speed_x, rotate_speed, 0};
    result = mCan->CAN_SendMsg(mvDirection_msgID, (uint8_t *) raw_data, DLSEND_42);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, mvDirection_msgID);
}

HAL_StatusTypeDef Motor2Can::update_battery() {
    result = mCan->CAN_SendMsg(battery_msgID, nullptr, DLSEND_51);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the battery msgID is:0x%08x\n", battery_msgID);
#endif
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, battery_msgID);
}

HAL_StatusTypeDef Motor2Can::update_encoderdata() {
    result = mCan->CAN_SendMsg(EncoderData_msgID, NULL, DLSEND_48);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the update_encoderdata msgID is:0x%08x\n", EncoderData_msgID);
#endif
    HAL_Delay(encoder_taskdelay);
    return this->verifyReceive(result, EncoderData_msgID);

}

HAL_StatusTypeDef Motor2Can::clear_encoder() {
    result = this->mCan->CAN_SendMsg(ClearEncoder_msgID, nullptr, DLSEND_25);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the clear_encoder msgID is:0x%08x\n", ClearEncoder_msgID);
#endif
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, ClearEncoder_msgID);

}

HAL_StatusTypeDef Motor2Can::stop() {
    return this->setSpeed(0, 0, 0, 0);
}
