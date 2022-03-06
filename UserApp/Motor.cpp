/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/24 下午7:59
* @version: 1.0
* @description: 
********************************************************************************/
#include "Motor.h"
#include "command_length.h"
#include "Usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "UserConfig.h"

#if JLINK_DEBUG == 1

#include "SEGGER_RTT.h"

#endif

#include "ros.h"
#include "memory.h"
#include "cstdio"

#define taskdelaytick 3
extern ros::NodeHandle nh;

HAL_StatusTypeDef Motor::verifyReceive(const CanStatusTypeDef &transtatus, const uint32_t &ExtID) {
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
    }
    name += " is error because: ";
    if (transtatus == CAN_OK) {
        switch (this->mCan->CAN_ReadMsg(ExtID, this->CanRxBuffer)) {
            case CAN_OK:
                return HAL_OK;
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
    } else {
        name += "transmit data error";
    }
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, name.c_str());
#endif
    nh.logwarn(name.c_str());
    return HAL_ERROR;
}

Motor::Motor(CAN_HandleTypeDef &hcan) {
    this->mCan = new Can(hcan);
}

Motor::~Motor() {
    if (this->mCan != NULL) {
        delete this->mCan;
        this->mCan = NULL;
    }
}

HAL_StatusTypeDef Motor::setSpeed(int16_t FL, int16_t FR, int16_t BL, int16_t BR) {
    int16_t speed[4] = {FL, FR, BL, BR};
    result = this->mCan->CAN_SendMsg(setSpeed_msgID, (uint8_t *) speed, DLSEND_41);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the setSpeed msgID is:0x%08x\n", setSpeed_msgID);
#endif
    HAL_Delay(taskdelaytick);
    if (this->verifyReceive(result, setSpeed_msgID) != HAL_OK) {
        return HAL_ERROR;
    }
    string temp = "The original speed is [";
    string temp2 = "The current speed has been set to [";
    for (int i = 0; i < 4; ++i) {
        temp += (to_string(this->motor_state.setted_speed[i]) + "  ");
        temp2 += (to_string(speed[i]) + "  ");
        this->motor_state.setted_speed[i] = speed[i];
    }
    nh.loginfo(temp.c_str());
    nh.loginfo(temp2.c_str());
    return HAL_OK;
}

HAL_StatusTypeDef Motor::motion_system_reset() {
    result = this->mCan->CAN_SendMsg(sysReset_msgID, nullptr, DLSEND_25);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the reset msgID is:0x%08x\n", sysReset_msgID);
#endif
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, sysReset_msgID);
}

HAL_StatusTypeDef Motor::XY_motion(int16_t speed_x, int16_t speed_y) {
    int16_t raw_data[4] = {0, 0, speed_y, speed_x};
    result = mCan->CAN_SendMsg(mvDirection_msgID, (uint8_t *) raw_data, DLSEND_42);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, mvDirection_msgID);
}

HAL_StatusTypeDef Motor::swerve_motion(int16_t radius, int16_t speed) {
    if (radius == 0) return HAL_ERROR;
    int16_t raw_data[4] = {0, (int16_t) (speed / radius), 0, speed};
    result = mCan->CAN_SendMsg(mvDirection_msgID, (uint8_t *) raw_data, DLSEND_42);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, mvDirection_msgID);
}


HAL_StatusTypeDef Motor::rotate_motion(int16_t rotate_speed) {
    int16_t raw_data[4] = {0, rotate_speed, 0, 0};
    result = mCan->CAN_SendMsg(mvDirection_msgID, (uint8_t *) raw_data, DLSEND_42);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, mvDirection_msgID);
}

HAL_StatusTypeDef Motor::update_battery() {
    result = mCan->CAN_SendMsg(battery_msgID, nullptr, DLSEND_51);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the battery msgID is:0x%08x\n", battery_msgID);
#endif
    HAL_Delay(taskdelaytick);
    if (HAL_OK == this->verifyReceive(result, battery_msgID)) {
        this->motor_state.battery_votage = *((uint32_t *) this->CanRxBuffer);
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::update_oneMs_encoder() {
    result = mCan->CAN_SendMsg(oneMS_Encoder_msgID, NULL, DLSEND_48);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the update_oneMs_encoder msgID is:0x%08x\n", oneMS_Encoder_msgID);
#endif
    HAL_Delay(taskdelaytick + 1);
    if (HAL_OK == this->verifyReceive(result, oneMS_Encoder_msgID)) {
        for (int i = 0; i < 4; ++i) {
            this->motor_state.oneMs_encoder[i] = (int8_t) (((uint32_t *) this->CanRxBuffer)[i]);
        }
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::stop() {
    if (HAL_OK == this->setSpeed(0, 0, 0, 0)) {
        nh.loginfo("Stop success!");
        return HAL_OK;
    } else {
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef Motor::InitState() {
    if ((this->mCan->CAN_Init() != HAL_OK)
        || (this->motion_system_reset() != HAL_OK)
        || (this->stop() != HAL_OK)
        || (this->clear_encoder() != HAL_OK)) {
        return HAL_ERROR;
    }
    for (int i = 0; i < 4; ++i) {
        this->motor_state.setted_speed[i] = 0;
        this->motor_state.oneMs_encoder[i] = 0;
        this->motor_state.last_encData.encoder_data[i] = 0;
        this->motor_state.current_encData.encoder_data[i] = 0;
        this->motor_state.current_encData.check_time = xTaskGetTickCount();
    }
    if (HAL_OK == this->update_battery()) {
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::update_encoderdata() {
    result = mCan->CAN_SendMsg(EncoderData_msgID, NULL, DLSEND_48);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the update_oneMs_encoder msgID is:0x%08x\n", EncoderData_msgID);
#endif
    HAL_Delay(taskdelaytick + 1);
    if (HAL_OK == this->verifyReceive(result, EncoderData_msgID)) {
        this->motor_state.last_encData.check_time = this->motor_state.current_encData.check_time;
        this->motor_state.current_encData.check_time = xTaskGetTickCount();
        memcpy(this->motor_state.last_encData.encoder_data, this->motor_state.current_encData.encoder_data, 16);
        memcpy(this->motor_state.current_encData.encoder_data, this->CanRxBuffer, 16);
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::clear_encoder() {
    result = this->mCan->CAN_SendMsg(ClearEncoder_msgID, nullptr, DLSEND_25);
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "the reset msgID is:0x%08x\n", ClearEncoder_msgID);
#endif
    HAL_Delay(taskdelaytick);
    if (HAL_OK == this->verifyReceive(result, ClearEncoder_msgID)) {
        this->motor_state.current_encData.check_time = xTaskGetTickCount();
        this->motor_state.last_encData.check_time = this->motor_state.current_encData.check_time;
        memset(this->motor_state.current_encData.encoder_data, 0, 16);
        memset(this->motor_state.last_encData.encoder_data, 0, 16);
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::run_cmd(const uint8_t &cmd, uint16_t *TxData) {
    HAL_StatusTypeDef cmd_result;
    string temp;
    switch (cmd) {
        case cmd_sysReset:
            cmd_result = this->motion_system_reset();
            if (cmd_result == HAL_OK) {
                temp = "the system has been reset!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_setSpeed:
            cmd_result = this->setSpeed(TxData[0], TxData[1], TxData[2], TxData[3]);
            break;
        case cmd_Stop:
            cmd_result = this->stop();
            break;
        case cmd_updateBattery:
            cmd_result = this->update_battery();
            if (cmd_result == HAL_OK) {
                temp = "the Battery value has been update!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_updateEncoderData:
            cmd_result = this->update_encoderdata();
            if (cmd_result == HAL_OK) {
                temp = "the Encoder data has been update!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_updateOneMsEncoder:
            cmd_result = this->update_oneMs_encoder();
            if (cmd_result == HAL_OK) {
                temp = "the OneMsEncoder has been update!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_clearEncoder:
            cmd_result = this->clear_encoder();
            if (cmd_result == HAL_OK) {
                temp = "the Encoder has been clear!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_xyMotion:
            cmd_result = this->XY_motion(TxData[0], TxData[1]);
            if (cmd_result == HAL_OK) {
                temp = "the XYmotion is set successfully!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_swerveMotion:
            cmd_result = this->swerve_motion(TxData[0], TxData[1]);
            if (cmd_result == HAL_OK) {
                temp = "the swerveMotion is set successfully!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_rotateMotion:
            cmd_result = this->rotate_motion(TxData[0]);
            if (cmd_result == HAL_OK) {
                temp = "the rotateMotion is set successfully!";
                nh.loginfo(temp.c_str());
            }
            break;
        case cmd_checkAveSpeed:
            break;
        case cmd_checkInsSpeed:
            break;
        case cmd_checkEncoderData:
            break;
        case cmd_checkOneMsEncoder:
            break;
        default:
            nh.logwarn("undefined cmd!");
            cmd_result = HAL_ERROR;
            break;
    }
    return cmd_result;
}

const char *select_name(uint8_t &cmd) {
    switch (cmd) {
        case cmd_sysReset:
            return "cmd_sysReset";
        case cmd_setSpeed:
            return "setSpeed";
        case cmd_Stop:
            return "Stop";
        case cmd_updateBattery:
            return "updateBattery";
        case cmd_updateEncoderData:
            return "updateEncoderData";
        case cmd_updateOneMsEncoder:
            return "updateOneMsEncoder";
        case cmd_clearEncoder:
            return "clearEncoder";
        case cmd_xyMotion:
            return "xyMotion";
        case cmd_swerveMotion:
            return "swerveMotion";
        case cmd_rotateMotion:
            return "rotateMotion";
        case cmd_checkAveSpeed:
            return "checkAveSpeed";
        case cmd_checkInsSpeed:
            return "checkInsSpeed";
        case cmd_checkEncoderData:
            return "checkEncoderData";
        case cmd_checkOneMsEncoder:
            return "checkOneMsEncoder";
        default:
            return "error";
    }


}






