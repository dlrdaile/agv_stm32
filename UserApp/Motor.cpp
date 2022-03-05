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
#include "SEGGER_RTT.h"
#include "ros.h"

#define taskdelaytick 5
extern ros::NodeHandle nh;
HAL_StatusTypeDef Motor::verifyReceive(const CanStatusTypeDef &transtatus, const uint32_t &ExtID) {
    string name;
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
    if (transtatus == CAN_OK) {
        switch (this->mCan->CAN_ReadMsg(ExtID, this->CanRxBuffer)) {
            case CAN_OK:
                return HAL_OK;
            case CAN_GET_MESSAGE_ERROR:
                SEGGER_RTT_printf(0, "%s is error because:%s", name.c_str(), "CAN_GET_MESSAGE_ERROR");
                break;
            case CAN_RECEIVE_ERROR:
                SEGGER_RTT_printf(0, "%s is error because:%s", name.c_str(), "CAN_RECEIVE_ERROR");
                break;
            case CAN_NO_RECEIVE_ERROR:
                SEGGER_RTT_printf(0, "%s is error because:%s", name.c_str(), "CAN_NO_RECEIVE_ERROR");
                break;
            default:
                SEGGER_RTT_printf(0, "%s is error because:%s", name.c_str(), "no known the reason");
                break;
        }
    } else {
        SEGGER_RTT_printf(0, "%s is error because:%s", name.c_str(), "transmit data error");
    }
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
    SEGGER_RTT_printf(0, "the setSpeed msgID is:0x%08x\n", setSpeed_msgID);
    HAL_Delay(taskdelaytick);
    if (this->verifyReceive(result, setSpeed_msgID) != HAL_OK) {
        return HAL_ERROR;
    }
    for (int i = 0; i < 4; ++i) {
        this->motor_state.setted_speed[i] = speed[i];
    }
    return HAL_OK;
}

HAL_StatusTypeDef Motor::motion_system_reset() {
    result = this->mCan->CAN_SendMsg(sysReset_msgID, nullptr, DLSEND_25);
    SEGGER_RTT_printf(0, "the reset msgID is:0x%08x\n", sysReset_msgID);
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

HAL_StatusTypeDef Motor::check_battery() {
    result = mCan->CAN_SendMsg(battery_msgID, nullptr, DLSEND_51);
    SEGGER_RTT_printf(0, "the battery msgID is:0x%08x\n", battery_msgID);
    HAL_Delay(taskdelaytick);
    if (HAL_OK == this->verifyReceive(result, battery_msgID)) {
        this->motor_state.battery_votage = *((uint32_t *) this->CanRxBuffer);
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::check_oneMs_encoder() {
    result = mCan->CAN_SendMsg(oneMS_Encoder_msgID, NULL, DLSEND_48);
    SEGGER_RTT_printf(0, "the check_oneMs_encoder msgID is:0x%08x\n", oneMS_Encoder_msgID);
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
    this->last_tick = 0;
    for (int i = 0; i < 4; ++i) {
        this->motor_state.setted_speed[i] = 0;
        this->motor_state.oneMs_encoder[i] = 0;
        this->motor_state.fact_speed[i] = 0;
    }
    if (HAL_OK == this->check_battery()) {
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::check_encoderdata() {
    result = mCan->CAN_SendMsg(EncoderData_msgID, NULL, DLSEND_48);
    SEGGER_RTT_printf(0, "the check_oneMs_encoder msgID is:0x%08x\n", EncoderData_msgID);
    TickType_t current = xTaskGetTickCount();
    HAL_Delay(taskdelaytick + 1);
    if (HAL_OK == this->verifyReceive(result, EncoderData_msgID)) {
        //TODO:接收到编码器信息后如何组织数据
        for (int i = 0; i < 4; ++i) {
            int32_t temp = ((int32_t *) this->CanRxBuffer)[i];
            this->motor_state.fact_speed[i] =
                    (temp - this->motor_state.encoder_data[i]) / 4.096 / (current - this->last_tick);
            this->motor_state.encoder_data[i] = temp;
        }
        this->last_tick = current;
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::clear_encoder() {
    result = this->mCan->CAN_SendMsg(ClearEncoder_msgID, nullptr, DLSEND_25);
    SEGGER_RTT_printf(0, "the reset msgID is:0x%08x\n", ClearEncoder_msgID);
    HAL_Delay(taskdelaytick);
    if (HAL_OK == this->verifyReceive(result, ClearEncoder_msgID)) {
        for (long &i: this->motor_state.encoder_data) {
            i = 0;
        }
        return HAL_OK;
    } else
        return HAL_ERROR;
}

HAL_StatusTypeDef Motor::run_cmd(const uint8_t &cmd, uint16_t *TxData) {
    HAL_StatusTypeDef cmd_result;
    switch (cmd) {
        case cmd_sysReset:
            cmd_result = this->motion_system_reset();
            break;
        case cmd_setSpeed:
            cmd_result = this->setSpeed(TxData[0],TxData[1],TxData[2],TxData[3]);
            break;
        case cmd_Stop:
            cmd_result = this->stop();
            break;
        case cmd_checkBattery:
            cmd_result = this->check_battery();
            break;
        case cmd_checkEncoderData:
            cmd_result = this->check_encoderdata();
            break;
        case cmd_checkonemsEncoder:
            cmd_result = this->check_oneMs_encoder();
            break;
        case cmd_clearEncoder:
            cmd_result = this->clear_encoder();
            break;
        case cmd_xyMotion:
            cmd_result = this->XY_motion(TxData[0],TxData[1]);
            break;
        case cmd_swerveMotion:
            cmd_result = this->swerve_motion(TxData[0],TxData[1]);
            break;
        case cmd_rotateMotion:
            cmd_result = this->rotate_motion(TxData[0]);
            break;
        default:
            vPortEnterCritical();
            nh.logwarn("undefined cmd!");
            vPortExitCritical();
            cmd_result = HAL_ERROR;
            break;
    }
    return cmd_result;
}







