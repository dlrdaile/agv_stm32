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

#define taskdelaytick 5
extern Usart debug_uart;

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

HAL_StatusTypeDef Motor::setSpeed(uint16_t FL, uint16_t FR, uint16_t BL, uint16_t BR) {
    uint16_t speed[4] = {FL, FR, BL, BR};
    result = this->mCan->CAN_SendMsg(setSpeed_msgID, (uint8_t *) speed, DLSEND_41);
    SEGGER_RTT_printf(0, "the setSpeed msgID is:0x%08x\n", setSpeed_msgID);
    HAL_Delay(taskdelaytick);
    if (this->verifyReceive(result, setSpeed_msgID) != HAL_OK) {
        return HAL_ERROR;

    }
    for (int i = 0; i < 4; ++i) {
        this->motor_state.FL_speed[i] = speed[i];
    }
    return HAL_OK;
}

HAL_StatusTypeDef Motor::motion_system_reset() {
    uint8_t *null_char = 0;
    result = this->mCan->CAN_SendMsg(sysReset_msgID, null_char, DLSEND_25);
    SEGGER_RTT_printf(0, "the reset msgID is:0x%08x\n", sysReset_msgID);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, sysReset_msgID);
}

HAL_StatusTypeDef Motor::XY_motion(uint16_t speed_x, uint16_t speed_y) {
    uint16_t raw_data[4] = {0, 0, speed_y, speed_x};
    result = mCan->CAN_SendMsg(mvDirection_msgID, (uint8_t *) raw_data, DLSEND_42);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, mvDirection_msgID);
}

HAL_StatusTypeDef Motor::swerve_motion(uint16_t radius, uint16_t speed) {
    if (radius == 0) return HAL_ERROR;
    uint16_t raw_data[4] = {0, (uint16_t) (speed / radius), 0, speed};
    result = mCan->CAN_SendMsg(mvDirection_msgID, (uint8_t *) raw_data, DLSEND_42);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, mvDirection_msgID);
}

HAL_StatusTypeDef Motor::rotate_motion(uint16_t rotate_speed) {
    uint16_t raw_data[4] = {0, rotate_speed, 0, 0};
    result = mCan->CAN_SendMsg(mvDirection_msgID, (uint8_t *) raw_data, DLSEND_42);
    HAL_Delay(taskdelaytick);
    return this->verifyReceive(result, mvDirection_msgID);
}

HAL_StatusTypeDef Motor::check_battery() {
    result = mCan->CAN_SendMsg(battery_msgID, NULL, DLSEND_51);
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
            this->motor_state.oneSecond_encoder[i] = ((uint32_t *) this->CanRxBuffer)[i];
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
        || (this->stop() != HAL_OK)) {
        return HAL_ERROR;
    }
    for (int i = 0; i < 4; ++i) {
        this->motor_state.FL_speed[i] = 0;
        this->motor_state.oneSecond_encoder[i] = 0;
    }
    if (HAL_OK == this->check_battery()) {
        return HAL_OK;
    } else
        return HAL_ERROR;
}

uint32_t Motor::show_battery() {
    return this->motor_state.battery_votage;
}

uint16_t *Motor::show_speed() {
    return this->motor_state.FL_speed;
}




