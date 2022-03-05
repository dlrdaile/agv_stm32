/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/24 下午7:59
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_MOTOR_H
#define PROJECT_CAR_MOTOR_H

#include "Can.h"

enum {
    FL_motor = 0,
    FR_motor,
    BL_motor,
    BR_motor,
};
typedef struct {
    uint32_t battery_votage;
    uint16_t FL_speed[4];
    uint32_t oneSecond_encoder[4];
} CarState_TypeDef;


const uint32_t sysReset_msgID = Generate_msgID(0x19, 0);
const uint32_t mvDirection_msgID = Generate_msgID(0x2A, 0);
const uint32_t battery_msgID = Generate_msgID(0x33, 0);
const uint32_t setSpeed_msgID = Generate_msgID(0x29, 0);
const uint32_t oneMS_Encoder_msgID  = Generate_msgID(0x30,0);
const uint32_t EncoderData_msgID  = Generate_msgID(0x31,0);
const uint32_t ClearEncoder_msgID  = Generate_msgID(0x32,0);

class Motor {
public:
    Motor(CAN_HandleTypeDef &hcan);

    ~Motor();

    HAL_StatusTypeDef setSpeed(uint16_t FL = 0, uint16_t FR = 0, uint16_t BL = 0, uint16_t BR = 0);

    HAL_StatusTypeDef motion_system_reset();

    HAL_StatusTypeDef swerve_motion(uint16_t radius, uint16_t speed);

    HAL_StatusTypeDef XY_motion(uint16_t speed_x, uint16_t speed_y);

    HAL_StatusTypeDef rotate_motion(uint16_t rotate_speed);

    HAL_StatusTypeDef check_battery();
    HAL_StatusTypeDef check_oneMs_encoder();

    uint32_t show_battery();

    uint16_t *show_speed();

    HAL_StatusTypeDef stop();

    HAL_StatusTypeDef InitState();

private:

    HAL_StatusTypeDef verifyReceive(const CanStatusTypeDef &transtatus, const uint32_t &ExtID);

public:
    CarState_TypeDef motor_state;
    uint8_t CanRxBuffer[16];
private:
    Can *mCan;
    CanStatusTypeDef result;
};


#endif //PROJECT_CAR_MOTOR_H