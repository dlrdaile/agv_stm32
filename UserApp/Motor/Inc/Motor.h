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
#include "FreeRTOS.h"
#include "task.h"
#include "string"
#include "Motor2Can.h"
#include "communicate_with_stm32/MotorControl.h"
#include "communicate_with_stm32/InitConfig.h"
typedef enum {
    cmd_sysReset = 0,
    cmd_setSpeed = 1,
    cmd_Stop = 2,
    cmd_updateBattery = 3,
    cmd_updateEncoderData = 4,
    cmd_clearEncoder = 5,
    cmd_directeMotion = 6
} topic_cmd_set;
//10
typedef enum {
    srv_startupBattery = 13,
    srv_startupEncoder = 14,
    srv_startPowerCalc = 15,
    srv_setPowerZero = 16,
} server_cmd_set;

typedef struct {
    Encoderinfo encoder_info;
    BatteryInfo battery_info;
    float set_speed[4];
} CarState_TypeDef;

const char *select_name(uint8_t &cmd);

class Motor {
public:
    Motor(CAN_HandleTypeDef &hcan);

    ~Motor();

    /**
     * @brief 对小车进行初始化设置
     * @return
     */
    HAL_StatusTypeDef InitState();

    HAL_StatusTypeDef topic_cmd(const uint8_t &cmd,const int16_t *TxData = NULL);

    HAL_StatusTypeDef server_cmd(const communicate_with_stm32::MotorControl::Request &req,
                                 communicate_with_stm32::MotorControl::Response &res);


public:
    CarState_TypeDef motor_state;
    InitConfig encoderCtrl;
    InitConfig batteryCtrl;
    Motor2Can *motor2can;
    const ros::Duration timeOffSet;
    const ros::Duration entimeOffSet;
};


#endif //PROJECT_CAR_MOTOR_H