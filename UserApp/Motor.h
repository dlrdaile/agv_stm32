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

enum {
    FL_motor = 0,
    FR_motor,
    BL_motor,
    BR_motor,
};

typedef struct {
    TickType_t check_time;
    int32_t encoder_data[4];
} EncoderData_TypeDef;

typedef struct {
    uint32_t battery_votage;
    int16_t setted_speed[4];
    int8_t oneMs_encoder[4];
    EncoderData_TypeDef current_encData;
    EncoderData_TypeDef last_encData;
} CarState_TypeDef;

typedef enum {
    cmd_sysReset = 0,
    cmd_setSpeed,
    cmd_Stop,
    cmd_updateBattery,
    cmd_updateEncoderData,
    cmd_updateOneMsEncoder,
    cmd_clearEncoder,
    cmd_xyMotion,
    cmd_swerveMotion,
    cmd_rotateMotion,
    cmd_checkAveSpeed,
    cmd_checkInsSpeed,
    cmd_checkEncoderData,
    cmd_checkOneMsEncoder
} cmd_set;
const char *select_name(uint8_t &cmd);
const uint32_t sysReset_msgID = Generate_msgID(0x19, 0);
const uint32_t mvDirection_msgID = Generate_msgID(0x2A, 0);
const uint32_t battery_msgID = Generate_msgID(0x33, 0);
const uint32_t setSpeed_msgID = Generate_msgID(0x29, 0);
const uint32_t oneMS_Encoder_msgID = Generate_msgID(0x30, 0);
const uint32_t EncoderData_msgID = Generate_msgID(0x31, 0);
const uint32_t ClearEncoder_msgID = Generate_msgID(0x32, 0);

class Motor {
public:
    Motor(CAN_HandleTypeDef &hcan);

    ~Motor();

    /**
     * @brief 设置各个轮子的速度,设定的数值等于 （1e-4 * value）
     * @param FL 左前轮
     * @param FR 右前轮
     * @param BL 左后轮
     * @param BR 右后轮
     * @return
     */
    HAL_StatusTypeDef setSpeed(int16_t FL = 0, int16_t FR = 0, int16_t BL = 0, int16_t BR = 0);

    /**
     * @brief 对底盘的驱动系统进行重启
     * @return
     */
    HAL_StatusTypeDef motion_system_reset();

    /**
     * @brief 设置车子做一定半径的圆周运动
     * @param radius 半径，单位为 m
     * @param speed 转弯过程中的线速度，速度为 (value * 1e-4)m/s
     * @return
     */
    HAL_StatusTypeDef swerve_motion(int16_t radius, int16_t speed);

    /**
     * @brief 控制小车在平面做某一方向的直线运动
     * @param speed_x x方向的速度,速度为 （value * 1e-4）m/s
     * @param speed_y y方向的速度,（value * 1e-4）m/s
     * @return
     */
    HAL_StatusTypeDef XY_motion(int16_t speed_x, int16_t speed_y);

    /**
     * @brief 小车原地自转
     * @param rotate_speed 原地自转的角速度，角速度为（value * 1e-4）rad/s
     * @return
     */
    HAL_StatusTypeDef rotate_motion(int16_t rotate_speed);

    /**
     * @brief 查看电池的电量，并更新小车电量信息
     * @return
     */
    HAL_StatusTypeDef update_battery();

    /**
     * @brief 查看1ms内编码器的读取数值，编码器的精度为2^12,即每一圈编码器会读取2^12个脉冲信号
     * @return
     */
    HAL_StatusTypeDef update_oneMs_encoder();

    HAL_StatusTypeDef update_encoderdata();

    HAL_StatusTypeDef clear_encoder();

    /**
     * @brief 控制小车停止
     * @return
     */
    HAL_StatusTypeDef stop();

    /**
     * @brief 对小车进行初始化设置
     * @return
     */
    HAL_StatusTypeDef InitState();

    HAL_StatusTypeDef run_cmd(const uint8_t &cmd, uint16_t *TxData = NULL);

private:

    HAL_StatusTypeDef verifyReceive(const CanStatusTypeDef &transtatus, const uint32_t &ExtID);

public:
    CarState_TypeDef motor_state;
    uint8_t CanRxBuffer[16];
    Can *mCan;
private:
    CanStatusTypeDef result;
};


#endif //PROJECT_CAR_MOTOR_H