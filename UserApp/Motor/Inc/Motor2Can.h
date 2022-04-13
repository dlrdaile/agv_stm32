/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/4/12 下午9:32
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_MOTOR2CAN_H
#define PROJECT_CAR_MOTOR2CAN_H

#include "main.h"
#include "Can.h"
#include "ros.h"
#include "communicate_with_stm32/BatteryInfo.h"
#include "communicate_with_stm32/Encoderinfo.h"
#include "command_length.h"

#ifdef __cplusplus
using namespace communicate_with_stm32;

const uint32_t sysReset_msgID = Generate_msgID(0x19, 0);
const uint32_t mvDirection_msgID = Generate_msgID(0x2A, 0);
const uint32_t battery_msgID = Generate_msgID(0x33, 0);
const uint32_t setSpeed_msgID = Generate_msgID(0x29, 0);
const uint32_t oneMS_Encoder_msgID = Generate_msgID(0x30, 0);
const uint32_t EncoderData_msgID = Generate_msgID(0x31, 0);
const uint32_t ClearEncoder_msgID = Generate_msgID(0x32, 0);

#define taskdelaytick 1
#define encoder_taskdelay 1
extern "C" {
#endif
class Motor2Can {
public:
    Motor2Can(CAN_HandleTypeDef &hcan);

    ~Motor2Can();

    /**
     * @brief 设置各个轮子的速度,设定的数值等于 （1e-4 * value）
     * @param v1 左前轮
     * @param v2 右前轮
     * @param v3 左后轮
     * @param v4 右后轮
     * @return
     */
    HAL_StatusTypeDef setSpeed(int16_t v1 = 0, int16_t v2 = 0, int16_t v3 = 0, int16_t v4 = 0);

    /**
     * @brief 对底盘的驱动系统进行重启
     * @return
     */
    HAL_StatusTypeDef motion_system_reset();

    HAL_StatusTypeDef directe_motion(int16_t rotate_speed, int16_t speed_x, int16_t speed_y);

    /**
     * @brief 查看电池的电量，并更新小车电量信息
     * @return
     */
    HAL_StatusTypeDef update_battery();

    HAL_StatusTypeDef update_encoderdata();

    HAL_StatusTypeDef clear_encoder();

    /**
     * @brief 控制小车停止
     * @return
     */
    HAL_StatusTypeDef stop();
public:
    uint8_t CanRxBuffer[16];
    Can *mCan;
private:
    /**
     * @brief 对Can的接收数据做验证，并当接收的过程出错时显示错误原因
     * @param transtatus
     * @param ExtID
     * @return
     */
    HAL_StatusTypeDef verifyReceive(const CanStatusTypeDef &transtatus, const uint32_t &ExtID);
private:
    CanStatusTypeDef result;
};
#ifdef __cplusplus
}
#endif
#endif //PROJECT_CAR_MOTOR2CAN_H