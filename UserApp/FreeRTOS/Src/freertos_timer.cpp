/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/4/12 下午9:31
* @version: 1.0
* @description: 
********************************************************************************/
#include "freertos_timer.h"
#include "cmsis_os.h"
#include "communicate_with_stm32/MotorCmd.h"
#include "Motor.h"
#include "Key.h"

extern xQueueHandle canSendQueue;
extern xQueueHandle canUrgentQueue;
extern Key hsw2;
extern Key hsw3;

void encoderTimCallbk(TimerHandle_t xTimer) {
    communicate_with_stm32::MotorCmd encoder_cmd;
    encoder_cmd.cmd = cmd_updateEncoderData;
    encoder_cmd.isUrgent = false;
    xQueueSendToBack(canSendQueue, &encoder_cmd, 100);
}

void batteryTimCallbk(TimerHandle_t xTimer) {
    communicate_with_stm32::MotorCmd battery_cmd;
    battery_cmd.cmd = cmd_updateBattery;
    battery_cmd.isUrgent = false;
    xQueueSendToBack(canSendQueue, &battery_cmd, 100);
}

void sw2TimCallbk(TimerHandle_t xTimer) {
    hsw2.Key_timely_detect();
}

void sw3TimCallbk(TimerHandle_t xTimer) {
    hsw3.Key_timely_detect();
}