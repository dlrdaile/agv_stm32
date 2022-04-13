/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/4/12 下午9:31
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_FREERTOS_TIMER_H
#define PROJECT_CAR_FREERTOS_TIMER_H
#ifdef __cplusplus
#include "cmsis_os.h"
extern "C" {
#endif
//定时器回调函数
void encoderTimCallbk(TimerHandle_t xTimer);

void batteryTimCallbk(TimerHandle_t xTimer);

void sw2TimCallbk(TimerHandle_t xTimer);

void sw3TimCallbk(TimerHandle_t xTimer);
#ifdef __cplusplus
}
#endif
#endif //PROJECT_CAR_FREERTOS_TIMER_H