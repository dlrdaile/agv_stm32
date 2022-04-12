/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/4/12 下午9:44
* @version: 1.0
* @description: 
********************************************************************************/
#include "freertos_setup.h"
#include "UserConfig.h"
#include "Motor.h"
#include "freertos_timer.h"

#include "FreeRTOSConfig.h"
#include "cmsis_os.h"

#include "ros.h"
#include "communicate_with_stm32/MotorCmd.h"

TimerHandle_t encoderTimerHandle;
TimerHandle_t batteryTimerHandle;
TimerHandle_t sw2TimerHandle;
TimerHandle_t sw3TimerHandle;
#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
volatile unsigned long long run_time_stats_tick;
#endif


void timely_detect(TIM_HandleTypeDef *htim) {
#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
    if (htim->Instance == TIM6) {
        run_time_stats_tick++;
    }
#endif
}

HAL_StatusTypeDef start_timer() {
    extern Motor motor;
    uint16_t sw_period;
    sw_period = 20;
    encoderTimerHandle = xTimerCreate("checkEncoder",
                                      motor.encoderCtrl.freq,
                                      pdTRUE,
                                      (void *) 1,
                                      encoderTimCallbk);
    batteryTimerHandle = xTimerCreate("checkBattery",
                                      motor.batteryCtrl.freq,
                                      pdTRUE,
                                      (void *) 2,
                                      batteryTimCallbk);
    sw2TimerHandle = xTimerCreate("checkSW2",
                                  sw_period,
                                  pdTRUE,
                                  (void *) 3,
                                  sw2TimCallbk);
    sw3TimerHandle = xTimerCreate("checkSW3",
                                  sw_period,
                                  pdTRUE,
                                  (void *) 4,
                                  sw3TimCallbk);
    if ((encoderTimerHandle != NULL)
        && (batteryTimerHandle != NULL)
        && (sw2TimerHandle != NULL)
        && (sw3TimerHandle != NULL)
            ) {
        if (motor.encoderCtrl.isOpen) {
            if (!xTimerStart(encoderTimerHandle, 1000)) {
                return HAL_ERROR;
            }
        }
        if (motor.batteryCtrl.isOpen) {
            if (!xTimerStart(batteryTimerHandle, 1000)) {
                return HAL_ERROR;
            }
        }
        return HAL_OK;
    } else {
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef create_Queue() {
    extern xQueueHandle canSendQueue;
    extern xQueueHandle canUrgentQueue;
    extern ros::NodeHandle nh;

    canSendQueue = xQueueCreate(20, sizeof(communicate_with_stm32::MotorCmd));
    canUrgentQueue = xQueueCreate(20, sizeof(communicate_with_stm32::MotorCmd));
    if (canUrgentQueue != NULL && canSendQueue != NULL) {
        return HAL_OK;
    }
    nh.logerror("the freertos create error!");
    return HAL_ERROR;
}

#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
void configureTimerForRunTimeStats(void) {
    run_time_stats_tick = 0;
}
unsigned long getRunTimeCounterValue(void) {
    return run_time_stats_tick;
}
#endif
