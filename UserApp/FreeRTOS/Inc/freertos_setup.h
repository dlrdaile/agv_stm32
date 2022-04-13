/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/4/12 下午9:44
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_FREERTOS_SETUP_H
#define PROJECT_CAR_FREERTOS_SETUP_H
#ifdef __cplusplus
#include "main.h"
extern "C" {
#endif
/**
* @brief 当调用debug模式时查看cpu等调用情况
* @param htim 定时器hal库对象
*/
void timely_detect(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef start_timer();
HAL_StatusTypeDef create_Queue();
#ifdef __cplusplus
}
#endif
#endif //PROJECT_CAR_FREERTOS_SETUP_H