/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午4:08
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_STARTUP_H
#define PROJECT_CAR_STARTUP_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "FreeRTOSConfig.h"
#include "UserConfig.h"
//创建几个任务队列
void startup();
void timely_detect(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef start_timer();
HAL_StatusTypeDef create_Queue();
/*void on_UART_IDLE(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);*/
#ifdef __cplusplus
}
#endif
#endif //PROJECT_CAR_STARTUP_H