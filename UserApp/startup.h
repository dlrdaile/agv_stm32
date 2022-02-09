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
void startup();
void timely_detect(TIM_HandleTypeDef *htim);
#ifdef __cplusplus
}
#endif
#endif //PROJECT_CAR_STARTUP_H