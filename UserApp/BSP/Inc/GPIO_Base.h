/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午4:06
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_GPIO_BASE_H
#define PROJECT_CAR_GPIO_BASE_H
#ifdef __cplusplus
#include "main.h"
#define on_High true
#define on_Low false
class GPIO_Base {
public:
    GPIO_Base(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, bool OnState);
protected:
    GPIO_TypeDef *GPIO_Port;
    uint16_t GPIO_Pin;
    GPIO_PinState OnState;
    GPIO_PinState OffState;
};
#endif
#endif //PROJECT_CAR_GPIO_BASE_H