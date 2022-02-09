/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午4:05
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_LED_H
#define PROJECT_CAR_LED_H
#ifdef __cplusplus
#include "GPIO_Base.h"
#include "main.h"
class Led : protected GPIO_Base {
public:
    /**
     * @brief 创建LED对象
     * @param Led_GPIO_Port
     * @param Led_GPIO_Pin
     * @param Led_OnState
     */
    Led(GPIO_TypeDef *Led_GPIO_Port, uint16_t Led_GPIO_Pin, bool Led_OnState = on_High);

    ~Led();

    void On();

    void Off();

    void Toggle();
};

#endif
#endif //PROJECT_CAR_LED_H