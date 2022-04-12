/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 上午1:18
* @version: 1.0
* @description: 
********************************************************************************/
#include "GPIO_Base.h"

GPIO_Base::GPIO_Base(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, bool OnState) {
    this->GPIO_Port = GPIO_Port;
    this->GPIO_Pin = GPIO_Pin;
    if (OnState) {
        this->OnState = GPIO_PIN_SET;
        this->OffState = GPIO_PIN_RESET;
    } else {
        this->OnState = GPIO_PIN_RESET;
        this->OffState = GPIO_PIN_SET;
    }
}
