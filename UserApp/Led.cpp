/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/8 下午11:43
* @version: 1.0
* @description: 
********************************************************************************/
#include "Led.h"

Led::Led(GPIO_TypeDef *Led_GPIO_Port, uint16_t Led_GPIO_Pin, bool Led_OnState) :
        GPIO_Base(Led_GPIO_Port, Led_GPIO_Pin,
                  Led_OnState) {
    this->Off();
}

void Led::On() {
    HAL_GPIO_WritePin(this->GPIO_Port, this->GPIO_Pin, this->OnState);
}

Led::~Led() {
    this->Off();
}

void Led::Off() {
    HAL_GPIO_WritePin(this->GPIO_Port, this->GPIO_Pin, this->OffState);
}

void Led::Toggle() {
    HAL_GPIO_TogglePin(this->GPIO_Port, this->GPIO_Pin);
}

GPIO_PinState Led::Read_State() {
    return HAL_GPIO_ReadPin(this->GPIO_Port,this->GPIO_Pin);
}

