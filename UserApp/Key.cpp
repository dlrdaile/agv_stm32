/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/8 下午11:43
* @version: 1.0
* @description: 
********************************************************************************/
#include "Key.h"
#include "FreeRTOS.h"
#include "timers.h"

extern TimerHandle_t sw2TimerHandle;
extern TimerHandle_t sw3TimerHandle;

Key::~Key() {

}

Key::Key(GPIO_TypeDef *Key_GPIO_Port, uint16_t Key_GPIO_Pin, bool Key_OnState, KeyButtenState_TypeDef ButtenFlag,
         KeyState_TypeDef KeyState) : GPIO_Base(Key_GPIO_Port, Key_GPIO_Pin, Key_OnState) {
    this->ButtenFlag = ButtenFlag;
    this->KeyState = KeyState;
}

void Key::Key_timely_detect() {
    switch (this->KeyState) {
        case KEY_CHECK:
            if (HAL_GPIO_ReadPin(this->GPIO_Port, this->GPIO_Pin) == this->OnState) {
                this->KeyState = KEY_COMFIRM;
            }
            break;
        case KEY_COMFIRM:
            if (HAL_GPIO_ReadPin(this->GPIO_Port, this->GPIO_Pin) == this->OnState) {
                this->KeyState = KEY_RELEASE;
                this->ButtenFlag = BUTTEN_ON;
            } else {
                this->KeyState = KEY_CHECK;
            }
            break;
        case KEY_RELEASE:
            if (HAL_GPIO_ReadPin(this->GPIO_Port, this->GPIO_Pin) == this->OffState) {
                this->KeyState = KEY_CHECK;
                if(this->GPIO_Pin == SW2_Pin)
                {
                    xTimerStop(sw2TimerHandle,10);
                }
                else if(this->GPIO_Pin == SW3_Pin)
                {
                    xTimerStop(sw3TimerHandle,10);
                }
            }
            break;
        default:
            break;
    }
}