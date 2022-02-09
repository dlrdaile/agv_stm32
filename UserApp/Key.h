/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午4:04
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_KEY_H
#define PROJECT_CAR_KEY_H
#ifdef __cplusplus
#include "GPIO_Base.h"
#include "main.h"
typedef enum {
    KEY_CHECK = 0,
    KEY_COMFIRM,
    KEY_RELEASE
} KeyState_TypeDef;

typedef enum {
    BUTTEN_OFF = 0,
    BUTTEN_ON
} KeyButtenState_TypeDef;


class Key : protected GPIO_Base {
public:
    Key(GPIO_TypeDef *Key_GPIO_Port, uint16_t Key_GPIO_Pin, bool Key_OnState = on_High,
        KeyButtenState_TypeDef ButtenFlag = BUTTEN_OFF, KeyState_TypeDef KeyState = KEY_CHECK);

    ~Key();

    void Key_timely_detect();

public:
    KeyButtenState_TypeDef ButtenFlag;
private:
    KeyState_TypeDef KeyState;
};
#endif
#endif //PROJECT_CAR_KEY_H