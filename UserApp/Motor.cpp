/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/24 下午7:59
* @version: 1.0
* @description: 
********************************************************************************/
#include "Motor.h"
#include "command_length.h"
#include "Usart.h"

extern Usart debug_uart;

Motor::Motor(Can *mCan) {
    this->mCan = mCan;
    this->motion_system_reset();
    this->InitState();
}

void Motor::setSpeed(uint16_t FL, uint16_t FR, uint16_t RL, uint16_t RR) {
    uint16_t speed[4]={FL,FR,RL,RR};
    uint8_t *raw_data = (uint8_t *) speed;  //size = 8
    this->mCan->CAN_SendMsg(setSpeed_msgID,raw_data,DLSEND_41);
}

void Motor::motion_system_reset() {
    uint8_t *null_char = 0;
    this->mCan->CAN_SendMsg(sysReset_msgID,null_char,DLSEND_25);

}

void Motor::XY_motion(uint16_t speed_x, uint16_t speed_y) {
    uint16_t raw_data[4]={0, 0, speed_y,speed_x};
    mCan->CAN_SendMsg(mvDirection_msgID,(uint8_t*) raw_data,DLSEND_42);
}

void Motor::swerve_motion(uint16_t radius, uint16_t speed) {
    if(radius == 0) return;
    uint16_t raw_data[4]={0, (uint16_t)(speed / radius), 0, speed};
    mCan->CAN_SendMsg(mvDirection_msgID,(uint8_t*) raw_data,DLSEND_42);
}

void Motor::rotate_motion(uint16_t rotate_speed) {
    uint16_t raw_data[4]={0, rotate_speed, 0,0};
    mCan->CAN_SendMsg(mvDirection_msgID,(uint8_t*) raw_data,DLSEND_42);
}

void Motor::check_battery() {
    mCan->CAN_SendMsg(battery_msgID,NULL,DLSEND_51);
}

void Motor::stop() {
    this->setSpeed(0,0,0,0);
}

void Motor::InitState() {
    this->stop();
    for (int i = 0; i < 4; ++i) {
        this->motor_state.FL_speed[i] = 0;
    }
    this->check_battery();
    mCan->CAN_ReadMsg();
    HAL_Delay(100);
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) !=
        HAL_OK) {
        debug_uart.SendData('init car state error!\r\n');
        while (true);
    }
    if(RxHeader.ExtId == battery_msgID)
    {
        uint8_t *p = RxData;
        this->motor_state.battery_votage = *(uint16_t *)p;
    }
    else
    {
        debug_uart.SendData('read car power error!\r\n');
    }
}

uint32_t Motor::show_battery() {
    return this->motor_state.battery_votage;
}
