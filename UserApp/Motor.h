/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/24 下午7:59
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_MOTOR_H
#define PROJECT_CAR_MOTOR_H
#include "Can.h"

/*#define MOTION_UNKNOWN 0x00
#define MOTION_STOP    0x01
#define MOTION_RECT    0x02
#define MOTION_LATR    0x03
#define MOTION_CIRC    0x04
#define MOTION_ROTA    0x05
#define CMD_REBOOT     0x10


//error ,1 byte

#define ERROR_OK            0x00
#define ERROR_UNKNOWN       0xFF
#define ERROR_COLLISION_00  0x01
#define ERROR_COLLISION_01  0x02
#define ERROR_COLLISION_02  0x03
#define ERROR_COLLISION_03  0x04
#define ERROR_LOW_POWER     0x05
#define ERROR_SIGNAL_LOST   0x06*/

enum {
    FL_motor = 0,
    FR_motor,
    RL_motor,
    RR_motor,
};
typedef struct {
    uint16_t battery_votage;
    uint16_t FL_speed[4];

} CarState_TypeDef;

#define ADRRESS 1
#define Generate_msgID(Instruct,Index) (uint32_t)(((Index)<<16) | ((Instruct)<<8) | (ADRRESS))

#define setSpeed_msgID Generate_msgID(41,0)
#define sysReset_msgID Generate_msgID(25,0)
#define mvDirection_msgID Generate_msgID(42,0)
#define battery_msgID Generate_msgID(51,0)
//#define Encoder_msgID Generate_msgID()

class Motor {
public:
    Motor(Can *mCan);
    ~Motor();
    void setSpeed(uint16_t FL = 0, uint16_t FR = 0, uint16_t RL = 0, uint16_t RR = 0);
    void motion_system_reset();
    void swerve_motion(uint16_t radius, uint16_t speed);
    void XY_motion(uint16_t speed_x , uint16_t speed_y);
    void rotate_motion(uint16_t rotate_speed);
    void check_battery();
    uint32_t show_battery();
    void stop();
private:
    void InitState();
public:
    CarState_TypeDef motor_state;
private:
    Can *mCan;
};


#endif //PROJECT_CAR_MOTOR_H