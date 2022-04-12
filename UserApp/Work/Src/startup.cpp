/********************************************************************************
 * @author: dai le
 * @email: 965794928@qq.com
 * @date: 2022/2/8 下午11:42
 * @version: 1.0
 * @description:
 ********************************************************************************/
//STM32的库函数
#include "main.h"
#include "iwdg.h"
//C++官方库
#include "cstdio"
//自定义的库函数
#include "startup.h"
#include "Key.h"
#include "Led.h"
#include "Can.h"
#include "Motor.h"
#include "HX711.h"
#include "ros_Callback.h"
#include "UserConfig.h"
//ros库函数
#include "ros.h"
#include "communicate_with_stm32/InitMotor.h"

//Jlink调试打印的函数库
#if JLINK_DEBUG == 1

#include "SEGGER_RTT.h"
#endif
communicate_with_stm32::InitConfig pressCtrl;
//用户变量初始化区
Key hsw2(SW2_GPIO_Port, SW2_Pin);
Key hsw3(SW3_GPIO_Port, SW3_Pin);

Led hled0(LED0_GPIO_Port, LED0_Pin, on_Low);
Led hled1(LED1_GPIO_Port, LED1_Pin);

HX711 hx711(HX711_SCK_GPIO_Port, HX711_SCK_Pin,HX711_DT_GPIO_Port,HX711_DT_Pin);

Motor motor(hcan1);

extern ros::NodeHandle nh;
extern TickType_t last_cmd_tick;
extern ros::ServiceClient<communicate_with_stm32::InitMotor::Request,
        communicate_with_stm32::InitMotor::Response> initSrv;
void startup() {
    ros_init();
    nh.loginfo("i'm wait for init server!");
    while (true) {
        InitMotor::Request init_req;
        InitMotor::Response init_res;
        initSrv.call(init_req,init_res);
        nh.spinOnce();
        if(init_res.success)
        {
            motor.encoderCtrl.isOpen = init_res.encoder_config.isOpen;
            motor.encoderCtrl.freq = init_res.encoder_config.freq;
            motor.batteryCtrl.isOpen = init_res.battery_config.isOpen;
            motor.batteryCtrl.freq = init_res.battery_config.freq;
            pressCtrl.isOpen = init_res.press_config.isOpen;
            pressCtrl.freq = init_res.press_config.freq;
            hled1.Off();
            break;
        }
        hled1.Toggle();
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(1000);
    }
    nh.loginfo("init the system success!");
    last_cmd_tick = HAL_GetTick();
    if (motor.InitState() != HAL_OK) {
        nh.logwarn("initial error!Maybe no open the source!");
        while (true) {}
    }
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "it is time to start!\n");
#endif
}

