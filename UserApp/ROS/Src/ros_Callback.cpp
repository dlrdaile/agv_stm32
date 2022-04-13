/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/4/12 下午9:31
* @version: 1.0
* @description: 
********************************************************************************/
#include "ros_Callback.h"

#include "ros.h"
#include "communicate_with_stm32/MotorCmd.h"
#include "communicate_with_stm32/MotorControl.h"
#include "communicate_with_stm32/Pressinfo.h"
#include "communicate_with_stm32/InitMotor.h"

#include "Motor.h"
#include "Led.h"

#include "cmsis_os.h"
#include "UserConfig.h"

#include "iwdg.h"
#include "SEGGER_RTT.h"

using namespace communicate_with_stm32;
ros::NodeHandle nh;

void stm32TopicCtrl_cb(const communicate_with_stm32::MotorCmd &cmd);

void ConfigServerCallback(const communicate_with_stm32::MotorControl::Request &req,
                          communicate_with_stm32::MotorControl::Response &res);

extern Motor motor;
extern Led hled0;
extern Led hled1;
extern xQueueHandle canSendQueue;
extern xQueueHandle canUrgentQueue;
extern TickType_t last_cmd_tick;
bool isRestart = false;
Pressinfo pressinfo;
ros::Publisher EncoderPub("encoderInfo", &motor.motor_state.encoder_info);
ros::Publisher BatteryPub("batteryInfo", &motor.motor_state.battery_info);
ros::Publisher PressPub("PressInfo", &pressinfo);

ros::Subscriber<communicate_with_stm32::MotorCmd> motorSub("stm32TopicCtrl", &stm32TopicCtrl_cb);
ros::ServiceServer<communicate_with_stm32::MotorControl::Request,
        communicate_with_stm32::MotorControl::Response> motorSrv("stm32SeverCtrl", &ConfigServerCallback);

ros::ServiceClient<communicate_with_stm32::InitMotor::Request,
        communicate_with_stm32::InitMotor::Response> initSrv("stm32Init");


void ros_init() {

    nh.initNode();
    nh.advertise(EncoderPub);
    nh.advertise(BatteryPub);
    nh.advertise(PressPub);
    nh.subscribe(motorSub);
    nh.advertiseService(motorSrv);
    nh.serviceClient(initSrv);
    pressinfo.header.frame_id = "press";
    while (!nh.connected()) {
#if JLINK_DEBUG == 1
        SEGGER_RTT_printf(0, "no catch!\n");
#endif
        nh.spinOnce();
        HAL_IWDG_Refresh(&hiwdg);
        hled0.Toggle();
        HAL_Delay(500);
    }
    hled0.Off();
}

void stm32TopicCtrl_cb(const communicate_with_stm32::MotorCmd &cmd) {
    //系统保护机制
    last_cmd_tick = HAL_GetTick();
    if(isRestart)
    {
        isRestart = false;
    }
    string info;
    if (cmd.isUrgent) {
        if (pdTRUE != xQueueSendToFront(canUrgentQueue, &cmd, 100)) {
            info = "the important cmd: " + to_string(cmd.cmd) +
                   "send to queue fail!\nwe will try to send until success";
            nh.logfatal(info.c_str());
            while (pdTRUE != xQueueSendToFront(canUrgentQueue, &cmd, 0));
            nh.loginfo("sucesss send to queue!");
        }
    } else {
        if (pdTRUE != xQueueSendToFront(canSendQueue, &cmd, 100)) {
            nh.logwarn("a normal cmd send fail");
        }
    }
}


void ConfigServerCallback(const communicate_with_stm32::MotorControl::Request &req,
                          communicate_with_stm32::MotorControl::Response &res) {
    //系统保护机制
    last_cmd_tick = HAL_GetTick();
    if(isRestart)
    {
        isRestart = false;
    }
    if (motor.server_cmd(req, res) == HAL_OK) {
        res.success = true;
    } else
        res.success = false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5)
        nh.getHardware()->reset_rbuf();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5)
        nh.getHardware()->flush();
}