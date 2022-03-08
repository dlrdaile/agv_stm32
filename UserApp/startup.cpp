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

//ros库函数
#include "ros.h"
#include "communicate_with_stm32/MotorData.h"
#include "communicate_with_stm32/MotorCmd.h"
#include "communicate_with_stm32/MotorControl.h"
//Freertos的库函数
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "cmsis_os.h"

//Jlink调试打印的函数库
#if JLINK_DEBUG == 1

#include "SEGGER_RTT.h"

#endif

enum {
    rosEvent = 1,
    canNormalEvent = 1 << 1,
    canUrgentEvent = 1 << 2
};

//用户变量初始化区
Key hsw2(SW2_GPIO_Port, SW2_Pin);
Key hsw3(SW3_GPIO_Port, SW3_Pin);

Led hled0(LED0_GPIO_Port, LED0_Pin, on_Low);
Led hled1(LED1_GPIO_Port, LED1_Pin);

Motor motor(hcan1);

bool canSendFlag = false;
#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
volatile unsigned long long run_time_stats_tick;
#endif

void stm32TopicCtrl_cb(const communicate_with_stm32::MotorCmd &cmd);

void stm32ServerCtrl_cb(const communicate_with_stm32::MotorControl::Request &req,
                        communicate_with_stm32::MotorControl::Response &res);


ros::NodeHandle nh;
//TODO:注意这里是一个测试的注释
communicate_with_stm32::MotorData mStateData;
//ros::Publisher motorState("motorState", &mStateData);
ros::Publisher motorState("motorState", &mStateData);
ros::Subscriber<communicate_with_stm32::MotorCmd> motorSub("stm32TopicCtrl", &stm32TopicCtrl_cb);

ros::ServiceServer<communicate_with_stm32::MotorControl::Request,
        communicate_with_stm32::MotorControl::Response> motorSrv("stm32SeverCtrl", &stm32ServerCtrl_cb);

bool IsPublish = false;

extern xQueueHandle canSendQueue;
extern xQueueHandle canUrgentQueue;
extern SemaphoreHandle_t canMutex;
extern TaskHandle_t rosTaskHandle;
extern TaskHandle_t CanNormalTaskHandle;
extern TaskHandle_t CanUrgentTaskHandle;
extern TaskHandle_t feedDogTaskHandle;
extern EventGroupHandle_t feedDogEvent;
extern TimerHandle_t encoderTimerHandle;
extern TimerHandle_t batteryTimerHandle;
extern TimerHandle_t sw2TimerHandle;
extern TimerHandle_t sw3TimerHandle;

void startup() {
#if JLINK_DEBUG == 1
    SEGGER_RTT_Init();
#endif
    nh.initNode();
    nh.advertise(motorState);
    nh.subscribe(motorSub);
    nh.advertiseService(motorSrv);
    while (!nh.connected()) {
        SEGGER_RTT_printf(0, "no catch!\n");
        nh.spinOnce();
        HAL_Delay(1000);
    }
    motor.InitState();
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "it is time to start!\n");
#endif
    if (!nh.getParam("isPubMStat", &IsPublish, 1, 500)) {
        nh.logwarn("get param error!set the Pub to false default!");
        IsPublish = false;
    }
    nh.spinOnce();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4)
        nh.getHardware()->reset_rbuf();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4)
        nh.getHardware()->flush();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BaseType_t xHighPriorityTaskWoken;
    if (GPIO_Pin == SW2_Pin) {
        xTimerStartFromISR(sw2TimerHandle, &xHighPriorityTaskWoken);
    }
    if (GPIO_Pin == SW3_Pin) {
        xTimerStartFromISR(sw3TimerHandle, &xHighPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHighPriorityTaskWoken);
/*    if (GPIO_Pin == SW2_Pin) {
        hled1.On();
        hled0.Toggle();
    }
    if (GPIO_Pin == SW3_Pin) {
        hled1.Off();
        hled0.Toggle();
    }*/
}

void timely_detect(TIM_HandleTypeDef *htim) {
#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
    if (htim->Instance == TIM6) {
        run_time_stats_tick++;
    }
#endif
}

HAL_StatusTypeDef create_Queue() {
    canSendQueue = xQueueCreate(10, sizeof(communicate_with_stm32::MotorCmd));
    canUrgentQueue = xQueueCreate(10, sizeof(communicate_with_stm32::MotorCmd));
    if (canUrgentQueue != NULL && canSendQueue != NULL) {
        return HAL_OK;
    }
    nh.logerror("the freertos create error!");
    return HAL_ERROR;
}

HAL_StatusTypeDef start_timer() {
    int encoder_period;
    int battery_period;
    BaseType_t result;
    if (!nh.getParam("mEncorderPrd", &encoder_period, 1, 500)) {
        nh.logwarn("get param error!set the encorder period to 100ms default!");
        encoder_period = 100;
    }
    if (!nh.getParam("mBatteryPrd", &battery_period, 1, 500)) {
        nh.logwarn("get param error!set the battery period to 1000ms default!");
        battery_period = 1000;
    }
    if (motor.encoderCheckFlag) {
        result = osTimerStart(encoderTimerHandle, encoder_period);
    } else {
        result = xTimerChangePeriod(encoderTimerHandle, encoder_period, 1000);
    }
    if (motor.batteryCheckFlag) {
        if (!result) return HAL_ERROR;
        result = osTimerStart(batteryTimerHandle, encoder_period);
    } else {
        if (!result) return HAL_ERROR;
        result = xTimerChangePeriod(batteryTimerHandle, battery_period, 1000);
    }
    if (!result) return HAL_ERROR;
    result = xTimerChangePeriod(sw3TimerHandle, 20, 1000);
    if (!result) return HAL_ERROR;
    result = xTimerChangePeriod(sw2TimerHandle, 20, 1000);
    if (!result) return HAL_ERROR;
    return HAL_OK;
}

//对c的weak函数进行重写
extern "C" {

void rosCallback(void const *argument) {
    TickType_t begin;
    while (true) {
        begin = xTaskGetTickCount();
        if (IsPublish) {
            vPortEnterCritical();
            memcpy(&mStateData, &motor.motor_state.motorData, sizeof(motorState));
            vPortExitCritical();
            motorState.publish(&mStateData);
        }
        nh.spinOnce();
        xEventGroupSetBits(feedDogEvent, rosEvent);
//        xTaskNotify(feedDogTaskHandle,rosEvent,eSetBits);
        vTaskDelayUntil(&begin, 200);
    }
}

void CanNormalTCallbk(void const *argument) {
    communicate_with_stm32::MotorCmd normal;
    while (true) {
        if (xQueueReceive(canUrgentQueue, &normal, 100) == pdTRUE) {
            xSemaphoreTake(canMutex, portMAX_DELAY);
            if (HAL_OK != motor.topic_cmd(normal.cmd, normal.data)) {
                char temp[100];
                sprintf(temp, "the normal cmd:%s is error!", select_name(normal.cmd));
                nh.logwarn(temp);
            }
            xSemaphoreGive(canMutex);
        }
        xEventGroupSetBits(feedDogEvent, canNormalEvent);
//        xTaskNotify(feedDogTaskHandle,canNormalEvent,eSetBits);
    }
}


void CanUrgentCallbk(void const *argument) {
    communicate_with_stm32::MotorCmd urgentcmd;
    while (true) {
        if (xQueueReceive(canUrgentQueue, &urgentcmd, 100) == pdTRUE) {
            xSemaphoreTake(canMutex, portMAX_DELAY);
            HAL_CAN_AbortTxRequest(motor.mCan->hcan, motor.mCan->TxMailbox);
            switch (urgentcmd.cmd) {
                case cmd_Stop:
                    while (HAL_OK != motor.stop()) {
                        nh.logfatal("motor stop error!");
                        HAL_Delay(500);
                    }
                    break;
                case cmd_sysReset:
                    while (HAL_OK != motor.motion_system_reset()) {
                        nh.logfatal("motor system reset error!");
                        HAL_Delay(500);
                    }
                default:
                    if (HAL_OK != motor.topic_cmd(urgentcmd.cmd, urgentcmd.data)) {
                        char temp[50];
                        sprintf(temp, "the urgent cmd:%d is error!", urgentcmd.cmd);
                        nh.logwarn(temp);
                    }
                    break;
            }
            xSemaphoreGive(canMutex);
        }
        xEventGroupSetBits(feedDogEvent, canUrgentEvent);
//        xTaskNotify(feedDogTaskHandle,canUrgentEvent,eSetBits);
    }
}

void feedDogCallbk(void const *argument) {
    HAL_IWDG_Refresh(&hiwdg);
    uint32_t eventSum = (1 << 3) - 1;
//    uint32_t uNotisfyValue;
    while (true) {
        if (pdTRUE == xEventGroupWaitBits(feedDogEvent,
                                          eventSum,
                                          pdTRUE,
                                          pdTRUE,
                                          3000))
            HAL_IWDG_Refresh(&hiwdg);
        else {
            nh.logwarn("the dog is going to dead!");
        }
/*        xTaskNotifyWait(0x00,
                        0xffffffff,
                        &uNotisfyValue,
                        3000);
        if(uNotisfyValue == eventSum){
            HAL_IWDG_Refresh(&hiwdg);
        }*/
    }
}

void keyCheckCallbk(void const *argument) {
    while (true) {
        if (hsw2.ButtenFlag == BUTTEN_ON) {
            hled0.On();
            hled1.Toggle();
            hsw2.ButtenFlag = BUTTEN_OFF;
            xTimerStop(sw2TimerHandle, portMAX_DELAY);
        }
        if (hsw3.ButtenFlag == BUTTEN_ON) {
            hled1.Toggle();
            hled0.Off();
            hsw3.ButtenFlag = BUTTEN_OFF;
            xTimerStop(sw3TimerHandle, portMAX_DELAY);
        }
        vTaskDelay(1000);
    }
}
#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
void configureTimerForRunTimeStats(void) {
    run_time_stats_tick = 0;
}
unsigned long getRunTimeCounterValue(void) {
    return run_time_stats_tick;
}
#endif

void encoderTimCallbk(void const *argument) {
    communicate_with_stm32::MotorCmd encoder_cmd;
    encoder_cmd.cmd = cmd_getIncSpeed;
    xQueueOverwrite(canSendQueue, &encoder_cmd);
}

void batteryTimCallbk(void const *argument) {
    communicate_with_stm32::MotorCmd battery_cmd;
    battery_cmd.cmd = cmd_updateBattery;
    xQueueOverwrite(canSendQueue, &battery_cmd);
}

void sw2TimCallbk(void const *argument) {
    hsw2.Key_timely_detect();
}

void sw3TimCallbk(void const *argument) {
    hsw3.Key_timely_detect();
}

}


void stm32TopicCtrl_cb(const communicate_with_stm32::MotorCmd &cmd) {
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
        if (pdTRUE != xQueueOverwrite(canUrgentQueue, &cmd)) {
            info = "the normal cmd: " + to_string(cmd.cmd) + "send to queue fail!\nwe will try again";
            nh.logwarn(info.c_str());
            if (pdTRUE == xQueueOverwrite(canUrgentQueue, &cmd)) {
                nh.loginfo("try again success!");
            } else {
                nh.loginfo("the cmd has been abundant!");
            }
        }
    }
}

void stm32ServerCtrl_cb(const communicate_with_stm32::MotorControl::Request &req,
                        communicate_with_stm32::MotorControl::Response &res) {
    if (motor.server_cmd(req, res) == HAL_OK) {
        res.success = true;
    } else
        res.success = false;
}






