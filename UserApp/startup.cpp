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
enum {
    encoderInitFlag = 0,
    batteryInitFlag = 1 << 1,
    rosPUbInitFlag = 1 << 2
};

//用户变量初始化区
Key hsw2(SW2_GPIO_Port, SW2_Pin);
Key hsw3(SW3_GPIO_Port, SW3_Pin);

Led hled0(LED0_GPIO_Port, LED0_Pin, on_Low);
Led hled1(LED1_GPIO_Port, LED1_Pin);

Motor motor(hcan1);

#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
volatile unsigned long long run_time_stats_tick;
#endif

void stm32TopicCtrl_cb(const communicate_with_stm32::MotorCmd &cmd);

void stm32ServerCtrl_cb(const communicate_with_stm32::MotorControl::Request &req,
                        communicate_with_stm32::MotorControl::Response &res);

void encoderTimCallbk(TimerHandle_t xTimer);

void batteryTimCallbk(TimerHandle_t xTimer);

void sw2TimCallbk(TimerHandle_t xTimer);

void sw3TimCallbk(TimerHandle_t xTimer);

void rosPubCallbk(TimerHandle_t xTimer);


ros::NodeHandle nh;
ros::Publisher motorState("motorState", &motor.motor_state.motorData);
ros::Subscriber<communicate_with_stm32::MotorCmd> motorSub("stm32TopicCtrl", &stm32TopicCtrl_cb);

ros::ServiceServer<communicate_with_stm32::MotorControl::Request,
        communicate_with_stm32::MotorControl::Response> motorSrv("stm32SeverCtrl", &stm32ServerCtrl_cb);

bool isInitial = false;

extern xQueueHandle canSendQueue;
extern xQueueHandle canUrgentQueue;
extern SemaphoreHandle_t canMutex;
extern TaskHandle_t rosTaskHandle;
extern TaskHandle_t CanNormalTaskHandle;
extern TaskHandle_t CanUrgentTaskHandle;
extern TaskHandle_t feedDogTaskHandle;
extern EventGroupHandle_t feedDogEvent;
TimerHandle_t encoderTimerHandle;
TimerHandle_t batteryTimerHandle;
TimerHandle_t sw2TimerHandle;
TimerHandle_t sw3TimerHandle;
TimerHandle_t rosPubHandle;

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
    nh.loginfo("please publish a topic cmd ID greater than 100 to initial the system!");
    while (!isInitial){
        nh.spinOnce();
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(500);
    }
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "it is time to start!\n");
#endif
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


//对c的weak函数进行重写
extern "C" {

void rosCallback(void const *argument) {
    TickType_t begin;
    while (true) {
        begin = xTaskGetTickCount();
        nh.spinOnce();
        xEventGroupSetBits(feedDogEvent, rosEvent);
        vTaskDelayUntil(&begin, 300);
    }
}

void CanNormalTCallbk(void const *argument) {
    communicate_with_stm32::MotorCmd normal;
    while (true) {
        if (xQueueReceive(canSendQueue, &normal, 100) == pdTRUE) {
            xSemaphoreTake(canMutex, portMAX_DELAY);
            if (HAL_OK != motor.topic_cmd(normal.cmd, normal.data)) {
                char temp[100];
                sprintf(temp, "the normal cmd:%s is error!", select_name(normal.cmd));
                nh.logwarn(temp);
            }
            xSemaphoreGive(canMutex);
        }
        xEventGroupSetBits(feedDogEvent, canNormalEvent);
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
    }
}

void feedDogCallbk(void const *argument) {
    HAL_IWDG_Refresh(&hiwdg);
    uint32_t eventSum = (1 << 3) - 1;
    while (true) {
        xEventGroupWaitBits(feedDogEvent,
                            eventSum,
                            pdTRUE,
                            pdTRUE,
                            8000);
        HAL_IWDG_Refresh(&hiwdg);
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

}

void encoderTimCallbk(TimerHandle_t xTimer) {
    communicate_with_stm32::MotorCmd encoder_cmd;
    encoder_cmd.cmd = cmd_getAveSpeed;
    encoder_cmd.isUrgent = false;
    xQueueSendToBack(canSendQueue, &encoder_cmd, 100);
}

void batteryTimCallbk(TimerHandle_t xTimer) {
    communicate_with_stm32::MotorCmd battery_cmd;
    battery_cmd.cmd = cmd_updateBattery;
    battery_cmd.isUrgent = false;
    xQueueSendToBack(canSendQueue, &battery_cmd, 100);
}

void rosPubCallbk(TimerHandle_t xTimer) {
    if (motor.rosPubCtrl.IsOpen) {
        vPortEnterCritical();
        motorState.publish(&motor.motor_state.motorData);
        vPortExitCritical();
    }
}

void sw2TimCallbk(TimerHandle_t xTimer) {
    hsw2.Key_timely_detect();
}

void sw3TimCallbk(TimerHandle_t xTimer) {
    hsw3.Key_timely_detect();
}


void stm32TopicCtrl_cb(const communicate_with_stm32::MotorCmd &cmd) {
    if(!isInitial)
    {
        if(cmd.cmd <= 100)
        {
            nh.logwarn("please input a cmd greater than 100 to ensure the init!");
        }
        else
        {
            uint8_t openFlag = cmd.cmd - 100;
            if(openFlag & encoderInitFlag)
            {
                motor.encoderCtrl.IsOpen = true;
                motor.encoderCtrl.freq = cmd.data[0];
            }
            else
            {
                motor.encoderCtrl.IsOpen = false;
                motor.encoderCtrl.freq = 500;
            }
            if(openFlag & batteryInitFlag)
            {
                motor.batteryCtrl.IsOpen = true;
                motor.batteryCtrl.freq = cmd.data[1];
            }
            else
            {
                motor.batteryCtrl.IsOpen = false;
                motor.batteryCtrl.freq = 1000;
            }
            if(openFlag & rosPUbInitFlag)
            {
                motor.rosPubCtrl.IsOpen = true;
                motor.rosPubCtrl.freq = cmd.data[2];
            }
            else
            {
                motor.rosPubCtrl.IsOpen = false;
                motor.rosPubCtrl.freq = 1000;
            }
            isInitial = true;
            nh.loginfo("init the system success!");
        }
    }
    else{
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

}

void stm32ServerCtrl_cb(const communicate_with_stm32::MotorControl::Request &req,
                        communicate_with_stm32::MotorControl::Response &res) {
        if (motor.server_cmd(req, res) == HAL_OK) {
            res.success = true;
        } else
            res.success = false;
}


HAL_StatusTypeDef create_Queue() {
    canSendQueue = xQueueCreate(20, sizeof(communicate_with_stm32::MotorCmd));
    canUrgentQueue = xQueueCreate(10, sizeof(communicate_with_stm32::MotorCmd));
    if (canUrgentQueue != NULL && canSendQueue != NULL) {
        return HAL_OK;
    }
    nh.logerror("the freertos create error!");
    return HAL_ERROR;
}

HAL_StatusTypeDef start_timer() {
    uint16_t sw_period;
    sw_period = 20;

    encoderTimerHandle = xTimerCreate("checkEncoder",
                                      motor.encoderCtrl.freq,
                                      pdTRUE,
                                      (void *) 1,
                                      encoderTimCallbk);
    batteryTimerHandle = xTimerCreate("checkBattery",
                                      motor.batteryCtrl.freq,
                                      pdTRUE,
                                      (void *) 2,
                                      batteryTimCallbk);
    sw2TimerHandle = xTimerCreate("checkSW2",
                                  sw_period,
                                  pdTRUE,
                                  (void *) 3,
                                  sw2TimCallbk);
    sw3TimerHandle = xTimerCreate("checkSW3",
                                  sw_period,
                                  pdTRUE,
                                  (void *) 4,
                                  sw3TimCallbk);
    rosPubHandle = xTimerCreate("rosPub",
                                motor.rosPubCtrl.freq,
                                pdTRUE,
                                (void *) 5,
                                rosPubCallbk);
    if ((encoderTimerHandle != NULL)
        && (batteryTimerHandle != NULL)
        && (sw2TimerHandle != NULL)
        && (sw3TimerHandle != NULL)
            ) {
        if (motor.encoderCtrl.IsOpen) {
            if (!xTimerStart(encoderTimerHandle, 1000)) {
                return HAL_ERROR;
            }
        }
        if (motor.batteryCtrl.IsOpen) {
            if (!xTimerStart(batteryTimerHandle, 1000)) {
                return HAL_ERROR;
            }
        }
        if(motor.rosPubCtrl.IsOpen)
        {
            if (!xTimerStart(rosPubHandle, 1000)) {
                return HAL_ERROR;
            }
        }
        return HAL_OK;
    } else {
        return HAL_ERROR;
    }
}






