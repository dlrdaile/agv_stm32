/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/4/12 下午9:30
* @version: 1.0
* @description: 
********************************************************************************/
#include "freertos_task.h"
#include "cmsis_os.h"

#include "main.h"

#include "ros.h"
#include "communicate_with_stm32/MotorCmd.h"
#include "communicate_with_stm32/Pressinfo.h"
#include "Led.h"
#include "Key.h"
#include "iwdg.h"
#include "Motor.h"
#include "HX711.h"

//任务句柄
extern TaskHandle_t rosTaskHandle;
extern TaskHandle_t CanNormalTaskHandle;
extern TaskHandle_t CanUrgentTaskHandle;
extern TaskHandle_t feedDogTaskHandle;
extern TaskHandle_t keyTaskHandle;
extern TaskHandle_t PressTaskHandle;
//时间句柄
extern TimerHandle_t sw2TimerHandle;
extern TimerHandle_t sw3TimerHandle;
//事件
extern EventGroupHandle_t feedDogEvent;
//队列
extern xQueueHandle canSendQueue;
extern xQueueHandle canUrgentQueue;
//互斥量
extern SemaphoreHandle_t canMutex;
//bsp的驱动对象
extern Motor motor;
extern Key hsw2;
extern Key hsw3;
extern Led hled0;
extern Led hled1;
extern HX711 hx711;
extern communicate_with_stm32::InitConfig pressCtrl;
extern ros::Publisher PressPub;
extern Pressinfo pressinfo;
extern ros::NodeHandle nh;
//小车未接收到指令超过一定时间则停止
#define stoptime 5000
volatile TickType_t last_cmd_tick;
extern "C" {

void rosCallback(void const *argument) {
    TickType_t begin;
    while (true) {
        extern bool isRestart;
        begin = xTaskGetTickCount();
        nh.spinOnce();
        if ((HAL_GetTick() - last_cmd_tick >= stoptime) && !isRestart) //当系统超过10s没收到指令时
        {
            if (xSemaphoreTake(canMutex, 0)) {
                motor.motor2can->stop();
                nh.loginfo("the motor has lose the signal more than 10s,and it will stop!");
                isRestart = true;
                xSemaphoreGive(canMutex);
            }
        }
        if (nh.connected()) {
            xEventGroupSetBits(feedDogEvent, rosEvent);
        }
        vTaskDelayUntil(&begin, 10);
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
    BaseType_t flag = pdTRUE;
    while (true) {
        if (xQueueReceive(canUrgentQueue, &urgentcmd, 100) == pdTRUE) {
            //保证紧急指令可以被执行
            flag = xSemaphoreTake(canMutex, 100);
            HAL_CAN_AbortTxRequest(motor.motor2can->mCan->hcan, motor.motor2can->mCan->TxMailbox);
            switch (urgentcmd.cmd) {
                case cmd_Stop:
                    while (HAL_OK != motor.topic_cmd(cmd_Stop)) {
                        nh.logfatal("motor stop error!");
                        HAL_Delay(500);
                    }
                    break;
                case cmd_sysReset:
                    while (HAL_OK != motor.topic_cmd(cmd_sysReset)) {
                        nh.logfatal("motor system reset error!");
                        HAL_Delay(500);
                    }
                default:
                    if (HAL_OK != motor.topic_cmd(urgentcmd.cmd, urgentcmd.data)) {
                        char temp[50];
                        sprintf(temp, "the urgent cmd:%s is error!", select_name(urgentcmd.cmd));
                        nh.logwarn(temp);
                    }
                    break;
            }
            if (flag)
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
                            10000);
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

void PressCallbk(void const * argument){
    if(pressCtrl.isOpen)
        vTaskSuspend(NULL);
    TickType_t begin;
    hx711.Get_Maopi();
    vTaskDelay(500);
    while (true){
        begin = xTaskGetTickCount();
        if(pressCtrl.isOpen)
        {
            pressinfo.weight = hx711.Get_Weight();
            pressinfo.header.stamp = nh.now();
            PressPub.publish(&pressinfo);
        }
        vTaskDelayUntil(&begin,pressCtrl.freq);
    }
}
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