/********************************************************************************
 * @author: dai le
 * @email: 965794928@qq.com
 * @date: 2022/2/8 下午11:42
 * @version: 1.0
 * @description:
 ********************************************************************************/
 //STM32的库函数
#include "main.h"

//C++官方库
#include "string"
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

//Freertos的库函数
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//Jlink调试打印的函数库
#if JLINK_DEBUG == 1
#include "SEGGER_RTT.h"
#endif

//用户变量初始化区
Key hsw2(SW2_GPIO_Port, SW2_Pin);
Key hsw3(SW3_GPIO_Port, SW3_Pin);

Led hled0(LED0_GPIO_Port, LED0_Pin);
Led hled1(LED1_GPIO_Port, LED1_Pin);

#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
volatile unsigned long long run_time_stats_tick;
#endif

Motor motor(hcan1);
ros::NodeHandle nh;

xQueueHandle canSendQueue;
xQueueHandle canParseQueue;
xQueueHandle canUrgentQueue;

void startup() {
    nh.initNode();
    SEGGER_RTT_Init();
    while (1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    nh.getHardware()->reset_rbuf();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    nh.getHardware()->flush();
}


void timely_detect(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
        run_time_stats_tick++;
#endif
    }
}

/*void on_UART_IDLE(UART_HandleTypeDef *huart) {
    if (huart->Instance == debug_uart.huart->Instance) {
        debug_uart.UART_IDLECB();
    }
    if (huart->Instance == ros_uart.huart->Instance) {
        ros_uart.UART_IDLECB();
    }
}*/

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == debug_uart.huart->Instance) {
        debug_uart.UART_RxCpltCB();
    }
    if (huart->Instance == ros_uart.huart->Instance) {
        ros_uart.UART_RxCpltCB();
    }
}*/

HAL_StatusTypeDef create_Queue() {
    canSendQueue = xQueueCreate(3, sizeof(communicate_with_stm32::MotorCmd));
    canUrgentQueue = xQueueCreate(3, sizeof(communicate_with_stm32::MotorCmd));
    canParseQueue = xQueueCreate(3, sizeof(CarState_TypeDef));
    if (canUrgentQueue != NULL && canSendQueue != NULL && canParseQueue != NULL) {
        return HAL_OK;
    }
    return HAL_ERROR;
}

//对c的weak函数进行重写
extern "C" {


void rosPubCallbk(void const *argument) {
    while (true) {

    }
}

void CanProcessTCallbk(void const *argument) {
    while (true) {

    }
}

void rosSubCallbk(void const *argument) {
    while (true) {

    }
}

void CanUrgentCallbk(void const *argument) {
    communicate_with_stm32::MotorCmd urgentcmd;
    while (true) {
        xQueueReceive(canUrgentQueue, &urgentcmd, portMAX_DELAY);
        HAL_CAN_AbortTxRequest(motor.mCan->hcan, motor.mCan->TxMailbox);
        switch (urgentcmd.cmd) {
            case cmd_Stop:
                while(HAL_OK!=motor.stop()){
                    nh.logfatal("motor stop error!");
                    HAL_Delay(500);
                }
                break;
            case cmd_sysReset:
                while (HAL_OK !=motor.motion_system_reset())
                {
                    nh.logfatal("motor system reset error!");
                    HAL_Delay(500);
                }
            default:
                if(HAL_OK!=motor.run_cmd(urgentcmd.cmd))
                {
                    char temp[50];
                    sprintf(temp,"the urgent cmd:%d is error!",urgentcmd.cmd);
                    nh.logwarn(temp);
                }
                break;
        }
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
/*void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        motor_can.CAN_ReadMsg_IT(CAN_RX_FIFO0);
    }
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        motor_can.CAN_ReadMsg_IT(CAN_RX_FIFO1);
    }
}*/
}






