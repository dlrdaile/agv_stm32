/********************************************************************************
 * @author: dai le
 * @email: 965794928@qq.com
 * @date: 2022/2/8 下午11:42
 * @version: 1.0
 * @description:
 ********************************************************************************/
#include "startup.h"
#include "Key.h"
#include "Led.h"
#include "Can.h"
#include "Motor.h"

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#if JLINK_DEBUG == 1
#include "SEGGER_RTT.h"
#endif
Key hsw2(SW2_GPIO_Port, SW2_Pin);
Key hsw3(SW3_GPIO_Port, SW3_Pin);

Led hled0(LED0_GPIO_Port, LED0_Pin);
Led hled1(LED1_GPIO_Port, LED1_Pin);

#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
volatile unsigned long  long run_time_stats_tick;
#endif
Motor motor(hcan1);
void startup() {
    if(motor.InitState() != HAL_OK)
    {
        SEGGER_RTT_printf(0,"motor init error!please check the error\n");
        while (1);
    }
/*    uint8_t temp[8];
    SEGGER_RTT_Init();
    SEGGER_RTT_printf(0,"hello world!\n");
    if(HAL_OK != motor_can.CAN_Init())
    {
        SEGGER_RTT_printf(0,"can init error!\n");
        while (1);
    }
    temp[0] = (uint8_t)(0xAABBCCDD);
    motor_can.CAN_SendMsg(Generate_msgID(42,0),temp,4);
    uint32_t rand;
    while (1)
    {
        HAL_RNG_GenerateRandomNumber(&hrng,&rand);
        temp[3] = rand & 0x000000FF;
        temp[2] = (rand & 0x0000FF00)>>8;
        temp[1] = (rand & 0x00FF0000)>>16;
        temp[0] = (rand & 0xFF000000)>>24;
        for (int i = 0; i < 4; ++i) {
            if(i == 0)
            SEGGER_RTT_printf(0,"the sending data is :0x:");
            SEGGER_RTT_printf(0,"%x",temp[i]);
        }
        SEGGER_RTT_printf(0,"\n");
        motor_can.CAN_SendMsg(Generate_msgID(temp[0],temp[1]),temp,4);
        hled1.Toggle();
        HAL_Delay(1000);
    }*/
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

//对c的weak函数进行重写
extern "C" {
void rosPubCallbk(void const * argument)
{

}

void CanProcessTCallbk(void const * argument){

}

void rosSubCallbk(void const * argument){

}
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

#if (configGENERATE_RUN_TIME_STATS == 1) && (JLINK_DEBUG == 1)
extern "C" {
void configureTimerForRunTimeStats(void) {
    run_time_stats_tick = 0;
}
unsigned long getRunTimeCounterValue(void){
    return run_time_stats_tick;
}
}
#endif




