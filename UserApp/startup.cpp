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
#include "Usart.h"
#include "Can.h"

Key hsw2(SW2_GPIO_Port, SW2_Pin);
Key hsw3(SW3_GPIO_Port, SW3_Pin);

Led hled0(LED0_GPIO_Port, LED0_Pin);
Led hled1(LED1_GPIO_Port, LED1_Pin);

Usart debug_uart(&huart4, false);
Usart ros_uart(&huart5, true);

Can motor_can(&hcan1);

void startup() {
    debug_uart.SendData("hello,%s\n", "world");
    if (motor_can.CAN_Init() == HAL_OK)
        debug_uart.SendData("start work!\n");
    while (true) {
        if (!debug_uart.RxBuffer.empty() || !motor_can.RxBUFFER.empty() || !debug_uart.TxBuffer.empty()) {
            while (!debug_uart.RxBuffer.empty()) {
                string str = debug_uart.RxBuffer.front();
                hled0.On();
                //TODO:处理字符串
                debug_uart.RxBuffer.pop();
                HAL_Delay(50);
            }
            while (!motor_can.RxBUFFER.empty()) {
                string str = motor_can.RxBUFFER.front();
                hled1.On();
                //TODO:处理字符串
                motor_can.RxBUFFER.pop();
                HAL_Delay(50);
            }
            while (!debug_uart.TxBuffer.empty()) {
                hled0.Off();
                string str = debug_uart.TxBuffer.front();
                debug_uart.SendData(str);
                debug_uart.TxBuffer.pop();
                HAL_Delay(50);
            }
        } else {
            hled0.Toggle();
            hled1.Toggle();
            HAL_Delay(500);
        }
    }
}

void timely_detect(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        hsw2.Key_timely_detect();
        hsw3.Key_timely_detect();
    }
}

void on_UART_IDLE(UART_HandleTypeDef *huart) {
    if (huart->Instance == debug_uart.huart->Instance) {
        debug_uart.UART_IDLECB();
    }
    if (huart->Instance == ros_uart.huart->Instance) {
        ros_uart.UART_IDLECB();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == debug_uart.huart->Instance) {
        debug_uart.UART_RxCpltCB();
    }
    if (huart->Instance == ros_uart.huart->Instance) {
        ros_uart.UART_RxCpltCB();
    }
}



