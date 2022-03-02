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
#include "ros.h"
#include <string>
Key hsw2(SW2_GPIO_Port, SW2_Pin);
Key hsw3(SW3_GPIO_Port, SW3_Pin);

Led hled0(LED0_GPIO_Port, LED0_Pin);
Led hled1(LED1_GPIO_Port, LED1_Pin);

Usart debug_uart(huart4, false);
Usart ros_uart(huart5, false);

//Can motor_can(&hcan1);

void startup() {
    std::string s1 = "hello world from cpp2\n";
    debug_uart.SendData(s1);
    debug_uart.SendData('a',10);
    debug_uart.SendData("hello format\n");
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



