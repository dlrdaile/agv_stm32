/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午5:32
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_USART_H
#define PROJECT_CAR_USART_H

#include "main.h"
#include <queue>
#include <string>
#include "usart.h"
#include <cstdarg>
#define MAX_BUFFER_SIZE 100
#define LIMIT_BuFFER_SIZE 10
using namespace std;
/*typedef enum {
    UART_FREE = 0,
    UART_BUSY
} UartState_TypeDef;*/

class Usart {
public:
    Usart(UART_HandleTypeDef *huart, bool IslimitLen,uint32_t Timeout = HAL_MAX_DELAY);

//    ~Usart();
    HAL_StatusTypeDef SendData(string &data);
    HAL_StatusTypeDef SendData(const char data,uint16_t len = 1);
    HAL_StatusTypeDef SendData(const char *format,...);
    void UART_RxCpltCB();
    void UART_IDLECB();

private:
    int Print_data(char * str,const char *format, va_list args);

public:
    UART_HandleTypeDef *huart;
    bool IslimitLen;
//    uint8_t *p_Char_Num;
//    UartState_TypeDef *p_work_flag;
    uint8_t Char_Num;
//    UartState_TypeDef work_flag;
    queue<string> RxBuffer;
    queue<string> TxBuffer;
    uint8_t *Rx_CMD_Buffer;
    uint8_t *Rx_Uart_Data;
    uint32_t Timeout;
};


#endif //PROJECT_CAR_USART_H