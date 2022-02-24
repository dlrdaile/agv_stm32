/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午5:32
* @version: 1.0
* @description: 
********************************************************************************/
#include "Usart.h"
#include <cstdio>

/*Usart::Usart(UART_HandleTypeDef *huart, bool IslimitLen) {
    this->huart = huart;
    this->IslimitLen = IslimitLen;
    this->p_Char_Num = NULL;
    this->p_work_flag = NULL;
    if(!this->IslimitLen)
    {
           this->p_Char_Num = new uint8_t;
           this->p_work_flag = new UartState_TypeDef;
    }
}*/

Usart::~Usart() {
    if (this->Rx_CMD_Buffer != NULL) {
        delete[] this->Rx_CMD_Buffer;
        this->Rx_CMD_Buffer = NULL;
    }
    if (this->Rx_Uart_Data != NULL) {
        delete[] this->Rx_Uart_Data;
        this->Rx_Uart_Data = NULL;
    }
}

Usart::Usart(UART_HandleTypeDef *huart, bool IslimitLen, uint32_t Timeout) {
    this->huart = huart;
    this->IslimitLen = IslimitLen;
    this->Char_Num = 0;
    this->Timeout = Timeout;
    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(huart, UART_IT_TC);
    if (this->IslimitLen) {
        this->Rx_CMD_Buffer = new uint8_t[LIMIT_BuFFER_SIZE + 1];
        this->Rx_CMD_Buffer[LIMIT_BuFFER_SIZE] = '\0';
        this->Rx_Uart_Data = NULL;
        HAL_UART_Receive_DMA(this->huart, this->Rx_CMD_Buffer, LIMIT_BuFFER_SIZE);
    } else {
        this->Rx_CMD_Buffer = new uint8_t[1];
        this->Rx_Uart_Data = new uint8_t[MAX_BUFFER_SIZE];
        HAL_UART_Receive_DMA(this->huart, this->Rx_CMD_Buffer, 1);
    }
}

HAL_StatusTypeDef Usart::SendData(string &data) {
    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(this->huart, (uint8_t *) data.c_str(), data.size(), this->Timeout);
    return status;
}

HAL_StatusTypeDef Usart::SendData(const char data, uint16_t len) {
    HAL_StatusTypeDef status;
    string temp(len, data);
    status = HAL_UART_Transmit(this->huart, (uint8_t *) temp.c_str(), temp.size(), this->Timeout);
    return status;
}

HAL_StatusTypeDef Usart::SendData(const char *format, ...) {
    HAL_StatusTypeDef status;
    char temp_txBuffer[100];
    va_list args;
    va_start(args, format);
    this->Print_data(temp_txBuffer, format, args);
    va_end(args);
    string temp(temp_txBuffer);
    status = HAL_UART_Transmit(this->huart, (uint8_t *) temp.c_str(), temp.size(), this->Timeout);
    return status;
}

void Usart::UART_RxCpltCB() {
    if (this->IslimitLen) {
        string rxdata = (char *) this->Rx_CMD_Buffer;
        this->RxBuffer.push(rxdata);
    } else {
        if (this->Rx_CMD_Buffer[0] == ';') {
            if (this->Char_Num != 0) {
                this->Rx_Uart_Data[this->Char_Num] = '\0';
                string rxdata = (char *) this->Rx_Uart_Data;
                this->RxBuffer.push(rxdata);
                this->Char_Num = 0;
            } else {
                this->SendData("the current instruct is empty!\n");
            }
        } else {
            this->Rx_Uart_Data[this->Char_Num] = this->Rx_CMD_Buffer[0];
            this->Char_Num++;
            if (this->Char_Num >= MAX_BUFFER_SIZE) {
                this->SendData("the instruct is so long,and it cannot be show!\n");
                this->Char_Num = 0;
            }
        }
    }
    __HAL_UART_ENABLE_IT(this->huart, UART_IT_IDLE);
}

void Usart::UART_IDLECB() {
    if (__HAL_UART_GET_IT_SOURCE(this->huart, UART_IT_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(this->huart);
        __HAL_UART_DISABLE_IT(this->huart, UART_IT_IDLE);
        uint8_t data_len = this->IslimitLen ? LIMIT_BuFFER_SIZE : 1;
        HAL_UART_Receive_DMA(this->huart, this->Rx_CMD_Buffer, data_len);
    }
}

int Usart::Print_data(char *str, const char *format, va_list args) {
    int done;
    done = vsprintf(str, format, args);
    return done;
}

Usart::Usart(Usart &u) {
    this->huart = u.huart;
    this->IslimitLen = u.IslimitLen;
    this->Char_Num = 0;
    this->Timeout = u.Timeout;
    __HAL_UART_DISABLE_IT(u.huart, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(u.huart, UART_IT_TC);
    if (this->IslimitLen) {
        this->Rx_CMD_Buffer = new uint8_t[LIMIT_BuFFER_SIZE + 1];
        this->Rx_Uart_Data = NULL;
        HAL_UART_Receive_DMA(this->huart, this->Rx_CMD_Buffer, LIMIT_BuFFER_SIZE);
    } else {
        this->Rx_CMD_Buffer = new uint8_t[1];
        this->Rx_Uart_Data = new uint8_t[MAX_BUFFER_SIZE];
        HAL_UART_Receive_DMA(this->huart, this->Rx_CMD_Buffer, 1);
    }
}



