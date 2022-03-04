/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午11:12
* @version: 1.0
* @description: 
********************************************************************************/
#ifndef PROJECT_CAR_CAN_H
#define PROJECT_CAR_CAN_H
#include "main.h"
#include "can.h"
using namespace std;
#define ADRRESS 1
#define Generate_msgID(Instruct,Index) (uint32_t)(((Index)<<16) | ((Instruct)<<8) | (ADRRESS))

typedef enum {
    CAN_OK = 0,
    CAN_NO_RECEIVE_ERROR,
    CAN_GET_MESSAGE_ERROR,
    CAN_RECEIVE_ERROR,
    CAN_ADD_MESSAGE_ERROR
} CanStatusTypeDef;

class Can {
public:
    Can(CAN_HandleTypeDef &hcan);
    HAL_StatusTypeDef CAN_Init(CAN_FilterTypeDef *canFilter = NULL,uint16_t filtersize = 0);
    CanStatusTypeDef CAN_SendMsg(const uint32_t &ExtID,uint8_t *TxData,const uint8_t &Data_Len=0);
    CanStatusTypeDef CAN_ReadMsg (const uint32_t &ExtID,uint8_t *Rxdata);
//    void CAN_ReadMsg_IT (uint32_t RxFifo);
public:
    CAN_HandleTypeDef *hcan;
//    uint8_t Rx_CMD_BUFFER[8];
};
#endif //PROJECT_CAR_CAN_H