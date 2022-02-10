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
#include <string>
#include <queue>
using namespace std;
#define ADRRESS 1
#define Generate_msgID(Instruct,Index) (uint32_t)(((Index)<<16) | ((Instruct)<<8) | (ADRRESS))

class Can {
public:
    Can(CAN_HandleTypeDef *hcan,CAN_FilterTypeDef *canFilter = NULL);
    ~Can();
    HAL_StatusTypeDef CAN_Init();
    void CAN_SendMsg(uint8_t Instruct,uint8_t Index, uint8_t *TxData, uint8_t Data_Len);
    void CAN_ReadMsg ();
public:
    CAN_FilterTypeDef *canFilter;
    CAN_HandleTypeDef *hcan;
    queue<string> RxBUFFER;
    uint8_t Rx_CMD_BUFFER[16];
};
#endif //PROJECT_CAR_CAN_H