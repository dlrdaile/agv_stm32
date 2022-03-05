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
    /**
     * @brief Can类构造
     * @param hcan cubemx生成的hcan
     */
    Can(CAN_HandleTypeDef &hcan);
    /**
     * @brief 对can进行初始化，包括设置过滤器和使能器工作
     * @param canFilter 过滤器数组，默认无过滤
     * @param filtersize 过滤器的数量，默认为0
     * @return 初始化状态
     */
    HAL_StatusTypeDef CAN_Init(CAN_FilterTypeDef *canFilter = NULL,uint16_t filtersize = 0);
    /**
     * @brief Can发送数据
     * @param ExtID 发送的ID,任务原因，默认都是扩展ID
     * @param TxData 发送数据的数组
     * @param Data_Len 发送的字节数
     * @return 发送状态
     */
    CanStatusTypeDef CAN_SendMsg(const uint32_t &ExtID,uint8_t *TxData,const uint8_t &Data_Len=0);
    /**
     * @brief 轮询读取接受到的数据
     * @param ExtID 刚刚发送数据的EXTid
     * @param Rxdata 用以接收数据的数组
     * @return 接收状态
     */
    CanStatusTypeDef CAN_ReadMsg (const uint32_t &ExtID,uint8_t *Rxdata);
public:
    CAN_HandleTypeDef *hcan;
    uint32_t TxMailbox;
};
#endif //PROJECT_CAR_CAN_H