/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午11:12
* @version: 1.0
* @description: 
********************************************************************************/
#include "Can.h"
//#include "startup.h"
#include "Usart.h"

extern Usart debug_uart;

Can::Can(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *canFilter) {
    this->hcan = hcan;
    this->canFilter = canFilter;
    if (canFilter == NULL) {
        this->canFilter = new CAN_FilterTypeDef;
        this->canFilter->FilterBank = 0;
        this->canFilter->FilterMode = CAN_FILTERMODE_IDMASK;
        this->canFilter->FilterScale = CAN_FILTERSCALE_32BIT;

        this->canFilter->FilterIdHigh = 0;
        this->canFilter->FilterIdLow = 0; //后四位是1100=0xC
//    this->canFilter->FilterMaskIdHigh = 0xF800;
//    thi->scanFilter->FilterMaskIdLow = 0x000E;
        this->canFilter->FilterMaskIdHigh = 0;
        this->canFilter->FilterMaskIdLow = 0;

        this->canFilter->FilterFIFOAssignment = CAN_RX_FIFO0;
        this->canFilter->FilterActivation = ENABLE;
        this->canFilter->SlaveStartFilterBank = 14;
    } else {
        this->canFilter = new CAN_FilterTypeDef;
        this->canFilter->FilterBank = canFilter->FilterBank;
        this->canFilter->FilterMode = canFilter->FilterMode;
        this->canFilter->FilterScale = canFilter->FilterScale;

        this->canFilter->FilterIdHigh = canFilter->FilterIdHigh;
        this->canFilter->FilterIdLow = canFilter->FilterIdLow; //后四位是1100=0xC
//    thiscanFilter.FilterMaskIdHigh = 0xF800;
//    thiscanFilter.FilterMaskIdLow = 0x000E;
        this->canFilter->FilterMaskIdHigh = canFilter->FilterMaskIdHigh;
        this->canFilter->FilterMaskIdLow = canFilter->FilterMaskIdLow;

        this->canFilter->FilterFIFOAssignment = canFilter->FilterFIFOAssignment;
        this->canFilter->FilterActivation = canFilter->FilterActivation;
        this->canFilter->SlaveStartFilterBank = canFilter->SlaveStartFilterBank;
    }
}

Can::~Can() {
    if (this->canFilter != NULL) {
        delete this->canFilter;
        this->canFilter = NULL;
    }
}

HAL_StatusTypeDef Can::CAN_Init() {
    HAL_StatusTypeDef result = HAL_CAN_ConfigFilter(this->hcan, this->canFilter);
    if(result != HAL_OK) {
        return result;
    }
    debug_uart.SendData("ID Filter has been set\n");
    result = HAL_CAN_Start(this->hcan);
    if(result != HAL_OK) {
        return result;
    }
    debug_uart.SendData("CAN is started\n");
    if(this->canFilter->FilterFIFOAssignment == CAN_RX_FIFO0)
        __HAL_CAN_ENABLE_IT(this->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    else
        __HAL_CAN_ENABLE_IT(this->hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    return result;
}

void Can::CAN_SendMsg(uint8_t Instruct, uint8_t Index, uint8_t *TxData, uint8_t Data_Len) {
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.ExtId = Generate_msgID(Instruct, Index);
    TxHeader.StdId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = Data_Len;
    TxHeader.TransmitGlobalTime = DISABLE;
    while (HAL_CAN_GetTxMailboxesFreeLevel(this->hcan) < 1) {
    }
    debug_uart.SendData("Send MsgID = 0x%08x\n", TxHeader.ExtId);
    uint32_t TxMailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        debug_uart.SendData("Send to mailbox error!\n");
    }
}

void Can::CAN_ReadMsg() {
    CAN_RxHeaderTypeDef RxHeader;
    if (HAL_CAN_GetRxMessage(this->hcan, this->canFilter->FilterFIFOAssignment, &RxHeader, this->Rx_CMD_BUFFER) !=
        HAL_OK) {
        Rx_CMD_BUFFER[RxHeader.DLC] = '\0';
        string temp = (char *) Rx_CMD_BUFFER;
        this->RxBUFFER.push(temp);
    } else {
        debug_uart.SendData("receive data error!\n");
    }
}


