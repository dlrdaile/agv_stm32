/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午11:12
* @version: 1.0
* @description: 
********************************************************************************/
#include "Can.h"
#include "UserConfig.h"

#if JLINK_DEBUG == 1

#include "SEGGER_RTT.h"

#endif

Can::Can(CAN_HandleTypeDef &hcan) {
    this->hcan = &hcan;
}

HAL_StatusTypeDef Can::CAN_Init(CAN_FilterTypeDef *canFilter, uint16_t filtersize) {
    HAL_StatusTypeDef result;
    if (canFilter == NULL) {
        CAN_FilterTypeDef canFilter;
        canFilter.FilterBank = 0;
        canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
        canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
        canFilter.FilterIdHigh = 0;
        canFilter.FilterIdLow = 0; //后四位是1100=0xC
        canFilter.FilterMaskIdHigh = 0;
        canFilter.FilterMaskIdLow = 0;
        canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
        canFilter.FilterActivation = ENABLE;
        canFilter.SlaveStartFilterBank = 14;
        result = HAL_CAN_ConfigFilter(this->hcan, &canFilter);
        if (HAL_OK != result) {
            return result;
        }
    } else {
        for (int i = 0; i < filtersize; ++i) {
            result = HAL_CAN_ConfigFilter(this->hcan, canFilter + i);
            if (HAL_OK != result) {
                return result;
            }
        }
    }
    result = HAL_CAN_Start(this->hcan);
    if (result != HAL_OK) {
        return result;
    }
    SEGGER_RTT_printf(0, "CAN is started\n");
/*    __HAL_CAN_ENABLE_IT(this->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(this->hcan, CAN_IT_RX_FIFO1_MSG_PENDING);*/
    return result;
}

CanStatusTypeDef Can::CAN_SendMsg(const uint32_t &ExtID, uint8_t *TxData, const uint8_t &Data_Len) {
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.ExtId = ExtID;
    TxHeader.StdId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = Data_Len;
    TxHeader.TransmitGlobalTime = DISABLE;
    while (HAL_CAN_GetTxMailboxesFreeLevel(this->hcan) < 1) {
    }
    uint32_t TxMailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        SEGGER_RTT_printf(0, "Send to mailbox error!\n");
        return CAN_ADD_MESSAGE_ERROR;
    } else
        return CAN_OK;
}

CanStatusTypeDef Can::CAN_ReadMsg(const uint32_t &EXTID, uint8_t *Rxdata) {
    CAN_RxHeaderTypeDef RxHeader;
    if (HAL_CAN_GetRxFifoFillLevel(this->hcan, CAN_RX_FIFO0) == 0) {
        return CAN_NO_RECEIVE_ERROR;
    }
    if(HAL_CAN_GetRxMessage(this->hcan,CAN_RX_FIFO0,&RxHeader,Rxdata) != HAL_OK)
    {
        return CAN_GET_MESSAGE_ERROR;
    }
    if(EXTID != RxHeader.ExtId)
    {
        return CAN_RECEIVE_ERROR;
    }
    return CAN_OK;
}

/*void Can::CAN_ReadMsg_IT(uint32_t RxFifo) {
    
    string temp;
    CAN_RxHeaderTypeDef RxHeader;
    if (HAL_CAN_GetRxMessage(this->hcan, RxFifo, &RxHeader, this->Rx_CMD_BUFFER) == HAL_OK) {
        if (RxFifo == CAN_RX_FIFO0) {
            temp = "FIFO0";
        } else {
            temp = "FIFO1";
        }
        SEGGER_RTT_printf(0, "%s has receive the data\n", temp.c_str());
        SEGGER_RTT_printf(0, "StdID = 0x%04x\n", RxHeader.StdId);
        SEGGER_RTT_printf(0, "ExtID = 0x%08x\n", RxHeader.ExtId);
        SEGGER_RTT_printf(0, "RTR(0=Data,2=Remote) = %d\n", RxHeader.RTR);
        SEGGER_RTT_printf(0, "IDE(0=Std,4=Ext) = %d\n", RxHeader.IDE);
        if (RxHeader.RTR != 2) {
            for (int i = 0; i < RxHeader.DLC; ++i) {
                if (i == 0) {
                    SEGGER_RTT_printf(0, "DLC = %d\n", RxHeader.DLC);
                    SEGGER_RTT_printf(0, "the received data is :0x");
                }
                SEGGER_RTT_printf(0, "%x", this->Rx_CMD_BUFFER[i]);
                if(i == (RxHeader.DLC - 1))
                {
                    SEGGER_RTT_printf(0, "\n");
                }
            }
        }
        temp.assign(20,'-');
        SEGGER_RTT_printf(0, "%S\n",temp.c_str());
    } else {
        SEGGER_RTT_printf(0, "%s receive data error!\n", temp.c_str());
    }
}*/



