/********************************************************************************
* @author: dai le
* @email: 965794928@qq.com
* @date: 2022/2/9 下午11:12
* @version: 1.0
* @description: 
********************************************************************************/
#include "Can.h"
#include "UserConfig.h"
#include "string"
#include "Motor.h"

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
#if JLINK_DEBUG == 1
    SEGGER_RTT_printf(0, "CAN is started\n");
#endif
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
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &this->TxMailbox) != HAL_OK) {
#if JLINK_DEBUG == 1
        SEGGER_RTT_printf(0, "Send to mailbox error!\n");
#endif
        return CAN_ADD_MESSAGE_ERROR;
    } else {
#if JLINK_DEBUG == 1
        if ((ExtID == oneMS_Encoder_msgID) || (ExtID == oneMS_Encoder_msgID)) {
            while (HAL_CAN_GetTxMailboxesFreeLevel(this->hcan) < 1) {
            }
            TxHeader.ExtId = (uint32_t) (ExtID | (1 << 16));
            if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
                return CAN_ADD_MESSAGE_ERROR;
            }
        }
#endif
        return CAN_OK;
    }
}

CanStatusTypeDef Can::CAN_ReadMsg(const uint32_t &EXTID, uint8_t *Rxdata) {
    CAN_RxHeaderTypeDef RxHeader;
    if (HAL_CAN_GetRxFifoFillLevel(this->hcan, CAN_RX_FIFO0) == 0) {
        return CAN_NO_RECEIVE_ERROR;
    }
    if (HAL_CAN_GetRxMessage(this->hcan, CAN_RX_FIFO0, &RxHeader, Rxdata) != HAL_OK) {
        return CAN_GET_MESSAGE_ERROR;
    }
    if (EXTID != RxHeader.ExtId) {
        return CAN_RECEIVE_ERROR;
    }

#if JLINK_DEBUG == 1
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
            SEGGER_RTT_printf(0, "%02x", Rxdata[i]);
            if (i == (RxHeader.DLC - 1)) {
                SEGGER_RTT_printf(0, "\n");
            }
        }
    }
    SEGGER_RTT_printf(0, "--------------------------\n\n");

#endif
    if ((EXTID == oneMS_Encoder_msgID) || (EXTID == EncoderData_msgID)) {
        uint8_t *p = (uint8_t *) ((uint32_t *) Rxdata + 1);
        if (HAL_OK != HAL_CAN_GetRxMessage(this->hcan, CAN_RX_FIFO0, &RxHeader, p)) {
            return CAN_NO_RECEIVE_ERROR;
        }
#if JLINK_DEBUG == 1
        SEGGER_RTT_printf(0, "the oneMS_Encoder_msgID's second ID:\n");
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
                SEGGER_RTT_printf(0, "%02x", p[i]);
                if (i == (RxHeader.DLC - 1)) {
                    SEGGER_RTT_printf(0, "\n");
                }
            }
        }
        SEGGER_RTT_printf(0, "--------------------------\n\n");
#endif
    }
    return CAN_OK;
}


