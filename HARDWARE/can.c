#include "can.h"
#include "string.h"

extern volatile int flag_can;
extern CAN_HandleTypeDef hcan;
static CanTxMsgTypeDef canTxMsg;
static CanRxMsgTypeDef canRxMsg;

CAN_FilterConfTypeDef CanFilterConfig;

void Config_CanFilter(void)
{
    CanFilterConfig.FilterNumber        =   0;
    CanFilterConfig.FilterMode          =   CAN_FILTERMODE_IDLIST;
    CanFilterConfig.FilterScale         =   CAN_FILTERSCALE_32BIT;
    
    CanFilterConfig.FilterIdHigh        =   (0x052d<<5);
    CanFilterConfig.FilterIdLow         =   0x0000;
    CanFilterConfig.FilterMaskIdHigh    =   (0x052d<<5);
    CanFilterConfig.FilterMaskIdLow     =   0x0000;
    
    CanFilterConfig.FilterFIFOAssignment    =   CAN_FILTER_FIFO0;
    CanFilterConfig.FilterActivation        =   ENABLE;
    
    HAL_CAN_ConfigFilter(&hcan,&CanFilterConfig);
    
    hcan.pTxMsg     =   &canTxMsg;
    hcan.pRxMsg     =   &canRxMsg;

    HAL_CAN_Receive_IT(&hcan,CAN_FILTER_FIFO0);
}

void sendCanData(uint32_t id,uint8_t * buf,uint32_t len)
{
    hcan.pTxMsg->StdId      =       id;
    hcan.pTxMsg->RTR        =       CAN_RTR_DATA;
    hcan.pTxMsg->IDE        =       CAN_ID_STD;
    hcan.pTxMsg->DLC        =       len;
    
    memcpy(canTxMsg.Data,buf,len);
    
    while(HAL_CAN_Transmit(&hcan,100) != HAL_OK);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
    HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
	
	flag_can = 1;
}
