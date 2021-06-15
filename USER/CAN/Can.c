
#include "mInclude.h"

static void CAN_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_GPIOA_CLK_ENABLE();
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void CAN_Configuration(uint8_t Bauderate)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	CAN_GPIO_Configuration();
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);	
	NVIC_InitTypeDef NVIC_InitStructure; 
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CAN_InitStructure.CAN_TTCM = DISABLE; 
	CAN_InitStructure.CAN_ABOM = DISABLE; 
	CAN_InitStructure.CAN_AWUM = DISABLE; 
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS2_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS1_7tq; 
	switch(Bauderate){
		case 0://125k
			CAN_InitStructure.CAN_Prescaler = 24;
			break;
		case 1://250k
			CAN_InitStructure.CAN_Prescaler = 12;
			break;
		case 2://500k
			CAN_InitStructure.CAN_Prescaler = 6;
			break;
		case 3://1000k
			CAN_InitStructure.CAN_Prescaler = 3;
			break;
		default:
			CAN_InitStructure.CAN_Prescaler = 12;
			break;			
	}
	CAN_Init(CAN1,&CAN_InitStructure);	
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);	
}

void CanWriteData(uint32_t ID,uint8_t *databuf,uint8_t datalen)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = ID;
	TxMessage.RTR 	= CAN_RTR_DATA;
	TxMessage.IDE 	= CAN_ID_STD;
	TxMessage.DLC 	= datalen;
	for(uint8_t i = 0;i < datalen;i++)
	{
		TxMessage.Data[i] = databuf[i]; 
	}
	CAN_Transmit(CAN1,&TxMessage);
}

