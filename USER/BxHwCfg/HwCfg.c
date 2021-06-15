#include "mInclude.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"
////////////

cyc_task_fun cycTask[]={cyc2ms,cyc4ms,cyc20ms,cyc100ms,cyc500ms,cycLongTsk};
uint32_t ComBauderate[4] = {9600,19200,38400,115200};
//////////Delare/////////
gMachineIO_type gMachineIO;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

/////for LED/////
uint8_t cntFlash,cntFlash500;
bool flashLED[10];

void TIM2_PWM_Init(u32 arr,u32 psc);
////////////////////
#define Uart6RS485RE SetDO(RS485DE,0)
#define Uart6RS485SE SetDO(RS485DE,1)
#define Uart6RS485RE2 SetDO(RS485DE2,0)
#define Uart6RS485SE2 SetDO(RS485DE2,1)
void SystemConfig()
{
	delay_init(168);
	HwCfgInit();
};
void HwCfgInit()
{
	/////////////initial output///////////
	CfgPINOut(DOLED);	
	CfgPINOut(RS485DE);	
	CfgPINOut(RS485DE2);	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/////////////Obtain the buffParameter/////////
	//CfgUartx(Uartx6,3,Uart6TX,Uart6RX);
	CfgUartx(Uartx1,3,Uart1TX,Uart1RX);
	//Uart6RS485RE;//
	Uart6RS485RE2;
	////////////
	TimCfg(1000,TIMx3);
	////////////
};
void CfgPINIn(uint32_t GPIOx, uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  APBCLKCfg(GPIOx, ENABLE); 						// 使能PC端口时钟
	//////////////设置成输入的，频率是50M/////////////
  GPIO_InitStructure.GPIO_Mode = GPIOMode;
	GPIO_InitStructure.GPIO_PuPd = GPIOPupd;
	///////////////////////////////////////////////////
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;	
  GPIO_Init((GPIO_TypeDef *)GPIOx,&GPIO_InitStructure);
};
void CfgPINOut(uint32_t GPIOx,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd,GPIOOType_TypeDef OType)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	APBCLKCfg(GPIOx, ENABLE); 						// 使能PC端口时钟
	//////////////设置成输出的，频率是50M//////////////
  GPIO_InitStructure.GPIO_Mode = GPIOMode;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIOPupd;
	GPIO_InitStructure.GPIO_OType = OType;
	////////////////////////////////////////////////////
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;	
  GPIO_Init((GPIO_TypeDef *)GPIOx,&GPIO_InitStructure);
  GPIO_ResetBits((GPIO_TypeDef *)GPIOx,GPIO_Pin);
};

void APBCLKCfg( uint32_t GPIOx,FunctionalState NewState)
{
		if(GPIOx>=GPIOA_BASE && GPIOx<=DMA1_BASE)
			RCC_AHB1PeriphClockCmd(GPIO2APB2CLK(GPIOx), NewState);	
		else if(GPIOx>=TIM1_BASE && GPIOx<=SAI1_BASE)
			RCC_APB2PeriphClockCmd(GPIO2APB2CLK(GPIOx), NewState);	
		else if(GPIOx>=TIM2_BASE && GPIOx<=UART8_BASE)
			RCC_APB1PeriphClockCmd(GPIO2APB2CLK(GPIOx), NewState);	
	
}
uint32_t GPIO2APB2CLK(uint32_t  GPIOx)
{
	switch (GPIOx)
	{
		case PxA:
			return RCC_AHB1Periph_GPIOA;
	
		
		case PxB:
			return RCC_AHB1Periph_GPIOB;
		
		
		case PxC:
			return RCC_AHB1Periph_GPIOC;

		
		case PxD:
			return RCC_AHB1Periph_GPIOD;

		
		case Uartx1:
			return RCC_APB2Periph_USART1;

		
		case Uartx6:
			return RCC_APB2Periph_USART6;
		
		
		case Uartx3:
			return RCC_APB1Periph_USART3;
		
		
		case TIMx3:
			return RCC_APB1Periph_TIM3;
		
	}
	return 0;
}

uint8_t UAartAFR(uint32_t UartX)
{
	switch (UartX)
	{
		case USART1_BASE: 
			return GPIO_AF_USART1;

		case USART6_BASE:
			return GPIO_AF_USART6;

		case USART3_BASE:
			return GPIO_AF_USART3;
	}
	return 0;
}

uint8_t AFPinsource(uint16_t PinDefine)
{
	switch (PinDefine)
	{
		case GPIO_Pin_0: 
			return GPIO_PinSource0;

		case GPIO_Pin_1: 
			return GPIO_PinSource1;

		case GPIO_Pin_2: 
			return GPIO_PinSource2;

		case GPIO_Pin_3: 
			return GPIO_PinSource3;

		case GPIO_Pin_4: 
			return GPIO_PinSource4;

		case GPIO_Pin_5: 
			return GPIO_PinSource5;

		case GPIO_Pin_6: 
			return GPIO_PinSource6;

		case GPIO_Pin_7: 
			return GPIO_PinSource7;

		case GPIO_Pin_8: 
			return GPIO_PinSource8;

		case GPIO_Pin_9: 
			return GPIO_PinSource9;

		case GPIO_Pin_10: 
			return GPIO_PinSource10;

		case GPIO_Pin_11: 
			return GPIO_PinSource11;

		case GPIO_Pin_12: 
			return GPIO_PinSource12;

		case GPIO_Pin_13: 
			return GPIO_PinSource13;

		case GPIO_Pin_14: 
			return GPIO_PinSource14;

		case GPIO_Pin_15: 
			return GPIO_PinSource15;
	}
	return 0;
}

uint8_t ReadIn(uint32_t GPIOxADR, uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd)
{
	
	GPIO_TypeDef* GPIOx;
	uint8_t bitstatus = 0x00;
	GPIOx=(GPIO_TypeDef*)GPIOxADR;
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin)); 
  if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)Bit_RESET)
    bitstatus = (uint8_t)Bit_SET;
  else
    bitstatus = (uint8_t)Bit_RESET;
  return bitstatus;
	
}
void SetDO(uint32_t GPIOxADR,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd,GPIOOType_TypeDef OType,bool state)
{
	
	GPIO_TypeDef* GPIOx;
	GPIOx=(GPIO_TypeDef*)GPIOxADR;	
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
	if(state) GPIOx->BSRRL = GPIO_Pin;
	else GPIOx->BSRRH = GPIO_Pin;
}

void CfgUartx(uint32_t UartX,uint8_t uartPar,uint32_t GPTx, uint16_t GPTX_Pin,uint32_t GPRx, uint16_t GPRX_Pin)
{
	
	GPIO_InitTypeDef GPIO_InitStructureTx;
	GPIO_InitTypeDef GPIO_InitStructureRx;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	APBCLKCfg(UartX, ENABLE);
	APBCLKCfg(GPTx, ENABLE);
	APBCLKCfg(GPRx, ENABLE);
	
	GPIO_PinAFConfig((GPIO_TypeDef *)GPTx, AFPinsource(GPTX_Pin), UAartAFR(UartX));//Standard meathod
	GPIO_PinAFConfig((GPIO_TypeDef *)GPRx, AFPinsource(GPRX_Pin), UAartAFR(UartX));//Standard meathod
	
  GPIO_InitStructureTx.GPIO_Pin = GPTX_Pin;
  GPIO_InitStructureTx.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructureTx.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureTx.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructureTx.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init((GPIO_TypeDef *)GPTx, &GPIO_InitStructureTx); 
		
  GPIO_InitStructureRx.GPIO_Pin = GPRX_Pin;
  GPIO_InitStructureRx.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructureRx.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureRx.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructureRx.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init((GPIO_TypeDef *)GPRx, &GPIO_InitStructureRx); 
	
	USART_InitStructure.USART_BaudRate = ComBauderate[uartPar];											//波特率设置：115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;											//数据位数设置：8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 													//停止位设置：1位
	USART_InitStructure.USART_Parity = USART_Parity_No ;  													//是否奇偶校验：无
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制模式设置：没有使能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;									//接收与发送都使能
	
	USART_Init((USART_TypeDef *)UartX, &USART_InitStructure);  
	USART_Cmd((USART_TypeDef *)UartX, ENABLE);		
	USART_ITConfig((USART_TypeDef *)UartX,USART_IT_RXNE,ENABLE);										//初始化USART1
																																									// USART1使能
	NVIC_InitStruct.NVIC_IRQChannel = MapIRQn(UartX);
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;     											// 主优先级为1
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;            											// 次优先级为0
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
}
enum IRQn MapIRQn(uint32_t BASEType)
{
	switch (BASEType)
	{
		case Uartx1:
			return USART1_IRQn;
		case Uartx6:
			return USART6_IRQn;
		case Uartx3:
			return USART3_IRQn;
		case TIMx3:
			return TIM3_IRQn;	
	}
	return TIM3_IRQn;
}
void USART1_IRQHandler(void)
{
	FillUrtBuf(&(gMachineIO.Uart1Data),Uartx1);//////485-2//////
}
void USART6_IRQHandler(void)
{
	FillUrtBuf(&(gMachineIO.Uart6Data),Uartx6);//////485-1//////
}
void FillUrtBuf(UrtBuf_type * pBoxIO,uint32_t USARTx)
{
	USART_TypeDef * Uarts;
	Uarts = (USART_TypeDef*)USARTx;
	if(USART_GetITStatus(Uarts, USART_IT_RXNE) != RESET)
	{
		uint8_t ResData;
		uint8_t iFill;
		ResData = Uarts->DR;		
		iFill=pBoxIO->pRfil;
		pBoxIO->rBuffer[iFill] = ResData;
		pBoxIO->pRfil++;		
		if(pBoxIO->pRfil >= UrtBfLen)
		{
			if(pBoxIO->pRder>0)pBoxIO->pRder--;
			pBoxIO->pRfil = UrtBfLen-1;
			MBLArry((int8_t*)pBoxIO->rBuffer,UrtBfLen);
		}
	}
}
void MBLArry(int8_t *buffer,uint8_t bufLen)
{
	uint8_t iArry;
	if(bufLen<1) bufLen=1;
	for(iArry=0;iArry<bufLen-1;iArry++)
	{
		buffer[iArry] = buffer[iArry+1];
	}
}

void SendUrtBuf(UrtBuf_type * pBoxIO,uint32_t USARTx)
{
		unsigned char i;
		unsigned char bufLen = pBoxIO->sLen;
		USART_TypeDef * Uarts;
		Uarts = (USART_TypeDef*)USARTx;
		switch(USARTx)
		{
		  case Uartx6:
				Uart6RS485SE;
				break;
			case Uartx1:
				Uart6RS485SE2;
				break;
			default:
				;
				break;
		}
		for(i=0;i<bufLen;i++)
		{	
			USART_SendData(Uarts, pBoxIO->sBuffer[i]);
			while (USART_GetFlagStatus(Uarts, USART_FLAG_TXE) == RESET);			
		}
		while(USART_GetFlagStatus(Uarts, USART_FLAG_TC)==RESET);
		switch(USARTx)
		{
		  case Uartx6:
				Uart6RS485RE;
				break;
			case Uartx1:
				Uart6RS485RE2;
				break;
			default:
				;
				break;
		}
};
void TimCfg(uint32_t timeUs ,uint32_t BASEType)
{
	uint16_t arr=timeUs/100-1;

	TIM_TypeDef * TIMxS;
	TIMxS = (TIM_TypeDef * )BASEType;
	APBCLKCfg(BASEType, ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
  TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler=8400-1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIMxS,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIMxS,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIMxS,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=MapIRQn(BASEType);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
 
void TIM3_IRQHandler(void)
{
	static uint32_t oldTimeStamp[6];
	if(TIM3->SR&0X0001)
	{		
		timeBoot = timeBoot + 0.001;
		gMachineIO.TimeStamp++;
		oldTimeStamp[task2ms]++;
		oldTimeStamp[task4ms]++;
		oldTimeStamp[task20ms]++;
		oldTimeStamp[task100ms]++;
		oldTimeStamp[task500ms]++;
		oldTimeStamp[taskLong]++;		
		/////////////////2ms////////////////////
		if(oldTimeStamp[task2ms]>(time2ms))
		{
			cycTask[task2ms]();
			oldTimeStamp[task2ms] = 0;
		};
		/////////////////4mS////////////////////
		if(oldTimeStamp[task4ms]>(time4ms))
		{		
			cycTask[task4ms]();
			oldTimeStamp[task4ms] = 0;
		};
		/////////////////20mS////////////////////
		if(oldTimeStamp[task20ms]>(time20ms))
		{
			cycTask[task20ms]();
			oldTimeStamp[task20ms] = 0;
		};
		/////////////////100ms///////////////////
		if(oldTimeStamp[task100ms]>(time100ms))
		{
			cycTask[task100ms]();
			oldTimeStamp[task100ms] = 0;
		};
				/////////////////500ms///////////////////
		if(oldTimeStamp[task500ms]>(time500ms))
		{
			cycTask[task500ms]();
			oldTimeStamp[task500ms] = 0;
		};
		///////////////Long time cyc/////////
		if(oldTimeStamp[taskLong]>(timeLong))
		{	
			cycTask[taskLong]();
			oldTimeStamp[taskLong] = 0;
		};
	}
	TIM3->SR&=~(1<<0);	 
}

void usec_delay(unsigned int t)
{
	delay_us(100);
}
void PVMap(){
	SetDO(DOLED,gMachineIO.LED);
}
__weak void IdleApp(void)
{
	///////////////////////////////////
	PVMap();

}
__weak void cyc2ms(){}
__weak void cyc4ms(){}
__weak void cyc20ms(){}
__weak void cyc100ms(){}
__weak void cyc500ms(){}
__weak void cycLongTsk(){}


