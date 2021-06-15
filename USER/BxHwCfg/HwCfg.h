#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "string.h"
#include "stm32f4xx_spi.h"

/* stdbool.h standard header */
#ifndef _STDBOOL
#define _STDBOOL

#define __bool_true_false_are_defined        1

#ifndef __cplusplus
                /* TYPES */

#if 199901L <= __STDC_VERSION__


#else /* 199901L <= __STDC_VERSION__ */
#if __TI_STRICT_ANSI_MODE__
typedef unsigned char _Bool;
#endif
#endif /* 199901L <= __STDC_VERSION__ */

                /* MACROS */
#define bool        _Bool
#define false        0
#define true        1
#endif /* __cplusplus */


#endif /* _STDBOOL */

/*
* Copyright (c) 1992-2004 by P.J. Plauger.  ALL RIGHTS RESERVED.
* Consult your license regarding permissions and restrictions.
V4.02:1476 */

/////define SPI Interface
#define      macSPI1                                     SPI1
#define      macSPI2                                     SPI2

#define      macSPI1_APBxClock_FUN                        RCC_APB2PeriphClockCmd 
#define      macSPI1_CLK                                  RCC_APB2Periph_SPI1

#define      macSPI2_APBxClock_FUN                        RCC_APB1PeriphClockCmd 
#define      macSPI2_CLK                                  RCC_APB1Periph_SPI2

////Redefine
#define PxA GPIOA_BASE
#define PxB GPIOB_BASE
#define PxC GPIOC_BASE
#define PxD GPIOD_BASE
#define Uartx1 USART1_BASE
#define Uartx2 USART2_BASE
#define Uartx3 USART3_BASE
#define Uartx6 USART6_BASE
#define TIMx3  TIM3_BASE///使用TIM3作为1ms中断源
#define TIMx1  TIM1_BASE//pulse 1
#define TIMx2  TIM2_BASE//pulse 2
#define TIMx4  TIM4_BASE//encoder 1
#define TIMx5  TIM5_BASE//encoder 2
///////////
#define PIN0 	GPIO_Pin_0
#define PIN1 	GPIO_Pin_1
#define PIN2 	GPIO_Pin_2
#define PIN3 	GPIO_Pin_3
#define PIN4 	GPIO_Pin_4
#define PIN5 	GPIO_Pin_5
#define PIN6 	GPIO_Pin_6
#define PIN7 	GPIO_Pin_7
#define PIN8 	GPIO_Pin_8
#define PIN9 	GPIO_Pin_9
#define PIN10 GPIO_Pin_10
#define PIN11 GPIO_Pin_11
#define PIN12 GPIO_Pin_12
#define PIN13 GPIO_Pin_13
#define PIN14 GPIO_Pin_14
#define PIN15 GPIO_Pin_15
///////////MAPING-OUTPUT
#define DO1 		PxC,PIN3,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define DO2 		PxC,PIN2,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define DO3 		PxC,PIN1,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define DO4 		PxC,PIN0,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define AI1Mode 		PxC,PIN5,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define AI2Mode 		PxC,PIN4,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define DACClear 		PxC,PIN8,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define DIR1 		PxC,PIN9,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define DIR2 		PxC,PIN12,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define RS485DE	 	PxA,PIN5,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define DOLED		PxA,PIN4,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define DOBuzzer 	PxB,PIN11,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SPI1NSS  	PxD,PIN2,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SPI1SCK		PxB,PIN3,GPIO_Mode_AF,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SPI1MISO	PxB,PIN4,GPIO_Mode_AF,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SPI1MOSI	PxB,PIN5,GPIO_Mode_AF,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SPI2NSS  	PxB,PIN12,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SPI2SCK		PxB,PIN13,GPIO_Mode_AF,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SPI2MISO	PxB,PIN14,GPIO_Mode_AF,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SPI2MOSI	PxB,PIN15,GPIO_Mode_AF,GPIO_PuPd_NOPULL,GPIO_OType_PP

///////////MAPING-INPUT
#define DI1 	PxB,PIN0,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DI2 	PxB,PIN1,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DI3 	PxB,PIN2,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DI4 	PxB,PIN10,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DACREADY 	PxB,PIN15,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define KEY1 	PxC,PIN13,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define KEY2 	PxC,PIN14,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define KEY3 	PxC,PIN15,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define KEY4 	PxA,PIN3,GPIO_Mode_IN,GPIO_PuPd_NOPULL
///////////MAPING-URT6->485
#define Uart6TX PxC,PIN6
#define Uart6RX PxC,PIN7
///////////MAPING-URT3->232
#define Uart3TX PxC,PIN10
#define Uart3RX PxC,PIN11
///////////MAPING-TTL
#define Uart1TX PxA,PIN9
#define Uart1RX PxA,PIN10
///////////////////////////////////////
#define	InputNum 16
#define OutputNum 16
#define UrtBfLen  254
#define time2ms   2
#define time4ms   4
#define time20ms  20
#define time100ms 100
#define time500ms 500
#define timeLong  1500
typedef	enum {task2ms=0,task4ms,task20ms,task100ms,task500ms,taskLong} cycTaskIndex;
///////////////////////////////////////
#define LATCH_HIGH SetDO(SPI2NSS,1)
#define LATCH_LOW SetDO(SPI2NSS,0)
///////////////////////////////////////
typedef struct AO_type{
	
	int16_t Value;//DAC7750/DAC7760 12bit Only
	bool Alarms;
	bool IFLT;
	bool TFLT;//temperature over 142 degree
	int8_t OuterMode;//101:4ma~20ma 110:0ma~20ma 111:0ma~24ma
}AO_type;

typedef struct UrtBuf_type
{
	uint8_t	rBuffer[UrtBfLen];
	uint8_t  pRfil;
	uint8_t  pRder;
	
	uint8_t	sBuffer[UrtBfLen];
	uint8_t  sLen;
	uint8_t  flagS;
}UrtBuf_type;

typedef struct gMachineIO_type
{

	UrtBuf_type Uart6Data;


	double CanTimeOut[4];
	///////////////////
	bool DO[4];
	bool DI[4];
	///////////////////
	bool KEY[4];
	bool LED;
	bool Buzzer;
	///////////////////
	uint16_t AI[2];
	double AIVoltage[2];
	uint16_t AO[4];
	///////////////////
	int32_t Encoder[2];
	///////////////////
	uint32_t TimeStamp;
	///
	bool Directions[2];
	bool flagLed[4];
}gMachineIO_type;

typedef struct gMachine_type
{
	uint8_t AppMode;
	double Voltage;
}gMachine_type;


extern gMachine_type gMachine;
extern TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
extern DMA_InitTypeDef DMA_InitStructure;

////////////////some Function to config the Hardware
extern gMachineIO_type gMachineIO;


bool capStartDwn(UrtBuf_type * pSrcBuf);
bool capEndDwn(UrtBuf_type * pSrcBuf);
bool capBin(UrtBuf_type * pSrcBuf);


void SystemConfig(void);
void IdleApp(void);
uint32_t GPIO2APB2CLK(uint32_t  GPIOx);
enum IRQn MapIRQn(uint32_t BASEType);
void CfgPINOut(uint32_t  GPIOx,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd,GPIOOType_TypeDef OType);
void CfgPINIn(uint32_t GPIOx, uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd);
void CfgUartx(uint32_t UartX,uint8_t uartPar,uint32_t GPTx, uint16_t GPTX_Pin,uint32_t GPRx, uint16_t GPRX_Pin);
void HwCfgInit(void);
void TimCfg(uint32_t timeUs,uint32_t BASEType);

void JTAGOFF(void);
uint16_t ReceiveByte(void);
void APBCLKCfg( uint32_t  GPIOx,FunctionalState NewState);
extern void TIM4_PWM_Init(u32 arr,u32 psc);
extern uint8_t ReadIn(uint32_t GPIOxADR, uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd);
extern void SetDO(uint32_t GPIOxADR,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd,GPIOOType_TypeDef OType,bool state);
void FillUrtBuf(UrtBuf_type * pBoxIO,uint32_t USARTx);
extern void SendUrtBuf(UrtBuf_type * pBoxIO,uint32_t USARTx);
void MBLArry(int8_t *buffer,uint8_t bufLen);
void TimCfg(uint32_t timeUs ,uint32_t BASEType);
extern void usec_delay(unsigned int t);
