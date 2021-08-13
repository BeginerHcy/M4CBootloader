#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "string.h"
#include "stm32f4xx_spi.h"
#include "demo_fatfs.h"
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

typedef struct dwnCtrl_type{

	uint32_t fwlen;
	uint32_t status;
	uint32_t index;
	uint8_t numSend;
	uint32_t indexArry[100];
	uint8_t iArry;
	uint32_t demandIndex;
	uint32_t bufAddr;
	
	
}dwnCtrl_type;


extern dwnCtrl_type dwnCtrl;

extern float timeBoot;
typedef void(*cyc_task_fun)(void);
typedef void(*agv_mode_App)(void);

void cyc2ms(void);
void cyc4ms(void);
void cyc20ms(void);
void cyc100ms(void);
void cyc500ms(void);
void cycLongTsk(void);

void NullApp(void);
void BMSApp(void);
void CallerApp(void);
void ChargeStaionApp(void);
void SonarApp(void);
///////////////////////////////////////

uint16_t CRC16_Modbus(uint8_t *_pBuf, uint16_t _usLen);
