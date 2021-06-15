#include "mInclude.h"

#define R_STATUS ((uint32_t)0x020000)
#define R_NOOP ((uint32_t)0x000000)
#define R_CTRL_REG ((uint32_t)0x020002)
#define R_DAC_REG ((uint32_t)0x020001)

#define SET_CTRL_REG_CLS ((uint32_t)0x550008)
#define SET_CONFG_REG ((uint32_t)0x57300D)
#define SET_REST_REG ((uint32_t)0x560001)



static uint16_t DataOut[4]={0,0,0,0};

uint16_t Shif24bit(uint32_t data){	
	uint8_t b8data1 = data>>0 & 0x0000FF;
	uint8_t b8data2 = data>>8 & 0x0000FF;
	uint8_t b8data3 = data>>16 & 0x0000FF;
	uint8_t result[3];	
	result[0] = SPI2_ReadWriteByte(b8data3);
	result[1] = SPI2_ReadWriteByte(b8data2);
	result[2] = SPI2_ReadWriteByte(b8data1);
	return (result[1]<<8)| result[2]; 
}
void SetDAC7760(uint16_t * ui16DACOut){
	
	uint32_t SetAOReg=0x010000,SetAOReg0,SetAOReg1,SetAOReg2,SetAOReg3;

	uint16_t AOValue0 = ((uint16_t)(ui16DACOut[0] ))<<4 & 0xFFFF;	
	uint16_t AOValue1 = ((uint16_t)(ui16DACOut[1] ))<<4 & 0xFFFF;
	uint16_t AOValue2 = ((uint16_t)(ui16DACOut[2] ))<<4 & 0xFFFF;
	uint16_t AOValue3 = ((uint16_t)(ui16DACOut[3] ))<<4 & 0xFFFF;
	
	SetAOReg0 = SetAOReg | AOValue0;
	SetAOReg1 = SetAOReg | AOValue1;
	SetAOReg2 = SetAOReg | AOValue2;
	SetAOReg3 = SetAOReg | AOValue3;
	
	LATCH_LOW;
	Shif24bit(SetAOReg3);
	Shif24bit(SetAOReg2);
	Shif24bit(SetAOReg1);
	Shif24bit(SetAOReg0);
	LATCH_HIGH;
	delay_us(2);
};
void ReadDAC7760(void){//To asure that the controll register is setted and the dasiy chain is ok
	if(DataOut[3]==0){
		LATCH_LOW;
		Shif24bit(R_CTRL_REG);
		Shif24bit(R_CTRL_REG);
		Shif24bit(R_CTRL_REG);
		Shif24bit(R_CTRL_REG);
		LATCH_HIGH;
		delay_us(2);
		LATCH_LOW;
		DataOut[3] = Shif24bit(R_NOOP);
		DataOut[2] = Shif24bit(R_NOOP);
		DataOut[1] = Shif24bit(R_NOOP);
		DataOut[0] = Shif24bit(R_NOOP);
		LATCH_HIGH;
		delay_us(2);
		SetDO(DACClear,0);
		EnableDaisyChain(gSystemPara.RangeMode);
		delay_us(200);	
	}
}

void EnableDaisyChain(uint8_t * OutputRage){	
	
	uint32_t CTRLREG0 = 0x553008 | ((uint32_t) OutputRage[0]-1);
	uint32_t CTRLREG1 = 0x553008 | ((uint32_t) OutputRage[1]-1);
	uint32_t CTRLREG2 = 0x553008 | ((uint32_t) OutputRage[2]-1);
	uint32_t CTRLREG3 = 0x553008 | ((uint32_t) OutputRage[3]-1);
	///////////////
	LATCH_LOW;
	
	if(OutputRage[3]==0)
		Shif24bit(SET_CTRL_REG_CLS);
	else
		Shif24bit(CTRLREG3);
	
	if(OutputRage[2]==0)
		Shif24bit(SET_CTRL_REG_CLS);
	else
		Shif24bit(CTRLREG2);
	
	if(OutputRage[1]==0)
		Shif24bit(SET_CTRL_REG_CLS);
	else
		Shif24bit(CTRLREG1);
		
	if(OutputRage[0]==0)
		Shif24bit(SET_CTRL_REG_CLS);
	else
		Shif24bit(CTRLREG0);

	LATCH_HIGH;
	
	delay_us(2);
}

