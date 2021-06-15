#include "stm32f4xx.h"
#include "mInclude.h"

agv_mode_App AppTask[]={NullApp,BMSApp,CallerApp,ChargeStaionApp,SonarApp};
gMachine_type gMachine;

int main(void)
{
	uint8_t Parameter2[800] = {0};
	///////////////////
	SystemConfig();////
	///////////////////
  while(1){
		IdleApp();
		AppTask[gMachine.AppMode]();
		/////////////////////////////////////////////////////////

	}
}
__weak void NullApp(){;}
__weak void BMSApp(){;}
__weak void CallerApp(){;}
__weak void ChargeStaionApp(){;}
__weak void SonarApp(){;}
	
	
