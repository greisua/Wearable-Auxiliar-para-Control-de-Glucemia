#include "stateMachine/sm.h"

extern stateMachine_t system_cfg;

/* FIX TO USE NFC PINS AS GPIO */
const uint32_t UICR_ADDR_0x20C __attribute__ ((section(".uicrNfcPinsAddress"))) __attribute__((used)) = 0xFFFFFFFE;

void main(void)
{
	while(1)
	{
		SM_Run();
		k_msleep(system_cfg.sm_period);
	} 
}
