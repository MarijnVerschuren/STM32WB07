//
// Created by marijn on 11/3/24.
//

#include "WDG.h"



void config_WDG(WDG_prescaler_t prescaler, uint16_t reload) {
	RCC->APB1ENR |= 0x00004000UL;		// enable watchdog
	IWDG->KR = 0x00005555UL;			// enable access to PR and RLR
	while (IWDG->SR & 0x00000001UL);	// wait until the Prescaler Value Update (PVU) flag is reset
	IWDG->PR = prescaler & 0x7UL;		// clk / (4 << prescaler)
	while (IWDG->SR & 0x00000002UL);	// wait until the Reload Value Update (RVU) flag is reset
	IWDG->RLR = reload & 0xFFFUL;
}

void start_WDG()	{ IWDG->KR = 0x0000CCCCUL; }
void WDG_tick()		{ IWDG->KR = 0x0000AAAAUL; }
