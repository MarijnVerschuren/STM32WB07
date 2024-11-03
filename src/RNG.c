//
// Created by marijn on 11/3/24.
//

#include "RNG.h"


/*!< enable / disable */
void enable_RNG(void) {
	RCC->AHBENR |= 0x00040000UL;
}

void disable_RNG(void) {
	RCC->AHBRSTR |= 0x00040000UL;
}

/*!< usage */
uint32_t RNG_generate(void) {
	while (RNG->SR & 0x00000001UL);
	return RNG->VAL;
}