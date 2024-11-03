//
// Created by marijn on 11/3/24.
//

#include "CRC.h"


/*!< enable / disable */
void enable_CRC(void) {
	RCC->AHBENR |= 0x00001000UL;	// enable crc clock
	reset_CRC();
}

void config_CRC(uint32_t poly, uint32_t init, uint32_t flags) {
	enable_CRC();
	CRC->CR = flags;
	CRC->POL = poly;
	CRC->INIT = init;
}

void disable_CRC(void) {
	RCC->AHBRSTR |= 0x00001000UL;	// disable crc clock
}

/*!< usage */
void reset_CRC(void) {
	CRC->CR |= 0x00000001UL;			// reset crc
	while (CRC->CR & 0x00000001UL);		// wait until done
}