//
// Created by marijn on 11/1/24.
//

#include "RF.h"

void radio_init(uint32_t flags) {
	RCC->APB3ENR |= (flags & 0x03U);

	//TODO: read flash for timing values




}