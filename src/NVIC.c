//
// Created by marijn on 11/2/24.
//

#include "NVIC.h"


/*!<
 * init
 * */
void NVIC_enable_IRQ(IRQn_t irqn) {
	NVIC->ISER |= (0b1UL << irqn);
}

void NVIC_disable_IRQ(IRQn_t irqn) {
	NVIC->ISER &= ~(0b1UL << irqn);
	__DSB();  // wait for the data bus to clear
	__ISB();  // flush cpu pipeline
}

void NVIC_set_IRQ_priority(IRQn_t irqn, uint32_t priority) {
	NVIC->IP[irqn] = priority << 6U;
}
