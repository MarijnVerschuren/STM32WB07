//
// Created by marijn on 11/3/24.
//

#include "tim.h"


/*!< init / enable / disable */
void config_TIM(uint32_t prescaler, uint32_t limit) {
	RCC->APB1ENR |= 0x00000001UL;	// enable TIM1
	TIM1->CR1 = 0x00000000UL;
	TIM1->PSC = prescaler;
	TIM1->ARR = limit;
	TIM1->EGR = 0x00000001UL;		// update shadow registers
}
void disable_TIM(void) {
	RCC->APB1ENR &= ~0x00000001UL;	// disable TIM1
}


/*!< actions */
void start_TIM(void)	{ TIM1->CR1 |= 0x1UL; }
void stop_TIM(void)		{ TIM1->CR1 &= ~0x1UL; }
void delay_TIM(uint32_t count) {
	uint32_t start = TIM1->CNT;
	while (TIM1->CNT - start < count);
}


/*!< irq */
void start_TIM_update_irq(void) {
	NVIC_enable_IRQ(TIM1_IRQn);
	TIM1->DIER |= 0x1UL;
}
void stop_TIM_update_irq(void) {
	NVIC_disable_IRQ(TIM1_IRQn);
	TIM1->DIER &= ~0x1UL;
}
