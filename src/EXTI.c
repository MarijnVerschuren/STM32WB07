//
// Created by marijn on 11/1/24.
//

#include "EXTI.h"


/*!< init / enable / disable */
void reset_EXTI(GPIO_t* port, uint8_t pin) {
	pin += (GPIO_to_int(port) << 4);
	SYSCFG->IO_IER &=	~(0b1UL << pin);
	SYSCFG->IO_DTR &=	~(0b1UL << pin);
	SYSCFG->IO_IBER &=	~(0b1UL << pin);
	SYSCFG->IO_IEVR &=	~(0b1UL << pin);
}
void config_EXTI(GPIO_t* port, uint8_t pin, uint32_t flags) {
	RCC->APB1ENR |= 0x00000100UL;  // enable SYSCFG
	reset_EXTI(port, pin);	// TODO: ASM to optimize
	pin += (GPIO_to_int(port) << 4);
	SYSCFG->IO_DTR |=	((flags 		& 0b1UL) << pin);
	SYSCFG->IO_IBER |=	(((flags >> 1U)	& 0b1UL) << pin);
	SYSCFG->IO_IEVR |=	(((flags >> 2U)	& 0b1UL) << pin);
}
void config_EXTI_IRQ(GPIO_t* port, uint32_t priority) {
	set_IRQ_priority(GPIOA_IRQn + GPIO_to_int(port), priority);
}

void start_EXTI(GPIO_t* port, uint8_t pin) {
	pin += (GPIO_to_int(port) << 4);
	SYSCFG->IO_IER |=	(0b1UL << pin);
	enable_IRQ(GPIOA_IRQn + GPIO_to_int(port));
}

void stop_EXTI(GPIO_t* port, uint8_t pin) {
	pin += (GPIO_to_int(port) << 4);
	SYSCFG->IO_IER &=	~(0b1UL << pin);
	if (SYSCFG->IO_IER & (0xFFFFUL << (GPIO_to_int(port) << 4))) { return; }
	disable_IRQ(GPIOA_IRQn + GPIO_to_int(port));
}