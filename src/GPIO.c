//
// Created by marijn on 2/13/23.
//

#include "GPIO.h"


/*!< init / disable */
void enable_GPIO(GPIO_t* port) {
	RCC->AHBENR |= 0b1U << (GPIO_to_int(port) + 2);
}
void disable_GPIO(GPIO_t* port) {
	RCC->AHBENR &= ~(0b1U << (GPIO_to_int(port) + 2));
}


void reset_GPIO(GPIO_t* port, uint8_t pin) {
	port->MODER &=		~(0b11U << (pin << 1U));
	port->OSPEEDR &=	~(0b11U << (pin << 1U));
	port->PUPDR &=		~(0b11U << (pin << 1U));
	port->OTYPER &=		~(0b1U <<	pin);
}
void fconfig_GPIO(GPIO_t* port, uint8_t pin, uint32_t flags, uint8_t alternate_function) {
	enable_GPIO(port);
	reset_GPIO(port, pin);
	port->MODER |=		((flags        & 0b11U) << (pin << 1U));
	port->PUPDR |=		(((flags >> 2) & 0b11U) << (pin << 1U));
	port->OSPEEDR |=	(((flags >> 4) & 0b11U) << (pin << 1U));
	port->OTYPER |=		(((flags >> 6) & 0b1U)	<< pin);
	uint8_t pos = ((pin & 0x7U) << 2U);
	port->AFR[pin >> 3U] &= ~(0xFU << pos);							// clear AFR entry
	port->AFR[pin >> 3U] |= ((alternate_function & 0xFU) << pos);		// set AFR entry
}
void config_GPIO(GPIO_t* port, uint8_t pin, uint32_t flags) {
	fconfig_GPIO(port, pin, flags | GPIO_low_speed, 0U);
}


/*!< output */
void GPIO_write(GPIO_t* port, uint8_t pin, uint8_t data) {
	port->BSRR |= (1U << (pin + (16U * !data)));
}
void GPIO_toggle(GPIO_t* port, uint8_t pin) {
	port->ODR ^= (1U << pin);  // TODO: BSRR
}


/*!< input */
uint8_t GPIO_read(GPIO_t* port, uint8_t pin) {
	return (port->IDR >> pin) & 1U;
}