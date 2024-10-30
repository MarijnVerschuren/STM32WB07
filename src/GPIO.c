//
// Created by marijn on 2/13/23.
//
#include "GPIO.h"

/*!<
 * core hardware memory map
 * */  // TODO: valid (old cortex M4 code)
#define SCS_BASE			(0xE000E000UL)
#define ITM_BASE			(0xE0000000UL)
#define DWT_BASE			(0xE0001000UL)
#define TPI_BASE			(0xE0040000UL)
#define CORE_DEBUG_BASE		(0xE000EDF0UL)

#define SYS_TICK_BASE		(SCS_BASE +  0x0010UL)
#define NVIC_BASE			(SCS_BASE +  0x0100UL)
#define SCB_BASE			(SCS_BASE +  0x0D00UL)


/*!<
 * extended hardware memory map
 * */
#define FLASH_BASE			0x10040000UL
#define SRAM_BASE			0x20000000UL
#define PERIPH_BASE			0x40000000UL

#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		(PERIPH_BASE + 0x01000000UL)
#define APB3PERIPH_BASE		(PERIPH_BASE + 0x20000000UL)
#define AHBPERIPH_BASE		(PERIPH_BASE + 0x08000000UL)


/*!< AHB */
#define GPIOA_BASE			(AHBPERIPH_BASE + 0x000000UL)
#define GPIOB_BASE			(AHBPERIPH_BASE + 0x100000UL)
#define RCC_BASE			(AHBPERIPH_BASE + 0x400000UL)
/*!< misc */
uint8_t GPIO_to_int(GPIO_t* port) {
	return (((uint32_t)(port - AHBPERIPH_BASE) >> 20u) & 0x1u) + 2u;
}
GPIO_t* int_to_GPIO(uint8_t num) {
	return (GPIO_t*)((((num - 2u) & 0x1u) << 20u) + AHBPERIPH_BASE);
}

/*!< init / disable */
void enable_GPIO(GPIO_t* port) {
	RCC->AHBENR |= 0b1u << GPIO_to_int(port);
}
void disable_GPIO(GPIO_t* port) {
	RCC->AHBENR &= ~(0b1u << GPIO_to_int(port));
}


void reset_GPIO(GPIO_t* port, uint8_t pin) {
	port->MODER &= ~(0b11u << (pin << 1u));
	port->OSPEEDR &= ~(0b11u << (pin << 1u));
	port->PUPDR &= ~(0b11u << (pin << 1u));
	port->OTYPER &= ~(0b1u << pin);
}
void fconfig_GPIO(GPIO_t* port, uint8_t pin, GPIO_MODE_t mode, GPIO_PULL_t pull, GPIO_OT_t output_type, GPIO_SPEED_t speed, uint8_t alternate_function) {
	enable_GPIO(port);
	reset_GPIO(port, pin);
	port->MODER |= (mode << (pin << 1u));
	port->OSPEEDR |= (speed << (pin << 1u));
	port->PUPDR |= (pull << (pin << 1u));
	port->OTYPER |= output_type << pin;
	uint8_t pos = ((pin & 0x7) << 2);
	port->AFR[pin >> 3] &= ~(0xf << pos);							// clear AFR entry
	port->AFR[pin >> 3] |= ((alternate_function & 0xf) << pos);		// set AFR entry
}
void config_GPIO(GPIO_t* port, uint8_t pin, GPIO_MODE_t mode, GPIO_PULL_t pull, GPIO_OT_t output_type) {
	fconfig_GPIO(port, pin, mode, pull, output_type, GPIO_low_speed, 0);
}


/*!< output */
void GPIO_write(GPIO_t* port, uint8_t pin, uint8_t data) {
	port->BSRR |= (1u << (pin + (16 * !data)));
}
void GPIO_toggle(GPIO_t* port, uint8_t pin) {
	port->ODR ^= (1u << pin);  // TODO: BSRR
}


/*!< input */
uint8_t GPIO_read(GPIO_t* port, uint8_t pin) {
	return (port->IDR >> pin) & 1u;
}