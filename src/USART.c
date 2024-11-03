//
// Created by marijn on 11/2/24.
//

#include "USART.h"


/*!< static */
static USART_t* int_to_USART(uint8_t pnum) {
	return (USART_t*)(APB2PERIPH_BASE + (pnum << 12U));
}

static uint16_t UART_division(uint32_t baud) {
	// TODO: clock prescaler?
	return (16000000 + (baud >> 1)) / baud;
}

static void enable_USART(USART_t* uart) {
	if (uart == USART1)		{ RCC->APB2ENR |= 0x00000400UL; }
	if (uart == LPUART1)	{ RCC->APB2ENR |= 0x00000100UL; }
}


/*!< init / enable / disable */
void disable_USART(USART_t* uart) {
	if (uart == USART1)		{ RCC->APB2RSTR |= 0x00000400UL; }
	if (uart == LPUART1)	{ RCC->APB2RSTR |= 0x00000100UL; }
}

void fconfig_UART(USART_GPIO_t _tx, USART_GPIO_t _rx, uint32_t baud, uint32_t flags) {
	dev_pin_t tx, rx; *((uint32_t*)&tx) = _tx ; *((uint32_t*)&rx) = _rx;
	USART_t* uart = NULL;
	// configure GPIO
	if (_tx) {
		uart = int_to_USART(tx.periph);
		fconfig_GPIO(int_to_GPIO(tx.port), tx.port, GPIO_alt_func | GPIO_medium_speed, tx.alt);
	}
	if (_rx) {
		uart = int_to_USART(rx.periph);
		fconfig_GPIO(int_to_GPIO(rx.port), rx.port, GPIO_alt_func | GPIO_medium_speed, rx.alt);
	}
	// configure UART
	enable_USART(uart);
	uint16_t uart_div = UART_division(baud) * ((flags & 0b1UL) + 1);
	uart->PSC = 0x00000000UL;			// prescaler 1 TODO: setting?
	uart->BRR = ((uart_div & 0xfff0) | ((uart_div & 0xf) >> (flags & 0b1UL)));
	uart->CR1 = (
		((flags & 0b1UL) << 15U)	|	// oversample mode
		((_tx != 0) << 3U)			|	// transmitter enable
		((_rx != 0) << 2U)			|	// receiver enable
		0b1U							// uart enable
	);
}

void config_UART(USART_GPIO_t tx, USART_GPIO_t rx, uint32_t baud) {
	fconfig_UART(tx, rx, baud, 0);
}



/*!< input / output */
uint32_t USART_write(USART_t* usart, const uint8_t* buffer, uint32_t size, uint32_t timeout) {
	uint64_t start = tick;
	for (uint32_t i = 0; i < size; i++) {
		while (!(usart->ISR & 0x00000080UL)) { if ( tick - start > timeout) { return i; } }
		usart->TDR = buffer[i];
	} return size;
}
uint32_t USART_read(USART_t* usart, uint8_t* buffer, uint32_t size, uint32_t timeout) {
	uint64_t start = tick;
	for (uint32_t i = 0; i < size; i++) {
		while (!(usart->ISR & 0x00000020UL)) { if ( tick - start > timeout) { return i; } }
		buffer[i] = usart->RDR;
	} return size;
}
uint8_t USART_print(USART_t* usart, char* str, uint32_t timeout) {
	uint64_t start = tick;
	while (*str) {
		while (!(usart->ISR & 0x00000080UL)) { if (tick - start > timeout) { return -1; } }
		usart->TDR = *str++;
	}
	return 0;
}
