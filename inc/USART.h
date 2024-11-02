//
// Created by marijn on 11/2/24.
//

#ifndef STM32WB07_USART_H
#define STM32WB07_USART_H
#include "GPIO.h"
#include "sys.h"


/*!<
 * types
 * */
typedef enum {
	USART_PIN_DISABLE =	0x00000000UL,
	// USART1
	USART1_CTS_A0 =		0x10000041UL,	USART1_CLK_A2 =		0x12000041UL,
	USART1_RTS_A3 =		0x13000041UL,	USART1_RX_A8 =		0x08000041UL,
	USART1_TX_A9 =		0x09000041UL,	USART1_RX_B0 =		0x00100041UL,
	USART1_RTS_B2 =		0x02100041UL,	USART1_CTS_B2 =		0x03100041UL,
	USART1_CLK_B8 =		0x08100041UL,	USART1_TX_B9 =		0x09100041UL,
	// LPUART1
	LPUART1_CTS_A6 =	0x06000051UL,	LPUART1_RTS_A7 =	0x07000051UL,
	LPUART1_RTS_B0 =	0x10100051UL,	LPUART1_TX_B3 =		0x13100051UL,
	LPUART1_TX_B4 =		0x04100051UL,	LPUART1_RX_B5 =		0x05100051UL,
	LPUART1_TX_B6 =		0x36100051UL,	LPUART1_RX_B7 =		0x37100051UL,
	LPUART1_RX_B8 =		0x18100051UL,	LPUART1_CTS_B9 =	0x19100051UL
} USART_GPIO_t;

typedef enum {
	USART_OVER_SAMPLE_8	=	0b1UL << 0U
} USART_FLAG_t;


/*!< init / enable / disable */
void disable_USART(USART_t* uart);
void fconfig_UART(USART_GPIO_t tx, USART_GPIO_t rx, uint32_t baud, uint32_t flags);
void config_UART(USART_GPIO_t tx, USART_GPIO_t rx, uint32_t baud);
/*!< input / output */
uint32_t USART_write(USART_t* usart, const uint8_t* buffer, uint32_t size, uint32_t timeout);
uint32_t USART_read(USART_t* usart, uint8_t* buffer, uint32_t size, uint32_t timeout);
uint8_t USART_print(USART_t* usart, char* str, uint32_t timeout);

#endif //STM32WB07_USART_H
