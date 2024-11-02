//
// Created by marijn on 11/1/24.
//

#ifndef STM32WB07_NVIC_H
#define STM32WB07_NVIC_H
#include "types.h"
#include "periph.h"


/*!<
 * types
 * */
typedef void(*IVT_FUNC)(void);
typedef union {
	void*		ptr;
	IVT_FUNC	func;
} IVT_ELEMENT_t;

typedef enum {
	/*!< ARM cortex-M0+ core interrupts */
	reset_IRQn =				-15,
	NMI_IRQn =					-14,
	hard_fault_IRQn =			-13,
	memory_fault_IRQn =			-12,
	bus_fault_IRQn =			-11,
	exception_IRQn =			-10,
	SVC_IRQn =					-5,
	debug_IRQn =				-4,
	pending_SV_IRQn =			-2,
	sys_tick_IRQn =				-1,
	/*!< STM32WB07 interrupts */
	flash_IRQn =				0,
	RCC_IRQn =					1,
	PVD_IRQn =					2,
	I2C1_IRQn =					3,
	I2C2_IRQn =					4,
	SPI1_IRQn =					5,
	SPI2_IRQn =					6,
	SPI3_IRQn =					7,
	USART1_IRQn =				8,
	LPUART1_IRQn =				9,
	TIM1_IRQn =					10,
	RTC_IRQn =					11,
	ADC_IRQn =					12,
	PKA_IRQn =					13,
	GPIOA_IRQn =				15,
	GPIOB_IRQn =				16,
	DMA_IRQn =					17,
	RADIO_TXRX_IRQn =			18,
	RADIO_TIM_error_IRQn =		20,
	RADIO_TIM_CPU_WKUP_IRQn =	23,
	RADIO_TIM_TXRX_WKUP_IRQn =	24,
	RADIO_TXRX_SEQ_IRQn =		25,
} IRQn_t;


/*!<
 * constants
 * */
extern const IVT_ELEMENT_t IVT[];


/*!<
 * init
 * */
void enable_IRQ(IRQn_t irqn);
void disable_IRQ(IRQn_t irqn);
void set_IRQ_priority(IRQn_t irqn, uint32_t priority);

#endif //STM32WB07_NVIC_H
