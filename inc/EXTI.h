//
// Created by marijn on 11/1/24.
//

#ifndef STM32WB07_EXTI_H
#define STM32WB07_EXTI_H
#include "NVIC.h"
#include "GPIO.h"


/*!<
 * types
 * */
typedef enum {
	EXTI_edge =				0b00U << 0U,
	EXTI_level =			0b01U << 0U
} EXTI_MODE_t;
typedef enum {
	EXTI_single_edge =		0b00U << 1U,
	EXTI_both_edge =		0b01U << 1U
} EXTI_EDGE_t;
typedef enum {
	EXTI_faling_edge =		0b00U << 2U,
	EXTI_low_level =		0b00U << 2U,
	EXTI_rising_edge =		0b01U << 2U,
	EXTI_high_level =		0b01U << 2U
} EXTI_POL_t;


/*!< init / enable / disable */
void config_EXTI(GPIO_t* port, uint8_t pin, uint32_t flags);
void config_EXTI_IRQ(GPIO_t* port, uint32_t priority);
void start_EXTI(GPIO_t* port, uint8_t pin);
void stop_EXTI(GPIO_t* port, uint8_t pin);


#endif //STM32WB07_EXTI_H
