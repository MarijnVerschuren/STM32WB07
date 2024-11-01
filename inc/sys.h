//
// Created by marijn on 10/31/24.
//

#ifndef STM32WB07_SYS_H
#define STM32WB07_SYS_H
#include "vector.h"
#include "periph.h"
#include "GPIO.h"


/*!<
 * types
 * */
typedef enum {
	HSE_ENABLE =				0b1U << 0U,
	PLL_ENABLE =				0b1U << 1U,
	PLL64_buffer_ENABLE =		0b1U << 2U,
	LSI_ENABLE = 				0b1U << 3U,
	LSE_ENABLE = 				0b1U << 4U,
	LSE_BYP_ENABLE = 			0b1U << 5U,
	IO_BOOST_ENABLE =			0b1U << 6U,
	LS_CLK_SRC_LSI_LPMU = 		0b00U << 7U,
	LS_CLK_SRC_LSE = 			0b01U << 7U,
	LS_CLK_SRC_LSI = 			0b10U << 7U,
	LS_CLK_SRC_HSI_DIV_2048 = 	0b11U << 7U,
	SMPS_CLK_SPEED_8MHz =		0b0U << 9U,
	SMPS_CLK_SPEED_4MHz =		0b1U << 9U,
	SYS_CLK_SPEED_64MHz = 		0b000U << 10U,
	SYS_CLK_SPEED_32MHz = 		0b001U << 10U,
	SYS_CLK_SPEED_16MHz = 		0b010U << 10U,
	SYS_CLK_SPEED_8MHz = 		0b011U << 10U,
	SYS_CLK_SPEED_4MHz = 		0b100U << 10U,
	SYS_CLK_SPEED_2MHz = 		0b101U << 10U,
	SYS_CLK_SPEED_1MHz = 		0b110U << 10U,
	HSI_BLOCK_DISABLE =			0b1U << 13U,
	SYS_CLK_SRC_PLL = 			0b0U << 14U,
	SYS_CLK_SRC_HSI = 			0b0U << 14U,
	SYS_CLK_SRC_HSE = 			0b1U << 14U,
	SMPS_clk_invert_ENABLE =	0b1U << 15U,
	SYS_TICK_ENABLE =			0b1U << 16U,
	SYS_TICK_INT_ENABLE =		0b1U << 17U,
} SYS_FLAG_t;


/*!<
 * constants
 * */
extern const uint32_t HSE_clock_frequency;
extern const uint32_t HSI_clock_frequency;
extern const uint32_t PLL64M_clock_frequency;
extern const uint32_t LSE_clock_frequency;


/*!<
 * clock variables
 * */
extern uint32_t LSI_clock_frequency;
extern uint32_t LS_clock_frequency;
extern uint32_t SYS_clock_frequency;

volatile extern uint64_t tick;


/*!<
 * functions
 * */
void sys_restart(void);				// TODO: assembly
void sys_init(uint32_t flags);
void sys_reset(void);				// TODO: assembly

void delay_ms(uint64_t ms);
void sys_scale_clock(SYS_FLAG_t scale);



#endif //STM32WB07_SYS_H
