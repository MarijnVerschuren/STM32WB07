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
	HSE_DISABLE =				0b0U << 0U
} HSE_enable_t;

typedef enum {
	PLL_ENABLE =				0b1U << 1U,
	PLL_DISABLE =				0b0U << 1U
} PLL_enable_t;

typedef enum {
	PLL64_buffer_ENABLE =		0b1U << 2U,
	PLL64_buffer_DISABLE =		0b0U << 2U
} PLL64_buffer_t;

typedef enum {
	LSI_ENABLE = 				0b1U << 3U,
	LSI_DISABLE = 				0b0U << 3U
} LSI_enable_t;

typedef enum {
	LSE_ENABLE = 				0b1U << 4U,
	LSE_DISABLE = 				0b0U << 4U
} LSE_enable_t;

typedef enum {
	LSE_BYP_ENABLE = 			0b1U << 5U,
	LSE_BYP_DISABLE = 			0b0U << 5U
} LSE_BYP_enable_t;

typedef enum {
	IO_BOOST_ENABLE =			0b1U << 6U,
	IO_BOOST_DISABLE =			0b0U <<	6U
} IO_boost_t;

typedef enum {
	LS_CLK_SRC_LSI_LPMU = 		0b00U << 7U,
	LS_CLK_SRC_LSE = 			0b01U << 7U,
	LS_CLK_SRC_LSI = 			0b10U << 7U,
	LS_CLK_SRC_HSI_DIV_2048 = 	0b11U << 7U
} LS_CLK_SRC_t;

typedef enum {
	SMPS_CLK_SPEED_8MHz =		0b0U << 9U,
	SMPS_CLK_SPEED_4MHz =		0b1U << 9U
} SMPS_CLK_SPEED_t;

typedef enum {
	SYS_CLK_SPEED_64MHz = 		0b000U << 10U,
	SYS_CLK_SPEED_32MHz = 		0b001U << 10U,
	SYS_CLK_SPEED_16MHz = 		0b010U << 10U,
	SYS_CLK_SPEED_8MHz = 		0b011U << 10U,
	SYS_CLK_SPEED_4MHz = 		0b100U << 10U,
	SYS_CLK_SPEED_2MHz = 		0b101U << 10U,
	SYS_CLK_SPEED_1MHz = 		0b110U << 10U
} SYS_CLK_SPEED_t;

typedef enum {
	HSI_BLOCK_DISABLE =			0b1U << 13U
} HSI_BLOCK_DISABLE_t;

typedef enum {
	SYS_CLK_SRC_PLL = 			0b0U << 14U,
	SYS_CLK_SRC_HSI = 			0b0U << 14U,
	SYS_CLK_SRC_HSE = 			0b1U << 14U
} SYS_CLK_SRC_t;

typedef enum {
	SMPS_clk_invert_ENABLE =	0b1U << 15U,
	SMPS_clk_invert_DISABLE =	0b0U <<	15U
} SMPS_clk_invert_t;

typedef enum {
	SYS_TICK_ENABLE =			0b1U << 16U,
} SYS_TICK_EN_t;

typedef enum {
	SYS_TICK_INT_ENABLE =		0b1U << 17U,
} SYS_TICK_INT_EN_t;

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
void sys_scale_clock(SYS_CLK_SPEED_t scale);



#endif //STM32WB07_SYS_H
