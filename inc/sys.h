//
// Created by marijn on 10/31/24.
//

#ifndef STM32WB07_SYS_H
#define STM32WB07_SYS_H
#include "periph.h"
#include "GPIO.h"


/*!<
 * types
 * */
typedef enum {
	LS_CLK_SRC_LSI_LPMU = 		0b00U,	// TODO: what is LPMU
	LS_CLK_SRC_LSE = 			0b01U,
	LS_CLK_SRC_LSI = 			0b10U,
	LS_CLK_SRC_HSI_DIV_2048 = 	0b11U
} LS_CLK_SRC_t;

typedef enum {
	SYS_CLK_SPEED_64MHz = 		0b000U,
	SYS_CLK_SPEED_32MHz = 		0b001U,
	SYS_CLK_SPEED_16MHz = 		0b010U,
	SYS_CLK_SPEED_8MHz = 		0b011U,
	SYS_CLK_SPEED_4MHz = 		0b100U,
	SYS_CLK_SPEED_2MHz = 		0b101U,
	SYS_CLK_SPEED_1MHz = 		0b110U
} SYS_CLK_SPEED_t;

typedef enum {
	SYS_CLK_SRC_PLL = 			0b0U,
	SYS_CLK_SRC_HSE = 			0b1U
} SYS_CLK_SRC_t;

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
void sys_restart(void);
void sys_init(void);

void delay_ms(uint64_t ms);

/*!<
 * config functions
 * */
void sys_scale_clock(SYS_CLK_SPEED_t scale);
void sys_clock_control_config(uint8_t  HSIPLL_enable, uint8_t HSE_enable, uint8_t LSI_enable, uint8_t LSE_enable, uint8_t HSEPLL_BUF_enable, uint8_t LSE_BYP);
void sys_clock_configuration(uint8_t IO_boost_enable, uint8_t LS_clk_src, uint8_t SMPS_clk_div, uint8_t SYS_clk_speed, uint8_t HSI_PLL64_disable, uint8_t SYS_clk_src, uint8_t SMPS_clk_invert);
void sys_tick_config(uint8_t sys_tick_interrupt);


#endif //STM32WB07_SYS_H
