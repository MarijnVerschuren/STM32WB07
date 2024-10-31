//
// Created by marijn on 10/31/24.
//

#include "sys.h"

/*!<
 * constants
 * */
extern const uint32_t HSE_clock_frequency =		32000000;
extern const uint32_t HSI_clock_frequency =		64000000;
extern const uint32_t PLL64M_clock_frequency =	64000000;
extern const uint32_t LSE_clock_frequency =		32768;


/*!<
 * clock variables
 * */
extern uint32_t LSI_clock_frequency =	32000;
extern uint32_t LS_clock_frequency =	0;
extern uint32_t SYS_clock_frequency =	16000000;

volatile extern uint64_t tick = 0;


/*!<
 * config
 * */
volatile struct {
	// RCC->CR
	uint32_t HSE_enable					: 1;
	uint32_t PLL64_enable				: 1;
	uint32_t PLL64_buffer_enable		: 1;
	uint32_t LSI_enable					: 1;
	uint32_t LSE_enable					: 1;
	uint32_t LSE_BYP					: 1;
	// RCC->CFGR
	uint32_t IO_boost_enable			: 1;
	uint32_t LS_clk_src					: 2;	// LS_CLK_SRC_t
	uint32_t SMPS_clk_div				: 1;	// SMPS_CLK_DIV_t
	uint32_t SYS_clk_speed				: 3;	// SYS_CLK_SPEED_t
	uint32_t HSI_PLL64_disable			: 1;
	uint32_t SYS_clk_src				: 1;	// SYS_CLK_SRC_t
	uint32_t SMPS_clk_invert			: 1;
	// SYSTICK
	uint32_t SYS_tick_enable			: 1;
	uint32_t SYS_tick_interrupt_enable	: 1;
} sys_config;



/*!<
 * interrupts
 * */
void sys_tick_handler(void) { tick++; }


/*!<
 * functions
 * */
void sys_restart(void) {
	SCB->AIRCR |= 0x05FA0004UL;
	// TODO: test
}

void sys_init(void) {
	FLASH->CONFIG = (
		((sys_config.SYS_clk_speed == SYS_CLK_SPEED_64MHz) << 4U)	// set 1 wait-state if freq is 64MHz
	);
	PWR->CR1 = 0x00000000UL;									// disable APC
	if (sys_config.LSE_enable) {
		reset_GPIO(GPIOB, 12); reset_GPIO(GPIOB, 13);
	}
	RCC->CR = 0x00000400UL;  // disable all except HSI
	RCC->CR |= (sys_config.HSE_enable << 16U);					// enable HSE
	while (sys_config.HSE_enable	&& RCC->CR & 0x00020000UL);	// wait until HSE is ready
	RCC->CR |= (
		(0b111UL << 7U)							|				// max setting for LOCKDET_NSTOP
		(sys_config.PLL64_buffer_enable << 12U)	|				// enable PLL64 buffer (for RF)
		(sys_config.PLL64_enable << 13U)		|				// enable PLL64
		(sys_config.LSE_BYP << 6U)				|				// enable LSE bypass
		(sys_config.LSE_enable << 4U)			|				// enable LSE
		(sys_config.LSI_enable << 2U)							// enable LSI
	);
	while (sys_config.PLL64_enable	&& RCC->CR & 0x00004000UL);	// wait until PLL64 is ready
	while (sys_config.LSE_enable	&& RCC->CR & 0x00000020UL);	// wait until LSE is ready
	while (sys_config.LSI_enable	&& RCC->CR & 0x00000008UL);	// wait until LSI is ready
	RCC->CFGR = (
		(sys_config.IO_boost_enable << 17U)		|
		(sys_config.LS_clk_src << 15U)			|
		(sys_config.SMPS_clk_div << 12U)		|
		(sys_config.HSI_PLL64_disable << 2U)	|
		(sys_config.SYS_clk_src << 1U)			|
		(sys_config.SMPS_clk_invert)
	);
	RCC->CSCMDR |= (
		(sys_config.SYS_clk_speed << 1U)	 	|
		0b1UL
	);
	while(RCC->CSCMDR & 0b1U);

	SYS_clock_frequency	= (64000000 / (0b1U << sys_config.SYS_clk_speed));
	switch (sys_config.LS_clk_src) {	// TODO: what is LPMU
	case LS_CLK_SRC_LSI_LPMU:		LS_clock_frequency = LSI_clock_frequency;			break;
	case LS_CLK_SRC_LSE:			LS_clock_frequency = LSE_clock_frequency;			break;
	case LS_CLK_SRC_LSI:			LS_clock_frequency = LSI_clock_frequency;			break;
	case LS_CLK_SRC_HSI_DIV_2048:	LS_clock_frequency = HSI_clock_frequency / 2048;	break;
	}

	SYS_TICK->LOAD = (SYS_clock_frequency / 1000) - 1;						/* set reload register, clk src: AHB/8 */
	SYS_TICK->VAL  = 0;														/* load counter value  */
	SYS_TICK->CTRL = (														/* start SysTick timer */
			(sys_config.SYS_tick_enable << 0U)				|
			(sys_config.SYS_tick_interrupt_enable << 1U)	|
			0b100UL
	);
	// set IRQ priority
	//SCB->SHP[11U] = 0b11110000;  // TODO how?
}


void delay_ms(uint64_t ms) {
	uint64_t start = tick;
	while (tick - start < ms);
}

/*!<
 * config functions
 * */
void sys_scale_clock(SYS_CLK_SPEED_t speed) {
	RCC->CSCMDR |= (
		(speed << 1U) 	|
		0b1U
	);
	while(RCC->CSCMDR & 0b1U);
	SYS_clock_frequency	= (64000000 / (0b1U << sys_config.SYS_clk_speed));
}
void sys_clock_control_config(uint8_t  HSIPLL_enable, uint8_t HSE_enable, uint8_t LSI_enable, uint8_t LSE_enable, uint8_t HSEPLL_BUF_enable, uint8_t LSE_BYP) {
	sys_config.PLL64_enable = HSIPLL_enable;
	sys_config.HSE_enable = HSE_enable;
	sys_config.LSI_enable = LSI_enable;
	sys_config.LSE_enable = LSE_enable;
	sys_config.PLL64_buffer_enable = HSEPLL_BUF_enable;
	sys_config.LSE_BYP = LSE_BYP;
}

void sys_clock_configuration(uint8_t IO_boost_enable, uint8_t LS_clk_src, uint8_t SMPS_clk_div, uint8_t SYS_clk_speed, uint8_t HSI_PLL64_disable, uint8_t SYS_clk_src, uint8_t SMPS_clk_invert) {
	sys_config.IO_boost_enable = IO_boost_enable;
	sys_config.LS_clk_src = LS_clk_src;
	sys_config.SMPS_clk_div = SMPS_clk_div;
	sys_config.SYS_clk_speed = SYS_clk_speed;
	sys_config.HSI_PLL64_disable = HSI_PLL64_disable;
	sys_config.SYS_clk_src = SYS_clk_src;
	sys_config.SMPS_clk_invert = SMPS_clk_invert;
}

void sys_tick_config(uint8_t sys_tick_interrupt) {
	sys_config.SYS_tick_interrupt_enable = sys_tick_interrupt;
}