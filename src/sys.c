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
extern uint32_t LS_clock_frequency =	32000;
extern uint32_t SYS_clock_frequency =	16000000;

volatile extern uint64_t tick = 0;


/*!<
 * config TODO: linker
 * */
volatile struct {
	// RCC->CR
	uint32_t HSE_enable					: 1;
	uint32_t PLL_enable					: 1;
	uint32_t PLL64_buffer_enable		: 1;
	uint32_t LSI_enable					: 1;
	uint32_t LSE_enable					: 1;
	uint32_t LSE_BYP					: 1;
	// RCC->CFGR
	uint32_t IO_boost_enable			: 1;
	uint32_t LS_clk_src					: 2;	// LS_CLK_SRC_t
	uint32_t SMPS_clk_speed				: 1;	// SMPS_CLK_SPEED_t
	uint32_t SYS_clk_speed				: 3;	// SYS_CLK_SPEED_t
	uint32_t HSI_BLOCK_disable			: 1;
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
 * */  // TODO: assembly
void sys_restart(void) {
	__DSB();							// clear memory read pipeline
	SCB->AIRCR = (uint32_t)(
		(0x5FAUL << 16U)			|	// write key
		(SCB->AIRCR & (7UL << 8U))	|	// keep priority group unchanged
		1UL << 2U						// write software reset request
	);
	__DSB();							// clear memory read pipeline (ensure write complete)
	for(;;) { __NOP(); }
}

void sys_init(uint32_t flags) {
	if (flags) { *((uint32_t*)&sys_config) = flags; }

	FLASH->CONFIG = (
		((sys_config.SYS_clk_speed == SYS_CLK_SPEED_64MHz) << 4U)	// set 1 wait-state if freq is 64MHz
	);
	PWR->CR1 = 0x00000000UL;										// disable APC
	if (sys_config.LSE_enable) {
		reset_GPIO(GPIOB, 12); reset_GPIO(GPIOB, 13);
	}
	RCC->CR = (sys_config.HSE_enable << 16U);						// enable HSE (HSI stays on)
	while (sys_config.HSE_enable	&& RCC->CR & 0x00020000UL);		// wait until HSE is ready
	RCC->CR |= (
		(0b111UL << 7U)							|					// max setting for LOCKDET_NSTOP
		(sys_config.PLL64_buffer_enable << 12U)	|					// enable PLL64 buffer (for RF)
		(sys_config.PLL_enable << 13U)			|					// enable PLL64 if set as sys clock (disable if not used)
		(sys_config.LSE_BYP << 6U)				|					// enable LSE bypass
		(sys_config.LSE_enable << 4U)			|					// enable LSE
		(sys_config.LSI_enable << 2U)								// enable LSI
	);
	while ((!sys_config.SYS_clk_src)	&& RCC->CR & 0x00004000UL);	// wait until PLL64 is ready
	while (sys_config.LSE_enable		&& RCC->CR & 0x00000020UL);	// wait until LSE is ready
	while (sys_config.LSI_enable		&& RCC->CR & 0x00000008UL);	// wait until LSI is ready
	RCC->CFGR = (
		(sys_config.IO_boost_enable << 17U)		|
		(sys_config.LS_clk_src << 15U)			|
		(sys_config.SMPS_clk_speed << 12U)		|
		//(sys_config.SYS_clk_speed << 5U)		|
		(sys_config.HSI_BLOCK_disable << 2U)	|
		(sys_config.SYS_clk_src << 1U)			|
		(sys_config.SMPS_clk_invert)
	);
	while (sys_config.HSI_BLOCK_disable	&& RCC->CR & 0x00000400UL);	// wait until HSI / PLL64 are disabled

	SYS_clock_frequency	= (64000000 >> sys_config.SYS_clk_speed);
	switch (sys_config.LS_clk_src) {
	case LS_CLK_SRC_LSI_LPMU:		LS_clock_frequency = LSI_clock_frequency;			break;
	case LS_CLK_SRC_LSE:			LS_clock_frequency = LSE_clock_frequency;			break;
	case LS_CLK_SRC_LSI:			LS_clock_frequency = LSI_clock_frequency;			break;
	case LS_CLK_SRC_HSI_DIV_2048:	LS_clock_frequency = (HSI_clock_frequency >> 11);	break;
	}

	SYS_TICK->LOAD = (SYS_clock_frequency >> 10) - 1; 						/* set reload register */
	// TODO: division needs to be more precise
	SYS_TICK->VAL  = 0;														/* load counter value  */
	SYS_TICK->CTRL = (														/* start SysTick timer */
			(sys_config.SYS_tick_enable << 0U)				|
			(sys_config.SYS_tick_interrupt_enable << 1U)	|
			0b100UL
	);
	// set IRQ priority
	SCB->SHP[1] |= 0xC0000000UL;
}

void sys_reset(void) {
	SCB->VTOR = (uint32_t)(IVT);

	/* TODO If the reset reason is a wakeup from power save restore the context */
	//if ((RCC->CSR == 0) && ((PWR->SR1 != 0)||(PWR->SR3 != 0))) {
	//	RAM_VR.WakeupFromSleepFlag = 1; /* A wakeup from power save occurred */
	//	CPUcontextRestore();            /* Restore the context */
	//	/* if the context restore worked properly, we should never return here */
	//	while(1) {
	//		NVIC_SystemReset();
	//	}
	//}
	// TODO: ram bank retention
	// TODO: GPIO retention
	// TODO: SMPS
	// TODO: HSI calib

	RCC->CR |= 0x00006400UL;										// Reset PLL64 flag
	while (!(RCC->CR & 0x00000400UL));								// wait until HSI is ready
	RCC->CFGR =	0x00000440UL;										// Reset CFGR
	RCC->CR =	0x00001400UL;										// Reset CR
	RCC->CIER =	0x00000000UL;										// Clear CIER

	__asm volatile ("cpsie i" ::: "memory");
}

void delay_ms(uint64_t ms) {
	uint64_t start = tick;
	while (tick - start < ms);
}

void sys_scale_clock(SYS_FLAG_t speed) {
	RCC->CSCMDR |= (
		(((speed >> 10) & 0b111UL) << 1U) 	|
		0b1UL
	);
	while(RCC->CSCMDR & 0b1U);
	SYS_clock_frequency	= (64000000 >> sys_config.SYS_clk_speed);
}