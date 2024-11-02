#include "sys.h"
#include "GPIO.h"
#include "EXTI.h"


/*!<
 * variables
 * */
_IO uint8_t status = 0b00U;


/*!<
 * interrupts
 * */
void GPIOA_handler(void) {
	SYSCFG->IO_ISCR |= 0x00000001UL;
	status ^= 0b01U;
}

void GPIOB_handler(void) {
	SYSCFG->IO_ISCR |= 0x00200000UL;
	status ^= 0b10U;
}


/*!<
 * functions
 * */
void LED_sweep(uint8_t d) {
	GPIO_toggle(GPIOB, d	<< 1);	delay_ms(50);
	GPIO_toggle(GPIOB, 4);			delay_ms(50);
	GPIO_toggle(GPIOB, (!d)	<< 1);	delay_ms(50);
	GPIO_toggle(GPIOB, d	<< 1);	delay_ms(50);
	GPIO_toggle(GPIOB, 4);			delay_ms(50);
	GPIO_toggle(GPIOB, (!d)	<< 1);	delay_ms(50);
}


/*!<
 * main
 * */
void main(void) {
	sys_init(
		HSE_ENABLE | PLL_ENABLE | PLL64_buffer_ENABLE | SYS_CLK_SPEED_64MHz |
		SYS_CLK_SRC_PLL | SYS_TICK_ENABLE | SYS_TICK_INT_ENABLE
	);

	config_GPIO(GPIOB, 0, GPIO_output);		// B
	config_GPIO(GPIOB, 4, GPIO_output);		// G
	config_GPIO(GPIOB, 2, GPIO_output);		// R

	config_GPIO(GPIOA, 0, GPIO_input | GPIO_pull_up);		// BTN1
	config_EXTI(GPIOA, 0, EXTI_edge | EXTI_faling_edge);
	config_GPIO(GPIOB, 5, GPIO_input | GPIO_pull_up);		// BTN2
	config_EXTI(GPIOB, 5, EXTI_edge | EXTI_faling_edge);
	config_EXTI_IRQ(GPIOA, 0b11U); start_EXTI(GPIOA, 0);
	config_EXTI_IRQ(GPIOA, 0b11U); start_EXTI(GPIOB, 5);
	// TODO: BTN3 for restart (BTN3 does not work)


	for(;;) {
		GPIO_write(GPIOB, 0, 1);
		GPIO_write(GPIOB, 4, 1);
		GPIO_write(GPIOB, 2, 1);

		if(status & 0b01U) { LED_sweep(0); }
		if(status & 0b10U) { LED_sweep(1); }
	}

	sys_restart();
}

// TODO: (LP)uart I2C, SPI kernel frequencies
// TODO: SMPS stuff (AS bkup regulator etc)
// TODO: low power stuff PWR (GPIO ctrl is disabled in sysinit)