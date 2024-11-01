#include "sys.h"
#include "GPIO.h"


//b1
// application
void main(void) {
	sys_init(
		HSE_ENABLE | PLL_ENABLE | PLL64_buffer_ENABLE | SYS_CLK_SPEED_64MHz |
		SYS_CLK_SRC_PLL | SYS_TICK_ENABLE | SYS_TICK_INT_ENABLE
	);

	config_GPIO(GPIOB, 0, GPIO_output);		// B
	config_GPIO(GPIOB, 4, GPIO_output);		// G
	config_GPIO(GPIOB, 2, GPIO_output);		// R

	delay_ms(100);
	for (uint8_t i = 0; i < 12; i++) {
		GPIO_toggle(GPIOB, 0);
		delay_ms(50);
		GPIO_toggle(GPIOB, 4);
		delay_ms(50);
		GPIO_toggle(GPIOB, 2);
		delay_ms(50);
	}

	delay_ms(200);
	for (uint8_t i = 0; i < 10; i++) {
		GPIO_toggle(GPIOB, 0);
		GPIO_toggle(GPIOB, 4);
		GPIO_toggle(GPIOB, 2);
		delay_ms(100);
	}

	sys_restart();
}

// TODO: (LP)uart I2C, SPI kernel frequencies
// TODO: SMPS stuff (AS bkup regulator etc)
// TODO: low power stuff PWR (GPIO ctrl is disabled in sysinit)