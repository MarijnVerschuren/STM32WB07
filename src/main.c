#include "GPIO.h"


//b1
// application
void main(void) {
	config_GPIO(GPIOB, 0, GPIO_output);		// B
	config_GPIO(GPIOB, 2, GPIO_output);		// R
	config_GPIO(GPIOB, 4, GPIO_output);		// G

	for (;;) {
		GPIO_toggle(GPIOB, 0);
		GPIO_toggle(GPIOB, 2);
		GPIO_toggle(GPIOB, 4);
		for (_IO uint32_t i = 0; i < 0xFFFF; i++);
	}
}

// TODO: (LP)uart I2C, SPI kernel frequencies
// TODO: SMPS stuff (AS bkup regulator etc)
// TODO: low power stuff PWR (GPIO ctrl is disabled in sysinit)