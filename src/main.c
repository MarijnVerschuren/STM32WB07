#include "GPIO.h"


//b1
// application
void main(void) {
	config_GPIO(GPIOB, 0, GPIO_output);
	config_GPIO(GPIOB, 1, GPIO_output);
	config_GPIO(GPIOB, 2, GPIO_output);

	for (;;) {
		GPIO_toggle(GPIOB, 0);
		GPIO_toggle(GPIOB, 1);
		GPIO_toggle(GPIOB, 2);
		for (_IO uint32_t i = 0; i < 0xFFFF; i++);
	}
}
