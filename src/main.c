#include "GPIO.h"


//b1
// application
void main(void) {
	uint32_t a = GPIO_to_int(GPIOB);
	config_GPIO(GPIOB, 0, GPIO_output, GPIO_no_pull, GPIO_push_pull);
	config_GPIO(GPIOB, 1, GPIO_output, GPIO_no_pull, GPIO_push_pull);

	for (;;) {
		GPIO_toggle(GPIOB, 0);
		GPIO_toggle(GPIOB, 1);
	}
}
