#include "sys.h"
#include "GPIO.h"
#include "EXTI.h"
#include "USART.h"
#include "tim.h"
#include "CRC.h"
#include "RNG.h"
#include "WDG.h"
#include "RTC.h"


/*!<
 * variables
 * */
_IO uint8_t status = 0b00U;
_IO uint32_t timestamp = 1735685999;


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

void TIM1_handler(void) {
	TIM1->SR &= ~0x00000001UL;
	timestamp = RTC_unix();
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

	/*!< LEDs */
	config_GPIO(GPIOB, 0, GPIO_output);		// B
	config_GPIO(GPIOB, 4, GPIO_output);		// G
	config_GPIO(GPIOB, 2, GPIO_output);		// R

	/*!< buttons */
	config_GPIO(GPIOA, 0, GPIO_input | GPIO_pull_up);		// BTN1
	config_EXTI(GPIOA, 0, EXTI_edge | EXTI_faling_edge);
	config_GPIO(GPIOB, 5, GPIO_input | GPIO_pull_up);		// BTN2
	config_EXTI(GPIOB, 5, EXTI_edge | EXTI_faling_edge);
	config_EXTI_IRQ(GPIOA, 0b11U); start_EXTI(GPIOA, 0);
	config_EXTI_IRQ(GPIOA, 0b11U); start_EXTI(GPIOB, 5);

	/*!< timer */
	config_TIM(64000, 1000);
	start_TIM_update_irq();

	/*!< RTC */
	uconfig_RTC(timestamp, RTC_WAKEUP_DISABLE, RTC_WAKEUP_DIV16, 0);

	/*!< uart */
	config_UART(LPUART1_TX_B6, LPUART1_RX_B7, 115200);  // TODO: test (CN4 -> 35 tx, 37 rx)

	/*!< CRC */
	config_CRC(0x04C11DB7UL, 0xFFFFFFFFUL, CRC_POLY_SIZE_32);  // MPEG-2
	CRC->DR = 0x12345678UL;
	uint32_t crc = CRC->DR;

	/*!< RNG */
	enable_RNG();
	uint32_t rn = RNG_generate();

	/*!< watchdog */
	config_WDG(WDG_DIV_256, 0xFFFUL);

	/*!< main loop */
	start_TIM();
	start_WDG();
	uint64_t prev = tick;
	for(;;) {
		if (tick - prev > 1000) { USART_print(LPUART1, "Hello World!\n", 100); prev = tick; }

		GPIO_write(GPIOB, 0, 1);
		GPIO_write(GPIOB, 4, 1);
		GPIO_write(GPIOB, 2, 1);

		if(status & 0b01U) { LED_sweep(0); }
		if(status & 0b10U) { LED_sweep(1); }

		WDG_tick();
	}

	sys_restart();
}

// TODO: (LP)uart I2C, SPI kernel frequencies
// TODO: SMPS stuff (AS bkup regulator etc)
// TODO: low power stuff PWR (GPIO ctrl is disabled in sysinit)