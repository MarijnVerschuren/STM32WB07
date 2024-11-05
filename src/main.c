#include "sys.h"
#include "GPIO.h"
#include "EXTI.h"
#include "USART.h"
#include "tim.h"
#include "CRC.h"
#include "RNG.h"
#include "WDG.h"
#include "RTC.h"
#include "SPI.h"


/*!<
 * variables
 * */
_IO uint8_t status = 0b00U;
_IO uint32_t timestamp = 0;


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
		HSE_ENABLE | LSE_ENABLE | PLL_ENABLE | PLL64_buffer_ENABLE |
		SYS_CLK_SPEED_64MHz | SYS_CLK_SRC_PLL | LS_CLK_SRC_LSE |
		SYS_TICK_ENABLE | SYS_TICK_INT_ENABLE
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
	uconfig_RTC(1735689599, RTC_WAKEUP_DISABLE, RTC_WAKEUP_DIV16, 0);  // TODO!!

	/*!< uart */
	config_UART(USART1_TX_A9, USART1_RX_A8, 115200);  // TODO test

	/*!< CRC */
	config_CRC(0x04C11DB7UL, 0xFFFFFFFFUL, CRC_POLY_SIZE_32);  // MPEG-2
	CRC->DR = 0x12345678UL;
	uint32_t crc = CRC->DR;

	/*!< RNG */
	enable_RNG();
	uint32_t rn = RNG_generate();

	/*!< watchdog */
	config_WDG(WDG_DIV_256, 0xFFFUL);

	/*!< SPI */
	config_SPI_master(
		SPI2_SCK_A5, SPI2_MOSI_A6, SPI2_MISO_A7,
		SPI_CPHA_FIRST_EDGE | SPI_CPOL_LOW | SPI_CLK_DIV_2 |
		SPI_ENDIANNESS_MSB | SPI_MODE_DUPLEX | SPI_FRAME_MOTOROLA |
		SPI_DATA_8 | SPI_FIFO_TH_HALF
	);

	/*!< main loop */
	start_TIM();
	//start_WDG();
	uint64_t prev = tick;
	uint32_t dt = 0;
	for(;;) {
		if (tick - prev > 100) {
			prev = tick;
			USART_write(USART1, (void*)&timestamp, 4, 10);
			SPI_master_write8(SPI2, (void*)&timestamp, 4, 10);
		}

		GPIO_write(GPIOB, 0, 1);
		GPIO_write(GPIOB, 4, 1);
		GPIO_write(GPIOB, 2, 1);

		if(status & 0b01U) { LED_sweep(0); }
		if(status & 0b10U) { LED_sweep(1); }

		//WDG_tick();
	}

	sys_restart();
}

// TODO: (LP)uart I2C, SPI kernel frequencies
// TODO: SMPS stuff (AS bkup regulator etc)
// TODO: low power stuff PWR (GPIO ctrl is disabled in sysinit)