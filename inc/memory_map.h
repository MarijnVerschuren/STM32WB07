//
// Created by marijn on 9/7/24.
//

#ifndef STM32WB07_MEMORY_MAP_H
#define STM32WB07_MEMORY_MAP_H


/*!<
 * core hardware memory map
 * */
#define SCS_BASE			(0xE000E000UL)
#define ITM_BASE			(0xE0000000UL)
#define DWT_BASE			(0xE0001000UL)
#define TPI_BASE			(0xE0040000UL)
#define CORE_DEBUG_BASE		(0xE000EDF0UL)

#define SYS_TICK_BASE		(SCS_BASE +  0x0010UL)
#define NVIC_BASE			(SCS_BASE +  0x0100UL)
#define SCB_BASE			(SCS_BASE +  0x0D00UL)


/*!<
 * extended hardware memory map
 * */
#define FLASH_BASE			0x10040000UL
#define SRAM_BASE			0x20000000UL
#define PERIPH_BASE			0x40000000UL

#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		(PERIPH_BASE + 0x01000000UL)
#define APB3PERIPH_BASE		(PERIPH_BASE + 0x20000000UL)
#define AHBPERIPH_BASE		(PERIPH_BASE + 0x08000000UL)


/*!< AHB */
#define GPIOA_BASE			(AHBPERIPH_BASE + 0x000000UL)
#define GPIOB_BASE			(AHBPERIPH_BASE + 0x100000UL)
#define CRC_BASE			(AHBPERIPH_BASE + 0x200000UL)
#define PKA_BASE			(AHBPERIPH_BASE + 0x300000UL)
#define PKA_RAM_BASE		(AHBPERIPH_BASE + 0x300400UL)
#define RCC_BASE			(AHBPERIPH_BASE + 0x400000UL)
#define PWR_BASE			(AHBPERIPH_BASE + 0x500000UL)
#define RNG_BASE			(AHBPERIPH_BASE + 0x600000UL)
#define DMA1_BASE			(AHBPERIPH_BASE + 0x700000UL)
#define DMAMUX1_BASE		(AHBPERIPH_BASE + 0x800000UL)

/*!< APB1 */
#define SYSCFG_BASE			(APB1PERIPH_BASE + 0x0000UL)
#define FLASH_BASE			(APB1PERIPH_BASE + 0x1000UL)
#define TIM1_BASE			(APB1PERIPH_BASE + 0x2000UL)
#define IWDG_BASE			(APB1PERIPH_BASE + 0x3000UL)
#define RTC_BASE			(APB1PERIPH_BASE + 0x4000UL)
#define AHBUPCONV_BASE		(APB1PERIPH_BASE + 0x5000UL)

/*!< APB2 */
#define I2C1_BASE			(APB2PERIPH_BASE + 0x0000UL)
#define I2C2_BASE			(APB2PERIPH_BASE + 0x1000UL)
#define SPI1_BASE			(APB2PERIPH_BASE + 0x2000UL)
#define SPI2_BASE			(APB2PERIPH_BASE + 0x3000UL)
#define USART1_BASE			(APB2PERIPH_BASE + 0x4000UL)
#define LPUART1_BASE		(APB2PERIPH_BASE + 0x5000UL)
#define ADC1_BASE			(APB2PERIPH_BASE + 0x6000UL)
#define SPI3_BASE			(APB2PERIPH_BASE + 0x7000UL)

/*!< APB3 */
#define BLUE_BASE			(APB3PERIPH_BASE + 0x0000UL)
#define RADIO_CTRL_BASE		(APB3PERIPH_BASE + 0x1000UL)
#define RRM_BASE			(APB3PERIPH_BASE + 0x1400UL)
#define WAKEUP_BASE			(APB3PERIPH_BASE + 0x1800UL)


#endif //STM32WB07_MEMORY_MAP_H
