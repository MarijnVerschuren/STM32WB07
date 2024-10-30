//
// Created by marijn on 9/7/24.
//

#ifndef STM32WB07_MEMORY_MAP_H
#define STM32WB07_MEMORY_MAP_H


/*!<
 * core hardware memory map
 * */  // TODO: valid (old cortex M4 code)
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
#define RCC_BASE			(AHBPERIPH_BASE + 0x400000UL)


#endif //STM32WB07_MEMORY_MAP_H
