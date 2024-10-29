//
// Created by marijn on 9/7/24.
//

#ifndef ARM_BOOTLOADER_PERIPH_H
#define ARM_BOOTLOADER_PERIPH_H


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
#define FLASH_BASE			0x08000000UL
#define SRAM_BASE			0x20000000UL
#define PERIPH_BASE			0x40000000UL

#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		(PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE		(PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE		(PERIPH_BASE + 0x10000000UL)


/*!< APB1 */

/*!< APB2 */

/*!< AHB1 */

/*!< AHB2 */


#endif //ARM_BOOTLOADER_PERIPH_H
