//
// Created by marijn on 11/3/24.
//

#ifndef STM32WB07_WDG_H
#define STM32WB07_WDG_H
#include "periph.h"


/*!<
 * types
 * */
typedef enum {
	WDG_DIV_4 =		0b000UL,
	WDG_DIV_8 =		0b001UL,
	WDG_DIV_16 =	0b010UL,
	WDG_DIV_32 =	0b011UL,
	WDG_DIV_64 =	0b100UL,
	WDG_DIV_128 =	0b101UL,
	WDG_DIV_256 =	0b110UL,
} WDG_prescaler_t;


/*!< init / enable */
void config_WDG(WDG_prescaler_t prescaler, uint16_t reload);
// TODO: void config_window_watchdog(uint8_t prescaler, uint16_t reload);
/*!< actions */
void start_WDG();
void WDG_tick();


#endif //STM32WB07_WDG_H
