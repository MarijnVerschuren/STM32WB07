//
// Created by marijn on 11/3/24.
//

#ifndef STM32WB07_CRC_H
#define STM32WB07_CRC_H
#include "periph.h"


/*!<
 * types
 * */
typedef enum {
	CRC_REVERSE_OUT =		0b1UL << 7U,
	CRC_REVERSE_IN_BY_8 =	0b01UL << 5U,
	CRC_REVERSE_IN_BY_16 =	0b10UL << 5U,
	CRC_REVERSE_IN_BY_32 =	0b11UL << 5U,
	CRC_POLY_SIZE_32 =		0b00 << 3U,
	CRC_POLY_SIZE_16 =		0b01 << 3U,
	CRC_POLY_SIZE_8 =		0b10 << 3U,
	CRC_POLY_SIZE_7 =		0b11 << 3U
} CRC_flag_t;


/*!< enable / disable */
void enable_CRC(void);
void config_CRC(uint32_t poly, uint32_t init, uint32_t flags);
void disable_CRC(void);
/*!< actions */
void reset_CRC(void);


#endif //STM32WB07_CRC_H
