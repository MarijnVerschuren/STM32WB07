//
// Created by marijn on 11/1/24.
//

#ifndef STM32WB07_RF_H
#define STM32WB07_RF_H
#include "types.h"
#include "periph.h"

/*!<
 * types
 * */
typedef enum {
	RF_ENABLE =				0b1U << 0U,
	RF_CLK_DIV =			0b1U << 1U,
} RF_FLAG_t;

/*!<
 * functions
 * */
void radio_init(uint32_t flags);

#endif //STM32WB07_RF_H

