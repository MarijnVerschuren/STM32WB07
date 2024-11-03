//
// Created by marijn on 11/3/24.
//

#ifndef STM32WB07_RNG_H
#define STM32WB07_RNG_H
#include "periph.h"


/*!< enable / disable */
void enable_RNG(void);
void disable_RNG(void);
/*!< usage */
uint32_t RNG_generate(void);


#endif //STM32WB07_RNG_H
