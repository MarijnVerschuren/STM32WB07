//
// Created by marijn on 11/3/24.
//

#ifndef STM32WB07_TIM_H
#define STM32WB07_TIM_H
#include "types.h"
#include "periph.h"
#include "NVIC.h"


/*!< init / enable / disable */
void config_TIM(uint32_t prescaler, uint32_t limit);
void disable_TIM(void);
/*!< actions */
void start_TIM(void);
void stop_TIM(void);
void delay_TIM(uint32_t count);
/*!< irq */
void start_TIM_update_irq(void);
void stop_TIM_update_irq(void);

#endif //STM32WB07_TIM_H
