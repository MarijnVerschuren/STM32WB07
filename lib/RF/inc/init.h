//
// Created by marijn on 11/13/24.
//

#ifndef STM32WB07_INIT_H
#define STM32WB07_INIT_H
// NOTE: this header file acts as a barrier between my custom code and the ST HAL library which is of extremely poor quality
void MX_RADIO_Init(void);
void MX_RADIO_TIMER_Init(void);
void MX_APPE_Init(void);

#endif //STM32WB07_INIT_H
