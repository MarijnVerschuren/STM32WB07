//
// Created by marijn on 11/1/24.
//
#ifndef STM32WB07_RF_H
#define STM32WB07_RF_H

#include "periph.h"
#include "NVIC.h"

/* RRM Config */
#define BLE_IBIAS           (0x08)
#define BLE_IPTAT           (0x07)
#define BLE_VBG             (0x08)

/* AFC Configuration */
#define AFC_DELAY_BEFORE    (0x05)
#define AFC_DELAY_AFTER     (0x05)
#define CR_GAIN_BEFORE      (0x06)
#define CR_GAIN_AFTER       (0x06)
#define CR_LR_GAIN_BEFORE   (0x05)
#define CR_LR_GAIN_AFTER    (0x05)
#define LR_RSSI_THR         (0x1D)
#define LR_PD_THR           (0x59)
#define LR_AAC_THR          (0x32)

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
void radio_init();
void radio_reset(void);
void radio_msp_init(void);

#endif //STM32WB07_RF_H

