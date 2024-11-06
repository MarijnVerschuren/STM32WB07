#ifndef STM32WB07_WAKEUP_H
#define STM32WB07_WAKEUP_H
#include "periph.h"

typedef struct                                  /*!< Wakeup structure                                                          */
{
  _IO uint32_t  WAKEUP_BLOCK_VERSION;        /*!< (@ 0x00000000) Wakeup block version register                              */
  _IO uint32_t  RESERVED;
  _IO uint32_t  WAKEUP_OFFSET[2];            /*!< (@ 0x00000008) Wakeup offset_x register                                   */
  _IO uint32_t  ABSOLUTE_TIME;               /*!< (@ 0x00000010) Absolute time register                                     */
  _IO uint32_t  MINIMUM_PERIOD_LENGTH;       /*!< (@ 0x00000014) Minimum period length register                             */
  _IO uint32_t  AVERAGE_PERIOD_LENGTH;       /*!< (@ 0x00000018) Average period length register                             */
  _IO uint32_t  MAXIMUM_PERIOD_LENGTH;       /*!< (@ 0x0000001C) Maximum period length register                             */
  _IO uint32_t  STATISTICS_RESTART;          /*!< (@ 0x00000020) Statistics restart register                                */
  _IO uint32_t  BLUE_WAKEUP_TIME;            /*!< (@ 0x00000024) BLE wakeup time register                                   */
  _IO uint32_t  BLUE_SLEEP_REQUEST_MODE;     /*!< (@ 0x00000028) BLE sleep request mode register                            */
  _IO uint32_t  CM0_WAKEUP_TIME;             /*!< (@ 0x0000002C) CPU wakeup time register                                   */
  _IO uint32_t  CM0_SLEEP_REQUEST_MODE;      /*!< (@ 0x00000030) CPU sleep request mode register                            */
  _IO uint32_t  RESERVED1[3];
  _IO uint32_t  WAKEUP_BLE_IRQ_ENABLE;       /*!< (@ 0x00000040) Wakeup BLE interrupt enable register                       */
  _IO uint32_t  WAKEUP_BLE_IRQ_STATUS;       /*!< (@ 0x00000044) Wakeup BLE interrupt status register                       */
  _IO uint32_t  WAKEUP_CM0_IRQ_ENABLE;       /*!< (@ 0x00000048) Wakeup CPU interrupt enable register                       */
  _IO uint32_t  WAKEUP_CM0_IRQ_STATUS;       /*!< (@ 0x0000004C) Wakeup CPU interrupt status register                       */
  _IO uint32_t  RESERVED2;
} WAKEUP_t;                               /*!< Size = 84 (0x54)                                                          */

// TODO: define wakeup in periph
#define WAKEUP                      ((WAKEUP_t*)WAKEUP_BASE)
#endif