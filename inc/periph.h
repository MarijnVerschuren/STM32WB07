//
// Created by marijn on 10/30/24.
//

#ifndef STM32WB07_PERIPH_H
#define STM32WB07_PERIPH_H
#include "memory_map.h"
#include "types.h"


/*!< AHB peripherals */
#define GPIOA					((GPIO_t*)GPIOA_BASE)
#define GPIOB					((GPIO_t*)GPIOB_BASE)
#define RCC						((RCC_t*)RCC_BASE)


/*!<
 * core peripheral types
 * */
/*!< GPIO */
typedef struct {
	_IO uint32_t	MODER;			/* port mode                         0x00 */
	_IO uint32_t	OTYPER;			/* port output type                  0x04 */
	_IO uint32_t	OSPEEDR;		/* port output speed                 0x08 */
	_IO uint32_t	PUPDR;			/* port pull-up/pull-down            0x0C */
	_IO uint32_t	IDR;			/* port input data                   0x10 */
	_IO uint32_t	ODR;			/* port output data                  0x14 */
	_IO uint32_t	BSRR;			/* port bit set/reset                0x18 */
	_IO uint32_t	LCKR;			/* port configuration lock           0x1C */
	_IO uint32_t	AFR[2];			/* alternate function           0x20-0x24 */
} GPIO_t;


typedef struct {
	_IO uint32_t  CR;
	_IO uint32_t  ICSCR;
	_IO uint32_t  CFGR;
	_IO uint32_t  CSSWCR;
	_IO uint32_t  RESERVED[2];
	_IO uint32_t  CIER;
	_IO uint32_t  CIFR;
	_IO uint32_t  CSCMDR;
	_IO uint32_t  RESERVED1[3];
	_IO uint32_t  AHBRSTR;
	_IO uint32_t  APB0RSTR;
	_IO uint32_t  APB1RSTR;
	_IO uint32_t  RESERVED2;
	_IO uint32_t  APB2RSTR;
	_IO uint32_t  RESERVED3[3];
	_IO uint32_t  AHBENR;
	_IO uint32_t  APB0ENR;
	_IO uint32_t  APB1ENR;
	_IO uint32_t  RESERVED4;
	_IO uint32_t  APB2ENR;
	_IO uint32_t  RESERVED5[12];
	_IO uint32_t  CSR;
	_IO uint32_t  RFSWHSECR;
	_IO uint32_t  RFHSECR;
} RCC_t;


#endif //STM32WB07_PERIPH_H
