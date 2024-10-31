//
// Created by marijn on 10/30/24.
//

#ifndef STM32WB07_PERIPH_H
#define STM32WB07_PERIPH_H
#include "memory_map.h"
#include "types.h"


/*!< CORE peripherals */
#define SYS_TICK				((SYS_TICK_t*)SYS_TICK_BASE)
#define NVIC					((NVIC_t*)NVIC_BASE)
#define SCB						((SCB_t*)SCB_BASE)

/*!< AHB peripherals */
#define GPIOA					((GPIO_t*)GPIOA_BASE)
#define GPIOB					((GPIO_t*)GPIOB_BASE)
#define RCC						((RCC_t*)RCC_BASE)
#define PWR						((PWR_t*)PWR_BASE)

/*!< APB1 peripherals */
#define FLASH					((FLASH_t*)FLASH_BASE)


/*!<
 * core peripheral types
 * */
typedef struct {
	_IO uint32_t	CTRL;			/* control and status                0x00 */
	_IO uint32_t	LOAD;			/* reload value                      0x04 */
	_IO uint32_t	VAL;			/* current value                     0x08 */
	_I  uint32_t	CALIB;			/* calibration                       0x0C */
} SYS_TICK_t;

typedef struct {
	_IO uint32_t ISER[8U];			/* interrupt enable                 0x000 */
	uint32_t _0[24U];
	_IO uint32_t ICER[8U];			/* interrupt disable                0x080 */
	uint32_t _1[24U];
	_IO uint32_t ISPR[8U];			/* interrupt set pending            0x100 */
	uint32_t _2[24U];
	_IO uint32_t ICPR[8U];			/* interrupt clear pending			0x180 */
	uint32_t _3[24U];
	_IO uint32_t IABR[8U];			/* interrupt active bit             0x200 */
	uint32_t _4[56U];
	_IO uint8_t  IP[240U];			/* interrupt priority               0x300 */
	uint32_t _5[644U];
	_O	uint32_t STIR;				/* software trigger interrupt       0xE00 */
} NVIC_t;

typedef struct {
	_I	uint32_t	CPUID;			/* CPU ID                            0x00 */
	_IO uint32_t	ICSR;			/* interrupt control and state       0x04 */
	_IO uint32_t	VTOR;			/* vector table offset               0x08 */
	_IO uint32_t	AIRCR;			/* application interrupt and reset   0x0C */
	_IO uint32_t	SCR;			/* system control                    0x10 */
	_I	uint32_t	CCR;			/* configuration and control         0x14 */
	_I	uint32_t	_;				/*                                   0x18 */
	_IO uint32_t	SHPR2;			/* system handler priority 2         0x1C */
	_IO uint32_t	SHPR3;			/* system handler priority 3         0x20 */
} SCB_t;


/*!<
 * peripheral types
 * */
/*!< RCC */
typedef struct {
	_IO uint32_t  CR;				/* clock source control              0x00 */
	_IO uint32_t  _0;				/*                                   0x04 */
	_IO uint32_t  CFGR;				/* clock configuration               0x08 */
	_IO uint32_t  _1[3];			/*                            0x0C - 0x14 */
	_IO uint32_t  CIER;				/* clock interrupt enable            0x18 */
	_IO uint32_t  CIFR;				/* clock interrupt flag              0x1C */
	_IO uint32_t  CSCMDR;			/* clock switch command              0x20 */
	_IO uint32_t  _2[3];			/*                            0x24 - 0x2C */
	_IO uint32_t  AHBRSTR;			/* AHB reset                         0x30 */
	_IO uint32_t  APB1RSTR;			/* APB1 reset                        0x34 */  // NOTE: datasheet refers to APB0
	_IO uint32_t  APB2RSTR;			/* APB2 reset                        0x38 */  // NOTE: datasheet refers to APB1
	_IO uint32_t  _3;				/*                                   0x3C */
	_IO uint32_t  APB3RSTR;			/* APB3 reset                        0x40 */  // NOTE: datasheet refers to APB2
	_IO uint32_t  _4[3];			/*                            0x44 - 0x4C */
	_IO uint32_t  AHBENR;			/* AHB enable                        0x50 */
	_IO uint32_t  APB1ENR;			/* APB1 reset                        0x54 */  // NOTE: datasheet refers to APB0
	_IO uint32_t  APB2ENR;			/* APB2 reset                        0x58 */  // NOTE: datasheet refers to APB1
	_IO uint32_t  _5;				/*                                   0x5C */
	_IO uint32_t  APB3ENR;			/* APB3 reset                        0x60 */  // NOTE: datasheet refers to APB2
	_IO uint32_t  _6[12];			/*                            0x64 - 0x90 */
	_IO uint32_t  CSR;				/* reset status                      0x94 */
	_IO uint32_t  RFSWHSECR;		/* RF software high speed external   0x98 */
	_IO uint32_t  RFHSECR;			/* RF high speed external            0x9C */
} RCC_t;

/*!< PWR */
typedef struct {	// TODO comments
	_IO uint32_t  CR1;
	_IO uint32_t  CR2;
	_IO uint32_t  CR3;
	_IO uint32_t  CR4;
	_IO uint32_t  SR1;
	_IO uint32_t  SR2;
	_IO uint32_t  _0;
	_IO uint32_t  CR5;
	_IO uint32_t  PUCRA;
	_IO uint32_t  PDCRA;
	_IO uint32_t  PUCRB;
	_IO uint32_t  PDCRB;
	_IO uint32_t  CR6;
	_IO uint32_t  CR7;
	_IO uint32_t  SR3;
	_IO uint32_t  _1;
	_IO uint32_t  IOxCFG;
	_IO uint32_t  _2[16];
	_IO uint32_t  DBGR;
	_IO uint32_t  EXTSRR;
	_IO uint32_t  DBGSMPS;
	_IO uint32_t  TRIMR;
	_IO uint32_t  ENGTRIM;
	_IO uint32_t  DBG1;
	_IO uint32_t  DBG2;
} PWR_t;

/*!< FLASH */
typedef struct {	// TODO comments
	_IO uint32_t  COMMAND;
	_IO uint32_t  CONFIG;
	_IO uint32_t  IRQSTAT;
	_IO uint32_t  IRQMASK;
	_IO uint32_t  IRQRAW;
	_IO uint32_t  SIZE;
	_IO uint32_t  ADDRESS;
	_IO uint32_t  _0[2];
	_IO uint32_t  LFSRVAL;
	_IO uint32_t  _1[3];
	_IO uint32_t  PAGEPROT0;
	_IO uint32_t  PAGEPROT1;
	_IO uint32_t  _2;
	_IO uint32_t  DATA0;
	_IO uint32_t  DATA1;
	_IO uint32_t  DATA2;
	_IO uint32_t  DATA3;
} FLASH_t;

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


#endif //STM32WB07_PERIPH_H
