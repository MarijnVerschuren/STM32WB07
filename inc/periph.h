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

/*!< APB3 peripherals */
#define RADIO					((RADIO_t*)RADIO_BASE)
#define RADIO_CTRL              ((RADIO_CTRL_t *)RADIO_CTRL_BASE)
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
	_IO uint32_t	ISER[8U];		/* interrupt enable                 0x000 */
		uint32_t	_0[24U];
	_IO uint32_t	ICER[8U];		/* interrupt disable                0x080 */
		uint32_t	_1[24U];
	_IO uint32_t	ISPR[8U];		/* interrupt set pending            0x100 */
		uint32_t	_2[24U];
	_IO uint32_t	ICPR[8U];		/* interrupt clear pending			0x180 */
		uint32_t	_3[24U];
	_IO uint32_t	IABR[8U];		/* interrupt active bit             0x200 */
		uint32_t	_4[56U];
	_IO uint8_t 	IP[240U];		/* interrupt priority               0x300 */
		uint32_t	_5[644U];
	_O	uint32_t	STIR;			/* software trigger interrupt       0xE00 */
} NVIC_t;

typedef struct {
	_I	uint32_t	CPUID;			/* CPU ID                            0x00 */
	_IO uint32_t	ICSR;			/* interrupt control and state       0x04 */
	_IO uint32_t	VTOR;			/* vector table offset               0x08 */
	_IO uint32_t	AIRCR;			/* application interrupt and reset   0x0C */
	_IO uint32_t	SCR;			/* system control                    0x10 */
	_I	uint32_t	CCR;			/* configuration and control         0x14 */
	_I	uint32_t	_;				/*                                   0x18 */
	_IO uint32_t	SHP[2];			/* system handler priority    0x1C - 0x20 */
	_IO uint32_t	SHCSR;			/* system handler status and control 0x24 */
} SCB_t;


/*!<
 * peripheral types
 * */
/*!< RCC */
typedef struct {
	_IO uint32_t	CR;				/* clock source control              0x00 */
	_IO uint32_t	_0;				/*                                   0x04 */
	_IO uint32_t	CFGR;			/* clock configuration               0x08 */
	_IO uint32_t	_1[3];			/*                            0x0C - 0x14 */
	_IO uint32_t	CIER;			/* clock interrupt enable            0x18 */
	_IO uint32_t	CIFR;			/* clock interrupt flag              0x1C */
	_IO uint32_t	CSCMDR;			/* clock switch command              0x20 */
	_IO uint32_t	_2[3];			/*                            0x24 - 0x2C */
	_IO uint32_t	AHBRSTR;		/* AHB reset                         0x30 */
	_IO uint32_t	APB1RSTR;		/* APB1 reset                        0x34 */  // NOTE: datasheet refers to APB0
	_IO uint32_t	APB2RSTR;		/* APB2 reset                        0x38 */  // NOTE: datasheet refers to APB1
	_IO uint32_t	_3;				/*                                   0x3C */
	_IO uint32_t	APB3RSTR;		/* APB3 reset                        0x40 */  // NOTE: datasheet refers to APB2
	_IO uint32_t	_4[3];			/*                            0x44 - 0x4C */
	_IO uint32_t	AHBENR;			/* AHB enable                        0x50 */
	_IO uint32_t	APB1ENR;		/* APB1 reset                        0x54 */  // NOTE: datasheet refers to APB0
	_IO uint32_t	APB2ENR;		/* APB2 reset                        0x58 */  // NOTE: datasheet refers to APB1
	_IO uint32_t	_5;				/*                                   0x5C */
	_IO uint32_t	APB3ENR;		/* APB3 reset                        0x60 */  // NOTE: datasheet refers to APB2
	_IO uint32_t	_6[12];			/*                            0x64 - 0x90 */
	_IO uint32_t	CSR;			/* reset status                      0x94 */
	_IO uint32_t	RFSWHSECR;		/* RF software high speed external   0x98 */
	_IO uint32_t	RFHSECR;		/* RF high speed external            0x9C */
} RCC_t;

/*!< PWR */
typedef struct {	// TODO find last 5 register names
	_IO uint32_t	CR1;			/* control  1						 0x00 */
	_IO uint32_t	CR2;			/* control  2						 0x04 */
	_IO uint32_t	CR3;			/* control  3						 0x08 */
	_IO uint32_t	CR4;			/* control  4						 0x0C */
	_IO uint32_t	SR1;			/* status  1						 0x10 */
	_IO uint32_t	SR2;			/* status  2						 0x14 */
	_IO uint32_t	_0;				/*									 0x18 */
	_IO uint32_t	CR5;			/* control  5						 0x1C */
	_IO uint32_t	PUCRA;			/* port A pull-up control 		     0x20 */
	_IO uint32_t	PDCRA;			/* port A pull-down control			 0x24 */
	_IO uint32_t	PUCRB;			/* port B pull-up control 			 0x28 */
	_IO uint32_t	PDCRB;			/* port B pull-down control 		 0x2C */
	_IO uint32_t	CR6;			/* control  6						 0x30 */
	_IO uint32_t	CR7;			/* control  7						 0x34 */
	_IO uint32_t	SR3;			/* status  3						 0x38 */
	_IO uint32_t	_1;				/*									 0x3C */
	_IO uint32_t	IOxCFG;			/* I/O deepstop drive configuration  0x40 */
	_IO uint32_t	_2[16];			/*  						  0x44 - 0x80 */
	_IO uint32_t	DBGR;			/* debug 							 0x84 */
	_IO uint32_t	EXTSRR;			/* extended status and reset		 0x88 */
	_IO uint32_t	DBGSMPS;		/*									 0x8C */
	_IO uint32_t	TRIMR;			/*									 0x90 */
	_IO uint32_t	ENGTRIM;		/*									 0x94 */
	_IO uint32_t	DBG1;			/*									 0x98 */
	_IO uint32_t	DBG2;			/*									 0x9C */
} PWR_t;

/*!< FLASH */
typedef struct {
	_IO uint32_t	COMMAND;		/* command 							 0x00 */
	_IO uint32_t	CONFIG;			/* configuration					 0x04 */
	_IO uint32_t	IRQSTAT;		/* interrupt state					 0x08 */
	_IO uint32_t	IRQMASK;		/* interrupt mask					 0x0C */
	_IO uint32_t	IRQRAW;			/* raw status						 0x10 */
	_IO uint32_t	SIZE;			/* size								 0x14 */
	_IO uint32_t	ADDRESS;		/* address							 0x18 */
	_IO uint32_t	_0[2];			/*							  0x1C - 0x20 */
	_IO uint32_t	LFSRVAL;		/* linear feedback shift		     0x24 */
	_IO uint32_t	_1[3];			/*							  0x28 - 0x30 */
	_IO uint32_t	PAGEPROT0;		/* flash page protection 1		     0x34 */
	_IO uint32_t	PAGEPROT1;		/* flash page protection 2		     0x38 */
	_IO uint32_t	_2;				/*								     0x3C */
	_IO uint32_t	DATA0;			/* data 0						     0x40 */
	_IO uint32_t	DATA1;			/* data 1						     0x44 */
	_IO uint32_t	DATA2;			/* data 2						     0x48 */
	_IO uint32_t	DATA3;			/* data 3						     0x4C */
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

/*!< RADIO */
typedef struct {
	_I	uint32_t	CID;			/* controller version number         0x00 */
	_IO uint32_t	ISR[2];			/* interrupt status           0x04 - 0x08 */
	_IO uint32_t	TODR;			/* destination timeout               0x0C */
	_IO uint32_t	TOR;			/* timeout                           0x10 */
	_I	uint32_t	TOCR;			/* capture timeout                   0x14 */
	_IO uint32_t	CR;				/* command                           0x18 */
	_I	uint32_t	SR1;			/* status                            0x1C */
	_I	uint32_t	IESR;			/* interrupt 1 enabled status        0x20 */
	_I	uint32_t	ILSR;			/* interrupt 1 latency status        0x24 */
	_IO uint32_t	AES_KEY[4];		/* AES key                    0x28 - 0x34 */
	_IO uint32_t	AES_TXT[4];		/* AES clear text             0x38 - 0x44 */
	_I	uint32_t	AES_CYP[4];		/* AES cypher text            0x48 - 0x54 */
	_IO uint32_t	AES_CMD;		/* AES command                       0x58 */
	_I	uint32_t	AES_SR;			/* AES status                        0x5C */
	_IO uint32_t	AES_PPTR;		/* AES private pointer               0x60 */
	_IO uint32_t	AES_PHASH;		/* AES private hash                  0x64 */
	_IO uint32_t	AES_PRAND;		/* AES private prand                 0x68 */  // TODO: prand?
	_IO uint32_t	AES_PCMD;		/* AES private command               0x6C */
	_I	uint32_t	AES_PSR;		/* AES private status                0x70 */
	_IO uint32_t	DBG_CMD;		/* debug command                     0x74 */
	_I	uint32_t	DBG_SR;			/* debug status                      0x78 */
	_I	uint32_t	SR2;			/* status 2                          0x7C */
} RADIO_t;

/*!< RADIO_CTRL */
typedef struct{
	_I	uint32_t	CID;			/* controller version number         0x00 */
	_IO uint32_t	WLR;			/* window length                     0x04 */
	_I	uint32_t	SCPR;			/* slow clock period                 0x08 */
	_I	uint32_t	SCFR;			/* slow clock frequency              0x0C */
	_IO uint32_t	ISR;			/* interrupt status                  0x10 */
	_IO uint32_t	IER;			/* interrupt control / enable        0x14 */
} RADIO_CTRL_t;

#endif //STM32WB07_PERIPH_H
