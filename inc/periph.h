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
#define CRC						((CRC_t*)CRC_BASE)
#define RCC						((RCC_t*)RCC_BASE)
#define PWR						((PWR_t*)PWR_BASE)
#define RNG						((RNG_t*)RNG_BASE)

/*!< APB1 peripherals */
#define SYSCFG					((SYSCFG_t*)SYSCFG_BASE)
#define FLASH					((FLASH_t*)FLASH_BASE)
#define TIM1					((TIM_t*)TIM1_BASE)
#define IWDG					((IWDG_t*)IWDG_BASE)
#define RTC						((RTC_t*)RTC_BASE)

/*!< APB2 */
#define USART1					((USART_t*)USART1_BASE)
#define LPUART1					((USART_t*)LPUART1_BASE)

/*!< APB3 peripherals */
#define RADIO					((RADIO_t*)RADIO_BASE)
#define RADIO_CTRL              ((RADIO_CTRL_t *)RADIO_CTRL_BASE)
#define RRM


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
	_IO uint32_t	ISER;			/* interrupt enable                 0x000 */
		uint32_t	_0[31U];
	_IO uint32_t	ICER;			/* interrupt disable                0x080 */
		uint32_t	_1[31U];
	_IO uint32_t	ISPR;			/* interrupt set pending            0x100 */
		uint32_t	_2[31U];
	_IO uint32_t	ICPR;			/* interrupt clear pending          0x180 */
		uint32_t	_3[95U];
	_IO uint8_t 	IP[32U];		/* interrupt priority               0x300 */
} NVIC_t;

typedef struct {
	_I	uint32_t	CPUID;			/* CPU ID                            0x00 */
	_IO uint32_t	ICSR;			/* interrupt control and state       0x04 */
	_IO uint32_t	VTOR;			/* vector table offset               0x08 */
	_IO uint32_t	AIRCR;			/* application interrupt and reset   0x0C */
	_IO uint32_t	SCR;			/* system control                    0x10 */
	_I	uint32_t	CCR;			/* configuration and control         0x14 */
		uint32_t	_;				/*                                   0x18 */
	_IO uint32_t	SHP[2];			/* system handler priority    0x1C - 0x20 */
	_IO uint32_t	SHCSR;			/* system handler status and control 0x24 */
} SCB_t;


/*!<
 * peripheral types
 * */
/*!< RCC */
typedef struct {
	_IO uint32_t	CR;				/* clock source control              0x00 */
		uint32_t	_0;				/*                                   0x04 */
	_IO uint32_t	CFGR;			/* clock configuration               0x08 */
		uint32_t	_1[3];			/*                            0x0C - 0x14 */
	_IO uint32_t	CIER;			/* clock interrupt enable            0x18 */
	_IO uint32_t	CIFR;			/* clock interrupt flag              0x1C */
	_IO uint32_t	CSCMDR;			/* clock switch command              0x20 */
		uint32_t	_2[3];			/*                            0x24 - 0x2C */
	_IO uint32_t	AHBRSTR;		/* AHB reset                         0x30 */
	_IO uint32_t	APB1RSTR;		/* APB1 reset                        0x34 */  // NOTE: datasheet refers to APB0
	_IO uint32_t	APB2RSTR;		/* APB2 reset                        0x38 */  // NOTE: datasheet refers to APB1
		uint32_t	_3;				/*                                   0x3C */
	_IO uint32_t	APB3RSTR;		/* APB3 reset                        0x40 */  // NOTE: datasheet refers to APB2
		uint32_t	_4[3];			/*                            0x44 - 0x4C */
	_IO uint32_t	AHBENR;			/* AHB enable                        0x50 */
	_IO uint32_t	APB1ENR;		/* APB1 enable                       0x54 */  // NOTE: datasheet refers to APB0
	_IO uint32_t	APB2ENR;		/* APB2 enable                       0x58 */  // NOTE: datasheet refers to APB1
		uint32_t	_5;				/*                                   0x5C */
	_IO uint32_t	APB3ENR;		/* APB3 enable                       0x60 */  // NOTE: datasheet refers to APB2
		uint32_t	_6[12];			/*                            0x64 - 0x90 */
	_IO uint32_t	CSR;			/* reset status                      0x94 */
	_IO uint32_t	RFSWHSECR;		/* RF software high speed external   0x98 */
	_IO uint32_t	RFHSECR;		/* RF high speed external            0x9C */
} RCC_t;

/*!< PWR */
typedef struct {	// TODO find last 5 register names
	_IO uint32_t	CR1;			/* control  1                        0x00 */
	_IO uint32_t	CR2;			/* control  2                        0x04 */
	_IO uint32_t	CR3;			/* control  3                        0x08 */
	_IO uint32_t	CR4;			/* control  4                        0x0C */
	_IO uint32_t	SR1;			/* status  1                         0x10 */
	_IO uint32_t	SR2;			/* status  2                         0x14 */
		uint32_t	_0;				/*                                   0x18 */
	_IO uint32_t	CR5;			/* control  5                        0x1C */
	_IO uint32_t	PUCRA;			/* port A pull-up control            0x20 */
	_IO uint32_t	PDCRA;			/* port A pull-down control          0x24 */
	_IO uint32_t	PUCRB;			/* port B pull-up control            0x28 */
	_IO uint32_t	PDCRB;			/* port B pull-down control          0x2C */
	_IO uint32_t	CR6;			/* control  6                        0x30 */
	_IO uint32_t	CR7;			/* control  7                        0x34 */
	_IO uint32_t	SR3;			/* status  3                         0x38 */
		uint32_t	_1;				/*                                   0x3C */
	_IO uint32_t	IOxCFG;			/* I/O deepstop drive configuration  0x40 */
		uint32_t	_2[16];			/*                            0x44 - 0x80 */
	_IO uint32_t	DBGR;			/* debug                             0x84 */
	_IO uint32_t	EXTSRR;			/* extended status and reset         0x88 */
	_IO uint32_t	DBGSMPS;		/*                                   0x8C */
	_IO uint32_t	TRIMR;			/*                                   0x90 */
	_IO uint32_t	ENGTRIM;		/*                                   0x94 */
	_IO uint32_t	DBG1;			/*                                   0x98 */
	_IO uint32_t	DBG2;			/*                                   0x9C */
} PWR_t;

/*!< FLASH */
typedef struct {
	_IO uint32_t	COMMAND;		/* command                           0x00 */
	_IO uint32_t	CONFIG;			/* configuration                     0x04 */
	_IO uint32_t	IRQSTAT;		/* interrupt state                   0x08 */
	_IO uint32_t	IRQMASK;		/* interrupt mask                    0x0C */
	_IO uint32_t	IRQRAW;			/* raw status                        0x10 */
	_IO uint32_t	SIZE;			/* size                              0x14 */
	_IO uint32_t	ADDRESS;		/* address                           0x18 */
		uint32_t	_0[2];			/*                            0x1C - 0x20 */
	_IO uint32_t	LFSRVAL;		/* linear feedback shift             0x24 */
		uint32_t	_1[3];			/*                            0x28 - 0x30 */
	_IO uint32_t	PAGEPROT0;		/* flash page protection 1           0x34 */
	_IO uint32_t	PAGEPROT1;		/* flash page protection 2           0x38 */
		uint32_t	_2;				/*                                   0x3C */
	_IO uint32_t	DATA0;			/* data 0                            0x40 */
	_IO uint32_t	DATA1;			/* data 1                            0x44 */
	_IO uint32_t	DATA2;			/* data 2                            0x48 */
	_IO uint32_t	DATA3;			/* data 3                            0x4C */
} FLASH_t;

/*!< WDG */
typedef struct {
	_IO uint32_t	KR;				/* key                               0x00 */
	_IO uint32_t	PR;				/* prescaler                         0x04 */
	_IO uint32_t	RLR;			/* reload                            0x08 */
	_IO uint32_t	SR;				/* status                            0x0C */
	_IO uint32_t	WINR;			/* window                            0x10 */
} IWDG_t;

/*!< TIM */
typedef struct {
	_IO uint32_t	CR1;			/* control 1                         0x00 */
	_IO uint32_t	CR2;			/* control 2                         0x04 */
	_IO uint32_t	SMCR;			/* slave mode control                0x08 */
	_IO uint32_t	DIER;			/* DMA/interrupt enable              0x0C */
	_IO uint32_t	SR;				/* status                            0x10 */
	_IO uint32_t	EGR;			/* event generation                  0x14 */
	_IO uint32_t	CCMR1;			/* capture/compare mode 1            0x18 */
	_IO uint32_t	CCMR2;			/* capture/compare mode 2            0x1C */
	_IO uint32_t	CCER;			/* capture/compare enable            0x20 */
	_IO uint32_t	CNT;			/* counter                           0x24 */
	_IO uint32_t	PSC;			/* prescaler,                        0x28 */
	_IO uint32_t	ARR;			/* auto-reload                       0x2C */
	_IO uint32_t	RCR;			/* repetition counter                0x30 */
	_IO uint32_t	CCR1;			/* capture/compare 1                 0x34 */
	_IO uint32_t	CCR2;			/* capture/compare 2                 0x38 */
	_IO uint32_t	CCR3;			/* capture/compare 3                 0x3C */
	_IO uint32_t	CCR4;			/* capture/compare 4                 0x40 */
	_IO uint32_t	BDTR;			/* break and dead-time               0x44 */
	_IO uint32_t	DCR;			/* DMA control                       0x48 */
	_IO uint32_t	DMAR;			/* DMA address for full transfer     0x4C */
	_IO uint32_t	OR;				/* option                            0x50 */
	_IO uint32_t	CCMR3;			/* output/compare mode 3             0x54 */
	_IO uint32_t	CCR5;			/* capture/compare 5                 0x58 */
	_IO uint32_t	CCR6;			/* capture/compare 6                 0x5C */
	_IO uint32_t	AF1;			/* TIM alternate function option 1   0x60 */
	_IO uint32_t	AF2;			/* TIM alternate function option 2   0x64 */
} TIM_t;

/*!< RTC */
typedef struct {
	_IO uint32_t	TR;				/* time                              0x00 */
	_IO uint32_t	DR;				/* date                              0x04 */
	_IO uint32_t	CR;				/* control                           0x08 */
	_IO uint32_t	ISR;			/* initialization and status         0x0C */
	_IO uint32_t	PRER;			/* prescaler                         0x10 */
	_IO uint32_t	WUTR;			/* wakeup timer                      0x14 */
	_IO uint32_t	CALIBR;			/* calibration                       0x18 */
	_IO uint32_t	ALRMAR;			/* alarm A                           0x1C */
	_IO uint32_t	ALRMBR;			/* alarm B                           0x20 */
	_IO uint32_t	WPR;			/* write protection                  0x24 */
	_IO uint32_t	SSR;			/* sub second                        0x28 */
	_IO uint32_t	SHIFTR;			/* shift control                     0x2C */
	_IO uint32_t	TSTR;			/* time stamp time                   0x30 */
	_IO uint32_t	TSDR;			/* time stamp date                   0x34 */
	_IO uint32_t	TSSSR;			/* time-stamp sub second             0x38 */
	_IO uint32_t	CALR;			/* calibration                       0x3C */
	_IO uint32_t	TAFCR;			/* tamper and af configuration       0x40 */
	_IO uint32_t	ALRMASSR;		/* alarm A sub second                0x44 */
	_IO uint32_t	ALRMBSSR;		/* alarm B sub second                0x48 */
		uint32_t	_;				/*                                   0x4C */
	_IO uint32_t	BKPR[2];		/* backup registers             0x50-0x54 */
} RTC_t;

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

/*!< SYSCFG */
typedef struct {
	_IO uint32_t	DIE_ID;			/* CPU die ID                        0x00 */
	_IO uint32_t	JTAG_ID;		/* JTAG ID                           0x04 */
	_IO uint32_t	I2C_FMP_CTRL;	/* I2C Fast Mode Plus control        0x08 */
	_IO uint32_t	IO_DTR;			/* EXTI detection type               0x0C */
	_IO uint32_t	IO_IBER;		/* EXTI detection edge               0x10 */
	_IO uint32_t	IO_IEVR;		/* EXTI polarity                     0x14 */
	_IO uint32_t	IO_IER;			/* EXTI enable                       0x18 */
	_IO uint32_t	IO_ISCR;		/* EXTI status                       0x1C */
	_IO uint32_t	PWRC_IER;		/* PWR controller interrupt enable   0x20 */
	_IO uint32_t	PWRC_ISCR;		/* PWR controller interrupt status   0x24 */
	_IO uint32_t	IO_SWA_CTRL;	/* IO analog switch control          0x28 */
	_IO uint32_t	BLERXTX_DTR;	/* BLE TX/RX interrupt type          0x2C */
	_IO uint32_t	BLERXTX_IBER;	/* BLE TX/RX interrupt edge          0x30 */
	_IO uint32_t	BLERXTX_IEVR;	/* BLE TX/RX interrupt polarity      0x34 */
	_IO uint32_t	BLERXTX_IER;	/* BLE TX/RX interrupt enable        0x38 */
	_IO uint32_t	BLERXTX_ISCR;	/* BLE TX/RX interrupt status        0x3C */
} SYSCFG_t;

/*!< USART */
typedef struct {
	_IO uint32_t	CR1;			/* control 1                         0x00 */
	_IO uint32_t	CR2;			/* control 2                         0x04 */
	_IO uint32_t	CR3;			/* control 3                         0x08 */
	_IO uint32_t	BRR;			/* baud rate                         0x0C */
	_IO uint32_t	GTPR;			/* guard time and prescaler          0x10 */
	_IO uint32_t	RTOR;			/* receiver timeout                  0x14 */
	_IO uint32_t	RQR;			/* request                           0x18 */
	_IO uint32_t	ISR;			/* interrupt and status              0x1C */
	_IO uint32_t	ICR;			/* interrupt flag clear              0x20 */
	_IO uint32_t	RDR;			/* receive data                      0x24 */
	_IO uint32_t	TDR;			/* transmit data                     0x28 */
	_IO uint32_t	PSC;			/* prescaler register                0x2C */
} USART_t;

/*!< CRC */
typedef struct {
	_IO uint32_t	DR;				/* data                              0x00 */
	_IO uint32_t	IDR;			/* independent data                  0x04 */
	_IO uint32_t	CR;				/* control                           0x08 */
		uint32_t	_;				/*                                   0x0C */
	_IO uint32_t	INIT;			/* initial value                     0x10 */
	_IO uint32_t	POL;			/* polynomial                        0x14 */
} CRC_t;

/*!< RNG */
typedef struct {
	_IO uint32_t	CR;				/* control                           0x00 */
	_IO uint32_t	SR;				/* status                            0x04 */
	_IO uint32_t	VAL;			/* value                             0x08 */
} RNG_t;

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

/*!< RRM */
typedef struct
{
	_IO uint32_t  RRM_ID;				/*!<  (@ 0x00) RRM_ID register                                                 */
	_IO uint32_t  RRM_CTRL;          	/*!<  (@ 0x04) RRM_CTRL register                                               */
	_IO uint32_t  RESERVED[2];
	_IO uint32_t  UDRA_CTRL0;           /*!<  (@ 0x10) UDRA_CTRL0 register                                             */
	_IO uint32_t  UDRA_IRQ_ENABLE;   	/*!<  (@ 0x14) UDRA_IRQ_ENABLE register                                        */
	_IO uint32_t  UDRA_IRQ_STATUS;    	/*!<  (@ 0x18) UDRA_IRQ_STATUS register                                        */
	_IO uint32_t  UDRA_RADIO_CFG_PTR;   /*!<  (@ 0x1C) UDRA_RADIO_CFG_PTR register                                     */
	_IO uint32_t  SEMA_IRQ_ENABLE;    	/*!<  (@ 0x20) SEMA_IRQ_ENABLE register                                        */
	_IO uint32_t  SEMA_IRQ_STATUS;   	/*!<  (@ 0x24) SEMA_IRQ_STATUS register                                        */
	_IO uint32_t  BLE_IRQ_ENABLE;   	/*!<  (@ 0x28) BLE_IRQ_ENABLE register                                         */
	_IO uint32_t  BLE_IRQ_STATUS;   	/*!<  (@ 0x2C) BLE_IRQ_STATUS register                                         */
	_IO uint32_t  RESERVED1[12];
	_IO uint32_t  VP_CPU_CMD_BUS;   	/*!<  (@ 0x60) VP_CPU_CMD_BUS register                                         */
	_IO uint32_t  VP_CPU_SEMA_BUS;    	/*!<  (@ 0x64) VP_CPU_SEMA_BUS register                                        */
	_IO uint32_t  VP_CPU_IRQ_ENABLE;    /*!<  (@ 0x68) VP_CPU_IRQ_ENABLE register                                      */
	_IO uint32_t  VP_CPU_IRQ_STATUS;    /*!<  (@ 0x6C) VP_CPU_IRQ_STATUS register                                      */
	_IO uint32_t  RESERVED2[36];
	_IO uint32_t  AA0_DIG_USR;          /*!<  (@ 0x100+0x00) AA0_DIG_USR register                                      */
	_IO uint32_t  AA1_DIG_USR;          /*!<  (@ 0x100+0x04) AA1_DIG_USR register                                      */
	_IO uint32_t  AA2_DIG_USR;          /*!<  (@ 0x100+0x08) AA2_DIG_USR register                                      */
	_IO uint32_t  AA3_DIG_USR;          /*!<  (@ 0x100+0x0C) AA3_DIG_USR register                                      */
	_IO uint32_t  DEM_MOD_DIG_USR;	    /*!<  (@ 0x100+0x10) DEM_MOD_DIG_USR register                                  */
	_IO uint32_t  RADIO_FSM_USR;   		/*!<  (@ 0x100+0x14) RADIO_FSM_USR register                                    */
	_IO uint32_t  PHYCTRL_DIG_USR; 		/*!<  (@ 0x100+0x18) PHYCTRL_DIG_USR register                                  */
	_IO uint32_t  RESERVED3[10];
	_IO uint32_t  AFC0_DIG_ENG;  		/*!<  (@ 0x100+0x44) AFC0_DIG_ENG register                                     */
	_IO uint32_t  AFC1_DIG_ENG;  		/*!<  (@ 0x100+0x48) AFC1_DIG_ENG register                                     */
	_IO uint32_t  AFC2_DIG_ENG;   		/*!<  (@ 0x100+0x4C) AFC2_DIG_ENG register                                     */
	_IO uint32_t  AFC3_DIG_ENG;   		/*!<  (@ 0x100+0x50) AFC3_DIG_ENG register                                     */
	_IO uint32_t  CR0_DIG_ENG;          /*!<  (@ 0x100+0x54) CR0_DIG_ENG register                                      */
	_IO uint32_t  RESERVED4[4];
	_IO uint32_t  CR0_LR;           	/*!<  (@ 0x100+0x68) CR0_LR register                                           */
	_IO uint32_t  VIT_CONF_DIG_ENG;   	/*!<  (@ 0x100+0x6C) VIT_CONF_DIG_ENG register                                 */
	_IO uint32_t  RESERVED5[5];
	_IO uint32_t  LR_PD_THR_DIG_ENG;    /*!<  (@ 0x100+0x84) LR_PD_THR_DIG_ENG register                                */
	_IO uint32_t  LR_RSSI_THR_DIG_ENG;  /*!<  (@ 0x100+0x88) LR_RSSI_THR_DIG_ENG register                              */
	_IO uint32_t  LR_AAC_THR_DIG_ENG;   /*!<  (@ 0x100+0x8C) LR_AAC_THR_DIG_ENG register                               */
	_IO uint32_t  RESERVED6[19];
	_IO uint32_t  DTB0_DIG_ENG;   		/*!<  (@ 0x100+0xDC) DTB0_DIG_ENG register                                     */
	_IO uint32_t  RESERVED7[4];
	_IO uint32_t  DTB5_DIG_ENG;   		/*!<  (@ 0x100+0xF0) DTB5_DIG_ENG register                                     */
	_IO uint32_t  RESERVED8[16];
	_IO uint32_t  MOD0_DIG_TST;   		/*!<  (@ 0x100+0x134) MOD0_DIG_TST register                                    */
	_IO uint32_t  MOD1_DIG_TST;   		/*!<  (@ 0x100+0x138) MOD1_DIG_TST register                                    */
	_IO uint32_t  MOD2_DIG_TST;   		/*!<  (@ 0x100+0x13C) MOD2_DIG_TST register                                    */
	_IO uint32_t  MOD3_DIG_TST;   		/*!<  (@ 0x100+0x140) MOD3_DIG_TST register                                    */
	_IO uint32_t  RESERVED9;
	_IO uint32_t  RXADC_ANA_USR;    	/*!<  (@ 0x100+0x148) RXADC_ANA_USR register                                   */
	_IO uint32_t  RESERVED10[2];
	_IO uint32_t  LDO_ANA_ENG;          /*!<  (@ 0x100+0x154) LDO_ANA_ENG register                                     */
	_IO uint32_t  RESERVED11[7];
	_IO uint32_t  CBIAS0_ANA_ENG;  		/*!<  (@ 0x100+0x174) CBIAS0_ANA_ENG register                                  */
	_IO uint32_t  CBIAS1_ANA_ENG;   	/*!<  (@ 0x100+0x178) CBIAS1_ANA_ENG register                                  */
	_IO uint32_t  CBIAS_ANA_TEST;   	/*!<  (@ 0x100+0x17C) CBIAS_ANA_TEST register                                  */
	_IO uint32_t  SYNTHCAL0_DIG_OUT;    /*!<  (@ 0x100+0x180) SYNTHCAL0_DIG_OUT register                               */
	_IO uint32_t  SYNTHCAL1_DIG_OUT;    /*!<  (@ 0x100+0x184) SYNTHCAL1_DIG_OUT register                               */
	_IO uint32_t  SYNTHCAL2_DIG_OUT;    /*!<  (@ 0x100+0x188) SYNTHCAL2_DIG_OUT register                               */
	_IO uint32_t  SYNTHCAL3_DIG_OUT;    /*!<  (@ 0x100+0x18C) SYNTHCAL3_DIG_OUT register                               */
	_IO uint32_t  SYNTHCAL4_DIG_OUT;    /*!<  (@ 0x100+0x190) SYNTHCAL4_DIG_OUT register                               */
	_IO uint32_t  SYNTHCAL5_DIG_OUT;    /*!<  (@ 0x100+0x194) SYNTHCAL5_DIG_OUT register                               */
	_IO uint32_t  FSM_STATUS_DIG_OUT;   /*!<  (@ 0x100+0x198) FSM_STATUS_DIG_OUT register                              */
	_IO uint32_t  IRQ_STATUS_DIG_OUT;   /*!<  (@ 0x100+0x19C) IRQ_STATUS_DIG_OUT register                              */
	_IO uint32_t  RESERVED12;
	_IO uint32_t  RSSI0_DIG_OUT;  	 	/*!<  (@ 0x100+0x1A4) RSSI0_DIG_OUT register                                   */
	_IO uint32_t  RSSI1_DIG_OUT;   		/*!<  (@ 0x100+0x1A8) RSSI1_DIG_OUT register                                   */
	_IO uint32_t  AGC_DIG_OUT;          /*!<  (@ 0x100+0x1AC) AGC_DIG_OUT register                                     */
	_IO uint32_t  DEMOD_DIG_OUT;   		/*!<  (@ 0x100+0x1B0) DEMOD_DIG_OUT register                                   */
	_IO uint32_t  AGC0_ANA_TST;   		/*!<  (@ 0x100+0x1B4) AGC0_ANA_TST register                                    */
	_IO uint32_t  AGC1_ANA_TST;   		/*!<  (@ 0x100+0x1B8) AGC1_ANA_TST register                                    */
	_IO uint32_t  AGC2_ANA_TST;   		/*!<  (@ 0x100+0x1BC) AGC2_ANA_TST register                                    */
	_IO uint32_t  AGC0_DIG_ENG;   		/*!<  (@ 0x100+0x1C0) AGC0_DIG_ENG register                                    */
	_IO uint32_t  AGC1_DIG_ENG;   		/*!<  (@ 0x100+0x1C4) AGC1_DIG_ENG register                                    */
	_IO uint32_t  AGC2_DIG_ENG;   		/*!<  (@ 0x100+0x1C8) AGC2_DIG_ENG register                                    */
	_IO uint32_t  AGC3_DIG_ENG;   		/*!<  (@ 0x100+0x1CC) AGC3_DIG_ENG register                                    */
	_IO uint32_t  AGC4_DIG_ENG;   		/*!<  (@ 0x100+0x1D0) AGC4_DIG_ENG register                                    */
	_IO uint32_t  AGC5_DIG_ENG;   		/*!<  (@ 0x100+0x1D4) AGC5_DIG_ENG register                                    */
	_IO uint32_t  AGC6_DIG_ENG;   		/*!<  (@ 0x100+0x1D8) AGC6_DIG_ENG register                                    */
	_IO uint32_t  AGC7_DIG_ENG;   		/*!<  (@ 0x100+0x1DC) AGC7_DIG_ENG register                                    */
	_IO uint32_t  AGC8_DIG_ENG;   		/*!<  (@ 0x100+0x1E0) AGC8_DIG_ENG register                                    */
	_IO uint32_t  AGC9_DIG_ENG;   		/*!<  (@ 0x100+0x1E4) AGC9_DIG_ENG register                                    */
	_IO uint32_t  AGC10_DIG_ENG;    	/*!<  (@ 0x100+0x1E8) AGC10_DIG_ENG register                                   */
	_IO uint32_t  AGC11_DIG_ENG;    	/*!<  (@ 0x100+0x1EC) AGC11_DIG_ENG register                                   */
	_IO uint32_t  AGC12_DIG_ENG;    	/*!<  (@ 0x100+0x1F0) AGC12_DIG_ENG register                                   */
	_IO uint32_t  AGC13_DIG_ENG;    	/*!<  (@ 0x100+0x1F4) AGC13_DIG_ENG register                                   */
	_IO uint32_t  AGC14_DIG_ENG;    	/*!<  (@ 0x100+0x1F8) AGC14_DIG_ENG register                                   */
	_IO uint32_t  AGC15_DIG_ENG;    	/*!<  (@ 0x100+0x1FC) AGC15_DIG_ENG register                                   */
	_IO uint32_t  AGC16_DIG_ENG;    	/*!<  (@ 0x100+0x200) AGC16_DIG_ENG register                                   */
	_IO uint32_t  AGC17_DIG_ENG;    	/*!<  (@ 0x100+0x204) AGC17_DIG_ENG register                                   */
	_IO uint32_t  AGC18_DIG_ENG;    	/*!<  (@ 0x100+0x208) AGC18_DIG_ENG register                                   */
	_IO uint32_t  AGC19_DIG_ENG;    	/*!<  (@ 0x100+0x20C) AGC19_DIG_ENG register                                   */
	_IO uint32_t  AGC20_DIG_ENG;    	/*!<  (@ 0x100+0x210) AGC20_DIG_ENG register                                   */
	_IO uint32_t  RESERVED13[4];
	_IO uint32_t  RXADC_HW_TRIM_OUT;    /*!<  (@ 0x100+0x224) RXADC_HW_TRIM_OUT register                               */
	_IO uint32_t  CBIAS0_HW_TRIM_OUT;   /*!<  (@ 0x100+0x228) CBIAS0_HW_TRIM_OUT register                              */
	_IO uint32_t  CBIAS1_HW_TRIM_OUT;   /*!<  (@ 0x100+0x22C) CBIAS1_HW_TRIM_OUT register                              */
	_IO uint32_t  AGC_HW_TRIM_OUT;      /*!<  (@ 0x100+0x230) AGC_HW_TRIM_OUT register                                 */
	_IO uint32_t  RESERVED14;
} RRM_t;                                /*!< Size = 824 (0x338)                                                        */

#endif //STM32WB07_PERIPH_H
