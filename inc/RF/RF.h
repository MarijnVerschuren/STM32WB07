//
// Created by marijn on 11/1/24.
//
#ifndef STM32WB07_RF_H
#define STM32WB07_RF_H

#include "periph.h"
#include "NVIC.h"
#include "WAKEUP.h"

/* RRM Config */
#define BLE_IBIAS                   (0x08)
#define BLE_IPTAT                   (0x07)
#define BLE_VBG                     (0x08)

/* AFC Configuration */
#define AFC_DELAY_BEFORE            (0x05)
#define AFC_DELAY_AFTER             (0x05)
#define CR_GAIN_BEFORE              (0x06)
#define CR_GAIN_AFTER               (0x06)
#define CR_LR_GAIN_BEFORE           (0x05)
#define CR_LR_GAIN_AFTER            (0x05)
#define LR_RSSI_THR                 (0x1D)
#define LR_PD_THR                   (0x59)
#define LR_AAC_THR                  (0x32)

/* initDelay */
#define INITDELAY_WAKEUP            (0x40U)
#define INITDELAY_TIMER12_CAL       (0x3FU)
#define INITDELAY_TIMER2_NOCAL      (0x9U)

/* Init_radio_delay */
#define DELAYCHK_TRANSMIT_CAL       (0x76U)
#define DELAYCHK_TRANSMIT_NOCAL     (0x3AU)
#define DELAYCHK_RECEIVE_CAL        (0x74U)
#define DELAYCHK_RECEIVE_NOCAL      (0x38U)

/* Tx delay */
#define TXDELAY_START               (0x10U)
#define TXDELAY_END                 (0x10U)
#define TXREADY_TIMEOUT             (0x5U)
#define CONFIG_END_DURATION         (0x14U)
#define CHECK_TXDATAREADY           (0x5U)

/* enable interrupts */
#define ChkFlagAutoclearEn          (0b1U)
#define NoActiveLErrorInterruptEn   (0b1U)
#define TxRxSkipInterruptEn         (0b1U)
#define Active2ErrorInterruptEn     (0b1U)

/* RRM register address for the hot table */
#define RRM_CBIAS1_ANA_ENG          (0x5E)
#define RRM_CBIAS0_ANA_ENG          (0x5D)
#define RRM_RXADC_ANA_USR           (0x52)
#define RRM_AFC1_DIG_ENG            (0x12)
#define RRM_CR0_DIG_ENG             (0x15)
#define RRM_CR0_LR                  (0x1A)
#define RRM_LR_RSSI_THR_DIG_ENG     (0x22)
#define RRM_LR_PD_THR_DIG_ENG       (0x21)
#define RRM_LR_AAC_THR_DIG_ENG      (0x23)
#define RRM_VIT_CONF_DIG_ENG        (0x1B)
#define RRM_ANTSW_DIG0_USR          (0x90)
#define RRM_ANTSW_DIG1_USR          (0x91)

/*!<
 * types
 * */
typedef enum {
	RF_ENABLE =				0b1U << 0U,
	RF_CLK_DIV =			0b1U << 1U,
} RF_FLAG_t;

typedef struct {
    // GLOBAL STATEMACHINE page 594 in software.pdf
    // word 0 INDEX 0
	_IO	uint32_t	CFG_PTR;
	// word 1 INDEX 1
	_IO	uint32_t	CONN_NUM	    : 7;
    _IO	uint32_t    ACTIVE          : 1;
    _IO	uint32_t    WAKE_INIT_DELAY : 8;
    _IO	uint32_t    TIM_12_DEL_CAL  : 8;
    _IO	uint32_t    TIM_12_DEL_NOCAL: 8;
    // word 2 INDEX 2
    _IO	uint32_t    TX_CAL_DEL      : 8;
    _IO	uint32_t    TX_NOCAL_DEL    : 8;
    _IO	uint32_t    RX_CAL_DEL      : 8;
    _IO	uint32_t    RX_NOCAL_DEL    : 8;
    // word 3 INDEX 3
    _IO	uint32_t    CFG_END_DUR     : 8;
    _IO	uint32_t    TX_RDY_CHK      : 8;
    _IO	uint32_t    TX_DEL_START    : 8;
    _IO	uint32_t    TX_DEL_END      : 6;
    _IO	uint32_t    TIME_CAP_SEL    : 1;
    _IO	uint32_t    TIME_CAP        : 1;
    // word 4 INDEX 4
    _IO	uint32_t    TX_RDY_TO       : 8;
    _IO	uint32_t    RCV_TO          :20;
    	uint32_t    _31_28          : 4;
    // word 5 INDEX 5
    _IO	uint32_t    AUTO_TXRX_SKIP  : 1;
    	uint32_t    _1              : 1;
    _IO	uint32_t    AUTO_CLR_BIT_EN : 1;
    	uint32_t    _7_3            : 5;
    /* interrupts */
    _IO	uint32_t    SEQ_ERR         : 5;
    	uint32_t    _19_13          : 7;
    _IO	uint32_t    ADD_PTR_ERR     : 1;
    _IO	uint32_t    TBL_RDY_ERR     : 1;
    _IO	uint32_t    TX_DATA_ERR     : 1;
    _IO	uint32_t    ACT_LBIT_ERR    : 1;
    	uint32_t    _24             : 1;
    _IO	uint32_t    RX_DATA_LEN_ERR : 1;
    _IO	uint32_t    SEMA_TO_ERR     : 1;
    	uint32_t    _27             : 1;
    _IO	uint32_t    EOT_SEQ_INT     : 1;
    _IO	uint32_t    TXRX_SKIP_INT   : 1;
    _IO	uint32_t    ACT_2_ERR       : 1;
    _IO	uint32_t    CFG_ERR         : 1;
	// word 6 INDEX 6
		uint32_t	_				:32;


    // STATEMACHINE RAM table page 599 in software.pdf
    // word 0 INDEX 7
    _IO uint32_t    UCHAN           : 6;
        uint32_t    _s1             : 1;
    _IO uint32_t    TX_MODE         : 1;
    _IO uint32_t    REMAP_CHAN      : 6;
    _IO uint32_t    SN              : 1;
    _IO uint32_t    NESN            : 1;
        uint32_t    _s2             : 4;
    _IO uint32_t    BUF_FULL        : 1;
    _IO uint32_t    ENCRYPTON       : 1;
    _IO uint32_t    TXENC           : 1;
    _IO uint32_t    RCV_ENC         : 1;
    _IO uint32_t    TX_PHY          : 3;
        uint32_t    _s3             : 1;
    _IO uint32_t    RX_PHY          : 3;
        uint32_t    _s4             : 1;
    // word 1 INDEX 8
    _IO uint32_t    TX_POINT;
    // word 2 INDEX 9
    _IO uint32_t    RCV_POINT;
    // word 3 INDEX 10
    _IO uint32_t    TX_POINT_PREV;
    // word 4 INDEX 11
    _IO uint32_t    RCV_POINT_PREV;
    // word 5 INDEX 12
    _IO uint32_t    RCV_POINT_NEXT;
    // word 6 INDEX 13
    _IO uint32_t    PCNT_TX_31_0;
    // word 7 INDEX 14
    _IO uint32_t    PCNT_TX_39_32  : 8;
    _IO uint32_t    PCNT_RCV_23_0  : 24;
    // word 8 INDEX 15
    _IO uint32_t    PCNT_RCV_39_24  : 16;
    _IO uint32_t    PA_REP          : 4;
    _IO uint32_t    EN_PA_REP       : 1;
    _IO uint32_t    DIS_PA_REP      : 1;
        uint32_t    _s5             : 1;
    _IO uint32_t    RX_MIC_DB       : 1;
    _IO uint32_t    INT_TX_ERR      : 5;
    _IO uint32_t    INT_ENC_ERR     : 1;
    _IO uint32_t    INT_RX_OERR     : 1;
    _IO uint32_t    RX_DB_CRC       : 1;
	// word 9 INDEX 16
	_IO uint32_t	ACC_ADDR;
	// word A INDEX 17
	_IO uint32_t	CRC_INIT		: 24;
	_IO uint32_t	MAX_REC_LEN		: 8;
	// word B INDEX 18
	_IO uint32_t	PA_PWR			: 5;
		uint32_t	_s6				: 3;
	_IO uint32_t	HOP_INC			: 6;
		uint32_t	_s7				: 2;
	_IO uint32_t	USD_CHLFLG_15_0 : 16;
	// word C INDEX 19
	_IO uint32_t	USD_CHLFLG_36_16: 22;
		uint32_t	_s8				: 10;
	// word D INDEX 20
	_IO uint32_t	CON_E_CNTR		: 16;
	_IO uint32_t	PA_E_CNTR		: 16;
	// word E INDEX 21
	_IO uint32_t	ENC_IV_31_0;
	// word F INDEX 22
	_IO uint32_t	ENC_IV_63_32;
	// word 10 INDEX 23
	_IO uint32_t	ENC_KEY_31_0;
	// word 11 INDEX 24
	_IO uint32_t	ENC_KEY_63_32;
	// word 12 INDEX 25
	_IO uint32_t	ENC_KEY_95_64;
	// word 13 INDEX 26
	_IO uint32_t	ENC_KEY_127_96;

	// TxRxPack RAM table page ? in software.pdf
	//word 0 INDEX 27
	_IO uint32_t	NXT_PTR;
	// word 1 INDEX 28
	_IO uint32_t	CAL_REQ			: 1;
	_IO uint32_t	CHL_GO_2_SEEL	: 1;
	_IO uint32_t	KP_SEMA_REQ		: 1;
		uint32_t	_t1				: 1;
	_IO uint32_t	CRC_INIT_SEL	: 1;
	_IO uint32_t	ADV				: 1;
	_IO uint32_t	SN_EN			: 1;
	_IO uint32_t	INC_CHAN		: 1;
	_IO uint32_t	NXT_TX_MODE		: 1;
	_IO uint32_t	A_TBL_RDY		: 1;
	_IO uint32_t	TX_DAT_RDY		: 1;
		uint32_t	_t2				: 1;
	_IO uint32_t	DIS_WHNG		: 1;
		uint32_t	_t3				: 19;
	// word 2 INDEX 29
	_IO uint32_t	DAT_PTR;
	// word 3 INDEX 30
	_IO uint32_t	TIM_2			: 20;
	_IO uint32_t	TIM_2_EN		: 1;
	_IO uint32_t	_t4				: 1;
	_IO uint32_t	TRIG_RCV		: 1;
	_IO uint32_t	TRIG_DONE		: 1;
	_IO uint32_t	INT_TX_OK		: 1;
	_IO uint32_t	INT_DONE		: 1;
	_IO uint32_t	INT_RCV_TOUT	: 1;
	_IO uint32_t	INT_RCV_NMD		: 1;
	_IO uint32_t	INT_TIM_CAPT	: 1;
	_IO uint32_t	INT_RCV_CRC_ERR : 1;
	_IO uint32_t	INT_RCV_OK		: 1;
	// word 4 INDEX 31
		uint32_t	_t5;
} RF_state_t;

// TODO: linker address define
extern RF_state_t RF_state;
#define RFW_state ((uint32_t*)&RF_state)

/*!<
 * functions
 * */
void radio_init(void);
void radio_reset(void);
void radio_msp_init(void);
void radio_conf_hot_table(void);

#endif //STM32WB07_RF_H

