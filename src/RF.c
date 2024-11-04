//
// Created by marijn on 11/1/24.
//

#include "RF.h"


/*!<
 * variables
 * */  // TODO: linker address
extern struct {
    // GLOBAL STATEMACHINE page 594 in software.pdf
    // word 0
	_IO	uint32_t	CFG_PTR;
	// word 1
	_IO	uint32_t	CONN_NUM	    : 7;
    _IO	uint32_t    ACTIVE          : 1;
    _IO	uint32_t    WAKE_INIT_DELAY : 8;
    _IO	uint32_t    TIM_12_DEL_CAL  : 8;
    _IO	uint32_t    TIM_12_DEL_NOCAL: 8;
    // word 2
    _IO	uint32_t    TX_CAL_DEL      : 8;
    _IO	uint32_t    TX_NOCAL_DEL    : 8;
    _IO	uint32_t    RX_CAL_DEL      : 8;
    _IO	uint32_t    RX_NOCAL_DEL    : 8;
    // word 3
    _IO	uint32_t    CFG_END_DUR     : 8;
    _IO	uint32_t    TX_RDY_CHK      : 8;
    _IO	uint32_t    TX_DEL_START    : 8;
    _IO	uint32_t    TX_DEL_END      : 6;
    _IO	uint32_t    TIME_CAP_SEL    : 1;
    _IO	uint32_t    TIME_CAP        : 1;
    // word 4
    _IO	uint32_t    TX_RDY_TO       : 8;
    _IO	uint32_t    RCV_TO          :20;
    	uint32_t    _31_28          : 4;
    // word 5
    _IO	uint32_t    AUTO_TXRX_SKIP  : 1;
    	uint32_t    _1              : 1;
    _IO	uint32_t    AUTO_CLR_BIT_EN : 1;
    	uint32_t    _7_3            : 5;
    /* error interrupts */
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

    // STATEMACHINE RAM table page 599 in software.pdf
    // word 0
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
    // word 1
    _IO uint32_t    TX_POINT;
    // word 2
    _IO uint32_t    RCV_POINT;
    // word 3
    _IO uint32_t    TX_POINT_PREV;
    // word 4
    _IO uint32_t    RCV_POINT_PREV;
    // word 5
    _IO uint32_t    RCV_POINT_NEXT;
    // word 6
    _IO uint32_t    PCNT_TX_31_0;
    // word 7
    _IO uint32_t    PCNT_TX_39_32  : 8;
    _IO uint32_t    PCNT_RCV_23_0  : 24;
    // word 8
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
	// word 9
	_IO uint32_t	ACC_ADDR;
	// word A
	_IO uint32_t	CRC_INIT		: 24;
	_IO uint32_t	MAX_REC_LEN		: 8;
	// word B
	_IO uint32_t	PA_PWR			: 5;
		uint32_t	_s6				: 3;
	_IO uint32_t	HOP_INC			: 6;
		uint32_t	_s7				: 2;
	_IO uint32_t	USD_CHLFLG_15_0 : 16;
	// word C
	_IO uint32_t	USD_CHLFLG_36_16: 22;
		uint32_t	_s8				: 10;
	// word D
	_IO uint32_t	CON_E_CNTR		: 16;
	_IO uint32_t	PA_E_CNTR		: 16;
	// word E
	_IO uint32_t	ENC_IV_31_0;
	// word F
	_IO uint32_t	ENC_IV_63_32;
	// word 10
	_IO uint32_t	ENC_KEY_31_0;
	// word 11
	_IO uint32_t	ENC_KEY_63_32;
	// word 12
	_IO uint32_t	ENC_KEY_95_64;
	// word 13
	_IO uint32_t	ENC_KEY_127_96;

	// TxRxPack RAM table page ? in software.pdf
	//word 0
	_IO uint32_t	NXT_PTR;
	// word 1
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
	// word 2
	_IO uint32_t	DAT_PTR;
	// word 3
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
	// word 4
		uint32_t	_t5;
} RF_state;
// TODO: linker address define
#define RFW_state ((uint32_t*)&RF_state)



/*!<
 * functions
 * */
void radio_init() {
	/* init MSP (MCU support package) */
	radio_msp_init();
	/* configure timers */
	RADIO->TODR = 0x00U;
	//TODO: add wakeup typedef to periph.h and disable BLE wakeup timer

	/* write radio trimming values */
	RRM->CBIAS0_ANA_ENG |= (BLE_IBIAS << 0U) |
	    (BLE_IPTAT << 4U);
	RRM->CBIAS1_ANA_ENG |= (BLE_VBG << 0U);

	/* radio AFC configuration (AA = Access Address) */
	RRM->AFC1_DIG_ENG  |= (AFC_DELAY_BEFORE << 4U) |
		(AFC_DELAY_AFTER << 0U);
	RRM->CR0_DIG_ENG |= (CR_GAIN_BEFORE << 4U) |
		(CR_GAIN_BEFORE << 0U);
	RRM->CR0_LR |= (CR_LR_GAIN_BEFORE << 4U) |
		(CR_LR_GAIN_AFTER << 0U);

	/* radio RSSI Threshold configuration */
	RRM->LR_RSSI_THR_DIG_ENG |= (LR_RSSI_THR << 0U);
	RRM->LR_PD_THR_DIG_ENG |= (LR_PD_THR << 0U);
	RRM->LR_AAC_THR_DIG_ENG |= (LR_AAC_THR << 0U);\

	/* enable Viterbi */
	RRM->VIT_CONF_DIG_ENG |= (0x01U << 0U);

}

void radio_msp_init(void) {
	RCC->APB3ENR |= 0x05U;

	/* configure interrupts */
	NVIC_set_IRQ_priority(RADIO_TXRX_IRQn, 0);
	NVIC_enable_IRQ(RADIO_TXRX_IRQn);
	NVIC_set_IRQ_priority(RADIO_TXRX_SEQ_IRQn, 0);
	NVIC_enable_IRQ(RADIO_TXRX_SEQ_IRQn);
}

void radio_reset(void) {
	RRM->CBIAS0_ANA_ENG = 0x00000078UL;
	RRM->CBIAS1_ANA_ENG = 0x00000007UL;
	RRM->AFC1_DIG_ENG = 0x00000044UL;
	RRM->CR0_DIG_ENG = 0x00000044UL;
	RRM->CR0_LR = 0x000000DCUL;
	RRM->VIT_CONF_DIG_ENG = 0x00U;
}