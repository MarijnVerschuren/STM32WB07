//
// Created by marijn on 11/1/24.
//

#include "RF.h"


/*!<
 * variables
 * */  // TODO: linker address
extern struct {
	_IO	uint32_t	CFG_PTR;
	_IO	uint32_t	CONN_NUM	: 7;

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