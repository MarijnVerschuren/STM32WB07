//
// Created by marijn on 11/1/24.
//

#include "RF/RF.h"

/*!<
 * variables
 * */  // TODO: linker address


/* hot table variables */
volatile uint32_t hot_table_radio_config_u32[12] = {0x00};
uint8_t *hot_table_radio_config = (uint8_t *)&hot_table_radio_config_u32[4];
uint8_t index;

/*!<
 * functions
 * */
void radio_init() {
	/* reset word 1 bit 0-7 */
	RFW_state[1] &= ~0xFFU;

	/* init MSP (MCU support package) */
	radio_msp_init();
	/* configure timers */
	RADIO->TODR = 0x00U;
	//TODO: add wakeup typedef to periph.h and disable BLE wakeup timer

	/* write radio trimming values */
	RRM->CBIAS0_ANA_ENG &= ~0xFFU;						// Reset bits to write to
	RRM->CBIAS0_ANA_ENG |= (BLE_IBIAS << 0U) |
	    (BLE_IPTAT << 4U);
	RRM->CBIAS1_ANA_ENG &= ~0b111U;						// Reset bits to write to
	RRM->CBIAS1_ANA_ENG |= (BLE_VBG << 0U);

	/* radio AFC configuration (AA = Access Address) */
	RRM->CBIAS0_ANA_ENG &= ~0xFFU;						// Reset bits to write to
	RRM->AFC1_DIG_ENG  |= (AFC_DELAY_BEFORE << 4U) |
		(AFC_DELAY_AFTER << 0U);
	RRM->CBIAS0_ANA_ENG &= ~0xFFU;						// Reset bits to write to
	RRM->CR0_DIG_ENG |= (CR_GAIN_BEFORE << 4U) |
		(CR_GAIN_BEFORE << 0U);
	RRM->CR0_LR &= ~0xFFU;								// Reset bits to write to
	RRM->CR0_LR |= (CR_LR_GAIN_BEFORE << 4U) |
		(CR_LR_GAIN_AFTER << 0U);

	/* radio RSSI Threshold configuration */
	RRM->LR_RSSI_THR_DIG_ENG &= ~0xFFU;					// Reset bits to write to
	RRM->LR_RSSI_THR_DIG_ENG |= (LR_RSSI_THR << 0U);
	RRM->LR_RSSI_THR_DIG_ENG &= ~0xFFU;					// Reset bits to write to
	RRM->LR_PD_THR_DIG_ENG |= (LR_PD_THR << 0U);
	RRM->LR_AAC_THR_DIG_ENG &= ~0xFFU;					// Reset bits to write to
	RRM->LR_AAC_THR_DIG_ENG |= (LR_AAC_THR << 0U);

	/* enable Viterbi */
	RRM->VIT_CONF_DIG_ENG |= (0x01U << 0U);

	/* set init delay */
	RFW_state[1] &= ~0xFFFFFF00UL;						// Reset bits to write to
	RFW_state[1] |= (INITDELAY_WAKEUP << 8U) |
		(INITDELAY_TIMER12_CAL << 16U) |
		(INITDELAY_TIMER2_NOCAL << 24U
	);

	/* set init_radio_delay */
	RFW_state[2] = (DELAYCHK_RECEIVE_CAL << 16U) |
		(DELAYCHK_RECEIVE_NOCAL << 24U) |
		(DELAYCHK_TRANSMIT_CAL << 0U) |
		(DELAYCHK_TRANSMIT_NOCAL << 8U
	);

	/* set Tx delay & duration */
	RFW_state[3] &= ~0x3FFFFFFFUL;					// Reset bits to write to
	RFW_state[3] |= (CONFIG_END_DURATION << 0U) |
		(CHECK_TXDATAREADY << 8U) |
		(TXDELAY_START << 16U) |
		(TXDELAY_END << 24U
	);

	/* Timeout for TX ready signal from the radio FSM after the 2nd init phase
	*  has expired
	*/
	RFW_state[4] &= ~0x000000FFUL;					// Reset bits to write to
	RFW_state[4] |= (TXREADY_TIMEOUT << 0U);


	RFW_state[5] |= (ChkFlagAutoclearEn << 2U) | (NoActiveLErrorInterruptEn << 23U) | (TxRxSkipInterruptEn << 29U);

	/* config hot table */
	radio_conf_hot_table();
	RFW_state[0] = hot_table_radio_config_u32[0];
	/* Reload radio config pointer */
	*(uint32_t *)(RRM_BASE + 0x10U) = 0x01U;
	RFW_state[5] |= (Active2ErrorInterruptEn << 30U);

	/*Clear all interrupts of the BLUE Controller*/
	uint32_t int_val_tmp = RADIO->ISR[1];
	RADIO->ISR[1] = int_val_tmp;




	WAKEUP->WAKEUP_BLE_IRQ_STATUS |= 0b1U;
	WAKEUP->WAKEUP_BLE_IRQ_ENABLE |= 0b1U;


	/* If the device is configured with
	 System clock = 64 MHz and BLE clock = 16 MHz
	 a register read is necessary to end fine
	 the clear interrupt register operation,
	 due the AHB down converter latency */
	int_val_tmp = RADIO->ISR[1];
}

void radio_reset(void) {
	RRM->CBIAS0_ANA_ENG = 0x00000078UL;
	RRM->CBIAS1_ANA_ENG = 0x00000007UL;
	RRM->AFC1_DIG_ENG = 0x00000044UL;
	RRM->CR0_DIG_ENG = 0x00000044UL;
	RRM->CR0_LR = 0x000000DCUL;
	RRM->VIT_CONF_DIG_ENG = 0x00U;
}

void radio_msp_init(void) {
	RCC->APB3ENR |= 0x05U;

	/* configure interrupts */
	NVIC_set_IRQ_priority(RADIO_TXRX_IRQn, 0);
	NVIC_enable_IRQ(RADIO_TXRX_IRQn);
	NVIC_set_IRQ_priority(RADIO_TXRX_SEQ_IRQn, 0);
	NVIC_enable_IRQ(RADIO_TXRX_SEQ_IRQn);
}

void radio_conf_hot_table(void) {
	index = 0;
    hot_table_radio_config[index++] = 0x01;
    hot_table_radio_config[index++] = RRM_CBIAS1_ANA_ENG;
    hot_table_radio_config[index++] = RRM->CBIAS1_ANA_ENG;
    hot_table_radio_config[index++] = 0x01;
    hot_table_radio_config[index++] = RRM_CBIAS0_ANA_ENG;
    hot_table_radio_config[index++] = RRM->CBIAS0_ANA_ENG;

    hot_table_radio_config[index++] = 0x01;
    hot_table_radio_config[index++] = RRM_RXADC_ANA_USR;
    hot_table_radio_config[index++] = RRM->RXADC_ANA_USR;

    hot_table_radio_config[index++] = 0x01;
    hot_table_radio_config[index++] = RRM_AFC1_DIG_ENG;
    hot_table_radio_config[index++] = RRM->AFC1_DIG_ENG;
    hot_table_radio_config[index++] = 0x01;
    hot_table_radio_config[index++] = RRM_CR0_DIG_ENG;
    hot_table_radio_config[index++] = RRM->CR0_DIG_ENG;
    hot_table_radio_config[index++] = 0x01;
    hot_table_radio_config[index++] = RRM_CR0_LR;
    hot_table_radio_config[index++] = RRM->CR0_LR;
    hot_table_radio_config[index++] = 0x01;
    hot_table_radio_config[index++] = RRM_LR_RSSI_THR_DIG_ENG;
    hot_table_radio_config[index++] = RRM->LR_RSSI_THR_DIG_ENG;
    hot_table_radio_config[index++] = 0x01;
    hot_table_radio_config[index++] = RRM_LR_PD_THR_DIG_ENG;
    hot_table_radio_config[index++] = RRM->LR_PD_THR_DIG_ENG;
    hot_table_radio_config[index++] = 0x01;
    hot_table_radio_config[index++] = RRM_LR_AAC_THR_DIG_ENG;
    hot_table_radio_config[index++] = RRM->LR_AAC_THR_DIG_ENG;
    hot_table_radio_config[index++] = 0x01;
    hot_table_radio_config[index++] = RRM_VIT_CONF_DIG_ENG;
    hot_table_radio_config[index++] = RRM->VIT_CONF_DIG_ENG;
    hot_table_radio_config[index++] = 0x00;

    hot_table_radio_config_u32[0] = (uint32_t)(&hot_table_radio_config_u32[4]); /* Point to Port 0 command list 1 executed when Wakeup timer triggers */
    hot_table_radio_config_u32[1] = (uint32_t)(&hot_table_radio_config_u32[4]); /* Point to Port 0 command list 2 executed when Timer1 triggers       */
    hot_table_radio_config_u32[2] = (uint32_t)(&hot_table_radio_config_u32[3]); /* Point to Port 0 command list 3 executed when Timer2 triggers       */
    hot_table_radio_config_u32[3] = 0x00000000;                /* Null command */
}
