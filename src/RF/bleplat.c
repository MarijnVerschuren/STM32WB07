//
// Created by marijn on 11/11/24.
//
#include "RF/RF.h"
#include "RF/RF_TIMER.h"
#include "RF/BLE.h"
#include "RF/bleplat_cntr.h"


tBleStatus P2P_SERVER_UpdateValue(P2P_SERVER_CharOpcode_t CharOpcode, P2P_SERVER_Data_t *pData) {
	tBleStatus ret = BLE_STATUS_SUCCESS;

	switch(CharOpcode) {
		case P2P_SERVER_LED_C:
			memcpy(led_c_val_buffer, pData->p_Payload, MIN(pData->Length, sizeof(led_c_val_buffer)));
			break;
		default:
			break;
	}
	return ret;
}


extern void Osal_MemCpy(void *dest, const void *src, unsigned int size);
void BLEPLAT_MemCpy(void *Dest, const void *Src, unsigned int Size) {
	Osal_MemCpy(Dest, Src, Size);
}

void BLEPLAT_MemSet(void *Ptr, int Value, unsigned int Size) {
	memset(Ptr, Value, Size);
}

int BLEPLAT_MemCmp(void *S1, void *S2, unsigned int Size) {
	return memcmp(S1, S2, Size);
}

/**
 * @brief Get Device ID, Version and Revision numbers
 */
//void BLEPLAT_GetPartInfo(uint8_t *pDeviceId, uint8_t *pMajorCut, uint8_t *pMinorCut) {
//	PartInfoType partInfo;
//	GetPartInfo(&partInfo);
//	*pDeviceId  = partInfo.die_id;
//	*pMajorCut = partInfo.die_major;
//	*pMinorCut = partInfo.die_cut;
//}
//
//BLEPLAT_PkaStatusTypeDef BLEPLAT_PkaStartP256Key(const uint32_t *private_key, BLEPLAT_PkaFuncCb funcCb) {
//	return (BLEPLAT_PkaStatusTypeDef)PKAMGR_StartP256PublicKeyGeneration(private_key, (PKAMGR_funcCB)funcCb);
//}
//
//BLEPLAT_PkaStatusTypeDef BLEPLAT_PkaStartDHkey(uint32_t* private_key,
//											   uint32_t* public_key,
//											   BLEPLAT_PkaFuncCb funcCb) {
//	return (BLEPLAT_PkaStatusTypeDef)HW_PKA_StartP256DHkeyGeneration(private_key, public_key, (PKAMGR_funcCB)funcCb);
//}
//
//void BLEPLAT_AesEcbEncrypt(const uint32_t *plainTextData,
//						   const uint32_t *key,
//						   uint32_t *encryptedData) {
//	HW_AES_Encrypt(plainTextData, key, encryptedData);
//}
//
//int32_t BLEPLAT_AesCMACEncryptInit(BLEPLAT_AESCMACctxTypeDef *pAESCMACctx) {
//	return AES_CMAC_Encrypt_Init((AESCMACctx_stt*)pAESCMACctx);
//}
//
//int32_t BLEPLAT_AesCMACEncryptAppend(BLEPLAT_AESCMACctxTypeDef *pAESCMACctx,
//									 const uint8_t  *pInputBuffer,
//									 int32_t InputSize) {
//	return AES_CMAC_Encrypt_Append((AESCMACctx_stt*)pAESCMACctx, pInputBuffer, InputSize);
//}
//
//int32_t BLEPLAT_AesCMACEncryptFinish(BLEPLAT_AESCMACctxTypeDef *pAESCMACctx,
//									 uint8_t *pOutputBuffer,
//									 int32_t *pOutputSize) {
//	return AES_CMAC_Encrypt_Finish((AESCMACctx_stt*)pAESCMACctx, pOutputBuffer, pOutputSize);
//}
//
//void BLEPLAT_RngGetRandom16(uint16_t* num) {
//	HW_RNG_GetRandom16(num);
//}
//
//void BLEPLAT_RngGetRandom32(uint32_t* num) {
//	HW_RNG_GetRandom32(num);
//}


#define TX_POWER_LEVELS                (32U)
static const int8_t normal_pa_level_table[TX_POWER_LEVELS] = {
		-54, -21, -20, -19, -17, -16, -15, -14,
		-13, -12, -11, -10,  -9,  -8,  -7,  -6,
		-6,  -4,  -3,  -3,  -2,  -2,  -1,  -1,
		0,   0,   1,   2,   3,   4,   5,   6
};
static const int8_t high_power_pa_level_table[TX_POWER_LEVELS] = {
		-54, -19, -18, -17, -16, -15, -14, -13,
		-12, -11, -10,  -9,  -8,  -7,  -6,  -5,
		-4,  -3,  -3,  -2,  -1,   0,   1,   2,
		3,   8,   8,   8,   8,   8,   8,   8
};
static uint8_t high_power = 0;


uint8_t BLEPLAT_DBmToPALevel(int8_t TX_dBm) {
	uint8_t i;
	const int8_t *pa_level_table = high_power ? high_power_pa_level_table : normal_pa_level_table;
	for(i = 0; i < TX_POWER_LEVELS; i++) {
		if(pa_level_table[i] >= TX_dBm) { break; }
	}
	if (((pa_level_table[i] > TX_dBm) && (i > 1)) || (i == TX_POWER_LEVELS)) { i--; }
	return i;
}

uint8_t BLEPLAT_DBmToPALevelGe(int8_t TX_dBm) {
	const int8_t *pa_level_table = high_power ? high_power_pa_level_table : normal_pa_level_table;
	uint8_t i;
	for(i = 1; i < TX_POWER_LEVELS; i++) {
		if (pa_level_table[i] >= TX_dBm) { break; }
	}
	if(i == TX_POWER_LEVELS) { i--; }
	return i;
}

int8_t BLEPLAT_PALevelToDBm(uint8_t PA_Level) {
	const int8_t *pa_level_table = high_power ? high_power_pa_level_table : normal_pa_level_table;
	if(PA_Level < 1 || PA_Level >= TX_POWER_LEVELS) { return 127; }
	return pa_level_table[PA_Level];
}

void BLEPLAT_ReadTransmitPower(int8_t *Min_Tx_Power, int8_t *Max_Tx_Power) {
	if (high_power) {
		*Min_Tx_Power = high_power_pa_level_table[1];
		*Max_Tx_Power = high_power_pa_level_table[TX_POWER_LEVELS - 1];
	} else {
		*Min_Tx_Power = normal_pa_level_table[1];
		*Max_Tx_Power = normal_pa_level_table[TX_POWER_LEVELS - 1];
	}
}

//uint8_t BLEPLAT_GetMaxPALevel(void) {
//	return RADIO_GetMaxPALevel();
//}
//
//uint8_t BLEPLAT_GetDefaultPALevel(void) {
//	return RADIO_GetDefaultPALevel();
//}

//void BLEPLAT_SetHighPower(uint8_t enable) {
//	RADIO_SetHighPower((FunctionalState)enable);
//}

int8_t BLEPLAT_CalculateRSSI(void) {
	int32_t rssi_dbm;
	uint32_t rssi0 = RRM->RSSI0_DIG_OUT;
	uint32_t rssi1 = RRM->RSSI1_DIG_OUT;

	uint32_t rssi_int16 = ((rssi1 & 0xFF) << 8) | (rssi0 & 0xFF);
	uint32_t reg_agc = RRM->AGC_DIG_OUT;

	if ((rssi_int16 == 0U) || (reg_agc > 0xbU))
	{
		rssi_dbm = 127 ;
	}
	else
	{
		rssi_dbm = (int32_t)reg_agc * 6 - 119;//127 ;
		while (rssi_int16 > 30U)
		{
			rssi_dbm = rssi_dbm + 6 ;
			rssi_int16 = (rssi_int16 >> 1) ;
		}
		rssi_dbm = rssi_dbm + (int32_t)(uint32_t)((417U * rssi_int16 + 18080U) >> 10);
	}
	return (int8_t)rssi_dbm;
}

//int8_t BLEPLAT_UpdateAvgRSSI(int8_t avg_rssi, int8_t rssi, uint8_t rssi_filter_coeff) {
//	return RADIO_UpdateAvgRSSI(avg_rssi, rssi, rssi_filter_coeff);
//}
//
uint8_t BLEPLAT_GetDemodCI(void) {
	/* Read the CI from the demodulator register */
	uint8_t demod_ci = (RRM->DEMOD_DIG_OUT) & 0x03U;

	/* Remap to the standard compliant values */
	uint8_t std_ci = (demod_ci == 0x02U ? 0x01U : 0x00U);

	return std_ci;;
}


#define SYNTH0_ANA_ENG      (*(volatile uint32_t *)0x60001610)
#define SYNTHCAL3_ANA_TST   (*(volatile uint32_t *)0x600015A4)
#define LL_DUMMY_ACCESS_ADDRESS (0x00000000U)
#define LL_DTM_ACCESS_ADDRESS   (0x71764129U)
uint32_t SYNTH0_ANA_ENG_bak, PWR_ENGTRIM_bak;


void BLEPLAT_InitCTE(uint8_t smNo) {
	SYNTH0_ANA_ENG_bak = SYNTH0_ANA_ENG;
	PWR_ENGTRIM_bak = PWR->ENGTRIM;
	SYNTHCAL3_ANA_TST = 0;
	SYNTH0_ANA_ENG &= ~(0x07UL);
	PWR->ENGTRIM = 0x00000001;
	ble_state[smNo].ACC_ADDR = LL_DUMMY_ACCESS_ADDRESS;
}

void BLEPLAT_DeinitCTE(void) {
	PWR->ENGTRIM = PWR_ENGTRIM_bak;
	SYNTH0_ANA_ENG = SYNTH0_ANA_ENG_bak;
	SYNTHCAL3_ANA_TST = 0;
}

void BLEPLAT_CalibrateCTE(uint8_t smNo) {
	uint32_t dac_word = RRM->SYNTHCAL4_DIG_OUT & 0x0000003fUL;
	dac_word += 2;
	dac_word &= 0x0000003fUL;
	// Set SYNTHCAL3_ANA_TST
	SYNTHCAL3_ANA_TST = dac_word | 0x80;
	ble_state[smNo].ACC_ADDR = LL_DTM_ACCESS_ADDRESS;
}

//void BLEPLAT_AntIdxRemap(uint8_t AntPattLen, uint8_t *pAntRamTable, const uint8_t* pAntPatt) {
//	RADIO_AntIdxRemap(AntPattLen, pAntRamTable, pAntPatt);
//}

uint64_t BLEPLAT_GetCurrentSysTime(void) {
	uint32_t _; return _get_system_time_and_machine(&RADIO_TIMER_Context, &_);
}

//uint64_t BLEPLAT_GetFutureSysTime64(uint32_t SysTime) {
//	return HAL_RADIO_TIMER_GetFutureSysTime64(SysTime);
//}

//int BLEPLAT_StartTimer(VTIMER_HandleType *TimerHandle, uint64_t Time) {
//	return HAL_RADIO_TIMER_StartVirtualTimerSysTime((VTIMER_HandleType*)TimerHandle, Time);
//}

void BLEPLAT_StopTimer(VTIMER_HandleType *TimerHandle) {
	HAL_RADIO_TIMER_StopVirtualTimer(TimerHandle);
}

uint8_t BLEPLAT_SetRadioTimerValue(uint32_t Time, uint8_t EventType, uint8_t CalReq) {
	uint8_t retVal = 0;

	uint64_t current_time;

	RADIO_TIMER_Context.radioTimer.event_type = EventType;
	RADIO_TIMER_Context.radioTimer.cal_req = CalReq;
	RADIO_TIMER_Context.radioTimer.expiryTime = RADIO_TIMER_Context.calibrationData.last_calibration_time + (uint32_t)(Time - (uint32_t)RADIO_TIMER_Context.calibrationData.last_calibration_time);
	RADIO_TIMER_Context.radioTimer.active = 0;
	RADIO_TIMER_Context.radioTimer.intTxRx_to_be_served = 0;
	RADIO_TIMER_Context.radioTimer.pending = 1;

	uint32_t current_machine_time;
	current_time = _get_system_time_and_machine(&RADIO_TIMER_Context, &current_machine_time);

  	if (RADIO_TIMER_Context.rootNode == NULL)
 	 {
 	   _check_radio_activity(&RADIO_TIMER_Context.radioTimer, &retVal);
 	 }
 	 else
 	 {
 	   if (RADIO_TIMER_Context.rootNode->expiryTime < current_time ||
	        ((RADIO_TIMER_Context.radioTimer.expiryTime < (RADIO_TIMER_Context.rootNode->expiryTime +
 	                                                      RADIO_TIMER_Context.hostMargin)) && RADIO_TIMER_Context.rootNode->active) || !RADIO_TIMER_Context.rootNode->active)
 	   {
      /* Program the radio timer */
      _check_radio_activity(&RADIO_TIMER_Context.radioTimer, &retVal);
      if ((RADIO_TIMER_Context.radioTimer.expiryTime >= RADIO_TIMER_Context.rootNode->expiryTime)
          && RADIO_TIMER_Context.rootNode->active)
      {
        /*The radio operation is before or too close the host timeout*/
        RADIO_TIMER_Context.hostIsRadioPending = 1;
      }
    }
    else
    {
      /* If radio timer is not programmed, an emulated host timer is already programmed.
      Make sure radio errors are disabled.
      This call is not needed if radio errors are not enabled by the BLE stack. */
      _set_controller_as_host();
    }
  }


	_virtualTimeBaseEnable(1);

	return retVal;;
}

uint8_t BLEPLAT_ClearRadioTimerValue(void) {
	return RADIO_TIMER_ClearRadioTimerValue();
}

//uint64_t BLEPLAT_GetAnchorPoint(uint64_t *pCurrentSysTime) {
//	return HAL_RADIO_TIMER_GetAnchorPoint(pCurrentSysTime);
//}

//void BLEPLAT_SetRadioCloseTimeout(void) {
//	HAL_RADIO_TIMER_SetRadioCloseTimeout();
//}

//uint8_t BLEPLAT_SetRadioTimerRelativeUsValue(uint32_t RelTimeoutUs, uint8_t Tx, uint8_t PLLCal) {
//	return HAL_RADIO_TIMER_SetRadioTimerRelativeUsValue(RelTimeoutUs, Tx, PLLCal);
//}


//BLEPLAT_CNTR_ResultStatus BLEPLAT_CNTR_Init(void)
//{
//
//	return BLEPLAT_CNTR_SUCCESS;
//}
//
//BLEPLAT_CNTR_ResultStatus BLEPLAT_CNTR_Deinit(void)
//{
//
//	return BLEPLAT_CNTR_SUCCESS;
//}

#define TX_DELAY_START 16
#define TX_DELAY_END 16
#define TIMER12_INIT_DELAY_CAL 63
#define TIMER2_INIT_DELAY_NO_CAL 9
#define RADIO_FSM_RX_DELAY_CAL 90U
#define RADIO_FSM_RX_DELAY_NO_CAL 50U
#define RADIO_FSM_TX_DELAY_CAL 92U
#define RADIO_FSM_TX_DELAY_NO_CAL 52U
uint32_t BLEPLAT_CNTR_GetTimer2TimeoutForIfs(uint32_t T_Ifs, BLEPLAT_CNTR_Transaction Transaction, uint8_t Cal_Enabled) {
	uint32_t Timeout = T_Ifs;
	uint32_t Tx_Delay_Comp;
	uint32_t Init_Delay = 0;

	if(Transaction == BLEPLAT_CNTR_RxTx) {
		const int32_t Adjust_Value = 6;
		Tx_Delay_Comp = (TX_DELAY_START >> 3) + Adjust_Value;
	}
	else if(Transaction == BLEPLAT_CNTR_TxRx) {
		const int32_t Adjust_Value = 4;
		Tx_Delay_Comp = (TX_DELAY_END>>3) + Adjust_Value;
	}
	else if(Transaction == BLEPLAT_CNTR_TxTx) {
		const int32_t Adjust_Value = 2;
		Tx_Delay_Comp = ((TX_DELAY_START + TX_DELAY_END)>>3) + Adjust_Value;
	} else {
		Tx_Delay_Comp = 0;
	}

	if((Transaction == BLEPLAT_CNTR_RxTx) || (Transaction == BLEPLAT_CNTR_TxTx)) {
		if(Cal_Enabled) {
			Init_Delay = TIMER12_INIT_DELAY_CAL + RADIO_FSM_TX_DELAY_CAL;
		} else {
			Init_Delay = TIMER2_INIT_DELAY_NO_CAL + RADIO_FSM_TX_DELAY_NO_CAL;
		}
	} else if((Transaction == BLEPLAT_CNTR_TxRx) || (Transaction == BLEPLAT_CNTR_RxRx)) {
		if(Cal_Enabled) {
			Init_Delay = TIMER12_INIT_DELAY_CAL + RADIO_FSM_RX_DELAY_CAL;
		} else {
			Init_Delay = TIMER2_INIT_DELAY_NO_CAL + RADIO_FSM_RX_DELAY_NO_CAL;
		}
	}

	Timeout -= (Init_Delay + Tx_Delay_Comp);
	return Timeout;
}

//void BLEPLAT_CNTR_ClearInterrupt(uint32_t x) {
//	LL_RADIO_BlueSetInterrupt1RegRegister(x);
//}
//
//void BLEPLAT_CNTR_ClearSemareq(void) {
//	LL_RADIO_BlueSetClearSemaphoreRequest(0x1U);
//}

void BLEPLAT_CNTR_TxRxSkip(void) {
	RADIO->CR |= 0x00000001UL;
}

//uint32_t* BLEPLAT_CNTR_GetCipherTextPtr() {
//	return (uint32_t*)&BLUE->MANAESCIPHERTEXT0REG;
//}
//
//uint32_t* BLEPLAT_CNTR_GetClrTextPtr() {
//	return (uint32_t*)&BLUE->MANAESCLEARTEXT0REG;
//}
//
//uint32_t* BLEPLAT_CNTR_GetEncKeyPtr() {
//	return (uint32_t*)&BLUE->MANAESKEY0REG;
//}
//
//uint8_t BLEPLAT_CNTR_GetEncryptDoneStatus() {
//	return (uint8_t)!LL_RADIO_BlueGetManAESStatusBusy();
//}

uint8_t BLEPLAT_CNTR_GetIqsamplesMissingError(void) {
	return (uint8_t)0;
}

uint8_t BLEPLAT_CNTR_GetIqsamplesNumber(void) {
	return (uint8_t)0;
}

uint8_t BLEPLAT_CNTR_getIqsamplesReady(void) {
	return (uint8_t)0;
}

//uint8_t BLEPLAT_CNTR_GetIsrLatency() {
//	return (uint8_t)LL_RADIO_BlueGetInterrupt1Latency();
//}
//
//uint32_t BLEPLAT_CNTR_GetTimercapture() {
//	return LL_RADIO_BlueGetTimerCapture();
//}

void BLEPLAT_CNTR_GlobDisableBlue() {
	ble_globstate.ACTIVE = 0;
}

//void BLEPLAT_CNTR_GlobEnableBlue() {
//	LL_RADIO_GlobalEnableBlue();
//}
//
//void BLEPLAT_CNTR_GlobEnableIntnoactivelerrorInt() {
//	LL_RADIO_NoActiveLErrorInterrupt_Enable();
//}
//
//void BLEPLAT_CNTR_GlobEnableOverrunAct2Int() {
//	LL_RADIO_Active2ErrorInterrupt_Enable();
//}

uint8_t BLEPLAT_CNTR_GlobGetDefaultAntennaid() {
	return (uint8_t)0;
}

//uint8_t BLEPLAT_CNTR_GlobGetWakeupinitdelay() {
//	return (uint8_t) LL_RADIO_GetWakeupInitDelay();
//}

void BLEPLAT_CNTR_GlobReloadRadioConfigP() {
	*(uint32_t*)(RRM_BASE + 0x10U) = 0x01U;
}

//void BLEPLAT_CNTR_GlobSetChkflagautoclearena() {
//	LL_RADIO_ChkFlagAutoclearEnable_Enable();
//}
//
void BLEPLAT_CNTR_GlobSetDefaultAntennaid(uint8_t x) { }

//void BLEPLAT_CNTR_GlobSetInitRadioDelayTxCal(uint8_t x) {
//	LL_RADIO_SetTransmitCalDelayChk((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobSetInitRadioDelayTxNocal(uint8_t x) {
//	LL_RADIO_SetTransmitNoCalDelayChk((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobSetRadioConfigP(volatile uint32_t* x) {
//	LL_RADIO_SetRadioConfigurationAddressPointer(x[0]);
//}
//
//void BLEPLAT_CNTR_GlobSetWakeupinitdelay(uint8_t x) {
//	LL_RADIO_SetWakeupInitDelay((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteConfigEnd(uint8_t x) {
//	LL_RADIO_SetConfigurationEndDuration((uint32_t) x);
//}
//
void BLEPLAT_CNTR_GlobWritePeriodslow(uint16_t x) {}
//
//void BLEPLAT_CNTR_GlobWriteRcvdelay(uint8_t x) {
//	LL_RADIO_SetReceivedCalDelayChk((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteRcvdelay1(uint8_t x) {
//	LL_RADIO_SetReceivedNoCalDelayChk((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteSlot(uint8_t slotNo) {
//	LL_RADIO_SetCurrentConnectionMachineNumber((uint32_t) slotNo);
//}

void BLEPLAT_CNTR_GlobWriteTimer12initdelaycal(uint8_t x) {
	ble_globstate.TIM_12_DEL_CAL = x;
}

//void BLEPLAT_CNTR_GlobWriteTimer2initdelaynocal(uint8_t x) {
//	LL_RADIO_SetTimer12InitDelayNoCal((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteTxDataReadyCheck(uint8_t x) {
//	LL_RADIO_SetTxDataReadyCheck((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteTxReadyTimeout(uint8_t x) {
//	LL_RADIO_SetTransmissionReadyTimeout((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteTxdelay(uint8_t x) {
//	LL_RADIO_SetTxDelayStart((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteTxdelayEnd(uint8_t x) {
//	LL_RADIO_SetTxDelayEnd((uint32_t) x);
//}
//
//uint32_t BLEPLAT_CNTR_IntGetIntStatusAnyHwError(uint32_t x) {
//	return (uint32_t)(x & ANY_HW_ERROR_INTERRUPT_Msk);
//}
//
//uint32_t BLEPLAT_CNTR_IntGetIntStatusRxOverflowError(uint32_t x) {
//	return (uint32_t)(x & BLUE_STATUSREG_RXOVERFLOWERROR_Msk);
//}
//
//uint32_t BLEPLAT_CNTR_IntGetIntStatusBitAct2Error(uint32_t x) {
//	return (uint32_t)(x & BLUE_STATUSREG_ACTIVE2ERROR_Msk);
//}

uint32_t BLEPLAT_CNTR_IntGetIntStatusBitTimerOverrun(uint32_t x) {
	return 0;
}

uint32_t BLEPLAT_CNTR_IntGetIntStatusCrcErr(uint32_t x) {
    return (uint32_t)(x & 0x40000000UL);
}

//uint32_t BLEPLAT_CNTR_IntGetIntStatusDone(uint32_t x) {
//	return (uint32_t)(x & BLUE_STATUSREG_DONE_Msk);
//}
//
//uint32_t BLEPLAT_CNTR_IntGetIntStatusEncErr(uint32_t x) {
//    return (uint32_t)(x & BLUE_STATUSREG_ENCERROR_Msk);
//}

uint32_t BLEPLAT_CNTR_IntGetIntStatusLenErr(uint32_t x) {
    return (uint32_t)(x & 0x00040000UL);
}

//uint32_t BLEPLAT_CNTR_IntGetIntStatusNoactiveError(uint32_t x) {
//    return (uint32_t)(x & BLUE_STATUSREG_NOACTIVELERROR_Msk);
//}
uint32_t BLEPLAT_CNTR_IntGetIntStatusTxRxSkip(uint32_t x) {
    return (uint32_t)(x & 0x00200000UL);
}

//uint32_t BLEPLAT_CNTR_IntGetIntStatusTxError1(uint32_t x) {
//    return (uint32_t)(x & BLUE_STATUSREG_TXERROR_1_Msk);
//}
//
//uint32_t BLEPLAT_CNTR_IntGetIntStatusTxError3(uint32_t x) {
//    return (uint32_t)(x & BLUE_STATUSREG_TXERROR_3_Msk);
//}
//
//uint32_t BLEPLAT_CNTR_IntGetIntStatusRxOk(uint32_t x) {
//    return (uint32_t)(x & BLUE_STATUSREG_RCVOK_Msk);
//}

uint32_t BLEPLAT_CNTR_IntGetIntStatusTimeout(uint32_t x) {
    return (uint32_t)(x & 0x04000000UL);
}

//uint32_t BLEPLAT_CNTR_IntGetIntStatusTrigRcv(uint32_t x) {
//    return (uint32_t)(x & BLUE_STATUSREG_TIMECAPTURETRIG_Msk);
//}
//
uint32_t BLEPLAT_CNTR_IntGetIntStatusTxDone(uint32_t x) {
    return (uint32_t)(x & 0x00000040UL);
}

//uint32_t BLEPLAT_CNTR_IntGetIntStatusTxOk(uint32_t x) {
//    return (uint32_t)(x & BLUE_STATUSREG_TXOK_Msk);
//}
//
//void BLEPLAT_CNTR_PacketClrCrcinitSel(TXRXPACK_STATE_t* packetP) {
//	LL_RADIO_SetCRCInitializationSelector((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x0);
//}
//
void BLEPLAT_CNTR_PacketClrCteSamplingEn(TXRXPACK_STATE_t* packetP) {}

void BLEPLAT_CNTR_PacketClrIncChan(TXRXPACK_STATE_t* packetP) {
	packetP->INC_CHAN = 0;
}

//void BLEPLAT_CNTR_PacketClrPllTrig(TXRXPACK_STATE_t* packetP) {
//	LL_RADIO_SetCalibrationRequest((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x0);
//}

void BLEPLAT_CNTR_PacketDisableWhitening(TXRXPACK_STATE_t* packetP) {
	packetP->DIS_WHNG = 1;
}

//uint8_t BLEPLAT_CNTR_PacketGetCteSamplingEn(TXRXPACK_STATE_t* packetP) {
//	return (uint8_t)0;
//}

uint8_t* BLEPLAT_CNTR_PacketGetDataPtr(TXRXPACK_STATE_t* packetP) {
	return (uint8_t *)packetP->DAT_PTR;
}

void BLEPLAT_CNTR_PacketInitTo0(TXRXPACK_STATE_t* packetP) {
	memset((void*)packetP, 0, sizeof(TXRXPACK_STATE_t));
}

void BLEPLAT_CNTR_PacketSetAdvPduFormat(TXRXPACK_STATE_t* packetP) {
	packetP->ADV = 1;
}

void BLEPLAT_CNTR_PacketSetCrcinitSel(TXRXPACK_STATE_t* packetP) {
	packetP->CRC_INIT_SEL = 1;
}

void BLEPLAT_CNTR_PacketSetCteSamplingEn(TXRXPACK_STATE_t* packetP) {}

//void BLEPLAT_CNTR_PacketSetDataPduFormat(TXRXPACK_STATE_t* packetP) {
//	LL_RADIO_SetAdvertise((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x0);
//}

void BLEPLAT_CNTR_PacketSetDataPtr(TXRXPACK_STATE_t* packetP, void* dataP) {
	packetP->DAT_PTR = (uint32_t)&dataP;
}

void BLEPLAT_CNTR_PacketSetIncChan(TXRXPACK_STATE_t* packetP) {
	packetP->INC_CHAN = 1;
}

void BLEPLAT_CNTR_PacketSetIntCrcErr(TXRXPACK_STATE_t* packetP) {
	packetP->INT_RCV_CRC_ERR = 1;
}

void BLEPLAT_CNTR_PacketSetIntDone(TXRXPACK_STATE_t* packetP) {
	packetP->INT_DONE = 1;
}

void BLEPLAT_CNTR_PacketSetIntRcvOk(TXRXPACK_STATE_t* packetP) {
	packetP->INT_RCV_OK = 1;
}

void BLEPLAT_CNTR_PacketSetIntTimeout(TXRXPACK_STATE_t* packetP) {
	packetP->INT_RCV_TOUT = 1;
}

//void BLEPLAT_CNTR_PacketSetIntTrigRcv(TXRXPACK_STATE_t* packetP) {
//	LL_RADIO_SetIntTimeCapture((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1U);
//}

void BLEPLAT_CNTR_PacketSetIntTxOk(TXRXPACK_STATE_t* packetP) {
	packetP->INT_TX_OK = 1;
}

void BLEPLAT_CNTR_PacketSetKeepsemareq(TXRXPACK_STATE_t* packetP) {
	packetP->KP_SEMA_REQ = 1;
}

void BLEPLAT_CNTR_PacketSetNextPtr(TXRXPACK_STATE_t* packetP, TXRXPACK_STATE_t* packetNextP) {
	packetP->NXT_PTR = (uint32_t)packetNextP;
}

void BLEPLAT_CNTR_PacketSetNextRxMode(TXRXPACK_STATE_t* packetP) {
	packetP->NXT_TX_MODE = 0;
}

void BLEPLAT_CNTR_PacketSetNextSlot(TXRXPACK_STATE_t* packetP, uint8_t slot) {}

void BLEPLAT_CNTR_PacketSetNextTxMode(TXRXPACK_STATE_t* packetP) {
	packetP->NXT_TX_MODE = 1;
}

//void BLEPLAT_CNTR_PacketSetNsEn(TXRXPACK_STATE_t* packetP) {
//	LL_RADIO_AutomaticSnNesnHardwareMechanism_Enable((TXRXPACK_TypeDef*)packetP);
//}
//
//void BLEPLAT_CNTR_PacketSetPllTrig(TXRXPACK_STATE_t* packetP) {
//	LL_RADIO_SetCalibrationRequest((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1);
//}

void BLEPLAT_CNTR_PacketSetRxReady(TXRXPACK_STATE_t* packetP) {
	packetP->A_TBL_RDY = 1;
}

void BLEPLAT_CNTR_PacketSetTimeout(TXRXPACK_STATE_t* packetP, uint32_t x) {
	packetP->TIM_2 = x;
}

void BLEPLAT_CNTR_PacketSetTimer2Active(TXRXPACK_STATE_t* packetP) {
	packetP->TIM_2_EN = 1;
}

void BLEPLAT_CNTR_PacketSetTimerTrigDone(TXRXPACK_STATE_t* packetP) {
	packetP->TRIG_DONE = 1;
}

void BLEPLAT_CNTR_PacketSetTimerTrigRcv(TXRXPACK_STATE_t* packetP) {
	packetP->TRIG_RCV = 1;
}

void BLEPLAT_CNTR_PacketSetTxReady(TXRXPACK_STATE_t* packetP) {
	packetP->TX_DAT_RDY = 1;
	packetP->A_TBL_RDY = 1;
}
void BLEPLAT_CNTR_SetRadioConfig(uint8_t* value) {}


void BLEPLAT_CNTR_SmCteOff(uint8_t smNo) {}

void BLEPLAT_CNTR_SmCteOn(uint8_t smNo) {}


void BLEPLAT_CNTR_SmEncOff(uint8_t smNo) {
	ble_state[smNo].ENCRYPTON = 0;
	ble_state[smNo].INT_ENC_ERR = 0;
}

void BLEPLAT_CNTR_SmEncOn(uint8_t smNo) {
	ble_state[smNo].ENCRYPTON = 1;
	ble_state[smNo].INT_ENC_ERR = 1;
}

uint32_t BLEPLAT_CNTR_SmGetAccessAddr(uint8_t smNo) {
	return ble_state[smNo].ACC_ADDR;
}

void BLEPLAT_CNTR_SmGetChannelMap(uint8_t smNo, uint8_t* chanMap) {
	chanMap[0] = (uint8_t)(ble_state[smNo].USD_CHLFLG_15_0);
	chanMap[1] = (uint8_t)(ble_state[smNo].USD_CHLFLG_15_0 >> 8U);
	chanMap[2] = (uint8_t)(ble_state[smNo].USD_CHLFLG_36_16);
	chanMap[3] = (uint8_t)(ble_state[smNo].USD_CHLFLG_36_16 >> 8U);
	chanMap[4] = (uint8_t)(ble_state[smNo].USD_CHLFLG_36_16 >> 16U);
}

uint8_t BLEPLAT_CNTR_SmGetCteAntennaPatternLen(uint8_t smNo){
	return (uint8_t)0;
}

uint8_t BLEPLAT_CNTR_SmGetCteAodNaoa(uint8_t smNo){
	return (uint8_t)0;
}

uint8_t BLEPLAT_CNTR_SmGetCteSlotWidth(uint8_t smNo) {
	return (uint8_t)0;
}

uint8_t BLEPLAT_CNTR_SmGetCteStatus(uint8_t smNo){
	return (uint8_t)0;
}

uint8_t BLEPLAT_CNTR_SmGetCteTime(uint8_t smNo){
	return (uint8_t)0;
}

//uint32_t* BLEPLAT_CNTR_SmGetEncIvPtr(uint8_t smNo) {
//	return (uint32_t*)&(bluedata + smNo)->ENCRYPTIV[0];
//}
//
//uint32_t* BLEPLAT_CNTR_SmGetEncKeyPtr(uint8_t smNo) {
//	return (uint32_t*)&(bluedata + smNo)->ENCRYPTK[0];
//}
//
//uint8_t BLEPLAT_CNTR_SmGetEncStatus(uint8_t smNo) {
//	return (uint8_t)LL_RADIO_Encryption_IsEnabled(smNo);
//}
//
//uint8_t BLEPLAT_CNTR_SmGetHopIncr(uint8_t smNo) {
//	return (uint8_t)LL_RADIO_GetHopIncrement(smNo);
//}
//
//uint8_t BLEPLAT_CNTR_SmGetMode(uint8_t smNo) {
//	return LL_RADIO_TxMode_IsEnabled(smNo);
//}
//
//uint8_t* BLEPLAT_CNTR_SmGetPrevRxPacketDataPtr(uint8_t smNo) {
//	return (uint8_t*)CONV_ADR(BLUE_TRANS_STRUCT_PTR_CAST(LL_RADIO_GetRcvPointPrevious(smNo))->DATAPTR);
//}
//
//TXRXPACK_STATE_t* BLEPLAT_CNTR_SmGetPrevRxPacketPtr(uint8_t smNo) {
//	return (TXRXPACK_STATE_t*)BLUE_TRANS_STRUCT_PTR_CAST(CONV_ADR(LL_RADIO_GetRcvPointPrevious(smNo)));
//}
//
//uint8_t* BLEPLAT_CNTR_SmGetPrevTxPacketDataPtr(uint8_t smNo) {
//	return (uint8_t*)CONV_ADR(BLUE_TRANS_STRUCT_PTR_CAST(LL_RADIO_GetTxPointPrevious(smNo))->DATAPTR);
//}
//
//TXRXPACK_STATE_t* BLEPLAT_CNTR_SmGetPrevTxPacketPtr(uint8_t smNo) {
//	return (TXRXPACK_STATE_t*)BLUE_TRANS_STRUCT_PTR_CAST(CONV_ADR(LL_RADIO_GetTxPointPrevious(smNo)));
//}

uint8_t BLEPLAT_CNTR_SmGetRemapChan(uint8_t smNo) {
	return ble_state[smNo].REMAP_CHAN;
}

//void BLEPLAT_CNTR_SmGetRxCount(uint8_t smNo, uint32_t* packetCount) {
//	packetCount[0] = LL_RADIO_GetPacketCounterRx_23_0(smNo);
//	packetCount[0] = packetCount[0] | ((uint32_t)LL_RADIO_GetPacketCounterRx_39_24(smNo) << 24U);
//	packetCount[1] = LL_RADIO_GetPacketCounterRx_39_24(smNo) >> 8U;
//}
//
uint8_t BLEPLAT_CNTR_SmGetRxPhy(uint8_t smNo) {
	return (uint8_t)ble_state[smNo].RX_PHY >> 28U;
}
//
//TXRXPACK_STATE_t* BLEPLAT_CNTR_SmGetTxPacketPtr(uint8_t smNo) {
//	return (TXRXPACK_STATE_t*)BLUE_TRANS_STRUCT_PTR_CAST(CONV_ADR(LL_RADIO_GetTxPoint(smNo)));
//}

uint8_t BLEPLAT_CNTR_SmGetTxPhy(uint8_t smNo) {
	return (uint8_t) ble_state[smNo].TX_PHY >> 24UL;
}

uint8_t BLEPLAT_CNTR_SmGetTxPwr(uint8_t smNo) {
	return ble_state[smNo].PA_PWR;
}

//uint8_t BLEPLAT_CNTR_SmGetUnmappedChan(uint8_t smNo) {
//	return (uint8_t)LL_RADIO_GetUnmappedChannel(smNo);
//}

void BLEPLAT_CNTR_SmInitTo0(uint8_t smNo) {
	memset((void*)&ble_state[smNo], 0, sizeof(RF_STATE_t));
	BLEPLAT_CNTR_SmEnRadioConfig(smNo, 0x01);
}

void BLEPLAT_CNTR_SmSetAccessAddr(uint8_t smNo, uint32_t x) {
	ble_state[smNo].ACC_ADDR = x;
}

void BLEPLAT_CNTR_SmSetChannelMap(uint8_t smNo, uint8_t* chanMap) {
	uint32_t value = (uint32_t)chanMap[0] | ((uint32_t)chanMap[1] << 8U);
	ble_state[smNo].USD_CHLFLG_15_0 = value;
	value = (uint32_t)chanMap[2] | ((uint32_t)chanMap[3] << 8U) | ((uint32_t)chanMap[4] << 16U);
	ble_state[smNo].USD_CHLFLG_36_16 = value;
}

void BLEPLAT_CNTR_SmSetCrcInit(uint8_t smNo, uint32_t x) {
	ble_state[smNo].CRC_INIT = x;
}

void BLEPLAT_CNTR_SmSetCteAntennaPatternLen(uint8_t smNo, uint8_t antPattLen) {}
uint32_t BLEPLAT_CNTR_SmGetCteAntennaPatternPtr(uint8_t smNo) {
	return 0x00UL;
}
void BLEPLAT_CNTR_SmSetCteAntennaPatternPtr(uint8_t smNo, uint8_t* antPattP) {}
void BLEPLAT_CNTR_SmSetCteAoa(uint8_t smNo) {}
void BLEPLAT_CNTR_SmSetCteAod(uint8_t smNo) {}
void BLEPLAT_CNTR_SmSetCteIqsamplesPtr(uint8_t smNo, uint32_t* iqSamplesP) {}
void BLEPLAT_CNTR_SmSetCteMaxIqsamplesNumb(uint8_t smNo, uint8_t iqsamplesNumb) {}
void BLEPLAT_CNTR_SmSetCteSlotWidth(uint8_t smNo, uint32_t cteSlot) {}
void BLEPLAT_CNTR_SmSetCteTime(uint8_t smNo, uint8_t cteTime) {}

void BLEPLAT_CNTR_SmSetDataLength(uint8_t smNo, uint8_t length) {
	ble_state[smNo].MAX_REC_LEN = length;
}

void BLEPLAT_CNTR_SmSetDataLengthExtnEn(uint8_t smNo) {}

void BLEPLAT_CNTR_SmSetHopIncr(uint8_t smNo, uint8_t x) {
	ble_state[smNo].HOP_INC = x;
}

//void BLEPLAT_CNTR_SmSetRemapChan(uint8_t smNo, uint8_t chan) {
//	LL_RADIO_SetRemapChannel(smNo, chan);
//}

void BLEPLAT_CNTR_SmSetRxCount(uint8_t smNo, uint32_t* packetCount) {
	ble_state[smNo].PCNT_RCV_23_0 = packetCount[0] & 0xFFFFFF;
	uint32_t value = (uint32_t)(packetCount[1] << 8U) | (uint32_t)(packetCount[0] >> 24U);
	ble_state[smNo].PCNT_RCV_39_24 = value & 0xFFFF;
}

void BLEPLAT_CNTR_SmSetRxCountDirectionBit(uint8_t smNo) {
	ble_state[smNo].PCNT_RCV_39_24 |= 0x8000U;
}

void BLEPLAT_CNTR_SmSetRxMode(uint8_t smNo) {
	ble_state[smNo].TX_MODE = 0;
}
//
void BLEPLAT_CNTR_SmSetRxPacketPtr(uint8_t smNo, TXRXPACK_STATE_t* packetP) {
	ble_state[smNo].RCV_POINT = (uint32_t)&packetP;
}

void BLEPLAT_CNTR_SmSetRxPhy(uint8_t smNo, uint8_t rxPhy) {
	ble_state[smNo].RX_PHY = rxPhy;
}

//void BLEPLAT_CNTR_SmSetTxCount(uint8_t smNo, uint32_t* packetCount) {
//	LL_RADIO_SetPacketCounterTx_31_0(smNo, (uint32_t) packetCount[0]);
//	LL_RADIO_SetPacketCounterTx_39_32(smNo, (uint32_t) packetCount[1]);
//}
//
//void BLEPLAT_CNTR_SmSetTxCountDirectionBit(uint8_t smNo) {
//	uint32_t value =  (LL_RADIO_GetPacketCounterTx_39_32(smNo) | 0x00000080U);
//	LL_RADIO_SetPacketCounterTx_39_32(smNo,  value);
//}

void BLEPLAT_CNTR_SmSetTxMode(uint8_t smNo) {
	ble_state[smNo].TX_MODE = 1;
}

void BLEPLAT_CNTR_SmSetTxPacketPtr(uint8_t smNo, TXRXPACK_STATE_t* packetP) {
	ble_state[smNo].TX_POINT = (uint32_t)&packetP;
}

void BLEPLAT_CNTR_SmSetTxPhy(uint8_t smNo, uint8_t txPhy) {
	ble_state[smNo].TX_PHY = txPhy;
}

void BLEPLAT_CNTR_SmEnTxHp(uint8_t smNo, uint8_t enable) {}

/* Consider PA Level 32 the one used to enable high power. */
void BLEPLAT_CNTR_SmSetTxPwr(uint8_t smNo, uint8_t paLevel) {
	ble_state[smNo].PA_PWR = paLevel;
}
//
//void BLEPLAT_CNTR_SmSetUnmappedChan(uint8_t smNo, uint8_t chan) {
//	LL_RADIO_SetUnmappedChannel(smNo, (uint32_t) chan);
//}

void BLEPLAT_CNTR_SmToggleNesn(uint8_t smNo) {
	ble_state[smNo].NESN ^= 1;
}

//void BLEPLAT_CNTR_SmToggleSn(uint8_t smNo) {
//	LL_RADIO_ToggleSequenceNumber(smNo);
//}
//
//void BLEPLAT_CNTR_StartEncrypt() {
//	LL_RADIO_BlueSetManAESCmdStart(0x1U);
//}

uint32_t BLEPLAT_CNTR_TimeDiff(uint32_t x, uint32_t y) {
	return (uint32_t)(x - y);
}

//uint8_t BLEPLAT_CNTR_DemodDelaySt(uint8_t RxPHY) {
//	return (uint8_t)((LL_PHY_CODED == RxPHY) ? 0x9DU : 0x12U);
//}


//BLEPLAT_CNTR_ResultStatus BLEPLAT_CNTR_Init(void)
//{
//
//	return BLEPLAT_CNTR_SUCCESS;
//}
//
//BLEPLAT_CNTR_ResultStatus BLEPLAT_CNTR_Deinit(void)
//{
//
//	return BLEPLAT_CNTR_SUCCESS;
//}

//void BLEPLAT_CNTR_ClearInterrupt(uint32_t x)
//{
//	LL_RADIO_BlueSetInterrupt1RegRegister(x);
//}
//
//void BLEPLAT_CNTR_ClearSemareq(void)
//{
//	LL_RADIO_BlueSetClearSemaphoreRequest(0x1U);
//}
//
//void BLEPLAT_CNTR_TxRxSkip(void)
//{
//	LL_RADIO_BlueSetTxRxSkip(0x1U);
//}
//
//uint32_t* BLEPLAT_CNTR_GetCipherTextPtr()
//{
//	return (uint32_t*)&BLUE->MANAESCIPHERTEXT0REG;
//}
//
//uint32_t* BLEPLAT_CNTR_GetClrTextPtr()
//{
//	return (uint32_t*)&BLUE->MANAESCLEARTEXT0REG;
//}
//
//uint32_t* BLEPLAT_CNTR_GetEncKeyPtr()
//{
//	return (uint32_t*)&BLUE->MANAESKEY0REG;
//}
//
//uint8_t BLEPLAT_CNTR_GetEncryptDoneStatus()
//{
//	return (uint8_t)!LL_RADIO_BlueGetManAESStatusBusy();
//}
//
//uint8_t BLEPLAT_CNTR_GetIqsamplesMissingError(void)
//{
//#if defined(STM32WB09) || defined(STM32WB05)
//	return (uint8_t)LL_RADIO_GetIQSamplesMissingError();
//#elif defined(STM32WB06) || defined(STM32WB07)
//	return (uint8_t)0;
//#endif
//}
//
//uint8_t BLEPLAT_CNTR_GetIqsamplesNumber(void)
//{
//#if defined(STM32WB09) || defined(STM32WB05)
//	return (uint8_t)LL_RADIO_GetIQSamplesNumber();
//#elif defined(STM32WB06) || defined(STM32WB07)
//	return (uint8_t)0;
//#endif
//}
//
//uint8_t BLEPLAT_CNTR_getIqsamplesReady(void)
//{
//#if defined(STM32WB09) || defined(STM32WB05)
//	return (uint8_t)LL_RADIO_GetIQSamplesReady();
//#elif defined(STM32WB06) || defined(STM32WB07)
//	return (uint8_t)0;
//#endif
//}
//
//uint8_t BLEPLAT_CNTR_GetIsrLatency()
//{
//	return (uint8_t)LL_RADIO_BlueGetInterrupt1Latency();
//}
//
//uint32_t BLEPLAT_CNTR_GetTimercapture()
//{
//	return LL_RADIO_BlueGetTimerCapture();
//}

void BLEPLAT_CNTR_GlobEnableBlue() { ble_globstate.ACTIVE = 1; }

//void BLEPLAT_CNTR_GlobEnableIntnoactivelerrorInt()
//{
//	LL_RADIO_NoActiveLErrorInterrupt_Enable();
//}
//
//void BLEPLAT_CNTR_GlobEnableOverrunAct2Int()
//{
//	LL_RADIO_Active2ErrorInterrupt_Enable();
//}
//
//uint8_t BLEPLAT_CNTR_GlobGetDefaultAntennaid()
//{
//#if defined(GLOBAL_WORD6_DEFAULTANTENNAID_Msk)
//	return (uint8_t)LL_RADIO_GetDefaultAntennaID();
//#else
//	return (uint8_t)0;
//#endif
//}
//
//uint8_t BLEPLAT_CNTR_GlobGetWakeupinitdelay()
//{
//	return (uint8_t) LL_RADIO_GetWakeupInitDelay();
//}
//
//void BLEPLAT_CNTR_GlobReloadRadioConfigP()
//{
//	*(uint32_t*)(RRM_BASE + 0x10U) = 0x01U;
//}
//
//void BLEPLAT_CNTR_GlobSetChkflagautoclearena()
//{
//	LL_RADIO_ChkFlagAutoclearEnable_Enable();
//}
//
//void BLEPLAT_CNTR_GlobSetDefaultAntennaid(uint8_t x)
//{
//#if defined(GLOBAL_WORD6_DEFAULTANTENNAID_Msk)
//	LL_RADIO_SetDefaultAntennaID((uint32_t)x);
//#else
//	/* nothing to do */
//#endif
//}
//
//void BLEPLAT_CNTR_GlobSetInitRadioDelayTxCal(uint8_t x)
//{
//	LL_RADIO_SetTransmitCalDelayChk((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobSetInitRadioDelayTxNocal(uint8_t x)
//{
//	LL_RADIO_SetTransmitNoCalDelayChk((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobSetRadioConfigP(volatile uint32_t* x)
//{
//	LL_RADIO_SetRadioConfigurationAddressPointer(x[0]);
//}
//
//void BLEPLAT_CNTR_GlobSetWakeupinitdelay(uint8_t x)
//{
//	LL_RADIO_SetWakeupInitDelay((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteConfigEnd(uint8_t x)
//{
//	LL_RADIO_SetConfigurationEndDuration((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWritePeriodslow(uint16_t x)
//{
//	/* nothing to do */
//}
//
//void BLEPLAT_CNTR_GlobWriteRcvdelay(uint8_t x)
//{
//	LL_RADIO_SetReceivedCalDelayChk((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteRcvdelay1(uint8_t x)
//{
//	LL_RADIO_SetReceivedNoCalDelayChk((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteSlot(uint8_t slotNo)
//{
//	LL_RADIO_SetCurrentConnectionMachineNumber((uint32_t) slotNo);
//}
//
//void BLEPLAT_CNTR_GlobWriteTimer12initdelaycal(uint8_t x)
//{
//	LL_RADIO_SetTimer12InitDelayCal((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteTimer2initdelaynocal(uint8_t x)
//{
//	LL_RADIO_SetTimer12InitDelayNoCal((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteTxDataReadyCheck(uint8_t x)
//{
//	LL_RADIO_SetTxDataReadyCheck((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteTxReadyTimeout(uint8_t x)
//{
//	LL_RADIO_SetTransmissionReadyTimeout((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteTxdelay(uint8_t x)
//{
//	LL_RADIO_SetTxDelayStart((uint32_t) x);
//}
//
//void BLEPLAT_CNTR_GlobWriteTxdelayEnd(uint8_t x)
//{
//	LL_RADIO_SetTxDelayEnd((uint32_t) x);
//}
//
//uint32_t BLEPLAT_CNTR_IntGetIntStatusAnyHwError(uint32_t x)
//{
//	return (uint32_t)(x & ANY_HW_ERROR_INTERRUPT_Msk);
//}
//
//uint32_t BLEPLAT_CNTR_IntGetIntStatusRxOverflowError(uint32_t x)
//{
//	return (uint32_t)(x & BLUE_STATUSREG_RXOVERFLOWERROR_Msk);
//}
//
//uint32_t BLEPLAT_CNTR_IntGetIntStatusBitAct2Error(uint32_t x)
//{
//	return (uint32_t)(x & BLUE_STATUSREG_ACTIVE2ERROR_Msk);
//}
//
//uint32_t BLEPLAT_CNTR_IntGetIntStatusBitTimerOverrun(uint32_t x)
//{
//	return 0;
//}

//#if defined(BLUE_STATUSREG_DONE_Msk)
//uint32_t BLEPLAT_CNTR_IntGetIntStatusDone(uint32_t x)
//{
//	return (uint32_t)(x & BLUE_STATUSREG_DONE_Msk);
//}
//#endif
//
//#if defined(BLUE_STATUSREG_ENCERROR_Msk)
//uint32_t BLEPLAT_CNTR_IntGetIntStatusEncErr(uint32_t x)
//{
//    return (uint32_t)(x & BLUE_STATUSREG_ENCERROR_Msk);
//}
//#endif

//#if defined(BLUE_STATUSREG_NOACTIVELERROR_Msk)
//uint32_t BLEPLAT_CNTR_IntGetIntStatusNoactiveError(uint32_t x)
//{
//    return (uint32_t)(x & BLUE_STATUSREG_NOACTIVELERROR_Msk);
//}
//#endif

//#if defined(BLUE_STATUSREG_TXERROR_1_Msk)
//uint32_t BLEPLAT_CNTR_IntGetIntStatusTxError1(uint32_t x)
//{
//    return (uint32_t)(x & BLUE_STATUSREG_TXERROR_1_Msk);
//}
//#endif
//
//#if defined(BLUE_STATUSREG_TXERROR_3_Msk)
//uint32_t BLEPLAT_CNTR_IntGetIntStatusTxError3(uint32_t x)
//{
//    return (uint32_t)(x & BLUE_STATUSREG_TXERROR_3_Msk);
//}
//#endif
uint32_t BLEPLAT_CNTR_IntGetIntStatusRxOk(uint32_t x) {
    return (uint32_t)(x & 0x01000000UL);
}
//#if defined(BLUE_STATUSREG_TIMECAPTURETRIG_Msk)
//uint32_t BLEPLAT_CNTR_IntGetIntStatusTrigRcv(uint32_t x)
//{
//    return (uint32_t)(x & BLUE_STATUSREG_TIMECAPTURETRIG_Msk);
//}
//#endif
//
//#if defined(BLUE_STATUSREG_PREVTRANSMIT_Msk)
//uint32_t BLEPLAT_CNTR_IntGetIntStatusTxDone(uint32_t x)
//{
//    return (uint32_t)(x & BLUE_STATUSREG_PREVTRANSMIT_Msk);
//}
//#endif
//
//#if defined(BLUE_STATUSREG_TXOK_Msk)
//uint32_t BLEPLAT_CNTR_IntGetIntStatusTxOk(uint32_t x)
//{
//    return (uint32_t)(x & BLUE_STATUSREG_TXOK_Msk);
//}
//#endif
//
//void BLEPLAT_CNTR_PacketClrCrcinitSel(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetCRCInitializationSelector((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x0);
//}
//
//void BLEPLAT_CNTR_PacketClrCteSamplingEn(TXRXPACK_STATE_t* packetP)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetCTEAndSamplingEnable((TXRXPACK_TypeDef*)packetP, 0x0);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
void BLEPLAT_CNTR_PacketClrPllTrig(TXRXPACK_STATE_t* packetP) {
	packetP->CAL_REQ = 0;
}
//uint8_t BLEPLAT_CNTR_PacketGetCteSamplingEn(TXRXPACK_STATE_t* packetP)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	return (uint8_t)LL_RADIO_GetCTEAndSamplingEnable((TXRXPACK_TypeDef *)packetP);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	return (uint8_t)0;
//#endif
//
//}
//
//uint8_t* BLEPLAT_CNTR_PacketGetDataPtr(TXRXPACK_STATE_t* packetP)
//{
//	return (uint8_t*)CONV_ADR(LL_RADIO_GetDataPointer((TXRXPACK_TypeDef*)packetP));
//}

//void BLEPLAT_CNTR_PacketSetAdvPduFormat(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetAdvertise((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1);
//}
//
//void BLEPLAT_CNTR_PacketSetCrcinitSel(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetCRCInitializationSelector((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1);
//}
//
//void BLEPLAT_CNTR_PacketSetCteSamplingEn(TXRXPACK_STATE_t* packetP)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetCTEAndSamplingEnable((TXRXPACK_TypeDef*)packetP, 0x01);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
//
//void BLEPLAT_CNTR_PacketSetDataPduFormat(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetAdvertise((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x0);
//}
//
//void BLEPLAT_CNTR_PacketSetDataPtr(TXRXPACK_STATE_t* packetP, void* dataP)
//{
//	LL_RADIO_SetDataPointer((TXRXPACK_TypeDef*)packetP, BLUE_DATA_PTR_CAST(dataP));
//}
//void BLEPLAT_CNTR_PacketSetIntCrcErr(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_IntRcvCRCErr_Enable((TXRXPACK_TypeDef*)packetP);
//}
//
//void BLEPLAT_CNTR_PacketSetIntDone(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_IntDone_Enable((TXRXPACK_TypeDef*)packetP);
//}
//
//void BLEPLAT_CNTR_PacketSetIntRcvOk(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_IntRcvOk_Enable((TXRXPACK_TypeDef*)packetP);
//}
//
//void BLEPLAT_CNTR_PacketSetIntTimeout(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_IntRcvTimeout_Enable((TXRXPACK_TypeDef*)packetP);
//}
//
//void BLEPLAT_CNTR_PacketSetIntTrigRcv(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetIntTimeCapture((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1U);
//}
//
//void BLEPLAT_CNTR_PacketSetIntTxOk(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetIntTxOk((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1);
//}
//
//void BLEPLAT_CNTR_PacketSetKeepsemareq(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetKeepSemaRequest((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1);
//}
//
//void BLEPLAT_CNTR_PacketSetNextPtr(TXRXPACK_STATE_t* packetP, TXRXPACK_STATE_t* packetNextP)
//{
//	LL_RADIO_SetNextPointer((TXRXPACK_TypeDef*)packetP, (uint32_t) BLUE_STRUCT_PTR_CAST((TXRXPACK_TypeDef*)packetNextP));
//}
//
//void BLEPLAT_CNTR_PacketSetNextRxMode(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetNextTxMode((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x0);
//}
//
//void BLEPLAT_CNTR_PacketSetNextSlot(TXRXPACK_STATE_t* packetP, uint8_t slot)
//{
//	/* nothing to do */
//}
//
//void BLEPLAT_CNTR_PacketSetNextTxMode(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetNextTxMode((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1);
//}
//
//void BLEPLAT_CNTR_PacketSetNsEn(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_AutomaticSnNesnHardwareMechanism_Enable((TXRXPACK_TypeDef*)packetP);
//}
//
void BLEPLAT_CNTR_PacketSetPllTrig(TXRXPACK_STATE_t* packetP) {
	packetP->CAL_REQ = 1;
}
//void BLEPLAT_CNTR_PacketSetTimeout(TXRXPACK_STATE_t* packetP, uint32_t x)
//{
//	LL_RADIO_SetTimer2Triggering((TXRXPACK_TypeDef*)packetP, x);
//}
//
//void BLEPLAT_CNTR_PacketSetTimer2Active(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_Timer2_Enable((TXRXPACK_TypeDef*)packetP);
//}
//
//void BLEPLAT_CNTR_PacketSetTimerTrigDone(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetTrigDone((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1);
//}
//
//void BLEPLAT_CNTR_PacketSetTimerTrigRcv(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetTrigRcv((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1);
//}
//
//void BLEPLAT_CNTR_PacketSetTxReady(TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetTransmissionDataReady((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1);
//	LL_RADIO_SetAllTableDataReady((TXRXPACK_TypeDef*)packetP, (uint32_t) 0x1);
//}
//
//void BLEPLAT_CNTR_SetRadioConfig(uint8_t* value)
//{
//	/* nothing to do */
//}

void BLEPLAT_CNTR_SetRcvLen(TXRXPACK_STATE_t* packetP, uint32_t rcvLen) {
	(void)packetP; ble_globstate.RCV_TO = (rcvLen & 0xFFFFFU);
}

//void BLEPLAT_CNTR_SmCteOff(uint8_t smNo)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetCTEDisable(smNo, 0x01);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
//
//void BLEPLAT_CNTR_SmCteOn(uint8_t smNo)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetCTEDisable(smNo, 0x00);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}

void BLEPLAT_CNTR_SmEnRadioConfig(uint8_t smNo, uint32_t enable) {
	ble_state[smNo].RF_COM_LST_EN = enable;
}

//void BLEPLAT_CNTR_SmEncOff(uint8_t smNo)
//{
//	LL_RADIO_Encryption_Disable(smNo);
//	LL_RADIO_ReceiveEncryptionErrorInterrupt_Disable(smNo);
//}
//
//void BLEPLAT_CNTR_SmEncOn(uint8_t smNo)
//{
//	LL_RADIO_Encryption_Enable(smNo);
//	LL_RADIO_ReceiveEncryptionErrorInterrupt_Enable(smNo);
//}
//void BLEPLAT_CNTR_SmGetChannelMap(uint8_t smNo, uint8_t* chanMap)
//{
//	chanMap[0] = (uint8_t)(LL_RADIO_GetUsedChannelFlags_15_0(smNo));
//	chanMap[1] = (uint8_t)(LL_RADIO_GetUsedChannelFlags_15_0(smNo) >> 8U);
//	chanMap[2] = (uint8_t)(LL_RADIO_GetUsedChannelFlags_36_16(smNo));
//	chanMap[3] = (uint8_t)(LL_RADIO_GetUsedChannelFlags_36_16(smNo) >> 8U);
//	chanMap[4] = (uint8_t)(LL_RADIO_GetUsedChannelFlags_36_16(smNo) >> 16U);
//}
//
//uint8_t BLEPLAT_CNTR_SmGetCteAntennaPatternLen(uint8_t smNo)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	return (uint8_t)LL_RADIO_GetAntennaPatternLength(smNo);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	return (uint8_t)0;
//#endif
//}
//
//uint8_t BLEPLAT_CNTR_SmGetCteAodNaoa(uint8_t smNo)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	return (uint8_t)LL_RADIO_GetAodNaoa(smNo);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	return (uint8_t)0;
//#endif
//}
//
//uint8_t BLEPLAT_CNTR_SmGetCteSlotWidth(uint8_t smNo)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	return (uint8_t)LL_RADIO_GetCTESlotWidth(smNo);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	return (uint8_t)0;
//#endif
//}
//
//uint8_t BLEPLAT_CNTR_SmGetCteStatus(uint8_t smNo)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	return (uint8_t)LL_RADIO_GetCTEDisable(smNo);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	return (uint8_t)0;
//#endif
//}
//
//uint8_t BLEPLAT_CNTR_SmGetCteTime(uint8_t smNo)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	return (uint8_t)LL_RADIO_GetCTETime(smNo);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	return (uint8_t)0;
//#endif
//}
//
//uint32_t* BLEPLAT_CNTR_SmGetEncIvPtr(uint8_t smNo)
//{
//	return (uint32_t*)&(bluedata + smNo)->ENCRYPTIV[0];
//}
//
//uint32_t* BLEPLAT_CNTR_SmGetEncKeyPtr(uint8_t smNo)
//{
//	return (uint32_t*)&(bluedata + smNo)->ENCRYPTK[0];
//}
//
//uint8_t BLEPLAT_CNTR_SmGetEncStatus(uint8_t smNo)
//{
//	return (uint8_t)LL_RADIO_Encryption_IsEnabled(smNo);
//}

uint8_t BLEPLAT_CNTR_SmGetHopIncr(uint8_t smNo) {
	return ble_state[smNo].HOP_INC;
}

//uint8_t BLEPLAT_CNTR_SmGetMode(uint8_t smNo)
//{
//	return LL_RADIO_TxMode_IsEnabled(smNo);
//}
//
//uint8_t* BLEPLAT_CNTR_SmGetPrevRxPacketDataPtr(uint8_t smNo)
//{
//	return (uint8_t*)CONV_ADR(BLUE_TRANS_STRUCT_PTR_CAST(LL_RADIO_GetRcvPointPrevious(smNo))->DATAPTR);
//}
//
//TXRXPACK_STATE_t* BLEPLAT_CNTR_SmGetPrevRxPacketPtr(uint8_t smNo)
//{
//	return (TXRXPACK_STATE_t*)BLUE_TRANS_STRUCT_PTR_CAST(CONV_ADR(LL_RADIO_GetRcvPointPrevious(smNo)));
//}
//
//uint8_t* BLEPLAT_CNTR_SmGetPrevTxPacketDataPtr(uint8_t smNo)
//{
//	return (uint8_t*)CONV_ADR(BLUE_TRANS_STRUCT_PTR_CAST(LL_RADIO_GetTxPointPrevious(smNo))->DATAPTR);
//}
//
//TXRXPACK_STATE_t* BLEPLAT_CNTR_SmGetPrevTxPacketPtr(uint8_t smNo)
//{
//	return (TXRXPACK_STATE_t*)BLUE_TRANS_STRUCT_PTR_CAST(CONV_ADR(LL_RADIO_GetTxPointPrevious(smNo)));
//}
//void BLEPLAT_CNTR_SmGetRxCount(uint8_t smNo, uint32_t* packetCount)
//{
//
//	packetCount[0] = LL_RADIO_GetPacketCounterRx_23_0(smNo);
//	packetCount[0] = packetCount[0] | ((uint32_t)LL_RADIO_GetPacketCounterRx_39_24(smNo) << 24U);
//	packetCount[1] = LL_RADIO_GetPacketCounterRx_39_24(smNo) >> 8U;
//}
//
//uint8_t BLEPLAT_CNTR_SmGetRxPhy(uint8_t smNo)
//{
//	return (uint8_t)LL_RADIO_GetReceptionPhy(smNo);
//}
//
//TXRXPACK_STATE_t* BLEPLAT_CNTR_SmGetTxPacketPtr(uint8_t smNo)
//{
//	return (TXRXPACK_STATE_t*)BLUE_TRANS_STRUCT_PTR_CAST(CONV_ADR(LL_RADIO_GetTxPoint(smNo)));
//}
//
//uint8_t BLEPLAT_CNTR_SmGetTxPhy(uint8_t smNo)
//{
//	return (uint8_t)LL_RADIO_GetTransmissionPhy(smNo);
//}
//
//uint8_t BLEPLAT_CNTR_SmGetTxPwr(uint8_t smNo)
//{
//	uint8_t pa_level = LL_RADIO_GetPAPower(smNo);
//
//#if defined(STM32WB09)
//	if (LL_RADIO_TxHp_IsEnabled(smNo) && (pa_level == MAX_PA_LEVEL))
//    {
//        pa_level = HP_PA_LEVEL;
//    }
//#endif
//	return pa_level;
//}
//
//uint8_t BLEPLAT_CNTR_SmGetUnmappedChan(uint8_t smNo)
//{
//	return (uint8_t)LL_RADIO_GetUnmappedChannel(smNo);
//}
//
//void BLEPLAT_CNTR_SmInitTo0(uint8_t smNo)
//{
//	memset((void*)&bluedata[smNo], 0, sizeof(STATMACH_TypeDef));
//	BLEPLAT_CNTR_SmEnRadioConfig(smNo, 0x01);
//}
//void BLEPLAT_CNTR_SmSetChannelMap(uint8_t smNo, uint8_t* chanMap)
//{
//	uint32_t value = (uint32_t)chanMap[0] | ((uint32_t)chanMap[1] << 8U);
//	LL_RADIO_SetUsedChannelFlags_15_0(smNo, value);
//	value = (uint32_t)chanMap[2] | ((uint32_t)chanMap[3] << 8U) | ((uint32_t)chanMap[4] << 16U);
//	LL_RADIO_SetUsedChannelFlags_36_16(smNo, value);
//}
//
//void BLEPLAT_CNTR_SmSetCrcInit(uint8_t smNo, uint32_t x)
//{
//	LL_RADIO_SetCRCInitializationValue(smNo, x);
//}
//
//void BLEPLAT_CNTR_SmSetCteAntennaPatternLen(uint8_t smNo, uint8_t antPattLen)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetAntennaPatternLength(smNo, (uint32_t) antPattLen);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
//
//uint32_t BLEPLAT_CNTR_SmGetCteAntennaPatternPtr(uint8_t smNo)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	return LL_RADIO_GetAntennaPatternPtr(smNo);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	return 0x00UL;
//#endif
//}
//
//void BLEPLAT_CNTR_SmSetCteAntennaPatternPtr(uint8_t smNo, uint8_t* antPattP)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetAntennaPatternPtr(smNo, (uint32_t)(uintptr_t)antPattP);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
//
//void BLEPLAT_CNTR_SmSetCteAoa(uint8_t smNo)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetAodNaoa(smNo, 0x0);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
//
//void BLEPLAT_CNTR_SmSetCteAod(uint8_t smNo)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetAodNaoa(smNo, 0x01);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
//
//void BLEPLAT_CNTR_SmSetCteIqsamplesPtr(uint8_t smNo, uint32_t* iqSamplesP)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetIQSamplesPtr(smNo, (uint32_t)(uintptr_t)iqSamplesP);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
//
//void BLEPLAT_CNTR_SmSetCteMaxIqsamplesNumb(uint8_t smNo, uint8_t iqsamplesNumb)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetMaximumIQSamplesNumber(smNo, (uint32_t) iqsamplesNumb);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
//
//void BLEPLAT_CNTR_SmSetCteSlotWidth(uint8_t smNo, uint32_t cteSlot)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetCTESlotWidth(smNo, cteSlot);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
//
//void BLEPLAT_CNTR_SmSetCteTime(uint8_t smNo, uint8_t cteTime)
//{
//#if defined(STM32WB05) || defined(STM32WB09)
//	LL_RADIO_SetCTETime(smNo, (uint32_t) cteTime);
//#elif defined(STM32WB06) || defined(STM32WB07)
//	/* nothing to do */
//#endif
//}
//
//void BLEPLAT_CNTR_SmSetDataLength(uint8_t smNo, uint8_t length)
//{
//	LL_RADIO_SetMaximumReceiveLength(smNo, (uint32_t) length);
//}
//
//void BLEPLAT_CNTR_SmSetDataLengthExtnEn(uint8_t smNo)
//{
//	/* nothing to do */
//}
//
//void BLEPLAT_CNTR_SmSetHopIncr(uint8_t smNo, uint8_t x)
//{
//	LL_RADIO_SetHopIncrement(smNo, (uint32_t) x);
//}

void BLEPLAT_CNTR_SmSetRemapChan(uint8_t smNo, uint8_t chan) {
	ble_state[smNo].REMAP_CHAN = chan;
}

//void BLEPLAT_CNTR_SmSetRxMode(uint8_t smNo)
//{
//	LL_RADIO_TxMode_Disable(smNo);
//}
//
//void BLEPLAT_CNTR_SmSetRxPacketPtr(uint8_t smNo, TXRXPACK_STATE_t* packetP)
//{
//	//LL_RADIO_SetRcvPoint(smNo, (uint32_t) BLUE_STRUCT_PTR_CAST((TXRXPACK_TypeDef*)packetP));
//}
//void BLEPLAT_CNTR_SmSetTxCount(uint8_t smNo, uint32_t* packetCount)
//{
//	LL_RADIO_SetPacketCounterTx_31_0(smNo, (uint32_t) packetCount[0]);
//	LL_RADIO_SetPacketCounterTx_39_32(smNo, (uint32_t) packetCount[1]);
//}
//
//void BLEPLAT_CNTR_SmSetTxCountDirectionBit(uint8_t smNo)
//{
//	uint32_t value =  (LL_RADIO_GetPacketCounterTx_39_32(smNo) | 0x00000080U);
//	LL_RADIO_SetPacketCounterTx_39_32(smNo,  value);
//}
//
//
//void BLEPLAT_CNTR_SmSetTxPacketPtr(uint8_t smNo, TXRXPACK_STATE_t* packetP)
//{
//	LL_RADIO_SetTxPoint(smNo, (uint32_t) BLUE_STRUCT_PTR_CAST((TXRXPACK_TypeDef*)packetP));
//}
//
//void BLEPLAT_CNTR_SmSetTxPhy(uint8_t smNo, uint8_t txPhy)
//{
//	LL_RADIO_SetTransmissionPhy(smNo, (uint32_t) txPhy);
//}
//
//void BLEPLAT_CNTR_SmEnTxHp(uint8_t smNo, uint8_t enable)
//{
//#if defined(STM32WB09)
//	if(enable)
//  {
//    LL_RADIO_TxHp_Enable(smNo);
//  }
//  else
//  {
//    LL_RADIO_TxHp_Disable(smNo);
//  }
//#endif
//}
//
///* Consider PA Level 32 the one used to enable high power. */
//void BLEPLAT_CNTR_SmSetTxPwr(uint8_t smNo, uint8_t paLevel)
//{
//#if defined(STM32WB09)
//	if(paLevel == HP_PA_LEVEL)
//  {
//    LL_RADIO_TxHp_Enable(smNo);
//    paLevel = MAX_PA_LEVEL;
//  }
//  else
//  {
//    LL_RADIO_TxHp_Disable(smNo);
//  }
//#endif
//
//	LL_RADIO_SetPAPower(smNo, (uint32_t) paLevel);
//}

void BLEPLAT_CNTR_SmSetUnmappedChan(uint8_t smNo, uint8_t chan) {
	ble_state[smNo].UCHAN = chan;
}

//void BLEPLAT_CNTR_SmToggleSn(uint8_t smNo)
//{
//	LL_RADIO_ToggleSequenceNumber(smNo);
//}
//
//void BLEPLAT_CNTR_StartEncrypt()
//{
//	LL_RADIO_BlueSetManAESCmdStart(0x1U);
//}
//
//uint32_t BLEPLAT_CNTR_TimeDiff(uint32_t x, uint32_t y)
//{
//	return (uint32_t)(x - y);
//}
//
uint8_t BLEPLAT_CNTR_DemodDelaySt(uint8_t RxPHY) {
	return (uint8_t)((0x04U == RxPHY) ? 0x9DU : 0x12U);
}


//BLEPLAT_NvmStatusTypeDef BLEPLAT_NvmGet(BLEPLAT_NvmSeekModeTypeDef Mode,
//										BLEPLAT_NvmRecordTypeDef Type,
//										uint16_t Offset,
//										uint8_t* pData,
//										uint16_t Size) {
//	NVMDB_RecordSizeType size_out;
//	NVMDB_status_t ret;
//	NVMDB_IdType db_id;
//	if(Type == BLEPLAT_NVM_REC_DEVICE_ID) {
//		curr_handle_p = &device_id_db_h;
//		db_id = 1;
//	} else {
//		curr_handle_p = &sec_gatt_db_h;
//		db_id = 0;
//	}
//	if(Mode == BLEPLAT_NVM_CURRENT) {
//		ret = NVMDB_ReadCurrentRecord(curr_handle_p, Offset, pData, Size, &size_out);
//	} else {
//		if(Mode == BLEPLAT_NVM_FIRST)
//		{
//			NVMDB_HandleInit(db_id, curr_handle_p);
//		}
//		ret = NVMDB_ReadNextRecord(curr_handle_p, Type, Offset, pData, Size, &size_out);
//	}
//
//	if(ret == NVMDB_STATUS_OK) {
//		return BLEPLAT_OK;
//	}
//
//	if(ret == NVMDB_STATUS_END_OF_DB) {
//		return BLEPLAT_EOF;
//	}
//
//	return BLEPLAT_BUSY;
//}