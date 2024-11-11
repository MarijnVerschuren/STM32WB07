//
// Created by quvx on 05/11/24.
//

#include "RF/BLE.h"


uint8_t a_AdvData[] = {
	2, AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
	8, AD_TYPE_COMPLETE_LOCAL_NAME, 'p', '2', 'p', 'S', '_', 'X', 'X',  /* Complete name */
	15, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x30, 0x00, 0x00 /*  */, 0x00 /*  */, 0x00 /*  */, 0x00 /*  */, 0x00 /*  */, 0x00 /*  */, 0x00 /*  */, 0x00 /*  */, 0x00 /*  */, 0x00 /*  */, 0x00 /*  */, 0x00 /*  */,
};

uint8_t SEQ_BitPosition(uint32_t Value)
{
	uint8_t n = 0U;
	uint32_t lvalue = Value;

	if ((lvalue & 0xFFFF0000U) == 0U)  { n  = 16U; lvalue <<= 16U;  }
	if ((lvalue & 0xFF000000U) == 0U)  { n +=  8U; lvalue <<=  8U;  }
	if ((lvalue & 0xF0000000U) == 0U)  { n +=  4U; lvalue <<=  4U;  }

	n += SEQ_clz_table_4bit[lvalue >> (32-4)];

	return (uint8_t)(31U-n);
}

void UTIL_SEQ_RegTask(UTIL_SEQ_bm_t TaskId_bm, uint32_t Flags, void (*Task)( void )) {
    (void)Flags;
	ENTER_CRITICAL_SECTION();
    TaskCb[SEQ_BitPosition(TaskId_bm)] = Task;
	EXIT_CRITICAL_SECTION();
}

void UTIL_SEQ_SetTask(UTIL_SEQ_bm_t TaskId_bm , uint32_t Task_Prio) {
	ENTER_CRITICAL_SECTION();
	TaskSet |= TaskId_bm;
	TaskPrio[Task_Prio].priority |= TaskId_bm;
	EXIT_CRITICAL_SECTION();
}

void UTIL_SEQ_Run( UTIL_SEQ_bm_t Mask_bm )
{
	uint32_t counter;
	UTIL_SEQ_bm_t current_task_set;
	UTIL_SEQ_bm_t super_mask_backup;
	UTIL_SEQ_bm_t local_taskset;
	UTIL_SEQ_bm_t local_evtset;
	UTIL_SEQ_bm_t local_taskmask;
	UTIL_SEQ_bm_t local_evtwaited;

	super_mask_backup = SuperMask;
	SuperMask &= Mask_bm;
	local_taskset = TaskSet;
	local_evtset = EvtSet;
	local_taskmask = TaskMask;
	local_evtwaited =  EvtWaited;
	while(((local_taskset & local_taskmask & SuperMask) != 0U) && ((local_evtset & local_evtwaited)==0U)){
		counter = 0U;
		while((TaskPrio[counter].priority & local_taskmask & SuperMask)== 0U){
			counter++;
		}
		current_task_set = TaskPrio[counter].priority & local_taskmask & SuperMask;
		if ((TaskPrio[counter].round_robin & current_task_set) == 0U){
			TaskPrio[counter].round_robin = UTIL_SEQ_ALL_BIT_SET;
		}
		CurrentTaskIdx = (SEQ_BitPosition(current_task_set & TaskPrio[counter].round_robin));
		TaskPrio[counter].round_robin &= ~(1U << CurrentTaskIdx);
		ENTER_CRITICAL_SECTION();
		TaskSet &= ~(1U << CurrentTaskIdx);
		for (counter = UTIL_SEQ_CONF_PRIO_NBR; counter != 0U; counter--){
			TaskPrio[counter - 1U].priority &= ~(1U << CurrentTaskIdx);
		}
		EXIT_CRITICAL_SECTION();
		TaskCb[CurrentTaskIdx]( );
		local_taskset = TaskSet;
		local_evtset = EvtSet;
		local_taskmask = TaskMask;
		local_evtwaited = EvtWaited;
	}

	CurrentTaskIdx = UTIL_SEQ_NOTASKRUNNING;
	if ((local_evtset & EvtWaited)== 0U){
		ENTER_CRITICAL_SECTION(); // UTIL_SEQ_ENTER_CRITICAL_SECTION_IDLE() points to this ENTER_CRITICAL_SECTION()
		local_taskset = TaskSet;
		local_evtset = EvtSet;
		local_taskmask = TaskMask;
		if ((local_taskset & local_taskmask & SuperMask) == 0U){
			if ((local_evtset & EvtWaited)== 0U){
				// UTIL_SEQ_Idle( ); NO CODE IN FUNCTION
			}
		}
		EXIT_CRITICAL_SECTION(); // UTIL_SEQ_EXIT_CRITICAL_SECTION_IDLE() points to this EXIT_CRITICAL_SECTION()
	}
	SuperMask = super_mask_backup;
	return;
}

void UTIL_SEQ_WaitEvt(UTIL_SEQ_bm_t EvtId_bm)
{
	UTIL_SEQ_bm_t event_waited_id_backup;
	UTIL_SEQ_bm_t current_task_idx;
	UTIL_SEQ_bm_t wait_task_idx;

	current_task_idx = CurrentTaskIdx;
	if(UTIL_SEQ_NOTASKRUNNING == CurrentTaskIdx)
	{
		wait_task_idx = 0u;
	}
	else
	{
		wait_task_idx = (uint32_t)1u << CurrentTaskIdx;
	}

	/* backup the event id that was currently waited */
	event_waited_id_backup = EvtWaited;
	EvtWaited = EvtId_bm;

	while ((EvtSet & EvtId_bm) == 0U)
	{
		UTIL_SEQ_Run(~wait_task_idx); // UTIL_SEQ_EvtIdle()
	}

	CurrentTaskIdx = current_task_idx;

	ENTER_CRITICAL_SECTION( );

	EvtSet &= (~EvtId_bm);

	EXIT_CRITICAL_SECTION( );

	EvtWaited = event_waited_id_backup;
	return;
}

static void BLEStack_Process(void) {	// VALID
	BLE_STACK_Tick();					// NOTE: EXTERNAL!

	if(BLE_STACK_SleepCheck() == 0) {	// NOTE: EXTERNAL!
		UTIL_SEQ_SetTask(1U << CFG_TASK_BLE_STACK, CFG_SEQ_PRIO_1);
	}
}

uint8_t NVMDB_get_info(NVMDB_info* info) {  // VALID
	uint32_t ptr = info->start_address;
	NVMDB_record_t* record;
	info->valid_records = 0;
	info->invalid_records = 0;
	info->free_space = 0;
	info->locked = 0;
	while (1) {
		record = (void*)ptr;
		if (record->header.valid_flag == 0xFFU) {  // NO record
			info->free_space = info->end_address - ptr - 4;
			return 0;
		}
		if (record->header.valid_flag == 0xFEU) {  // valid
			info->valid_records++;
		} else if (record->header.valid_flag == 0x00U) {  // invalid
			info->invalid_records++;
		} else {
			return 3;
		}
		ptr += (((record->header.length + 4) - 1) | 3) + 1;  // round to 4
		if(ptr + 5 >= info->end_address) {  // end
			return 0;
		}
	}
}

void NVMDB_init(void) {  // VALID
// Checks DB consistency. Reads number of records.

	uint8_t status;
	uint32_t page_address, offset;
	uint16_t clean_threshold;
	uint8_t id;

	// NOTE NO SMALL DBs

	/* Parse large DBs. */
	for(uint8_t id = 0; id < 2; id++) {
		DBInfo[id].start_address = NVM_BASE + (id * NVM_PAGE);
		DBInfo[id].end_address = DBInfo[id].start_address + (NVM_BASE - (id * 8));

		status = NVMDB_get_info(&DBInfo[id]);
		if(status) { return; }
	}
}

uint8_t NVMDB_handle_init(uint8_t NVMDB_id, NVMDB_HandleType *handle_p) {
	if(NVMDB_id >= 2) { return 1; }
	handle_p->address = DBInfo[NVMDB_id].start_address;
	handle_p->end_address = DBInfo[NVMDB_id].end_address;
	handle_p->first_read = 1;
	handle_p->id = NVMDB_id;
	handle_p->cache = 0;
	return 0;
}

tBleStatus aci_gatt_srv_profile_init(uint8_t Characteristics,
									 uint16_t *Service_Changed_Handle)
{
	tBleStatus ret;
	uint16_t gatt_srvc_handle;

	*Service_Changed_Handle = 0x0000;

	ret = aci_gatt_srv_add_service_api(&gatt_srvc);
	if (ret != BLE_STATUS_SUCCESS) { for(;;); }

	gatt_srvc_handle = aci_gatt_srv_get_service_handle_api(&gatt_srvc);

	if(Characteristics & GATT_INIT_SERVICE_CHANGED_BIT)
	{
		ret = aci_gatt_srv_add_char_api((ble_gatt_chr_def_t*)&gatt_srvc_changed_chr, gatt_srvc_handle);
		if (ret != BLE_STATUS_SUCCESS) { for(;;); }
	}
	*Service_Changed_Handle = aci_gatt_srv_get_char_decl_handle_api((ble_gatt_chr_def_t*)&gatt_srvc_changed_chr);

	ret = aci_gatt_srv_add_char_api((ble_gatt_chr_def_t*)&gatt_clt_supp_feat_chr, gatt_srvc_handle);
	if (ret != BLE_STATUS_SUCCESS) { for(;;); }

	ret = aci_gatt_srv_add_char_api((ble_gatt_chr_def_t*)&gatt_db_hash_chr, gatt_srvc_handle);
	if (ret != BLE_STATUS_SUCCESS) { for(;;); }

	ret = aci_gatt_srv_add_char_api((ble_gatt_chr_def_t*)&gatt_srv_supp_feat_chr, gatt_srvc_handle);
	if (ret != BLE_STATUS_SUCCESS) { for(;;); }

	return   BLE_STATUS_SUCCESS;
}


tBleStatus aci_gap_profile_init(uint8_t Role,
								uint8_t Privacy_Type,
								uint16_t *Dev_Name_Char_Handle,
								uint16_t *Appearance_Char_Handle,
								uint16_t *Periph_Pref_Conn_Param_Char_Handle)
{
	tBleStatus ret;
	uint16_t gap_srvc_handle;

	*Dev_Name_Char_Handle = 0x0000;
	*Appearance_Char_Handle= 0x0000;
	*Periph_Pref_Conn_Param_Char_Handle = 0x0000;

	if ((Role & 0x5) != 0x0U) {
		ret = aci_gatt_srv_add_service_api(&gap_srvc);
		if (ret != BLE_STATUS_SUCCESS) { for(;;); }
		*Dev_Name_Char_Handle = aci_gatt_srv_get_char_decl_handle_api(&gap_chrs[0U]);
		*Appearance_Char_Handle = aci_gatt_srv_get_char_decl_handle_api(&gap_chrs[1U]);
		gap_srvc_handle = aci_gatt_srv_get_service_handle_api(&gap_srvc);
		if ((Role & 1) != 0x0U) {
			ret = aci_gatt_srv_add_char_api(&gap_chrs[2U], gap_srvc_handle);
			if (ret != BLE_STATUS_SUCCESS) { for(;;); }
		}
		*Periph_Pref_Conn_Param_Char_Handle = aci_gatt_srv_get_char_decl_handle_api(&gap_chrs[2U]);
		if (Privacy_Type == 2U) {
			ret = aci_gatt_srv_add_char_api(&gap_chrs[3U], gap_srvc_handle);
			if (ret != BLE_STATUS_SUCCESS) { for(;;); }
		}
		// TODO: needed??Gap_profile_set_dev_name(0U, sizeof(gap_device_name_buff), (uint8_t *)gap_device_name_buff);
	}
	return BLE_STATUS_SUCCESS;
}

tBleStatus Gap_profile_set_char_value(uint16_t attr_h, uint16_t val_offset, uint16_t val_length, uint8_t *val_p) {
	uint8_t i;
	uint16_t handle;
	for (i = 0U; i < (sizeof(gap_chrs) / sizeof(gap_chrs[0U])); i++) {
		handle = aci_gatt_srv_get_char_decl_handle_api(&gap_chrs[i]);
		if ((handle != 0x0000) &&
			((handle + 1U) == attr_h)) {
			break;
		}
	}
	if (i == (sizeof(gap_chrs) / sizeof(gap_chrs[0U]))) { for(;;); }
	if ((val_offset + val_length) > gap_chrs[i].val_buffer_p->buffer_len) { for(;;); }
	memcpy(&gap_chrs[i].val_buffer_p->buffer_p[val_offset], val_p, val_length);
	if ((gap_chrs[i].val_buffer_p->op_flags & 2) != 0U) {
		gap_chrs[i].val_buffer_p->val_len = val_length;
	}

	return BLE_STATUS_SUCCESS;
}

tBleStatus Gap_profile_set_dev_name(uint16_t offset, uint16_t length, uint8_t *dev_name_p) {
	uint16_t handle = aci_gatt_srv_get_char_decl_handle_api(&gap_chrs[0U]) + 1U;
	return Gap_profile_set_char_value(handle, offset, length, dev_name_p);
}

tBleStatus Gap_profile_set_appearance(uint16_t offset, uint16_t length, uint8_t *appearance_p) {
	uint16_t handle = aci_gatt_srv_get_char_decl_handle_api(&gap_chrs[1U]) + 1U;
	return Gap_profile_set_char_value(handle, offset, length, appearance_p);
}

tBleStatus Gap_profile_set_pref_conn_par(uint16_t offset, uint16_t length, uint8_t *pref_conn_param_p) {
	uint16_t handle = aci_gatt_srv_get_char_decl_handle_api(&gap_chrs[2U]) + 1U;
	return Gap_profile_set_char_value(handle, offset, length, pref_conn_param_p);
}

static void fill_advData(uint8_t *p_adv_data, uint8_t tab_size, const uint8_t* p_bd_addr)
{
	uint16_t i =0;
	uint8_t bd_addr_1, bd_addr_0;
	uint8_t ad_length, ad_type;

	while(i < tab_size)
	{
		ad_length = p_adv_data[i];
		ad_type = p_adv_data[i + 1];

		switch (ad_type)
		{
			case AD_TYPE_FLAGS:
				break;
			case AD_TYPE_TX_POWER_LEVEL:
				break;
			case AD_TYPE_COMPLETE_LOCAL_NAME:
			{
				if((p_adv_data[i + ad_length] == 'X') && (p_adv_data[i + ad_length - 1] == 'X')) {
					bd_addr_1 = ((p_bd_addr[0] & 0xF0)>>4);
					bd_addr_0 = (p_bd_addr[0] & 0xF);

					/* Convert hex value into ascii */
					if(bd_addr_1 > 0x09) {
						p_adv_data[i + ad_length - 1] = bd_addr_1 + '7';
					}
					else {
						p_adv_data[i + ad_length - 1] = bd_addr_1 + '0';
					}

					if(bd_addr_0 > 0x09) {
						p_adv_data[i + ad_length] = bd_addr_0 + '7';
					}
					else {
						p_adv_data[i + ad_length] = bd_addr_0 + '0';
					}
				}
				break;
			}
			case AD_TYPE_MANUFACTURER_SPECIFIC_DATA:
			{
				p_adv_data[i+2] = ST_MANUF_ID_LSB;
				p_adv_data[i+3] = ST_MANUF_ID_MSB;
				p_adv_data[i+4] = BLUESTSDK_V2; /* blueST SDK version */
				p_adv_data[i+5] = BOARD_ID_NUCLEO_WB0; /* Board ID */
				p_adv_data[i+6] = FW_ID_P2P_SERVER; /* FW ID */
				p_adv_data[i+7] = 0x00; /* FW data 1 */
				p_adv_data[i+8] = 0x00; /* FW data 2 */
				p_adv_data[i+9] = 0x00; /* FW data 3 */
				p_adv_data[i+10] = p_bd_addr[5]; /* MSB BD address */
				p_adv_data[i+11] = p_bd_addr[4];
				p_adv_data[i+12] = p_bd_addr[3];
				p_adv_data[i+13] = p_bd_addr[2];
				p_adv_data[i+14] = p_bd_addr[1];
				p_adv_data[i+15] = p_bd_addr[0]; /* LSB BD address */
				break;
			}
			default:
				break;
		}
		i += ad_length + 1; /* increment the iterator to go on next element*/
	}
}

void BLE_Init() {  // TODO: MARIJN <
	uint8_t role;
	uint8_t privacy_type = 0;
	tBleStatus ret;
	uint16_t gatt_service_changed_handle;
	uint16_t gap_dev_name_char_handle;
	uint16_t gap_appearance_char_handle;
	uint16_t gap_periph_pref_conn_param_char_handle;
	uint8_t bd_address[6] = {0};
	uint8_t bd_address_len= 6;
	uint16_t appearance = 0x0000;

	BLE_STACK_InitTypeDef BLE_STACK_InitParams = {
		.BLEStartRamAddress = (uint8_t*)dyn_alloc_a,
		.TotalBufferSize = 9246,
		.NumAttrRecords = 6,
		.MaxNumOfClientProcs = 2,
		.NumOfRadioTasks = 2,
		.NumOfEATTChannels = 0,
		.NumBlockCount = 30,
		.ATT_MTU = 247,
		.MaxConnEventLength = 0xFFFFFFFFUL,
		.SleepClockAccuracy = 100,
		.NumOfAdvDataSet = 1,
		.NumOfSubeventsPAwR = 16,
		.MaxPAwRSubeventDataCount = 8,
		.NumOfAuxScanSlots = 0,
		.FilterAcceptListSizeLog2 = 3,
		.L2CAP_MPS = 247,
		.L2CAP_NumChannels = 1,
		.NumOfSyncSlots = 0,
		.CTE_MaxNumAntennaIDs = 0,
		.CTE_MaxNumIQSamples = 0,
		.NumOfSyncBIG = 1,
		.NumOfBrcBIG = 1,
		.NumOfSyncBIS = 2,
		.NumOfBrcBIS = 2,
		.NumOfCIG = 2,
		.NumOfCIS = 2,
		.isr0_fifo_size = 256,
		.isr1_fifo_size = 768,
		.user_fifo_size = 1024
	};

	/* Bluetooth LE stack init */
	ret = BLE_STACK_Init(&BLE_STACK_InitParams);  // EXTERNAL
	if (ret != BLE_STATUS_SUCCESS) { for(;;); }
	aci_hal_set_tx_power_level(0, CFG_TX_POWER);
	aci_gatt_srv_profile_init(0x1, &gatt_service_changed_handle); // MICK
	aci_gap_init(privacy_type, 0x1);
	aci_gap_profile_init(0x1, privacy_type,
						 &gap_dev_name_char_handle,
						 &gap_appearance_char_handle,
						 &gap_periph_pref_conn_param_char_handle);
	aci_hal_read_config_data(0x80, &bd_address_len, bd_address);
	Gap_profile_set_dev_name(0, sizeof(a_GapDeviceName), (uint8_t*)a_GapDeviceName);
	Gap_profile_set_appearance(0, sizeof(appearance), (uint8_t*)&appearance);
	hci_le_set_default_phy_api(0x00, 2, 2);
	bleAppContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = 1;
	aci_gap_set_io_capability_api(bleAppContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);
	bleAppContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode             = 1;
	bleAppContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin  = 8;
	bleAppContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax  = 16;
	bleAppContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode          = 1;
	fill_advData(&a_AdvData[0], sizeof(a_AdvData), bd_address);
	aci_gap_set_security_requirements_api(bleAppContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
									  bleAppContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
									  1,
									  0,
									  bleAppContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
									  bleAppContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
									  0);// MARIJN
	if (bleAppContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode) {
		aci_gap_configure_filter_accept_and_resolving_list(0x01);
	}
}

void BLEStack_Process_Schedule(void){
	UTIL_SEQ_SetTask( 1U << CFG_TASK_BLE_STACK, CFG_SEQ_PRIO_1);
}

void APP_BLE_Procedure_Gap_Peripheral(ProcGapPeripheralId_t ProcGapPeripheralId)
{
	tBleStatus status;
	uint32_t paramA = ADV_INTERVAL_MIN;
	uint32_t paramB = ADV_INTERVAL_MAX;
	uint32_t paramC, paramD;

	/* First set parameters before calling ACI APIs, only if needed */
	switch(ProcGapPeripheralId) {
		case PROC_GAP_PERIPH_ADVERTISE_START_FAST: {
			paramA = ADV_INTERVAL_MIN;
			paramB = ADV_INTERVAL_MAX;
			paramC = APP_BLE_ADV_FAST;
			break;
		}/* PROC_GAP_PERIPH_ADVERTISE_START_FAST */
		case PROC_GAP_PERIPH_ADVERTISE_START_LP: {
			paramA = ADV_LP_INTERVAL_MIN;
			paramB = ADV_LP_INTERVAL_MAX;
			paramC = APP_BLE_ADV_LP;
			break;
		}
		case PROC_GAP_PERIPH_ADVERTISE_STOP: {
			paramC = APP_BLE_IDLE;
			break;
		}
		case PROC_GAP_PERIPH_CONN_PARAM_UPDATE: {
			paramA = CONN_INT_MS(1000);
			paramB = CONN_INT_MS(1000);
			paramC = 0x0000;
			paramD = 0x01F4;
			if (bleAppContext.connIntervalFlag != 0) {
				bleAppContext.connIntervalFlag = 0;
				paramA = CONN_INT_MS(50);
				paramB = CONN_INT_MS(50);
			}
			else {
				bleAppContext.connIntervalFlag = 1;
				paramA = CONN_INT_MS(1000);
				paramB = CONN_INT_MS(1000);
			}
			break;
		}/* PROC_GAP_PERIPH_CONN_PARAM_UPDATE */
		case PROC_GAP_PERIPH_CONN_TERMINATE: {
			status = aci_gap_terminate_api(bleAppContext.BleApplicationContext_legacy.connectionHandle, 0x13); // NOTE: EXTERNAL!
			if (status == BLE_STATUS_SUCCESS) {
				UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_PROC_GAP_COMPLETE); /* gap_cmd_resp_wait(): waiting for HCI_DISCONNECTION_COMPLETE_EVT_CODE */
			}
			break;
		}
		default:
			break;
	}

	/* Call ACI APIs */
	switch(ProcGapPeripheralId) {
		case PROC_GAP_PERIPH_ADVERTISE_START_FAST:
		case PROC_GAP_PERIPH_ADVERTISE_START_LP: {
			Advertising_Set_Parameters_t Advertising_Set_Parameters = {0};
			status = aci_gap_set_advertising_configuration(0,
														   GAP_MODE_GENERAL_DISCOVERABLE,
														   ADV_TYPE,
														   paramA,
														   paramB,
														   HCI_ADV_CH_ALL,
														   0,
														   NULL, /* No peer address */
														   HCI_ADV_FILTER_NONE,
														   0, /* 0 dBm */
														   HCI_PHY_LE_1M, /* Primary advertising PHY */
														   0, /* 0 skips */
														   HCI_PHY_LE_1M, /* Secondary advertising PHY. Not used with legacy advertising. */
														   0, /* SID */
														   0 /* No scan request notifications */);
			if (status == BLE_STATUS_SUCCESS) {
				bleAppContext.Device_Connection_Status = (APP_BLE_ConnStatus_t)paramC;
			}
			status = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(a_AdvData), (uint8_t*) a_AdvData);
			//REMOVED UNUSED DEBUG IF ELSE
			/* Enable advertising */
			status = aci_gap_set_advertising_enable(1, 1, &Advertising_Set_Parameters);
			//REMOVED UNUSED DEBUG IF ELSE
			break;
		}
		case PROC_GAP_PERIPH_ADVERTISE_STOP: {
			status = aci_gap_set_advertising_enable(1, 0, NULL);
			if (status != BLE_STATUS_SUCCESS) {
				bleAppContext.Device_Connection_Status = (APP_BLE_ConnStatus_t)paramC;
			}
			break;
		}/* PROC_GAP_PERIPH_ADVERTISE_STOP */
		case PROC_GAP_PERIPH_ADVERTISE_DATA_UPDATE: {
			Advertising_Set_Parameters_t Advertising_Set_Parameters = {0};
			/* Disable advertising */ //TBR??? Do we need to disable advertising, set advertising data and then enable advertising?
			status = aci_gap_set_advertising_enable(0, 0, NULL);
			if (status != BLE_STATUS_SUCCESS) {
				bleAppContext.Device_Connection_Status = (APP_BLE_ConnStatus_t)paramC;
			}
			/* Set advertising data */
			status = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(a_AdvData), (uint8_t*) a_AdvData);
			//REMOVED UNUSED DEBUG IF ELSE
			/* Enable advertising */
			status = aci_gap_set_advertising_enable(1, 1, &Advertising_Set_Parameters);
			//REMOVED UNUSED DEBUG IF ELSE
			break;
		}/* PROC_GAP_PERIPH_ADVERTISE_DATA_UPDATE */
		case PROC_GAP_PERIPH_CONN_PARAM_UPDATE: {
			status = aci_l2cap_connection_parameter_update_req_api(
					bleAppContext.BleApplicationContext_legacy.connectionHandle,
					paramA,
					paramB,
					paramC,
					paramD);
			//REMOVED UNUSED DEBUG IF ELSE
			break;
		}/* PROC_GAP_PERIPH_CONN_PARAM_UPDATE */
		case PROC_GAP_PERIPH_SET_BROADCAST_MODE: {
			break;
		}/* PROC_GAP_PERIPH_SET_BROADCAST_MODE */
		default:
			break;
	}
	return;
}

static void Adv_Cancel(void)
{
	GPIO_write(GPIOB, 4, 0); // BSP_LED_Off

	APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_STOP);
	bleAppContext.Device_Connection_Status = APP_BLE_IDLE;
}

tBleStatus P2P_SERVER_NotifyValue(P2P_SERVER_CharOpcode_t CharOpcode, P2P_SERVER_Data_t *pData, uint16_t ConnectionHandle)
{
	tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

	switch(CharOpcode)
	{

		case P2P_SERVER_SWITCH_C:
			memcpy(switch_c_val_buffer, pData->p_Payload, MIN(pData->Length, sizeof(switch_c_val_buffer))); // TODO: CHECK WITH MARIJN WHAT DO THIS "string.h"
			ret = aci_gatt_srv_notify_api(ConnectionHandle,				// aci_gatt_srv_notify()
									  BLE_GATT_UNENHANCED_ATT_L2CAP_CID,
									  P2P_SERVER_Context.Switch_CCharHdle + 1,
									  GATT_NOTIFICATION,
									  pData->Length, /* charValueLen */
									  (uint8_t *)pData->p_Payload);			// NOTE! EXTERNAL
			// REMOVED UNUSED DEBUG IF ELSE
			break;

		default:
			break;
	}

	return ret;
}

void P2P_SERVER_Switch_c_SendNotification(void) /* Property Notification */
{
	P2P_SERVER_APP_SendInformation_t notification_on_off = Switch_c_NOTIFICATION_OFF;
	P2P_SERVER_Data_t p2p_server_notification_data;

	p2p_server_notification_data.p_Payload = (uint8_t*)a_P2P_SERVER_UpdateCharData;
	p2p_server_notification_data.Length = 0;

	if(P2P_SERVER_APP_Context.ButtonControl.ButtonStatus == 0x00)
	{
		P2P_SERVER_APP_Context.ButtonControl.ButtonStatus = 0x01;
	}
	else
	{
		P2P_SERVER_APP_Context.ButtonControl.ButtonStatus = 0x00;
	}
	a_P2P_SERVER_UpdateCharData[0] = 0x01; /* Device Led selection */
	a_P2P_SERVER_UpdateCharData[1] = P2P_SERVER_APP_Context.ButtonControl.ButtonStatus;
	/* Update notification data length */
	p2p_server_notification_data.Length = (p2p_server_notification_data.Length) + 2;

	if(P2P_SERVER_APP_Context.Switch_c_Notification_Status == Switch_c_NOTIFICATION_ON)
	{
		notification_on_off = Switch_c_NOTIFICATION_ON;
	}

	if (notification_on_off != Switch_c_NOTIFICATION_OFF && P2P_SERVER_APP_Context.ConnectionHandle != 0xFFFF)
	{
		P2P_SERVER_NotifyValue(P2P_SERVER_SWITCH_C, &p2p_server_notification_data, P2P_SERVER_APP_Context.ConnectionHandle);
	}
}

void P2P_SERVER_APP_LED_BUTTON_context_Init(void)
{
	GPIO_write(GPIOB, 0, 0);
	P2P_SERVER_APP_Context.LedControl.Device_Led_Selection=0x01; /* Device1 */
	P2P_SERVER_APP_Context.LedControl.Led1=0x00; /* led OFF */
	P2P_SERVER_APP_Context.ButtonControl.Device_Button_Selection=0x01;/* Device1 */
	P2P_SERVER_APP_Context.ButtonControl.ButtonStatus=0x00;

	return;
}

int BLEEVT_RegisterGattEvtHandler(BLEEVT_GattEvtHandlerFunc_t EvtHandlerFunc)
{
	if(BLEEVT_GattEvtHandler.NbrOfRegisteredHandlers == 1)
	{
		return -1;
	}

	//REMOVED IF CUZ DEFINE WAS 1
	BLEEVT_GattEvtHandler.BLEEVT_SvcHandlerTab[BLEEVT_GattEvtHandler.NbrOfRegisteredHandlers] = EvtHandlerFunc;
	BLEEVT_GattEvtHandler.NbrOfRegisteredHandlers++;

	return 0;
}

void P2P_SERVER_Notification(P2P_SERVER_NotificationEvt_t *p_Notification)
{
	switch(p_Notification->EvtOpcode)
	{
		case P2P_SERVER_LED_C_READ_EVT:
			break;

		case P2P_SERVER_LED_C_WRITE_NO_RESP_EVT:
			if(p_Notification->DataTransfered.p_Payload[1] == 0x01)
			{
				// BSP_LED_On(LED_BLUE);
				P2P_SERVER_APP_Context.LedControl.Led1 = 0x01; /* LED1 ON */
			}
			if(p_Notification->DataTransfered.p_Payload[1] == 0x00)
			{
				// BSP_LED_Off(LED_BLUE);
				P2P_SERVER_APP_Context.LedControl.Led1 = 0x00; /* LED1 OFF */
			}
			break;

		case P2P_SERVER_SWITCH_C_NOTIFY_ENABLED_EVT:
			P2P_SERVER_APP_Context.Switch_c_Notification_Status = Switch_c_NOTIFICATION_ON;
			break;

		case P2P_SERVER_SWITCH_C_NOTIFY_DISABLED_EVT:
			P2P_SERVER_APP_Context.Switch_c_Notification_Status = Switch_c_NOTIFICATION_OFF;
			break;

		default:
			break;
	}
	return;
}

static BLEEVT_EvtAckStatus_t P2P_SERVER_EventHandler(aci_blecore_event *p_evt)
{
	BLEEVT_EvtAckStatus_t return_value = BLEEVT_NoAck;
	aci_gatt_srv_attribute_modified_event_rp0 *p_attribute_modified;
	aci_gatt_srv_write_event_rp0   *p_write;
	aci_gatt_srv_read_event_rp0    *p_read;
	P2P_SERVER_NotificationEvt_t notification;

	switch(p_evt->ecode)
	{
		case ACI_GATT_SRV_ATTRIBUTE_MODIFIED_VSEVT_CODE:
		{

			p_attribute_modified = (aci_gatt_srv_attribute_modified_event_rp0*)p_evt->data;
			notification.ConnectionHandle         = p_attribute_modified->Connection_Handle;
			notification.AttributeHandle          = p_attribute_modified->Attr_Handle;
			notification.DataTransfered.Length    = p_attribute_modified->Attr_Data_Length;
			notification.DataTransfered.p_Payload = p_attribute_modified->Attr_Data;
			if(p_attribute_modified->Attr_Handle == (P2P_SERVER_Context.Switch_CCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
			{
				return_value = BLEEVT_Ack;

				switch(p_attribute_modified->Attr_Data[0])
				{


					/* Disabled Notification management */
					case (!BLE_GATT_SRV_CCCD_NOTIFICATION):

						notification.EvtOpcode = P2P_SERVER_SWITCH_C_NOTIFY_DISABLED_EVT;
						P2P_SERVER_Notification(&notification);

						break;

						/* Enabled Notification management */
					case BLE_GATT_SRV_CCCD_NOTIFICATION:

						notification.EvtOpcode = P2P_SERVER_SWITCH_C_NOTIFY_ENABLED_EVT;
						P2P_SERVER_Notification(&notification);

						break;

					default:

						break;
				}
			}  /* if(p_attribute_modified->Attr_Handle == (P2P_SERVER_Context.Switch_CCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

			else if(p_attribute_modified->Attr_Handle == (P2P_SERVER_Context.Led_CCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
			{
				return_value = BLEEVT_Ack;

				notification.EvtOpcode = P2P_SERVER_LED_C_WRITE_NO_RESP_EVT;
				notification.DataTransfered.Length = p_attribute_modified->Attr_Data_Length;
				notification.DataTransfered.p_Payload = p_attribute_modified->Attr_Data;
				P2P_SERVER_Notification(&notification);
			} /* if(p_attribute_modified->Attr_Handle == (P2P_SERVER_Context.Led_CCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/

			break;
		}
		case ACI_GATT_SRV_READ_VSEVT_CODE :
		{
			p_read = (aci_gatt_srv_read_event_rp0*)p_evt->data;
			if(p_read->Attribute_Handle == (P2P_SERVER_Context.Led_CCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
			{
				return_value = BLEEVT_Ack;
			} /* if(p_read->Attribute_Handle == (P2P_SERVER_Context.Led_CCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/

			break;
		}
		case ACI_GATT_SRV_WRITE_VSEVT_CODE:
		{
			p_write = (aci_gatt_srv_write_event_rp0*)p_evt->data;
			if(p_write->Attribute_Handle == (P2P_SERVER_Context.Led_CCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
			{
				return_value = BLEEVT_Ack;
			} /*if(p_write->Attribute_Handle == (P2P_SERVER_Context.Led_CCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/

			break;
		}
		case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
		{
			aci_gatt_tx_pool_available_event_rp0 *p_tx_pool_available_event;
			p_tx_pool_available_event = (aci_gatt_tx_pool_available_event_rp0 *) p_evt->data;
			(void)(p_tx_pool_available_event);

			break;
		}
		case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
		{
			aci_att_exchange_mtu_resp_event_rp0 *p_exchange_mtu;
			p_exchange_mtu = (aci_att_exchange_mtu_resp_event_rp0 *)  p_evt->data;
			(void)(p_exchange_mtu);

			break;
		}
			/* Manage ACI_GATT_INDICATION_VSEVT_CODE occurring on Android 12 */
		case ACI_GATT_CLT_INDICATION_VSEVT_CODE:
		{
			tBleStatus status = BLE_STATUS_FAILED;
			aci_gatt_clt_indication_event_rp0 *pr = (void*)p_evt->data;
			status = aci_gatt_clt_confirm_indication_api(pr->Connection_Handle, BLE_GATT_UNENHANCED_ATT_L2CAP_CID);
			if (status != BLE_STATUS_SUCCESS) { for(;;); }
		}
			break;
		default:
			break;
	}

	/* USER CODE BEGIN Service1_EventHandler_2 */

	/* USER CODE END Service1_EventHandler_2 */

	return(return_value);
}




void P2P_SERVER_Init(void)
{
	tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
	(void)P2P_SERVER_Context;

	/**
	 *  Register the event handler to the BLE controller
	 */
	BLEEVT_RegisterGattEvtHandler(P2P_SERVER_EventHandler);

	ret = aci_gatt_srv_add_service_api((ble_gatt_srv_def_t *)&p2p_server_service); //NOTE! EXTERNAL

	P2P_SERVER_Context.P2p_serverSvcHdle = aci_gatt_srv_get_service_handle_api((ble_gatt_srv_def_t *) &p2p_server_service);
	P2P_SERVER_Context.Led_CCharHdle = aci_gatt_srv_get_char_decl_handle_api((ble_gatt_chr_def_t *)&p2p_server_chars[0]);
	P2P_SERVER_Context.Switch_CCharHdle = aci_gatt_srv_get_char_decl_handle_api((ble_gatt_chr_def_t *)&p2p_server_chars[1]);
}

void P2P_SERVER_APP_Init(void)
{
	P2P_SERVER_APP_Context.ConnectionHandle = 0xFFFF;
	P2P_SERVER_Init(); //TODO: QUINTEN

	UTIL_SEQ_RegTask( 1U << CFG_TASK_SEND_NOTIF_ID, UTIL_SEQ_RFU, P2P_SERVER_Switch_c_SendNotification);

	/**
	 * Initialize LedButton Service
	 */
	P2P_SERVER_APP_Context.Switch_c_Notification_Status= Switch_c_NOTIFICATION_OFF;
	P2P_SERVER_APP_LED_BUTTON_context_Init();
}

static void Adv_Cancel_Req(void *arg)
{
	UTIL_SEQ_SetTask(1 << CFG_TASK_ADV_CANCEL_ID, CFG_SEQ_PRIO_0);
}

static void Switch_OFF_GPIO(void *arg)
{
	GPIO_write(GPIOB, 4, 0);
}


//
void APP_BLE_Init(void) { // VALID
	UTIL_SEQ_RegTask(1U << CFG_TASK_BLE_STACK, UTIL_SEQ_RFU, BLEStack_Process);
	UTIL_SEQ_RegTask(1U << CFG_TASK_VTIMER, UTIL_SEQ_RFU, RADIO_TIMER_Tick);
	// UTIL_SEQ_RegTask(1U << CFG_TASK_NVM, UTIL_SEQ_RFU, NVMDB_tick);  // NOTE this function is empty so log as AUTO_CLEAN == 0

	// modules init()
	NVMDB_init();
	NVMDB_handle_init(0, &sec_gatt_db_h);
	NVMDB_handle_init(1, &device_id_db_h);
	curr_handle_p = &sec_gatt_db_h;
	// ~modules init()
	internal_state = 1;

	BLE_Init();

	BLEStack_Process_Schedule();

	bleAppContext.Device_Connection_Status = APP_BLE_IDLE;
	bleAppContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;

	UTIL_SEQ_RegTask(1<<CFG_TASK_ADV_CANCEL_ID, UTIL_SEQ_RFU, Adv_Cancel);

	/* Create timer to handle the Advertising Stop */
	bleAppContext.Advertising_mgr_timer_Id.callback = Adv_Cancel_Req;

	/* Create timer to handle the Led Switch OFF */
	bleAppContext.SwitchOffGPIO_timer_Id.callback = Switch_OFF_GPIO;

	/**
	* Initialize Services and Characteristics.
	*/
	P2P_SERVER_APP_Init();

	if (aci_hal_set_radio_activity_mask(0x0006)) { for(;;); }
	/* Start to Advertise to accept a connection */
	APP_BLE_Procedure_Gap_Peripheral(PROC_GAP_PERIPH_ADVERTISE_START_FAST);
	//HAL_RADIO_TIMER_StartVirtualTimer(&bleAppContext.Advertising_mgr_timer_Id, ADV_TIMEOUT_MS);

	bleAppContext.connIntervalFlag = 0;
}

void MX_APPE_Init(void) {
	// TODO USART init?
	// TODO RNG init?
	// TODO AES init?
	// TODO PKA init?

	APP_BLE_Init();
	// TODO: LPM_init(); ?

}

void MX_APPE_Process(void) {
	UTIL_SEQ_Run(~0);
}





