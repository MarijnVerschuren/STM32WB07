t = """
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: CMakeFiles/STM32WB07.dir/src/RF/bleplat.c.obj: in function `BLEPLAT_MemCpy':
/home/marijn/Github/STM32WB07/src/RF/bleplat.c:26: undefined reference to `Osal_MemCpy'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(aci_hal.o): in function `HAL_Set_TX_Power_Level':
(.text.HAL_Set_TX_Power_Level+0x8): undefined reference to `BLEPLAT_GetMaxPALevel'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.HAL_Set_TX_Power_Level+0x24): undefined reference to `BLEPLAT_SetHighPower'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(bluenrg_main_func.o): in function `_BlueNRG_Stack_Initialization':
(.text._BlueNRG_Stack_Initialization+0x22): undefined reference to `BLEPLAT_CNTR_ClearInterrupt'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text._BlueNRG_Stack_Initialization+0xae): undefined reference to `BLEPLAT_GetPartInfo'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(gat_srv.o): in function `GAT_srv_db_hash_tsk':
(.text.GAT_srv_db_hash_tsk+0x2a): undefined reference to `BLEPLAT_AesCMACEncryptInit'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.GAT_srv_db_hash_tsk+0xdc): undefined reference to `BLEPLAT_AesCMACEncryptAppend'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.GAT_srv_db_hash_tsk+0xe8): undefined reference to `BLEPLAT_AesCMACEncryptFinish'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(gat_srv.o): in function `GAT_srv_aes_cmac_append':
(.text.GAT_srv_aes_cmac_append+0x28): undefined reference to `BLEPLAT_AesCMACEncryptAppend'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(llc_cig_cmn_sap.o): in function `llc_cig_cmn_get_enc_keys':
(.text.llc_cig_cmn_get_enc_keys+0x1e): undefined reference to `BLEPLAT_CNTR_SmGetEncIvPtr'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.llc_cig_cmn_get_enc_keys+0x52): undefined reference to `BLEPLAT_CNTR_SmGetEncKeyPtr'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(llc_connect_sap.o): in function `llc_conn_set_blue_packets':
(.text.llc_conn_set_blue_packets+0x22): undefined reference to `BLEPLAT_CNTR_PacketSetNsEn'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_cpf_cte.o): in function `LLC_connection_cte_request_enable':
(.text.LLC_connection_cte_request_enable+0x32): undefined reference to `BLEPLAT_CNTR_PacketGetCteSamplingEn'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_cpf_pcl.o): in function `LLC_pcl_update_avg_rssi':
(.text.LLC_pcl_update_avg_rssi+0x36): undefined reference to `BLEPLAT_UpdateAvgRSSI'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_cte_functions.o): in function `LLC_le_set_connectionless_cte_transmit_parameters':
(.text.LLC_le_set_connectionless_cte_transmit_parameters+0xa6): undefined reference to `BLEPLAT_AntIdxRemap'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_cte_functions.o): in function `LLC_set_antenna_pattern':
(.text.LLC_set_antenna_pattern+0xc): undefined reference to `BLEPLAT_AntIdxRemap'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_cte_functions.o): in function `llc_cte_timer_start':
(.text.llc_cte_timer_start+0x3c): undefined reference to `BLEPLAT_GetFutureSysTime64'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.llc_cte_timer_start+0x46): undefined reference to `BLEPLAT_StartTimer'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_dtm.o): in function `LL_dtm_rx_isr':
(.text.LL_dtm_rx_isr+0x14): undefined reference to `BLEPLAT_CNTR_IntGetIntStatusDone'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_ext_adv.o): in function `ADV_ISR':
(.text.ADV_ISR+0x1b2): undefined reference to `BLEPLAT_SetRadioCloseTimeout'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_ext_adv.o): in function `LL_eadv_EadvIsr':
(.text.LL_eadv_EadvIsr+0x14): undefined reference to `BLEPLAT_CNTR_IntGetIntStatusAnyHwError'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_ext_adv.o): in function `LL_eadv_EauxIsr':
(.text.LL_eadv_EauxIsr+0x26): undefined reference to `BLEPLAT_CNTR_IntGetIntStatusAnyHwError'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_routines.o): in function `LL_init_ucfg_weak':
(.text.LL_init_ucfg_weak+0x28e): undefined reference to `BLEPLAT_GetDefaultPALevel'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.LL_init_ucfg_weak+0x29c): undefined reference to `BLEPLAT_SetHighPower'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(ll_scheduler.o): in function `LL_sched_engine':
(.text.LL_sched_engine+0x2c6): undefined reference to `BLEPLAT_CNTR_ClearSemareq'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.LL_sched_engine+0x2f4): undefined reference to `BLEPLAT_CNTR_GlobWriteSlot'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.LL_sched_engine+0x346): undefined reference to `BLEPLAT_SetRadioCloseTimeout'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(sdb.o): in function `SDB_sap_write_peer_bonded_gatt_client_data':
(.text.SDB_sap_write_peer_bonded_gatt_client_data+0x26): undefined reference to `BLEPLAT_NvmCompare'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_nvm.o): in function `nvm_gatt_db_read_next_record':
(.text.nvm_gatt_db_read_next_record+0x12): undefined reference to `BLEPLAT_NvmGet'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_nvm.o): in function `nvm_gatt_db_read_full_current_record':
(.text.nvm_gatt_db_read_full_current_record+0xc): undefined reference to `BLEPLAT_NvmGet'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_nvm.o): in function `nvm_gatt_db_update_record':
(.text.nvm_gatt_db_update_record+0xe): undefined reference to `BLEPLAT_NvmAdd'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_nvm.o): in function `nvm_sec_db_read_next_record':
(.text.nvm_sec_db_read_next_record+0x10): undefined reference to `BLEPLAT_NvmGet'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_nvm.o): in function `nvm_sec_db_add_record':
(.text.nvm_sec_db_add_record+0xc): undefined reference to `BLEPLAT_NvmAdd'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_nvm.o): in function `nvm_sec_db_get_record_count':
(.text.nvm_sec_db_get_record_count+0x14): undefined reference to `BLEPLAT_NvmGet'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_nvm.o): in function `nvm_discard_all_records':
(.text.nvm_discard_all_records+0x4): undefined reference to `BLEPLAT_NvmDiscard'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_nvm.o): in function `nvm_discard_current_record':
(.text.nvm_discard_current_record+0x4): undefined reference to `BLEPLAT_NvmDiscard'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_nvm.o): in function `nvm_dev_id_data_get_record':
(.text.nvm_dev_id_data_get_record+0xe): undefined reference to `BLEPLAT_NvmGet'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_nvm.o): in function `nvm_dev_id_data_add_record':
(.text.nvm_dev_id_data_add_record+0xc): undefined reference to `BLEPLAT_NvmAdd'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(stacklib_timers.o): in function `Timer_StartFirstActive_sysT':
(.text.Timer_StartFirstActive_sysT+0x46): undefined reference to `BLEPLAT_StartTimer'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(aes128cmac.o): in function `AES_CMAC_Encrypt':
(.text.AES_CMAC_Encrypt+0x1e): undefined reference to `BLEPLAT_AesCMACEncryptInit'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.AES_CMAC_Encrypt+0x28): undefined reference to `BLEPLAT_AesCMACEncryptAppend'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.AES_CMAC_Encrypt+0x32): undefined reference to `BLEPLAT_AesCMACEncryptFinish'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(bluenrgx_aes.o): in function `BLEPLAT_aes128_encrypt':
(.text.BLEPLAT_aes128_encrypt+0x2): undefined reference to `BLEPLAT_AesEcbEncrypt'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(bluenrgx_rng.o): in function `Hal_Get_Random_Number':
(.text.Hal_Get_Random_Number+0x6): undefined reference to `BLEPLAT_RngGetRandom16'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(bluenrgx_rng.o): in function `Hal_Get_Random_Number32':
(.text.Hal_Get_Random_Number32+0x4): undefined reference to `BLEPLAT_RngGetRandom32'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(hci_encryption_pka.o): in function `hci_le_read_local_p256_public_key_api':
(.text.hci_le_read_local_p256_public_key_api+0x16): undefined reference to `BLEPLAT_RngGetRandom32'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.hci_le_read_local_p256_public_key_api+0x20): undefined reference to `BLEPLAT_PkaStartP256Key'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(hci_encryption_pka.o): in function `hci_le_generate_dhkey_api':
(.text.hci_le_generate_dhkey_api+0x32): undefined reference to `BLEPLAT_PkaStartDHkey'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(llc_connect_isr.o): in function `llc_conn_isr':
(.text.llc_conn_isr+0x12c): undefined reference to `BLEPLAT_CNTR_IntGetIntStatusTxOk'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.llc_conn_isr+0x27a): undefined reference to `BLEPLAT_CNTR_IntGetIntStatusEncErr'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.llc_conn_isr+0x4e4): undefined reference to `BLEPLAT_CNTR_SmGetMode'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.llc_conn_isr+0x516): undefined reference to `BLEPLAT_CNTR_SmGetEncKeyPtr'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.llc_conn_isr+0x52a): undefined reference to `BLEPLAT_CNTR_SmGetEncIvPtr'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.llc_conn_isr+0x872): undefined reference to `BLEPLAT_CNTR_SmSetTxCount'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: (.text.llc_conn_isr+0x87e): undefined reference to `BLEPLAT_CNTR_SmSetTxCountDirectionBit'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(llc_connect_isr.o): in function `llc_conn_decrement_enc_count_rx':
(.text.llc_conn_decrement_enc_count_rx+0x6): undefined reference to `BLEPLAT_CNTR_SmGetRxCount'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(llc_padv_wr_isr.o): in function `llc_padv_wr_isr_rx':
(.text.llc_padv_wr_isr_rx+0x60): undefined reference to `BLEPLAT_SetRadioCloseTimeout'
/usr/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: /home/marijn/Github/STM32WB07/lib/libble_stack.a(llc_big_sync_callback.o): in function `llc_big_sync_isr':
(.text.llc_big_sync_isr+0xa): undefined reference to `BLEPLAT_CNTR_IntGetIntStatusNoactiveError'
collect2: error: ld returned 1 exit status
make[3]: *** [CMakeFiles/STM32WB07.dir/build.make:430: STM32WB07] Error 1
make[2]: *** [CMakeFiles/Makefile2:89: CMakeFiles/STM32WB07.dir/all] Error 2
make[1]: *** [CMakeFiles/Makefile2:96: CMakeFiles/STM32WB07.dir/rule] Error 2
make: *** [Makefile:124: STM32WB07] Error 2
"""

if __name__ == "__main__":
	f = [l[l.find("`"):].replace("`", "'").replace("'", "") for l in t.split("\n") if "undefined reference to `" in l]
	s = set(f)
	d = []
	for _f in s:
		d.append((f.count(_f), _f))
	d.sort(key=lambda x: x[0], reverse=True)
	print("\n".join([f"{c}\t{f}" for c, f in d]))
	