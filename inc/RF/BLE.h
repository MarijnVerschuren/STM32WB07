//
// Created by quvx on 05/11/24.
//

#ifndef STM32WB07_BLE_H
#define STM32WB07_BLE_H

#include "types.h"
#include "RF/RF_TIMER.h"
#include "GPIO.h"
#include "string.h"

#define P2P_SERVER_UUID			0x8f,0xe5,0xb3,0xd5,0x2e,0x7f,0x4a,0x98,0x2a,0x48,0x7a,0xcc,0x40,0xfe,0x00,0x00
#define LED_C_UUID			0x19,0xed,0x82,0xae,0xed,0x21,0x4c,0x9d,0x41,0x45,0x22,0x8e,0x41,0xfe,0x00,0x00
#define SWITCH_C_UUID			0x19,0xed,0x82,0xae,0xed,0x21,0x4c,0x9d,0x41,0x45,0x22,0x8e,0x42,0xfe,0x00,0x00

#define BLE_GATT_SRV_CHAR_PROP_NOTIFY                   (0x10U)
#define BLE_GATT_SRV_CHAR_PROP_WRITE_NO_RESP            (0x04U)
#define GATT_NOTIFICATION                          (0x00)
#define BLE_GATT_UNENHANCED_ATT_L2CAP_CID               (0x0004)
#define BLE_ERROR_INVALID_HCI_CMD_PARAMS            ((tBleStatus)(0x12))
#define BLE_STATUS_INVALID_PARAMS           ((tBleStatus)(BLE_ERROR_INVALID_HCI_CMD_PARAMS))
#define BLE_STATUS_FAILED                   ((tBleStatus)(0x81))
#define UTIL_SEQ_CONF_PRIO_NBR  (2)
#define UTIL_SEQ_ALL_BIT_SET    (~0U)
#define AD_TYPE_FLAGS                           (0x01)
#define FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE   (0x02)
#define FLAG_BIT_BR_EDR_NOT_SUPPORTED           (0x04)
#define AD_TYPE_COMPLETE_LOCAL_NAME             (0x09)
#define AD_TYPE_MANUFACTURER_SPECIFIC_DATA      (0xFF)
#define AD_TYPE_TX_POWER_LEVEL                  (0x0A)
#define HCI_ADV_EVENT_PROP_LEGACY           0x0010U
#define HCI_ADV_EVENT_PROP_CONNECTABLE      0x0001U
#define HCI_ADV_EVENT_PROP_SCANNABLE        0x0002U
#define HCI_ADV_CH_ALL                      0x07U
#define HCI_ADV_FILTER_NONE                 0x00U
#define HCI_PHY_LE_1M                   	0x01
#define ADV_COMPLETE_DATA               	(0x03)
#define UTIL_SEQ_RFU 0
#define ADV_INTERVAL_MIN                    (0x0080)
#define ADV_INTERVAL_MAX                    (0x00A0)
#define ADV_LP_INTERVAL_MIN                 (0x0640)
#define ADV_LP_INTERVAL_MAX                 (0x0FA0)
#define CONN_INT_MS(x) 						((uint16_t)((x)/1.25f))
#define BLE_STATUS_SUCCESS                  ((tBleStatus)(0x00))
#define UTIL_SEQ_NOTASKRUNNING       		(0xFFFFFFFFU)
#define UTIL_SEQ_NO_BIT_SET     			(0U)
#define CFG_TX_POWER                        (0x18) /* 0x18 <=> 0 dBm */
#define ST_MANUF_ID_LSB         0x30
#define ST_MANUF_ID_MSB         0x00
#define  BLUESTSDK_V2           0x02
#define  BOARD_ID_NUCLEO_WB0    0x8D
#define  FW_ID_P2P_SERVER       0x83
#define GAP_MODE_GENERAL_DISCOVERABLE         2
#define ADV_TYPE                            (HCI_ADV_EVENT_PROP_LEGACY|HCI_ADV_EVENT_PROP_CONNECTABLE|HCI_ADV_EVENT_PROP_SCANNABLE)

#define MIN( x, y )          (((x)<(y))?(x):(y))

typedef enum {
	P2P_SERVER_LED_C,
	P2P_SERVER_SWITCH_C,
	P2P_SERVER_CHAROPCODE_LAST
} P2P_SERVER_CharOpcode_t;

typedef enum
{
	APP_BLE_IDLE,
	APP_BLE_LP_CONNECTING,
	APP_BLE_CONNECTED_SERVER,
	APP_BLE_CONNECTED_CLIENT,
	APP_BLE_ADV_FAST,
	APP_BLE_ADV_LP,
} APP_BLE_ConnStatus_t;

typedef enum
{
	CFG_TASK_BLE_STACK,
	CFG_TASK_VTIMER,
	CFG_TASK_NVM,
	TASK_BUTTON_1,
	TASK_BUTTON_2,
	TASK_BUTTON_3,
	CFG_TASK_ADV_CANCEL_ID,
	CFG_TASK_SEND_NOTIF_ID,
	CFG_TASK_NBR,
} CFG_Task_Id_t;

typedef enum {
	CFG_SEQ_PRIO_0,
	CFG_SEQ_PRIO_1,
	CFG_SEQ_PRIO_NBR
} CFG_SEQ_Prio_Id_t;

typedef struct
{
	uint32_t priority;
	uint32_t round_robin;
} UTIL_SEQ_Priority_t;

typedef struct {
	uint32_t start_address;
	uint32_t end_address;
	uint16_t valid_records;
	uint16_t invalid_records;
	uint16_t free_space;  // Free space at the end of last record. It is a real free space, not virtual. After a clean, the free space may increase. It takes also into account all the records in cache.
	uint8_t locked;
	uint16_t clean_threshold;
} NVMDB_info;

typedef struct {
	uint8_t valid_flag;
	uint8_t record_id;
	uint16_t length;
} NVMDB_header_t;

typedef struct {
	NVMDB_header_t header;
	uint8_t data[];
} NVMDB_record_t;

typedef struct  {
	uint8_t id;
	uint32_t address;
	uint32_t end_address;      // This info may also be retrieved from id.
	uint8_t first_read;
	uint8_t cache;             // If TRUE, the handle points in cache
	uint16_t cache_index;      // If cache is TRUE, cache_index is the index of the current write operation.
}NVMDB_HandleType;

typedef struct {
	uint8_t ioCapability;
	uint8_t mitm_mode;
	uint8_t bonding_mode;
	uint8_t encryptionKeySizeMin;
	uint8_t encryptionKeySizeMax;
	uint8_t initiateSecurity;
} SecurityParams_t;

typedef struct{
	SecurityParams_t bleSecurityParam;
	uint16_t gapServiceHandle;
	uint16_t devNameCharHandle;
	uint16_t appearanceCharHandle;
	uint16_t connectionHandle;
}BleGlobalContext_t;

typedef struct {
	BleGlobalContext_t BleApplicationContext_legacy;
	APP_BLE_ConnStatus_t Device_Connection_Status;
	VTIMER_HandleType Advertising_mgr_timer_Id;
	VTIMER_HandleType SwitchOffGPIO_timer_Id;
	uint8_t connIntervalFlag;
}BleApplicationContext_t;
static BleApplicationContext_t bleAppContext;

typedef struct{
	uint16_t  P2p_serverSvcHdle;				/**< P2p_server Service Handle */
	uint16_t  Led_CCharHdle;			/**< LED_C Characteristic Handle */
	uint16_t  Switch_CCharHdle;			/**< SWITCH_C Characteristic Handle */
}P2P_SERVER_Context_t;
static P2P_SERVER_Context_t P2P_SERVER_Context;

typedef struct {
	uint8_t* BLEStartRamAddress;      /**< Start address of the RAM buffer required by the Bluetooth stack. It must be 32-bit aligned. Use BLE_STACK_TOTAL_BUFFER_SIZE to calculate the correct size. */
	uint32_t TotalBufferSize;         /**< BLE_STACK_TOTAL_BUFFER_SIZE return value, used to check the MACRO correctness*/
	uint16_t NumAttrRecords;          /**< Maximum number of attributes that can be stored in the GATT database. */
	uint8_t MaxNumOfClientProcs;      /**< Maximum number of concurrent client's procedures. This value shall be less or equal to NumOfRadioTasks. */
	uint8_t NumOfRadioTasks;          /**< Maximum number of simultaneous radio tasks. Radio controller supports up to 128 simultaneous radio tasks, but actual usable max value depends on the available device RAM (NUM_LINKS used in the calculation of BLE_STACK_TOTAL_BUFFER_SIZE). */
	uint8_t NumOfEATTChannels;        /**< Maximum number of simultaneous EATT active channels */
	uint16_t NumBlockCount;           /**< Number of allocated memory blocks */
	uint16_t ATT_MTU;                 /**< Maximum supported ATT_MTU size [23-1020]*/
	uint32_t MaxConnEventLength;      /**< Maximum duration of the connection event when the device is peripheral, in units of 625/256 us (~2.44 us) */
	uint16_t SleepClockAccuracy;      /**< Sleep clock accuracy (ppm value)*/
	uint8_t NumOfAdvDataSet;          /**< Maximum number of advertising data sets, valid only when Advertising Extension Feature is enabled  */
	uint8_t NumOfSubeventsPAwR;       /**< Maximum number of Periodic Advertising with Responses subevents */
	uint8_t MaxPAwRSubeventDataCount; /**< Maximum number of Periodic Advertising with Responses subevents that data can be requested for */
	uint8_t NumOfAuxScanSlots;        /**< Maximum number of slots for scanning on the secondary advertising channel, valid only when Advertising Extension Feature is enabled  */
	uint8_t NumOfSyncSlots;           /**< Maximum number of slots for synchronizing to a periodic advertising train, valid only when Periodic Advertising and Synchronizing Feature is enabled  */
	uint8_t FilterAcceptListSizeLog2; /**< Two's logarithm of Filter Accept, Resolving and advertiser list size. */
	uint16_t L2CAP_MPS;               /**< The maximum size of payload data in octets that the L2CAP layer entity is capable of accepting [0-1024].*/
	uint8_t L2CAP_NumChannels;        /**< Maximum number of channels in LE Credit Based Flow Control mode [0-255].*/
	uint8_t CTE_MaxNumAntennaIDs;     /**< Maximum number of Antenna IDs in the antenna pattern used in CTE connection oriented mode. */
	uint8_t CTE_MaxNumIQSamples;      /**< Maximum number of IQ samples in the buffer used in CTE connection oriented mode. */
	uint8_t NumOfSyncBIG;             /**< Maximum number of ISO Synchronizer groups. */
	uint8_t NumOfBrcBIG;              /**< Maximum number of ISO Broadcaster groups. */
	uint8_t NumOfSyncBIS;             /**< Maximum number of ISO Synchronizer streams. */
	uint8_t NumOfBrcBIS;              /**< Maximum number of ISO Broadcaster streams. */
	uint8_t NumOfCIG;                 /**< Maximum number of Connected Isochronous Groups. */
	uint8_t NumOfCIS;                 /**< Maximum number of Connected Isochronous Streams. */
	uint16_t isr0_fifo_size;          /**< Size of the internal FIFO used for critical controller events produced by the ISR (e.g. rx data packets)*/
	uint16_t isr1_fifo_size;          /**< Size of the internal FIFO used for non-critical controller events produced by the ISR (e.g. advertising or IQ sampling reports)*/
	uint16_t user_fifo_size;          /**< Size of the internal FIFO used for controller and host events produced outside the ISR */
} BLE_STACK_InitTypeDef;


typedef enum {
	PROC_GAP_PERIPH_ADVERTISE_START_LP,
	PROC_GAP_PERIPH_ADVERTISE_START_FAST,
	PROC_GAP_PERIPH_ADVERTISE_STOP,
	PROC_GAP_PERIPH_ADVERTISE_DATA_UPDATE,
	PROC_GAP_PERIPH_CONN_PARAM_UPDATE,
	PROC_GAP_PERIPH_CONN_TERMINATE,
	PROC_GAP_PERIPH_SET_BROADCAST_MODE,
}ProcGapPeripheralId_t;

typedef enum
{
	CFG_IDLEEVT_PROC_GAP_COMPLETE,
} CFG_IdleEvt_Id_t;

typedef enum
{
	BLEEVT_NoAck,
	BLEEVT_Ack,
} BLEEVT_EvtAckStatus_t;

/** Documentation for C struct Advertising_Set_Parameters_t */
typedef struct Advertising_Set_Parameters_t_s {
	/** It is used to identify an advertising set.
	 *  Values:
	 *  - 0x00 ... 0xEF
	 */
	uint8_t Advertising_Handle;
	/** The Duration[i] parameter indicates the duration for which that advertising set
	 *  is enabled. The duration begins at the start of the first advertising
	 *  event of this advertising set. The Controller should not start an extended
	 *  advertising event that it cannot complete within the duration. Time = N *
	 *  10 ms 0x00 means no advertising duration: advertising will continue until
	 *  the Host disables it.
	 *  Values:
	 *  - 0x0000 (0 ms) : No advertising duration
	 *  - 0x0001 (10 ms)  ... 0xFFFF (655350 ms)
	 */
	uint16_t Duration;
	/** The Max_Extended_Advertising_Events[i] parameter, if non-zero, indicates the
	 *  maximum number of extended advertising events that shall be sent prior to
	 *  disabling the extended advertising set even if the Duration[i] parameter
	 *  has not expired.
	 *  Values:
	 *  - 0x00: No maximum number of advertising events.
	 *  - 0x01 ... 0xFF: Maximum number of extended advertising events.
	 */
	uint8_t Max_Extended_Advertising_Events;
} Advertising_Set_Parameters_t;

typedef struct{
	uint8_t             Device_Led_Selection;
	uint8_t             Led1;
}P2P_LedCharValue_t;

typedef struct{
	uint8_t             Device_Button_Selection;
	uint8_t             ButtonStatus;
}P2P_ButtonCharValue_t;

typedef enum {
	Switch_c_NOTIFICATION_OFF,
	Switch_c_NOTIFICATION_ON,
	P2P_SERVER_APP_SENDINFORMATION_LAST
} P2P_SERVER_APP_SendInformation_t;

typedef struct
{
	P2P_SERVER_APP_SendInformation_t     Switch_c_Notification_Status;
	P2P_LedCharValue_t              LedControl;
	P2P_ButtonCharValue_t           ButtonControl;
	uint16_t              ConnectionHandle;
} P2P_SERVER_APP_Context_t;
static P2P_SERVER_APP_Context_t P2P_SERVER_APP_Context;

typedef struct ble_uuid_s {
	union
	{
		struct {
			uint8_t v[16U];
		} v_128;
		struct {
			uint8_t base[12U];
			uint32_t v;
		} v_32;
		struct {
			uint8_t base[12U];
			uint16_t v;
			uint16_t empty;
		} v_16;
	};
	uint8_t uuid_type;
} ble_uuid_t;

typedef struct ble_gatt_val_buffer_def_s {
	uint8_t op_flags; /**< See @ref SRV_VALBUFFER_OP_FLAGS */
	uint16_t val_len; /**< Actual value length. It can differ from buffer_len if BLE_GATT_SRV_PERM_ENCRY_READ the BLE_GATT_SRV_OP_VALUE_VAR_LENGTH_FLAG is set in op_flags */
	uint16_t buffer_len; /**< Buffer length. */
	uint8_t *buffer_p; /**< Pointer to the storage buffer. */
} ble_gatt_val_buffer_def_t;

typedef struct ble_gatt_descr_def_s {
	uint8_t properties; /**< See @ref GATT_SRV_DESCR_PROP. */
	/** Minimum key size required to access descriptor value. Valid only if permissions are
	 *  BLE_GATT_SRV_PERM_ENCRY_READ or BLE_GATT_SRV_PERM_ENCRY_WRITE, otherwise ignored.
	 *  See @ref GATT_SRV_KEY_SIZE for minimum and maximum values.
	 */
	uint8_t min_key_size;
	uint8_t permissions; /**< Access permissions. See @ref GATT_SRV_PERMS. */
	ble_uuid_t uuid; /**< UUID for this descriptor */
	ble_gatt_val_buffer_def_t *val_buffer_p; /**< Pointer to the value buffer structure. */
} ble_gatt_descr_def_t;

typedef struct ble_gatt_chr_def_s {
	uint8_t properties; /**< @see GATT_SRV_CHAR_PROP */
	/** Minimum key size required to access characteristic value. Valid only if permissions are
	 *  BLE_GATT_SRV_PERM_ENCRY_READ or BLE_GATT_SRV_PERM_ENCRY_WRITE, otherwise ignored.
	 *  @see GATT_SRV_KEY_SIZE for minimum and maximum values.
	 */
	uint8_t min_key_size;
	uint8_t permissions; /**< Access permissions. See @ref GATT_SRV_PERMS. */
	ble_uuid_t uuid; /**< UUID for this characteristic. Macros can be used to initialize the UUID: see @ref BLE_UUID_INIT. */
	struct {
		uint8_t descr_count; /**< Number of descriptors. */
		ble_gatt_descr_def_t *descrs_p; /**< Pointer to the descriptors vector. */
	} descrs; /**< List of descriptors. */
	ble_gatt_val_buffer_def_t *val_buffer_p; /**< Pointer to the value buffer structure. */
} ble_gatt_chr_def_t;

typedef struct ble_gatt_srv_def_s {
	ble_uuid_t uuid; /**< UUID for this service. Macros can be used to initialize the UUID: see @ref BLE_UUID_INIT. */
	uint8_t type; /**< primary/secondary. See @ref GATT_SRV_TYPE */
	uint16_t group_size; /**< Define the number of attribute handles reserved for this service. */
	struct {
		uint8_t incl_srv_count; /**< Number of services. */
		struct ble_gatt_srv_def_s **included_srv_pp; /**< List of pointers to the included service definition. */
	} included_srv;
	struct {
		uint8_t chr_count; /**< Number of characteristics. */
		ble_gatt_chr_def_t *chrs_p; /**< Pointer to characteristics vector. */
	} chrs; /**< List of characteristics. */
} ble_gatt_srv_def_t;

typedef PACKED(struct) aci_blecore_event_s {
	uint16_t      ecode;      /*!< A proprietary ACI event code. See @ref ACI_HAL_evt_code (HAL/LL), @ref ACI_GAP_evt_code (GAP),
                                 @ref ACI_GATT_evt_code (GATT), @ref ACI_L2CAP_evt_code (L2CAP). */
	uint8_t       data[0];    /*!< Proprietary event parameters. To be casted to a struct of @ref ACI_hal_evt_structs, @ref ACI_gap_evt_structs,
                                 @ref ACI_gatt_evt_structs, @ref ACI_l2cap_evt_structs.  */
} aci_blecore_event;

typedef BLEEVT_EvtAckStatus_t (*BLEEVT_GattEvtHandlerFunc_t)(aci_blecore_event *evt_p);
typedef struct {
	BLEEVT_GattEvtHandlerFunc_t BLEEVT_SvcHandlerTab[1];
	uint8_t NbrOfRegisteredHandlers;
} BLEEVT_GattEvtHandler_t;
static BLEEVT_GattEvtHandler_t BLEEVT_GattEvtHandler;

#define BLE_GATT_SRV_PRIMARY_SRV_TYPE                   (1U)
#define BLE_UUID_BASE                                   0xFB, 0x34, 0x9B, 0x5F, \
                                                        0x80, 0x00, 0x00, 0x80, \
                                                        0x00, 0x10, 0x00, 0x00
#define BLE_UUID_TYPE_16BITS                            (16U)
#define BLE_UUID_INIT_16(UUID)      {                                           \
        .v_16.base = { BLE_UUID_BASE },                                         \
        .v_16.v = UUID,                                                         \
        .v_16.empty = 0x0000U,                                                  \
        .uuid_type = BLE_UUID_TYPE_16BITS,                                      \
}
#define BLE_UUID_TYPE_128BITS                            (128U)
#define BLE_UUID_INIT_128(UUID)     {                                           \
        .v_128.v = { UUID },                                                    \
        .uuid_type = BLE_UUID_TYPE_128BITS,                                     \
}
#define BLE_GATT_SRV_GATT_SERVICE_UUID                  (0x1801U)
static ble_gatt_srv_def_t gatt_srvc = {
		.type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
		.uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_GATT_SERVICE_UUID),
		.chrs = {0, NULL},
};

#define BLE_GATT_SRV_CHAR_PROP_INDICATE                 (0x20U)
#define BLE_GATT_SRV_PERM_NONE                          (0x00U)
#define BLE_GATT_SRV_SERVICE_CHANGE_CHR_UUID            (0x2A05U)
#define GATT_CHR_SERVICE_CHANGED_VALUE_LEN                  (4U)
static const uint8_t gatt_chr_srv_changed_buff[GATT_CHR_SERVICE_CHANGED_VALUE_LEN] =
		{0x00U, 0x00U, 0xffU, 0xffU};

static const ble_gatt_val_buffer_def_t gatt_chr_srv_changed_value_buff = {
		.buffer_len = GATT_CHR_SERVICE_CHANGED_VALUE_LEN,
		.buffer_p = (uint8_t *)gatt_chr_srv_changed_buff,
};

#define HCI_MAX_PAYLOAD_SIZE 532

typedef PACKED(struct) aci_gatt_srv_attribute_modified_event_rp0_s {
/**
 * The connection handle which modified the attribute.
 */
	uint16_t Connection_Handle;
/**
 * If equal to 0x0004, the event is related to an unenhanced ATT bearer.
 * Otherwise, the value is the local endpoint identifying the enhanced ATT
 * bearer.
 */
	uint16_t CID;
/**
 * Handle of the attribute that was modified.
 */
	uint16_t Attr_Handle;
/**
 * Length of Attr_Data in octets
 */
	uint16_t Attr_Data_Length;
/**
 * A concatenation of Handle, Length and Values for each of the attributes being
 * notified.
 */
	uint8_t Attr_Data[(HCI_MAX_PAYLOAD_SIZE - 8)/sizeof(uint8_t)];
} aci_gatt_srv_attribute_modified_event_rp0;

typedef PACKED(struct) aci_gatt_srv_write_event_rp0_s {
/**
 * Handle identifying the connection where the write operation has been
 * received.
 */
	uint16_t Connection_Handle;
/**
 * If equal to 0x0004, the event is related to an unenhanced ATT bearer.
 * Otherwise, the value is the local endpoint identifying the enhanced ATT
 * bearer.
 */
	uint16_t CID;
/**
 * If 1, application must call aci_gatt_srv_resp() to give a response to the
 * peer.
 */
	uint8_t Resp_Needed;
/**
 * Handle of the attribute to write.
 */
	uint16_t Attribute_Handle;
/**
 * Length of the data to write.
 */
	uint16_t Data_Length;
/**
 * Data to write.
 */
	uint8_t Data[(HCI_MAX_PAYLOAD_SIZE - 9)/sizeof(uint8_t)];
} aci_gatt_srv_write_event_rp0;

typedef PACKED(struct) aci_gatt_srv_read_event_rp0_s {
/**
 * Handle identifying the connection where the read operation has been received.
 */
	uint16_t Connection_Handle;
/**
 * If equal to 0x0004, the event is related to an unenhanced ATT bearer.
 * Otherwise, the value is the local endpoint identifying the enhanced ATT
 * bearer.
 */
	uint16_t CID;
/**
 * Handle of the attribute to read.
 */
	uint16_t Attribute_Handle;
/**
 * Offset from which the peer is requesting the attribute value.
 */
	uint16_t Data_Offset;
} aci_gatt_srv_read_event_rp0;

typedef enum
{
	P2P_SERVER_LED_C_READ_EVT,
	P2P_SERVER_LED_C_WRITE_NO_RESP_EVT,
	P2P_SERVER_SWITCH_C_NOTIFY_ENABLED_EVT,
	P2P_SERVER_SWITCH_C_NOTIFY_DISABLED_EVT,

	/* USER CODE BEGIN Service1_OpcodeEvt_t */

	/* USER CODE END Service1_OpcodeEvt_t */

	P2P_SERVER_BOOT_REQUEST_EVT
} P2P_SERVER_OpcodeEvt_t;

typedef struct
{
	uint8_t *p_Payload;
	uint8_t Length;

	/* USER CODE BEGIN Service1_Data_t */

	/* USER CODE END Service1_Data_t */

} P2P_SERVER_Data_t;

typedef struct
{
	P2P_SERVER_OpcodeEvt_t       EvtOpcode;
	P2P_SERVER_Data_t             DataTransfered;
	uint16_t                ConnectionHandle;
	uint16_t                AttributeHandle;
	uint8_t                 ServiceInstance;

	/* USER CODE BEGIN Service1_NotificationEvt_t */

	/* USER CODE END Service1_NotificationEvt_t */

} P2P_SERVER_NotificationEvt_t;
#define ACI_GATT_SRV_ATTRIBUTE_MODIFIED_VSEVT_CODE                             0x0C01
#define ACI_GATT_SRV_READ_VSEVT_CODE                                           0x0C19
#define ACI_GATT_SRV_WRITE_VSEVT_CODE                                          0x0C1A
#define ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE                                  0x0C16
#define ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE                                   0x0C03
#define ACI_GATT_CLT_INDICATION_VSEVT_CODE                                     0x0C0E
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET        2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET             1

typedef PACKED(struct) aci_gatt_tx_pool_available_event_rp0_s {
/**
 * Connection handle related to the request
 */
	uint16_t Connection_Handle;
/**
 * Not used.
 */
	uint16_t Available_Buffers;
} aci_gatt_tx_pool_available_event_rp0;

typedef PACKED(struct) aci_att_exchange_mtu_resp_event_rp0_s {
/**
 * Connection handle related to the response
 */
	uint16_t Connection_Handle;
/**
 * ATT_MTU value agreed between server and client. This is the minimum of the
 * Client Rx MTU and the Server Rx MTU.
 */
	uint16_t MTU;
} aci_att_exchange_mtu_resp_event_rp0;

/**
 *@name GATT_SRV_CCCD
 *@brief The Client Characteristic Configuration is a special descriptor that
 *       has to store the subscription of each client to receive notification
 *       and indication value.
 *@{
 */
#define BLE_GATT_SRV_CCCD_VALUE_LEN                     (2U)
#define BLE_GATT_SRV_CCCD_NOTIFICATION_BIT              (0U)
#define BLE_GATT_SRV_CCCD_INDICATION_BIT                (1U)
#define BLE_GATT_SRV_CCCD_NOTIFICATION                  (1U << BLE_GATT_SRV_CCCD_NOTIFICATION_BIT)
#define BLE_GATT_SRV_CCCD_INDICATION                    (1U << BLE_GATT_SRV_CCCD_INDICATION_BIT)

#define BLE_GATT_SRV_CCCD_NUM_BITS_PER_CONN             (2U)
#define BLE_GATT_SRV_CCCD_VAL_MASK                      (BLE_GATT_SRV_CCCD_NUM_BITS_PER_CONN | \
                                                         (BLE_GATT_SRV_CCCD_NUM_BITS_PER_CONN - 1))
#define BLE_GATT_SRV_CCCD_NUM_CONN_PER_BYTE             (8U / BLE_GATT_SRV_CCCD_NUM_BITS_PER_CONN)
/** Return the number of bytes needed to store a CCCD giving the number of connections. */
#define BLE_GATT_SRV_CCCD_BUFF_SIZE(NUM_CONN)           (((NUM_CONN) / BLE_GATT_SRV_CCCD_NUM_CONN_PER_BYTE) + 1U)
/** Declare a storage buffer with the giving name to store the CCCD. */
#define BLE_GATT_SRV_CCCD_BUFF_DECLARE(NAME, NUM_CONN)   uint8_t(NAME)[BLE_GATT_SRV_CCCD_BUFF_SIZE(NUM_CONN)]

#define BLE_GATT_SRV_CCCD_BUFFER_NAME(NAME)             NAME##_cccd_buffer
#define BLE_GATT_SRV_CCCD_VAL_BUFFER_NAME(NAME)         NAME##_cccd_val_buffer

#define BLE_GATT_SRV_CCCD_DEF_NAME(NAME)                NAME##_cccd
#define BLE_GATT_SRV_CCCD_DEF_STR_FIELDS(NAME, NUM_CONN, PERM)                      \
    .val_buffer_p = (ble_gatt_val_buffer_def_t *)&BLE_GATT_SRV_CCCD_VAL_BUFFER_NAME(NAME), \
    .properties = BLE_GATT_SRV_DESCR_PROP_WRITE | BLE_GATT_SRV_DESCR_PROP_READ,     \
    .permissions = (PERM),                                                          \
    .min_key_size = BLE_GATT_SRV_MIN_ENCRY_KEY_SIZE,                                \
    .uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_CLIENT_CHAR_CONF_DESCR_TYPE_UUID)


#define GATT_SRV_MAX_CONN                                (8U)
#define BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG        (0x01U)
#define BLE_GATT_SRV_CLIENT_CHAR_CONF_DESCR_TYPE_UUID   (0x2902U)
#define BLE_GATT_SRV_DESCR_PROP_READ                    (0x01U)
#define BLE_GATT_SRV_DESCR_PROP_WRITE                   (0x02U)
#define BLE_GATT_SRV_MIN_ENCRY_KEY_SIZE                 (7U)
#define CFG_NUM_RADIO_TASKS 							(2)

//BLE_GATT_SRV_CCCD_DECLARE(switch_c, CFG_NUM_RADIO_TASKS, BLE_GATT_SRV_CCCD_PERM_DEFAULT,  BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG);
static uint8_t BLE_GATT_SRV_CCCD_BUFFER_NAME(switch_c)[BLE_GATT_SRV_CCCD_BUFF_SIZE(2)];
static const ble_gatt_val_buffer_def_t BLE_GATT_SRV_CCCD_VAL_BUFFER_NAME(switch_c) = {
	.op_flags = (1 & BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG),
	.buffer_len = BLE_GATT_SRV_CCCD_BUFF_SIZE(2),
	.buffer_p = BLE_GATT_SRV_CCCD_BUFFER_NAME(switch_c),
};
static ble_gatt_descr_def_t BLE_GATT_SRV_CCCD_DEF_NAME(switch_c) = {
	.val_buffer_p = (ble_gatt_val_buffer_def_t *)&BLE_GATT_SRV_CCCD_VAL_BUFFER_NAME(switch_c),
    .properties = BLE_GATT_SRV_DESCR_PROP_WRITE | BLE_GATT_SRV_DESCR_PROP_READ,
    .permissions = (0),
    .min_key_size = BLE_GATT_SRV_MIN_ENCRY_KEY_SIZE,
    .uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_CLIENT_CHAR_CONF_DESCR_TYPE_UUID)
};

static uint8_t BLE_GATT_SRV_CCCD_BUFFER_NAME(gatt_chr_srv_changed)[BLE_GATT_SRV_CCCD_BUFF_SIZE(8)];
static const ble_gatt_val_buffer_def_t BLE_GATT_SRV_CCCD_VAL_BUFFER_NAME(gatt_chr_srv_changed) = {
	.op_flags = (1 & BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG),
	.buffer_len = BLE_GATT_SRV_CCCD_BUFF_SIZE(8),
	.buffer_p = BLE_GATT_SRV_CCCD_BUFFER_NAME(gatt_chr_srv_changed),
};
static ble_gatt_descr_def_t BLE_GATT_SRV_CCCD_DEF_NAME(gatt_chr_srv_changed) = {
	.val_buffer_p = (ble_gatt_val_buffer_def_t *)&BLE_GATT_SRV_CCCD_VAL_BUFFER_NAME(gatt_chr_srv_changed),
	.properties = BLE_GATT_SRV_DESCR_PROP_WRITE | BLE_GATT_SRV_DESCR_PROP_READ,
	.permissions = (0),
	.min_key_size = BLE_GATT_SRV_MIN_ENCRY_KEY_SIZE,
	.uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_CLIENT_CHAR_CONF_DESCR_TYPE_UUID)
};


static const ble_gatt_chr_def_t gatt_srvc_changed_chr = {
				/**< Service Changed Characteristic. */
				.properties = BLE_GATT_SRV_CHAR_PROP_INDICATE,
				.permissions = BLE_GATT_SRV_PERM_NONE,
				.uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_SERVICE_CHANGE_CHR_UUID),
				.val_buffer_p = (ble_gatt_val_buffer_def_t *)&gatt_chr_srv_changed_value_buff,
				.descrs = {
						.descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(gatt_chr_srv_changed),
						.descr_count = 1U,
				}
};

#define GATT_INIT_SERVICE_CHANGED_BIT                       (0x01)

#define BLE_GATT_SRV_CHAR_PROP_READ                     (0x02U)
#define BLE_GATT_SRV_CHAR_PROP_WRITE                    (0x08U)
#define BLE_GATT_SRV_CLIENT_SUPP_FEATURE_CHR_UUID       (0x2B29U)

#define BLE_GATT_SRV_CLIENT_SUP_FEATURE_NUM_BITS_PER_CONN    (4U)
#define BLE_GATT_SRV_CLIENT_SUP_FEATURE_NUM_CONN_PER_BYTE    (8U / BLE_GATT_SRV_CLIENT_SUP_FEATURE_NUM_BITS_PER_CONN)
#define BLE_GATT_SRV_CLIENT_SUP_FEATURE_SIZE_X_CONN(NUM_CONN)  (((NUM_CONN) / \
                                                                 BLE_GATT_SRV_CLIENT_SUP_FEATURE_NUM_CONN_PER_BYTE) + 1U)

static uint8_t gatt_client_supp_feature_buff[BLE_GATT_SRV_CLIENT_SUP_FEATURE_SIZE_X_CONN(GATT_SRV_MAX_CONN)];

static const ble_gatt_val_buffer_def_t gatt_client_supp_feature_val_buff = {
		.buffer_len = BLE_GATT_SRV_CLIENT_SUP_FEATURE_SIZE_X_CONN(GATT_SRV_MAX_CONN),
		.buffer_p = gatt_client_supp_feature_buff,
};

static const ble_gatt_chr_def_t gatt_clt_supp_feat_chr = {
				/**< Client Supported Features Characteristic. */
				.properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_WRITE,
				.permissions = BLE_GATT_SRV_PERM_NONE,
				.uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_CLIENT_SUPP_FEATURE_CHR_UUID),
				.val_buffer_p = (ble_gatt_val_buffer_def_t *)&gatt_client_supp_feature_val_buff,

};

#define BLE_GATT_SRV_DB_HASH_CHR_UUID                   (0x2B2AU)
static const ble_gatt_chr_def_t gatt_db_hash_chr = {
				/**< Database Hash Characteristic. Value buffer is allocated into the stack. */
				.properties = BLE_GATT_SRV_CHAR_PROP_READ,
				.permissions = BLE_GATT_SRV_PERM_NONE,
				.uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_DB_HASH_CHR_UUID)
};

#define BLE_GATT_SRV_SUPPORTED_FEATURES_CHR_UUID        (0x2B3AU)
#define BLE_GATT_SRV_SUPPORTED_FEATURES_VAL_LEN         (1U)
static const uint8_t gatt_server_supp_feature_buff = 1U;
static const ble_gatt_val_buffer_def_t gatt_server_supp_feature_val_buff = {
		.buffer_len = BLE_GATT_SRV_SUPPORTED_FEATURES_VAL_LEN,
		.buffer_p = (uint8_t *)&gatt_server_supp_feature_buff,
};

static const ble_gatt_chr_def_t gatt_srv_supp_feat_chr =
		{
				/**< Server Supported Feature Characteristic. */
				.properties = BLE_GATT_SRV_CHAR_PROP_READ,
				.permissions = BLE_GATT_SRV_PERM_NONE,
				.uuid = BLE_UUID_INIT_16(BLE_GATT_SRV_SUPPORTED_FEATURES_CHR_UUID),
				.val_buffer_p = (ble_gatt_val_buffer_def_t *)&gatt_server_supp_feature_val_buff,
		};

typedef PACKED(struct) aci_gatt_clt_indication_event_rp0_s {
/**
 * Connection handle related to the response
 */
	uint16_t Connection_Handle;
/**
 * If equal to 0x0004, the event is related to an unenhanced ATT bearer.
 * Otherwise, the value is the local endpoint identifying the enhanced ATT
 * bearer.
 */
	uint16_t CID;
/**
 * The handle of the attribute
 */
	uint16_t Attribute_Handle;
/**
 * Length of Attribute_Value in octets
 */
	uint16_t Attribute_Value_Length;
/**
 * The current value of the attribute
 */
	uint8_t Attribute_Value[(HCI_MAX_PAYLOAD_SIZE - 8)/sizeof(uint8_t)];
} aci_gatt_clt_indication_event_rp0;

/* Utils defines */
typedef uint32_t UTIL_SEQ_bm_t;
/**
 * @brief current task id.
 */
static uint32_t CurrentTaskIdx = 0U;
/**
 * @brief evt expected mask.
 */
static volatile UTIL_SEQ_bm_t EvtWaited = UTIL_SEQ_NO_BIT_SET;
/**
 * @brief evt set mask.
 */
static volatile UTIL_SEQ_bm_t EvtSet = UTIL_SEQ_NO_BIT_SET;
static UTIL_SEQ_bm_t SuperMask = UTIL_SEQ_ALL_BIT_SET;
static volatile UTIL_SEQ_bm_t TaskMask = UTIL_SEQ_ALL_BIT_SET;
static uint8_t a_P2P_SERVER_UpdateCharData[247];
static uint8_t switch_c_val_buffer[2];


static const char a_GapDeviceName[] = {'S', 'E', 'S', ' ', 'B', 'e', 'a', 'c', 'o', 'n'}; /* Gap Device Name */
static uint32_t dyn_alloc_a[2311] __attribute__((section(".noinit")));


typedef uint8_t tBleStatus;
static uint32_t primask_bit;
static NVMDB_info DBInfo[2];
static NVMDB_HandleType sec_gatt_db_h, device_id_db_h, *curr_handle_p;
static volatile uint32_t internal_state;

inline static void ENTER_CRITICAL_SECTION(void){
		primask_bit = __GET_PRIMASK();
		__IRQ_D();
}

inline static void EXIT_CRITICAL_SECTION(void){
	__SET_PRIMASK(primask_bit);
	__IRQ_E();
}

static void (*TaskCb[32])( void );
static const uint8_t SEQ_clz_table_4bit[16U] = { 4U, 3U, 2U, 2U, 1U, 1U, 1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
static volatile UTIL_SEQ_bm_t TaskSet;
static volatile UTIL_SEQ_Priority_t TaskPrio[2];

static uint8_t gap_device_name_buff[50];
static ble_gatt_val_buffer_def_t gap_device_name_val_buff = {
		.buffer_len = sizeof(gap_device_name_buff),
		.op_flags = 3,
		.buffer_p = gap_device_name_buff,
};

static uint8_t gap_appearance_buff[2];
static const ble_gatt_val_buffer_def_t gap_appearance_val_buff = {
		.buffer_len = 2,
		.buffer_p = gap_appearance_buff,
};

static uint8_t gap_peripheral_preferred_con_params_buff[8] = {
		0xFFU, 0xFFU, 0xFFU, 0xFFU,
		0x00U, 0x00U, 0xFFU, 0xFFU
};
static const ble_gatt_val_buffer_def_t gap_peripheral_preferred_con_params_val_buff = {
		.buffer_len = 8,
		.buffer_p = gap_peripheral_preferred_con_params_buff,
};

static uint8_t gap_central_address_resolution_buff[1] = { 1U };
static const ble_gatt_val_buffer_def_t gap_central_address_resolution_val_buff = {
		.buffer_len = 1,
		.buffer_p = gap_central_address_resolution_buff,
};

static ble_gatt_chr_def_t gap_chrs[] = {
	{ /**< Device Name Characteristic. */
		.properties = 2,
		.permissions = 0,
		.uuid = BLE_UUID_INIT_16(0x2A00U),
		.min_key_size = 7U,
		.val_buffer_p = (ble_gatt_val_buffer_def_t *)&gap_device_name_val_buff,
	}, { /**< Appearance Characteristic. */
		.properties = 2,
		.permissions = 0,
		.uuid = BLE_UUID_INIT_16(0x2A01U),
		.min_key_size = 7U,
		.val_buffer_p = (ble_gatt_val_buffer_def_t *)&gap_appearance_val_buff,
	}, { /**< Peripheral Preferred Connection Parameters Characteristic. */
		.properties = 2,
		.permissions = 0,
		.uuid = BLE_UUID_INIT_16(0x2A04U),
		.min_key_size = 7U,
		.val_buffer_p = (ble_gatt_val_buffer_def_t *)&gap_peripheral_preferred_con_params_val_buff,
	}, { /**< Central Address Resolution Characteristic. */
		.properties = 2,
		.permissions = 0,
		.min_key_size = 7U,
		.uuid = BLE_UUID_INIT_16(0x2AA6U),
		.val_buffer_p = (ble_gatt_val_buffer_def_t *)&gap_central_address_resolution_val_buff,
	}
};
static ble_gatt_srv_def_t gap_srvc = {
	.type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
	.uuid = BLE_UUID_INIT_16(0x1800U),
	.chrs = {
			.chrs_p = gap_chrs,
			.chr_count = 2U,
	},
};

static uint8_t led_c_val_buffer[2];
static ble_gatt_val_buffer_def_t led_c_val_buffer_def = {
		.op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG,
		.val_len = 2,
		.buffer_len = sizeof(led_c_val_buffer),
		.buffer_p = led_c_val_buffer
};

static ble_gatt_val_buffer_def_t switch_c_val_buffer_def = {
		.op_flags = BLE_GATT_SRV_OP_MODIFIED_EVT_ENABLE_FLAG,
		.val_len = 2,
		.buffer_len = sizeof(switch_c_val_buffer),
		.buffer_p = switch_c_val_buffer
};

static ble_gatt_chr_def_t p2p_server_chars[] = {
		{
				.properties = BLE_GATT_SRV_CHAR_PROP_READ | BLE_GATT_SRV_CHAR_PROP_WRITE_NO_RESP,
				.permissions = BLE_GATT_SRV_PERM_NONE,
				.min_key_size = 0x10,
				.uuid = BLE_UUID_INIT_128(LED_C_UUID),
				.val_buffer_p = &led_c_val_buffer_def
		},
		{
				.properties = BLE_GATT_SRV_CHAR_PROP_NOTIFY,
				.permissions = BLE_GATT_SRV_PERM_NONE,
				.min_key_size = 0x10,
				.uuid = BLE_UUID_INIT_128(SWITCH_C_UUID),
				.descrs = {
						.descrs_p = &BLE_GATT_SRV_CCCD_DEF_NAME(switch_c),
						.descr_count = 1U,
				},
				.val_buffer_p = &switch_c_val_buffer_def
		},
};

/* p2p Server service definition */
static ble_gatt_srv_def_t p2p_server_service = {
		.type = BLE_GATT_SRV_PRIMARY_SRV_TYPE,
		.uuid = BLE_UUID_INIT_128(P2P_SERVER_UUID),
		.chrs = {
				.chrs_p = (ble_gatt_chr_def_t *)p2p_server_chars,
				.chr_count = 2U,
		},
};

uint8_t SEQ_BitPosition(uint32_t Value);
void UTIL_SEQ_RegTask(uint32_t TaskId_bm, uint32_t Flags, void (*Task)( void ));
void UTIL_SEQ_SetTask(UTIL_SEQ_bm_t TaskId_bm , uint32_t Task_Prio);
static void BLEStack_Process(void);

//NOTE: !EXTERNAL FUNCTIONS
void BLE_STACK_Tick(void);
uint8_t BLE_STACK_SleepCheck(void);
tBleStatus aci_gap_terminate_api(uint16_t Connection_Handle, uint8_t Reason);
tBleStatus aci_gap_set_advertising_configuration(uint8_t Advertising_Handle, uint8_t Discoverable_Mode, uint16_t Advertising_Event_Properties, uint32_t Primary_Advertising_Interval_Min, uint32_t Primary_Advertising_Interval_Max, uint8_t Primary_Advertising_Channel_Map, uint8_t Peer_Address_Type, uint8_t Peer_Address[6], uint8_t Advertising_Filter_Policy, int8_t Advertising_Tx_Power, uint8_t Primary_Advertising_PHY, uint8_t Secondary_Advertising_Max_Skip, uint8_t Secondary_Advertising_PHY, uint8_t Advertising_SID, uint8_t Scan_Request_Notification_Enable);
tBleStatus aci_gap_set_advertising_data(uint8_t Advertising_Handle, uint8_t Operation, uint16_t Advertising_Data_Length, uint8_t Advertising_Data[]);
tBleStatus aci_gap_set_advertising_enable(uint8_t Enable, uint8_t Number_of_Sets, Advertising_Set_Parameters_t Advertising_Set_Parameters[]);
tBleStatus aci_l2cap_connection_parameter_update_req_api(uint16_t Connection_Handle, uint16_t Connection_Interval_Min, uint16_t Connection_Interval_Max, uint16_t Peripheral_Latency, uint16_t Timeout_Multiplier);
tBleStatus aci_gatt_srv_notify_api(uint16_t Connection_Handle, uint16_t CID, uint16_t Attr_Handle, uint8_t Flags, uint16_t Val_Length, uint8_t* Val_p);
tBleStatus BLE_STACK_Init(const BLE_STACK_InitTypeDef *BLE_STACK_InitStruct);
tBleStatus aci_hal_set_tx_power_level(uint8_t En_High_Power, uint8_t PA_Level);
tBleStatus aci_gatt_srv_add_service_api(ble_gatt_srv_def_t*);
tBleStatus aci_gatt_srv_get_service_handle_api(ble_gatt_srv_def_t*);
tBleStatus aci_gatt_srv_add_char_api(ble_gatt_chr_def_t*, uint16_t);
tBleStatus aci_gatt_srv_get_char_decl_handle_api(ble_gatt_chr_def_t*);
tBleStatus aci_gap_init(uint8_t Privacy_Type, uint8_t Identity_Address_Type);
tBleStatus aci_hal_read_config_data(uint8_t, uint8_t*, uint8_t[]);
tBleStatus hci_le_set_default_phy_api(uint8_t, uint8_t, uint8_t);
tBleStatus aci_gap_set_io_capability_api(uint8_t);
tBleStatus aci_gap_set_security_requirements_api(uint8_t Bonding_Mode, uint8_t MITM_Mode, uint8_t SC_Support, uint8_t KeyPress_Notification_Support, uint8_t Min_Encryption_Key_Size, uint8_t Max_Encryption_Key_Size, uint8_t Pairing_Response);
tBleStatus aci_gap_configure_filter_accept_and_resolving_list(uint8_t Lists);
tBleStatus aci_hal_set_radio_activity_mask(uint16_t Radio_Activity_Mask);
tBleStatus aci_gatt_clt_confirm_indication_api(uint16_t, uint16_t);
// NOTE ~external

// normal functions
void ble_init(void);
void modules_init(void);
void blenvm_init(void);
void MX_APPE_Init(void);

#endif // STM32WB07_BLE_H
