#ifndef STM32WB07_RF_TIMER_H
#define STM32WB07_RF_TIMER_H

#include "WAKEUP.h"
#include "NVIC.h"
#include "RF/RF.h"
#include "types.h"
/** @defgroup RADIO_TIMER_Private_Defines RADIO TIMER Private Defines
  * @{
  */
#define MULT64_THR_FREQ (806)
#define MULT64_THR_PERIOD (1589)

/* Margin to add to the calibration interval in order to guarantee
 * enough time to program the radio timer after the calibration.
 * It is expressed in STU. */
#define RADIO_ACTIVITY_MARGIN (204800)

/* Threshold to take into account the calibration duration. */
#define CALIB_SAFE_THR (370)

/* Minimum threshold to safely program the radio timer (expressed in STU) */
#define TIMER1_MARGIN (10)

/*  Delay to program a radio timer in the worst case (in STU).
   This is the sum of: 1st init dalay, 2 init delay and tx delay (118 + 65 + 2)  */
#define TIMER1_INIT_DELAY (76)

/* Radio event types */
#define RX (0)
#define TX (1)

#define MARGIN_EXT (200)

/* Minimum threshold in STU to safely clear radio timers.
The wakeup timer in the worst case triggers about 30us in advance.
This must be considered when the radio timer is cleared.
Then a window of about 30 us is considered as critical, that is
it is not sure the timer can be cleared properly */
#define CLEAR_MIN_THR (15)

/* Extra margin to consider before going in low power mode.
   This is the time (STU) needed for the system to go to sleep from the time the
   HAL_RADIO_TIMER_PowerSaveLevelCheck() is called. */
#define LOW_POWER_THR (82)  // Around 200 us.

/* HOST_MARGIN is the margin in STU needed to program a pending radio operation after the host timer is triggered */
#define HOST_MARGIN (200)

#define RADIO_TX_RX_EXCEPTION_NUMBER 18

// startuptime can't be smaller than 200
#define XTAL_StartupTime 320
#define enableInitialCalibration 0

/**
  * @brief  VIRTUAL TIMER Callback pointer definition
  */
typedef void (*VTIMER_CallbackType)(void *);

/**
  * @brief VIRTUAL TIMER handle Structure definition
  */
typedef struct VTIMER_HandleTypeS
{
	uint64_t expiryTime; /*!< Managed internally when the timer is started */
	VTIMER_CallbackType callback; /*!< Pointer to the user callback */
	uint8_t active; /*!< Managed internally when the timer is started */
	struct VTIMER_HandleTypeS *next; /*!< Managed internally when the timer is started */
	void *userData; /*!< Pointer to user data */
} VTIMER_HandleType;

typedef struct
{
	uint8_t periodicCalibration; /*!< Periodic calibration enable status */
	uint32_t periodicCalibrationInterval;  /*!< Periodic calibration interval in ms, to disable set to 0 */
	uint8_t calibration_in_progress;  /*!< Flag to indicate that a periodic calibration has been started */
} CalibrationSettingsTypeDef;

typedef struct
{
	uint32_t period;        /** Number of 16 MHz clock cycles in (2*(SLOW_COUNT+1)) low speed oscillator periods */
	uint32_t freq;          /** 2^39/period */
	int32_t freq1;          /** Round(((freq/64)*0x753)/256) */
	int32_t period1;        /** Round (( ((period /256) * 0x8BCF6) + (((period % 256)* 0x8BCF6)/256)) / 32) */
	int32_t last_period1;   /** Period global in last calibration */
	uint64_t last_calibration_time; /** Absolute system time when last calibration was performed */
	uint32_t calibration_machine_interval; /** Calibration Interval MTU */
	uint8_t calibration_data_available; /** Flag to signal if a new calibration data is available or not */
} CalibrationDataTypeDef;

typedef struct
{
	uint8_t tx_cal_delay; /**time in MTU to be compensated if transmission and pll calibration are requested. The value in RAM must be initialized before the TIMER is initialized*/
	uint8_t tx_no_cal_delay; /**time in MTU to be compensated if transmission is requested. The value in RAM must be initialized before the TIMER is initialized*/
	uint8_t rx_cal_delay; /**time in MTU to be compensated if reception and pll calibration are requested. The value in RAM must be initialized before the TIMER is initialized*/
	uint8_t rx_no_cal_delay; /**time in MTU to be compensated if reception is requested. The value in RAM must be initialized before the TIMER is initialized*/
	uint8_t tx_cal_delay_st; /**time in STU to be compensated if transmission and pll calibration are requested. The value in RAM must be initialized before the TIMER is initialized*/
	uint8_t tim12_delay_mt;
} TxRxDelayTypeDef;

typedef struct
{
	uint64_t expiryTime;
	uint8_t cal_req;
	uint8_t active;
	uint8_t pending;
	uint8_t intTxRx_to_be_served;
	uint8_t event_type;
} RADIO_TIMER_RadioHandleTypeDef;

typedef struct
{
	CalibrationSettingsTypeDef calibrationSettings;
	CalibrationDataTypeDef calibrationData;
	TxRxDelayTypeDef TxRxDelay;
	VTIMER_HandleType calibrationTimer;
	RADIO_TIMER_RadioHandleTypeDef radioTimer;
	uint32_t hs_startup_time; /*!< HS startup time */
	uint64_t cumulative_time; /** Absolute system time since power up */
	uint64_t last_system_time; /** Last System Time */
	uint32_t last_machine_time; /** Last machine time used to update cumulative time */
	uint8_t last_setup_time; /**setup time of last timer programmed*/
	uint32_t last_anchor_mt;
	VTIMER_HandleType *rootNode; /*!< First timer of the host timer queue */
	uint8_t enableTimeBase;      /*!< Internal flag. User can ignore it*/
	uint8_t expired_count; /*!< Progressive number to indicate expired timers */
	uint8_t served_count; /*!< Progressive number to indicate served expired timers */
	uint8_t stop_notimer_action; /*!< Flag to indicate DEEPSTOP no timer action */
	uint8_t wakeup_calibration; /*!< Flag to indicate if start a calibration after  wakeup */

	uint8_t hostIsRadioPending; /*!< If hostIsRadioPending is true, the virtual timer callback will be triggered when the wakeup timer triggers */
	uint32_t hostMargin; /*!< It depends on the hs startup time. See HOST_MARGIN */
	uint8_t waitCal; /*!< Wait the next calibration to get the latest values */
} RADIO_TIMER_ContextTypeDef;


static RADIO_TIMER_ContextTypeDef RADIO_TIMER_Context;
uint32_t blue_unit_conversion(uint32_t time, uint32_t period_freq,
							  uint32_t thr);
uint64_t _get_system_time_and_machine(RADIO_TIMER_ContextTypeDef *context, uint32_t *current_machine_time);
void _check_radio_activity(RADIO_TIMER_RadioHandleTypeDef *timerHandle, uint8_t *expired);
void _set_controller_as_host(void);
void _virtualTimeBaseEnable(uint8_t state);

void radio_timer_init(void);
void RADIO_TIMER_Tick(void);

#endif