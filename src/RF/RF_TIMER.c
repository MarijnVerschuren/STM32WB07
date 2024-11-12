#include "RF/RF_TIMER.h"


/* ! copied from cubeide */

uint32_t uwPRIMASK_Bit;
__attribute__((always_inline)) static inline void ATOMIC_SECTION_BEGIN(void){
	uwPRIMASK_Bit = __GET_PRIMASK();
	__IRQ_D();
}

__attribute__((always_inline)) static inline void ATOMIC_SECTION_END(void){
	__SET_PRIMASK(uwPRIMASK_Bit);
	__IRQ_E();
}
#define DIFF8(a,b) ((a)>=(b) ? ((a)-(b)) : (256+((a)-(b))))


#define TIMER_MAX_VALUE 0xFFFFFFFFUL
#define TIME_ABSDIFF(a, b)       ((a - b) & TIMER_MAX_VALUE)
#define INCREMENT_EXPIRE_COUNT_ISR (RADIO_TIMER_Context.expired_count\
= ((RADIO_TIMER_Context.expired_count + 1) == RADIO_TIMER_Context.served_count) ? RADIO_TIMER_Context.expired_count : (RADIO_TIMER_Context.expired_count + 1))
#define INCREMENT_EXPIRE_COUNT ATOMIC_SECTION_BEGIN(); INCREMENT_EXPIRE_COUNT_ISR ; ATOMIC_SECTION_END();
#define WAKEUP_INIT_DELAY (27) /* about 65us in STU */

uint64_t _get_system_time_and_machine(RADIO_TIMER_ContextTypeDef *context, uint32_t *current_machine_time) {
    uint32_t difftime;
    uint64_t new_time;

    ATOMIC_SECTION_BEGIN();
    new_time = context->cumulative_time;
    *current_machine_time =  WAKEUP->ABSOLUTE_TIME;
    difftime = TIME_ABSDIFF(*current_machine_time, context->last_machine_time);
    new_time += blue_unit_conversion(difftime, context->calibrationData.period1, MULT64_THR_PERIOD);
    if (new_time < context->last_system_time) {
        new_time += blue_unit_conversion(TIMER_MAX_VALUE, context->calibrationData.period1, MULT64_THR_PERIOD);
    }
    context->last_system_time = new_time;
    ATOMIC_SECTION_END();

    return new_time;
}

static uint8_t TIMER_SetRadioTimerValue(uint32_t timeout, uint8_t event_type, uint8_t cal_req)
{
  uint32_t current_time, delay, radio_init_delay, device_delay, rel_timeout, rel_timeout_mt;
  uint8_t ret_val;

  /*choose the 2nd init duration. Check the event_type and cal. request*/
  if (event_type == TX)
  {
    if (cal_req)
    {
      radio_init_delay = RADIO_TIMER_Context.TxRxDelay.tx_cal_delay;
      device_delay = RADIO_TIMER_Context.TxRxDelay.tx_cal_delay_st;
    }
    else
    {
      radio_init_delay = RADIO_TIMER_Context.TxRxDelay.tx_no_cal_delay;
      device_delay = RADIO_TIMER_Context.TxRxDelay.tx_cal_delay_st;
    }
  }
  else
  {
    if (cal_req)
    {
      radio_init_delay = RADIO_TIMER_Context.TxRxDelay.rx_cal_delay;
      device_delay = RADIO_TIMER_Context.TxRxDelay.tx_cal_delay_st;
    }
    else
    {
      radio_init_delay = RADIO_TIMER_Context.TxRxDelay.rx_no_cal_delay;
      device_delay = RADIO_TIMER_Context.TxRxDelay.tx_cal_delay_st;
    }
  }

  /* At this point, it is care of the upper layers to guarantee that the timeout represents an absolute time in the future */
  rel_timeout = timeout - (uint32_t)_get_system_time_and_machine(&RADIO_TIMER_Context, &current_time);

  rel_timeout_mt =  blue_unit_conversion(rel_timeout, RADIO_TIMER_Context.calibrationData.freq1, MULT64_THR_FREQ);

  /*Check if the timeout is beyond the wakeup time offset. Then program either the WakeUp timer or the Timer1*/
  if (rel_timeout > (device_delay + RADIO_TIMER_Context.hs_startup_time + MARGIN_EXT))
  {
    /*The timeout is after the wakeup_time_offset, So it is ok to program the wakeup timer*/
    delay = rel_timeout_mt - ble_globstate.WAKE_INIT_DELAY  - radio_init_delay;
    WAKEUP->BLUE_WAKEUP_TIME = ((current_time + delay) & TIMER_MAX_VALUE);
    WAKEUP->BLUE_SLEEP_REQUEST_MODE = 0;
    RADIO->TODR = 0b00;
    WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0b1U << 30U);
      WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0b1U << 29U);
    radio_init_delay += ble_globstate.WAKE_INIT_DELAY ;
  }
  else
  {
    delay = rel_timeout_mt - RADIO_TIMER_Context.TxRxDelay.tim12_delay_mt - radio_init_delay;
    RADIO->TOR = ((current_time + delay) & TIMER_MAX_VALUE);
      WAKEUP->BLUE_SLEEP_REQUEST_MODE &= 0xBFFFFFFF;
      RADIO->TODR = 0b10;
    radio_init_delay += RADIO_TIMER_Context.TxRxDelay.tim12_delay_mt;
  }

  RADIO_TIMER_Context.last_anchor_mt = (current_time + rel_timeout_mt) & TIMER_MAX_VALUE;

	ble_globstate.ACTIVE = 0b1;
	ble_globstate.ADD_PTR_ERR = 0b1;
	ble_globstate.TBL_RDY_ERR = 0b1;
	ble_globstate.TX_DATA_ERR = 0b1;
	ble_globstate.ACT_LBIT_ERR = 0b1;
	BLE_GLOB_STATE_W[5] |= 0xFF000000;

  /* Basic low level check with an extra margin of machine units */
  if ((delay + radio_init_delay) < (radio_init_delay + 5))
  {
      RADIO->TODR = 0b00;
      WAKEUP->BLUE_SLEEP_REQUEST_MODE &= 0xBFFFFFFF;
    ret_val =  1;
  }
  else
  {
    RADIO_TIMER_Context.last_setup_time = blue_unit_conversion(radio_init_delay, RADIO_TIMER_Context.calibrationData.period1, MULT64_THR_PERIOD);
    ret_val = 0;
  }

  return ret_val;
}

void _check_radio_activity(RADIO_TIMER_RadioHandleTypeDef *timerHandle, uint8_t *expired)
{
    uint64_t nextCalibrationEvent, currentTime;

    *expired = 0;

    if (timerHandle->pending)
    {
        nextCalibrationEvent = RADIO_TIMER_Context.calibrationData.last_calibration_time + \
                               RADIO_TIMER_Context.calibrationSettings.periodicCalibrationInterval;

        ATOMIC_SECTION_BEGIN();
        uint32_t current_machine_time;
        currentTime = _get_system_time_and_machine(&RADIO_TIMER_Context, &current_machine_time);
        if ((timerHandle->expiryTime < (nextCalibrationEvent + RADIO_ACTIVITY_MARGIN)) || \
            (currentTime > (nextCalibrationEvent + CALIB_SAFE_THR)))
        {
            if (timerHandle->expiryTime - TIMER1_INIT_DELAY > (currentTime + TIMER1_MARGIN))
            {
                *expired = TIMER_SetRadioTimerValue(timerHandle->expiryTime, timerHandle->event_type, timerHandle->cal_req);
                timerHandle->pending = 0b0; /* timer has been served. No more pending */
                timerHandle->active = 0b1; /* timer has been programmed and it becomes ACTIVE */
                timerHandle->intTxRx_to_be_served = 0b1;
            }
            else
            {
                RADIO_TIMER_Context.radioTimer.pending = 0b0;
                *expired = 1;
            }
        }
        else
        {
            RADIO_TIMER_Context.waitCal = 1;
        }
        ATOMIC_SECTION_END();
    }
}

void _set_controller_as_host(void)
{
	ble_globstate.ACTIVE = 0b0;
	BLE_GLOB_STATE_W[5] &= 0x0000FFFFUL;
}

static uint32_t TIMER_SetRadioHostWakeupTime(uint32_t delay, uint8_t *share)
{
    uint32_t current_time;

    delay = blue_unit_conversion(delay, RADIO_TIMER_Context.calibrationData.freq1, MULT64_THR_FREQ) ;
    /* If the delay is too small round to minimum 2 tick */
    delay = delay >= 32 ? delay : 32;
    current_time = WAKEUP->ABSOLUTE_TIME;
    /* 4 least significant bits are not taken into account. Then let's round the value */
    WAKEUP->CM0_WAKEUP_TIME = ((current_time + (delay + 8)) & TIMER_MAX_VALUE);
    WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0b1U << 29U);
    WAKEUP->CM0_SLEEP_REQUEST_MODE |= (0b1U << 30U);
    if ((RADIO->TODR == 0b10U || RADIO->TODR == 0b11U || (*share != 0b1)))
    {
        *share = 0b0;
        return current_time;
    }
    _set_controller_as_host();
    /* 4 least significant bits are not taken into account. Then let's round the value */
    WAKEUP->BLUE_WAKEUP_TIME = (current_time + delay + 8) & 0xFFFFFFF0;
    WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0 & 0x7);
    WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0b1 << 30U);
    WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0b1 << 29U);
    return current_time;
}

static VTIMER_HandleType *_update_user_timeout(VTIMER_HandleType *rootNode, uint8_t *expired)
{
  VTIMER_HandleType *curr = rootNode;
  VTIMER_HandleType *rootOrig = rootNode;
  int64_t delay;
  *expired = 0;
  while (curr != NULL)
  {
    if (curr->active)
    {
      ATOMIC_SECTION_BEGIN();

      uint8_t dummy;
      uint8_t share = 0b01U;
      _check_radio_activity(&RADIO_TIMER_Context.radioTimer, &dummy);
        uint32_t current_machine_time;
      delay = curr->expiryTime - _get_system_time_and_machine(&RADIO_TIMER_Context, &current_machine_time);;
      if (delay > 0)
      {
        /* Protection against interrupt must be used to avoid that the called function will be interrupted
          and so the timer programming will happen after the target time is already passed
          leading to a timer expiring after timer wraps, instead of the expected delay */

        /* Is the active radio operation before or too close the host timeout? */
        if (((RADIO_TIMER_Context.radioTimer.expiryTime) < (curr->expiryTime + RADIO_TIMER_Context.hostMargin))
            && RADIO_TIMER_Context.radioTimer.active)
        {
          if ((RADIO_TIMER_Context.radioTimer.expiryTime >= curr->expiryTime) && RADIO_TIMER_Context.radioTimer.active)
          {
            RADIO_TIMER_Context.hostIsRadioPending = 1;
          }
        }
        else
        {
          /* It's fine to program the wakeup timer for an host wakeup */
          share = 0b1;
        }
        TIMER_SetRadioHostWakeupTime(delay, &share);
        if (share == 0b1)
        {
          RADIO_TIMER_Context.radioTimer.pending |= RADIO_TIMER_Context.radioTimer.active;
          RADIO_TIMER_Context.radioTimer.active = 0b0;
        }
      }
      else
      {
        *expired = 1;
      }
      ATOMIC_SECTION_END();
      break;
    }
    curr = curr->next;
  }
  if (*expired)
  {
    return rootOrig;
  }

  return curr;
}

static VTIMER_HandleType* _insert_timer_in_queue(VTIMER_HandleType* rootNode, VTIMER_HandleType* handle) {
    VTIMER_HandleType *current = rootNode;
    VTIMER_HandleType *prev = NULL;
    VTIMER_HandleType *returnValue = rootNode;

    while ((current != NULL) && (current->expiryTime < handle->expiryTime)) {
        prev = current;
        current = current->next;
    }
    handle->next = current;
    if (prev == NULL) {
        /* We are the new root */
        returnValue = handle;
    }
    else {
        prev->next = handle;
    }

    return returnValue;
}

static int32_t _start_timer(VTIMER_HandleType *timerHandle, uint64_t time) {
    uint8_t expired = 0;

    /* The timer is already started*/
    if (timerHandle->active) {
        return 1;
    }
    timerHandle->expiryTime = time;
    timerHandle->active = 0b1U;
	// TODO MARIJN << NOTE INVALID FROM THIS POINT!
    if (_insert_timer_in_queue(RADIO_TIMER_Context.rootNode, timerHandle) == timerHandle) {  // TODO: error prone
        RADIO_TIMER_Context.rootNode = _update_user_timeout(timerHandle, &expired);
        if (expired) {
            /* A new root timer is already expired, mimic timer expire that is normally signaled
             through the interrupt handler that increase the number of expired timers*/
            INCREMENT_EXPIRE_COUNT;
        }
    }
    return expired;
}

static uint32_t _us_to_machinetime(uint32_t time)
{
    uint64_t tmp = (uint64_t)RADIO_TIMER_Context.calibrationData.freq * (uint64_t)time * (uint64_t)3U;
    uint32_t time_mt = ((tmp + (1 << 26)) >> 27) & TIMER_MAX_VALUE;

    return time_mt;
}

static uint32_t _us_to_systime(uint32_t time)
{
    uint32_t t1, t2;
    t1 = time * 0x68;
    t2 = time * 0xDB;
    return (t1 >> 8) + (t2 >> 16);
}

static void _configureTxRxDelay(RADIO_TIMER_ContextTypeDef *context, uint8_t calculate_st)
{
    uint8_t tx_delay_start;

    tx_delay_start = (ble_globstate.TX_DEL_START * 125 / 1000) + 1;
	ble_globstate.WAKE_INIT_DELAY =  blue_unit_conversion(WAKEUP_INIT_DELAY, context->calibrationData.freq1, MULT64_THR_FREQ);
    context->TxRxDelay.tim12_delay_mt = _us_to_machinetime(ble_globstate.TIM_12_DEL_CAL);
    context->TxRxDelay.tx_cal_delay = _us_to_machinetime(ble_globstate.TX_CAL_DEL + tx_delay_start);
    context->TxRxDelay.tx_no_cal_delay = _us_to_machinetime(ble_globstate.TX_NOCAL_DEL + tx_delay_start);
    context->TxRxDelay.rx_cal_delay = _us_to_machinetime(ble_globstate.RX_CAL_DEL);
    context->TxRxDelay.rx_no_cal_delay = _us_to_machinetime(ble_globstate.RX_NOCAL_DEL);

    if (calculate_st) {
        context->TxRxDelay.tx_cal_delay_st    = _us_to_systime(ble_globstate.TX_CAL_DEL + tx_delay_start) + WAKEUP_INIT_DELAY;
    }

}

static void _calibration_callback(void *handle) {
	if (RADIO_TIMER_Context.calibrationSettings.periodicCalibration)
	{
		// _timer_start_calibration(); // need to implement if we want calibration
	}
	RADIO_TIMER_Context.calibrationSettings.calibration_in_progress = 0b1;
}

void radio_timer_init(void){
    /* Wait to be sure that the Radio Timer is active */
    while(WAKEUP->ABSOLUTE_TIME < 0x10);

    WAKEUP->WAKEUP_CM0_IRQ_STATUS |= 0b1U;
    WAKEUP->WAKEUP_CM0_IRQ_ENABLE |= 0b1U;
    NVIC_enable_IRQ(RADIO_TIM_CPU_WKUP_IRQn);
    NVIC_enable_IRQ(RADIO_TIM_error_IRQn);

    WAKEUP->WAKEUP_BLE_IRQ_STATUS |= 0b1U;
    WAKEUP->WAKEUP_BLE_IRQ_ENABLE |= 0b1U;
    NVIC_enable_IRQ(RADIO_TIM_TXRX_WKUP_IRQn);
    RADIO_TIMER_Context.hostMargin = XTAL_StartupTime;

    /* Calibration Setting */
    /* Assume fix frequency at 32.768 kHz */
    RADIO_TIMER_Context.calibrationData.last_period1 = 0x00190000;
    RADIO_TIMER_Context.calibrationData.period1 = 0x00190000 ;
    RADIO_TIMER_Context.calibrationData.freq1 = 0x0028F5C2 ;
    RADIO_TIMER_Context.calibrationData.period = 23437;
    RADIO_TIMER_Context.calibrationData.freq = 23456748;
    RADIO_TIMER_Context.calibrationSettings.periodicCalibrationInterval = blue_unit_conversion(0x50000000,RADIO_TIMER_Context.calibrationData.period1,MULT64_THR_PERIOD);
    RADIO_TIMER_Context.calibrationSettings.calibration_in_progress = 0;
\
    /* XTAL startup time configuration */
    RADIO_TIMER_Context.hs_startup_time = XTAL_StartupTime;
    int32_t time1 = blue_unit_conversion(RADIO_TIMER_Context.hs_startup_time, RADIO_TIMER_Context.calibrationData.freq1, MULT64_THR_FREQ);
    if (time1 > 4095)	{ time1 = 4095; }
    if (time1 < 16)		{ time1 = 16; }
    WAKEUP->WAKEUP_OFFSET[0] = ((time1 >> 4) & 0xFF);

    /* Init Radio Timer Context */
    RADIO_TIMER_Context.last_setup_time = 0;
    RADIO_TIMER_Context.cumulative_time = 0;
    RADIO_TIMER_Context.last_machine_time = WAKEUP->ABSOLUTE_TIME;
    RADIO_TIMER_Context.last_system_time = 0;
    RADIO_TIMER_Context.calibrationData.last_calibration_time = 0;
    RADIO_TIMER_Context.calibrationData.calibration_data_available = 0;
    RADIO_TIMER_Context.calibrationData.calibration_machine_interval = blue_unit_conversion(RADIO_TIMER_Context.calibrationSettings.periodicCalibrationInterval,
                                                                       RADIO_TIMER_Context.calibrationData.freq1, MULT64_THR_FREQ);
    RADIO_TIMER_Context.wakeup_calibration = RADIO_TIMER_Context.calibrationSettings.periodicCalibration;

    /* Init the Virtual Timer queue */
    RADIO_TIMER_Context.rootNode = NULL;
    RADIO_TIMER_Context.enableTimeBase = 0b1U;
    RADIO_TIMER_Context.stop_notimer_action = 0b0U;
    RADIO_TIMER_Context.expired_count = 0;
    RADIO_TIMER_Context.served_count = 0;

    /* Init Radio Timer queue */
    RADIO_TIMER_Context.radioTimer.active = 0b0U;
    RADIO_TIMER_Context.radioTimer.pending = 0b0U;
    RADIO_TIMER_Context.radioTimer.intTxRx_to_be_served = 0b0U;
    RADIO_TIMER_Context.radioTimer.expiryTime = 0;

    /* Configure the Calibration callback and schedule the next calibration */
    RADIO_TIMER_Context.calibrationTimer.callback = _calibration_callback;
    RADIO_TIMER_Context.calibrationTimer.userData = NULL;

	uint32_t current_machine_time;
	(void)_start_timer(  // TODO MARIJN unfinnished (stupid func tbh)
		&RADIO_TIMER_Context.calibrationTimer,
		_get_system_time_and_machine(&RADIO_TIMER_Context, &current_machine_time) +\
		RADIO_TIMER_Context.calibrationSettings.periodicCalibrationInterval
	);

    /* Tx & Rx delay configuration */
    _configureTxRxDelay(&RADIO_TIMER_Context, 0b1);
}

static void _check_host_activity(void) {
	uint8_t expired;
	RADIO_TIMER_Context.rootNode = _update_user_timeout(RADIO_TIMER_Context.rootNode, &expired);
	if (expired == 1) {
		INCREMENT_EXPIRE_COUNT_ISR;
	}
}

uint32_t RADIO_TIMER_ClearRadioTimerValue(void) {
	int64_t time_diff;

	RADIO->TODR &= ~(0x00000003UL);
	WAKEUP->BLUE_SLEEP_REQUEST_MODE &= ~(0x40000000UL);
	RADIO_TIMER_Context.radioTimer.active = 0;
	RADIO_TIMER_Context.radioTimer.pending = 0;
	RADIO_TIMER_Context.radioTimer.intTxRx_to_be_served = 0;

	/*The rfSetup is different if Timer1 or Wakeup timer is programmed*/
	uint32_t _;
	ATOMIC_SECTION_BEGIN();
	time_diff = RADIO_TIMER_Context.radioTimer.expiryTime \
              - _get_system_time_and_machine(&RADIO_TIMER_Context, &_) \
              - RADIO_TIMER_Context.last_setup_time;
	ATOMIC_SECTION_END();

	/* Check if the routine is executed in the Tx/Rx interrupt handler or not */
	if (((SCB->ICSR & 0x1FFUL) - 16) != RADIO_TX_RX_EXCEPTION_NUMBER) {
		_check_host_activity();
	}

	if (time_diff <= 0)				{ return 1; }
	if (time_diff < CLEAR_MIN_THR)	{ return 2; }
	return 0;
}

/* Quinten */
static VTIMER_HandleType *check_callbacks(VTIMER_HandleType *rootNode, VTIMER_HandleType **expiredList) {
	VTIMER_HandleType *curr = rootNode;
	VTIMER_HandleType *prev = NULL;
	VTIMER_HandleType *returnValue = rootNode;
	*expiredList = rootNode;
	int64_t delay;
	uint32_t expiredCount = 0;

	while (curr != NULL) {
		if (curr->active) {
			uint32_t current_machine_time;
			delay = curr->expiryTime - _get_system_time_and_machine(&RADIO_TIMER_Context, &current_machine_time);

			if (delay > 5) { break; }
		}

		prev = curr;
		curr = curr->next;
		expiredCount++;
	}

	if (expiredCount) {
		/* Some timers expired */
		prev->next = NULL;
		returnValue = curr;
	} else {
		/* No timer expired */
		*expiredList = NULL;
	}

	return returnValue;
}

static void get_calibration_data(CalibrationDataTypeDef *calibration_data) {
	int32_t period;
	int32_t freq;
	int32_t mul1;
	int32_t b1;
	int32_t b2;
	int32_t mult;
	int32_t a1;
	int32_t a2;

	// 0x0003FFFFUL

	period =  RADIO_CTRL->SCPR; // LL_RADIO_TIMER_GetLSIPeriod
	while (period != (RADIO_CTRL->SCPR) || period == 0) // LL_RADIO_TIMER_GetLSIPeriod
	{
		period = RADIO_CTRL->SCPR; // LL_RADIO_TIMER_GetLSIPeriod
	}

	mul1 = 0x8BCF6 ;
	b1 = period >> 8 ;
	b2 = period & 0xff ;
	calibration_data->period1 = ((mul1 * b1) + ((b2 * mul1) >> 8) + 16) >> 5;
	calibration_data->period = period;

	mult = 0x753 ;

	// 0x04FFFFFFUL

	freq = RADIO_CTRL->SCFR; // LL_RADIO_TIMER_GetLSIFrequency

	while (freq != (RADIO_CTRL->SCFR) || freq == 0) // LL_RADIO_TIMER_GetLSIFrequency
	{
		freq = RADIO_CTRL->SCFR; // LL_RADIO_TIMER_GetLSIFrequency
	}
	a1 = freq >> 6 ;
	a2 = a1 * mult ;
	calibration_data->freq1 = (a2 + 128) >> 8 ;
	calibration_data->freq = freq;
}

static void update_xtal_startup_time(uint16_t hs_startup_time, int32_t freq1)
{
	int32_t time1;

	time1 = blue_unit_conversion(hs_startup_time, freq1, MULT64_THR_FREQ);
	if (time1 > 4095)	{ time1 = 4095; }
	if (time1 < 16)		{ time1 = 16; }
	WAKEUP->WAKEUP_OFFSET[0] = ((time1 >> 4) & 0xFF);
}

static void update_system_time(RADIO_TIMER_ContextTypeDef *context)
{
	uint32_t current_machine_time;
	uint32_t period;

	current_machine_time = WAKEUP->ABSOLUTE_TIME; // LL_RADIO_TIMER_GetAbsoluteTime
	period = context->calibrationData.last_period1;
	context->cumulative_time = context->calibrationData.last_calibration_time + \
                             blue_unit_conversion(TIME_ABSDIFF(current_machine_time,
															   context->last_machine_time),
												  period, MULT64_THR_PERIOD);

	if ((context->calibrationSettings.periodicCalibration == 0)
		&& (TIME_ABSDIFF(current_machine_time,
						 context->last_machine_time) < context->calibrationData.calibration_machine_interval))
	{
		context->cumulative_time += blue_unit_conversion(TIMER_MAX_VALUE, period, MULT64_THR_PERIOD);
	}
	context->last_machine_time = current_machine_time;
	context->calibrationData.last_calibration_time = context->cumulative_time;
	context->calibrationData.last_period1 = context->calibrationData.period1;
}

static void update_calibration_data(void) {
	if (RADIO_TIMER_Context.calibrationSettings.periodicCalibration)
	{
		get_calibration_data(&RADIO_TIMER_Context.calibrationData);
		update_xtal_startup_time(RADIO_TIMER_Context.hs_startup_time, RADIO_TIMER_Context.calibrationData.freq1);
		_configureTxRxDelay(&RADIO_TIMER_Context, 0);
		RADIO_TIMER_Context.calibrationData.calibration_data_available = 1;
	}
	ATOMIC_SECTION_BEGIN();
	update_system_time(&RADIO_TIMER_Context);
	ATOMIC_SECTION_END();
}

static VTIMER_HandleType *remove_timer_in_queue(VTIMER_HandleType *rootNode, VTIMER_HandleType *handle)
{
	VTIMER_HandleType *current = rootNode;
	VTIMER_HandleType *prev = NULL;
	VTIMER_HandleType *returnValue = rootNode;

	while ((current != NULL) && (current != handle)) {
		prev = current;
		current = current->next;
	}

	if (current == NULL) {
		/* Not found */
	}
	else if (current == rootNode) {
		/* New root node */
		returnValue = current->next;
	}
	else {
		prev->next = current->next;
	}

	return returnValue;
}

// start TODO

void HAL_RADIO_TIMER_StopVirtualTimer(VTIMER_HandleType *timerHandle)
{
	VTIMER_HandleType *rootNode = remove_timer_in_queue(RADIO_TIMER_Context.rootNode, timerHandle);
	uint8_t expired = 0;
	timerHandle->active = 0;
	if (RADIO_TIMER_Context.rootNode != rootNode)
	{
		RADIO_TIMER_Context.rootNode = _update_user_timeout(rootNode, &expired);
		if (expired)
		{
			/* A new root timer is already expired, mimic timer expire */
			INCREMENT_EXPIRE_COUNT;
		}
	}
	else
	{
		RADIO_TIMER_Context.rootNode = rootNode;
	}
}

static VTIMER_HandleType *_remove_timer_in_queue(VTIMER_HandleType *rootNode, VTIMER_HandleType *handle)
{
	VTIMER_HandleType *current = rootNode;
	VTIMER_HandleType *prev = NULL;
	VTIMER_HandleType *returnValue = rootNode;

	while ((current != NULL) && (current != handle))
	{
		prev = current;
		current = current->next;
	}

	if (current == NULL)
	{
		/* Not found */
	}
	else if (current == rootNode)
	{
		/* New root node */
		returnValue = current->next;
	}
	else
	{
		prev->next = current->next;
	}

	return returnValue;
}

void _virtualTimeBaseEnable(uint8_t state)
{
	if (state != 0)
	{
		if (RADIO_TIMER_Context.enableTimeBase == 0)
		{
			RADIO_TIMER_Context.calibrationSettings.calibration_in_progress = 1;
			RADIO_TIMER_Context.enableTimeBase = 1;
		}
	}
	else
	{
		VTIMER_HandleType *rootNode = _remove_timer_in_queue(RADIO_TIMER_Context.rootNode, &RADIO_TIMER_Context.calibrationTimer);
		uint8_t expired = 0;
		RADIO_TIMER_Context.calibrationTimer.active = 0;
		if (RADIO_TIMER_Context.rootNode != rootNode)
		{
			RADIO_TIMER_Context.rootNode = _update_user_timeout(rootNode, &expired);
			if (expired)
			{
				/* A new root timer is already expired, mimic timer expire */
				INCREMENT_EXPIRE_COUNT;
			}
		}
		else
		{
			RADIO_TIMER_Context.rootNode = rootNode;
		}
		RADIO_TIMER_Context.enableTimeBase = 0;
	}
}

void RADIO_TIMER_Tick(void) { // VALID until comment
	uint8_t expired = 0;
	ATOMIC_SECTION_BEGIN();
	if (RADIO_TIMER_Context.radioTimer.active) {
		uint32_t current_machine_time;
		if (RADIO_TIMER_Context.radioTimer.expiryTime < _get_system_time_and_machine(&RADIO_TIMER_Context, &current_machine_time)) {
			RADIO_TIMER_Context.radioTimer.active = 0;
		}
	}
	ATOMIC_SECTION_END();


	/* Check for expired timers */
	while (DIFF8(RADIO_TIMER_Context.expired_count, RADIO_TIMER_Context.served_count)) {
		VTIMER_HandleType *expiredList, *curr;
		uint8_t to_be_served = DIFF8(RADIO_TIMER_Context.expired_count, RADIO_TIMER_Context.served_count);

		RADIO_TIMER_Context.rootNode = check_callbacks(RADIO_TIMER_Context.rootNode, &expiredList);

		/* Call all the user callbacks */
		curr = expiredList;
		while (curr != NULL)
		{
			/* Save next pointer, in case callback start the timer again */
			VTIMER_HandleType *next = curr->next;
			curr->active = 0;
			if (curr->callback)
			{
				curr->callback(curr); /* we are sure a callback is set?*/
			}
			curr = next;
		}

		RADIO_TIMER_Context.rootNode = _update_user_timeout(RADIO_TIMER_Context.rootNode, &expired);
		if (expired == 1)
		{
			/* A new root timer is already expired, mimic timer expire */
			INCREMENT_EXPIRE_COUNT;
		}
		RADIO_TIMER_Context.served_count += to_be_served;

		/* Check for periodic calibration */
		if (RADIO_TIMER_Context.calibrationSettings.calibration_in_progress)
		{
			if ((RADIO_CTRL->ISR & 0b1U))
			{
				/* Calibration is completed */
				RADIO_TIMER_Context.calibrationSettings.calibration_in_progress = 0;
				if ((RADIO_TIMER_Context.wakeup_calibration == 0) && RADIO_TIMER_Context.stop_notimer_action)
				{
					RADIO_TIMER_Context.stop_notimer_action = 0;
				}
				else
				{
					/* Collect calibration data */
					update_calibration_data();
				}


				if (RADIO_TIMER_Context.waitCal)
			    {
					RADIO_TIMER_Context.waitCal = 0;
					RADIO_TIMER_Context.radioTimer.pending = 1	;
					_check_radio_activity(&RADIO_TIMER_Context.radioTimer, &expired);
					RADIO_TIMER_Context.rootNode = _update_user_timeout(RADIO_TIMER_Context.rootNode, &expired);
			    }

				HAL_RADIO_TIMER_StopVirtualTimer(&RADIO_TIMER_Context.calibrationTimer);
				/* Schedule next calibration event */
				uint32_t current_machine_time;
				_start_timer(&RADIO_TIMER_Context.calibrationTimer,
							 _get_system_time_and_machine(&RADIO_TIMER_Context, &current_machine_time) + RADIO_TIMER_Context.calibrationSettings.periodicCalibrationInterval); // HAL_RADIO_TIMER_GetCurrentSysTime
			}
		}
		/* if there is a periodic calibration, start it in advance during the active phase */
		else
		{
			if (RADIO_TIMER_Context.calibrationSettings.periodicCalibration)
			{
				uint32_t current_machine_time;
				if (_get_system_time_and_machine(&RADIO_TIMER_Context, &current_machine_time) > (RADIO_TIMER_Context.calibrationData.last_calibration_time + 2048000)) // HAL_RADIO_TIMER_GetCurrentSysTime
				{
					_calibration_callback(&RADIO_TIMER_Context.calibrationTimer);
				}
			}
		}
	}
}