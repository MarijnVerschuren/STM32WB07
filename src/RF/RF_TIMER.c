#include "RF_TIMER.h"

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





/* ! copied from cubeide */

__attribute__((always_inline)) static inline uint32_t __get_PRIMASK(void)
{
    uint32_t result;

    __asm volatile ("MRS %0, primask" : "=r" (result) );
    return(result);
}

__attribute__((always_inline)) static inline void __disable_irq(void)
{
    __asm volatile ("cpsid i" : : : "memory");
}

__attribute__((always_inline)) static inline void __set_PRIMASK(uint32_t priMask)
{
    __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}

#define ATOMIC_SECTION_BEGIN() uint32_t uwPRIMASK_Bit = __get_PRIMASK(); \
__disable_irq(); \
/* Must be called in the same scope of ATOMIC_SECTION_BEGIN */
#define ATOMIC_SECTION_END() __set_PRIMASK(uwPRIMASK_Bit)

#define TIMER_BITS (32)
#define TIMER_MAX_VALUE (0xFFFFFFFFU >> (32-TIMER_BITS))
#define TIME_ABSDIFF(a, b)       ((a - b) & TIMER_MAX_VALUE)
#define INCREMENT_EXPIRE_COUNT_ISR (RADIO_TIMER_Context.expired_count\
= ((RADIO_TIMER_Context.expired_count + 1) == RADIO_TIMER_Context.served_count) ? RADIO_TIMER_Context.expired_count : (RADIO_TIMER_Context.expired_count + 1))
#define INCREMENT_EXPIRE_COUNT ATOMIC_SECTION_BEGIN(); INCREMENT_EXPIRE_COUNT_ISR ; ATOMIC_SECTION_END();

static uint64_t _get_system_time_and_machine(RADIO_TIMER_ContextTypeDef *context, uint32_t *current_machine_time)
{
    uint32_t difftime;
    uint64_t new_time;

    ATOMIC_SECTION_BEGIN();
    new_time = context->cumulative_time;
    *current_machine_time =  WAKEUP->ABSOLUTE_TIME;
    difftime = TIME_ABSDIFF(*current_machine_time, context->last_machine_time);
    new_time += blue_unit_conversion(difftime, context->calibrationData.period1, MULT64_THR_PERIOD);
    if (new_time < context->last_system_time)
    {
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
    delay = rel_timeout_mt - (uint8_t)(RFW_state[1] >> 8U) - radio_init_delay;
    WAKEUP->BLUE_WAKEUP_TIME = ((current_time + delay) & TIMER_MAX_VALUE);
    WAKEUP->BLUE_SLEEP_REQUEST_MODE = 0;
    RADIO->TODR = 0b00;
    WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0b1U << 30U);
      WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0b1U << 29U);
    radio_init_delay += (uint8_t)(RFW_state[1] >> 8U);
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

    RFW_state[1] &= ~0xFFU;
  RFW_state[1] |= 0b1U << 7U;
    RFW_state[5] &= 0xFF0000FF;
  RFW_state[5] = 0xF0 << 16U;
  RFW_state[5] = 0xFF << 24U;


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




static void _check_radio_activity(RADIO_TIMER_RadioHandleTypeDef *timerHandle, uint8_t *expired)
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

static void _set_controller_as_host(void)
{
    RFW_state[1] &= 0xBFU;
    RFW_state[5] &= 0xFF0000F;
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
    WAKEUP->BLUE_WAKEUP_TIME ((current_time + delay + 8) & 0xFFFFFFF0);
    WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0 & 0x7);
    WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0b1 << 30U);
    WAKEUP->BLUE_SLEEP_REQUEST_MODE |= (0b1 << 29U);
    return current_time;
}


/* Set timeout and skip non active timers */
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


static VTIMER_HandleType *_insert_timer_in_queue(VTIMER_HandleType *rootNode, VTIMER_HandleType *handle)
{
    VTIMER_HandleType *current = rootNode;
    VTIMER_HandleType *prev = NULL;
    VTIMER_HandleType *returnValue = rootNode;

    while ((current != NULL) && (current->expiryTime < handle->expiryTime))
    {
        prev = current;
        current = current->next;
    }

    handle->next = current;

    if (prev == NULL)
    {
        /* We are the new root */
        returnValue = handle;
    }
    else
    {
        prev->next = handle;
    }

    return returnValue;
}

static int32_t _start_timer(VTIMER_HandleType *timerHandle, uint64_t time)
{
    uint8_t expired = 0;

    /* The timer is already started*/
    if (timerHandle->active)
    {
        return 1;
    }
    timerHandle->expiryTime = time;
    timerHandle->active = 0b1U;
    if (_insert_timer_in_queue(RADIO_TIMER_Context.rootNode, timerHandle) == timerHandle)
    {
        RADIO_TIMER_Context.rootNode = _update_user_timeout(timerHandle, &expired);
        if (expired)
        {
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
 // TODO
static void _configureTxRxDelay(RADIO_TIMER_ContextTypeDef *context, uint8_t calculate_st)
{
    uint8_t tx_delay_start;

    tx_delay_start = (BLUEGLOB->TXDELAYSTART * 125 / 1000) + 1;

    BLUEGLOB->WAKEUPINITDELAY =  blue_unit_conversion(WAKEUP_INIT_DELAY, context->calibrationData.freq1, MULT64_THR_FREQ);
    context->TxRxDelay.tim12_delay_mt = _us_to_machinetime(BLUEGLOB->TIMER12INITDELAYCAL);
    context->TxRxDelay.tx_cal_delay = _us_to_machinetime(BLUEGLOB->TRANSMITCALDELAYCHK + tx_delay_start);
    context->TxRxDelay.tx_no_cal_delay = _us_to_machinetime(BLUEGLOB->TRANSMITNOCALDELAYCHK + tx_delay_start);
    context->TxRxDelay.rx_cal_delay = _us_to_machinetime(BLUEGLOB->RECEIVECALDELAYCHK);
    context->TxRxDelay.rx_no_cal_delay = _us_to_machinetime(BLUEGLOB->RECEIVENOCALDELAYCHK);

    if (calculate_st)
    {
        context->TxRxDelay.tx_cal_delay_st    = _us_to_systime(BLUEGLOB->TRANSMITCALDELAYCHK + tx_delay_start) + WAKEUP_INIT_DELAY;
    }

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
    RADIO_TIMER_Context.hostMargin = 320;

    /* Calibration Setting */
    /* Assume fix frequency at 32.768 kHz */
    RADIO_TIMER_Context.calibrationData.last_period1 = 0x00190000;
    RADIO_TIMER_Context.calibrationData.period1 = 0x00190000 ;
    RADIO_TIMER_Context.calibrationData.freq1 = 0x0028F5C2 ;
    RADIO_TIMER_Context.calibrationData.period = 23437;
    RADIO_TIMER_Context.calibrationData.freq = 23456748;

    RADIO_TIMER_Context.calibrationSettings.periodicCalibrationInterval = blue_unit_conversion(0x50000000,RADIO_TIMER_Context.calibrationData.period1,MULT64_THR_PERIOD);
    RADIO_TIMER_Context.calibrationSettings.calibration_in_progress = 0;

    /* XTAL startup time configuration */
    RADIO_TIMER_Context.hs_startup_time = 320;
    int32_t time1 = blue_unit_conversion(RADIO_TIMER_Context.hs_startup_time, RADIO_TIMER_Context.calibrationData.freq1, MULT64_THR_FREQ);
    if (time1 >= 4096)
    {
        time1 = 4095;
    }
    if (time1 < 16)
    {
        time1 = 16;
    }
    WAKEUP->WAKEUP_OFFSET[0] |= ((time1 >> 4) & 0xFF);

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
    RADIO_TIMER_Context.calibrationSettings.calibration_in_progress = 0b1U;
    RADIO_TIMER_Context.calibrationTimer.userData = NULL;

    uint32_t current_machine_time;
    _start_timer(&RADIO_TIMER_Context.calibrationTimer,
    _get_system_time_and_machine(&RADIO_TIMER_Context, &current_machine_time) + RADIO_TIMER_Context.calibrationSettings.periodicCalibrationInterval);

    /* Tx & Rx delay configuration */ // TODO
    _configureTxRxDelay(&RADIO_TIMER_Context, 0b1);
}