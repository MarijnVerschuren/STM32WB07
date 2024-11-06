#ifndef STM32WB07_RF_TIMER_H
#define STM32WB07_RF_TIMER_H

#include "WAKEUP.h"
#include "NVIC.h"
#include "RF/RF.h"
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

void radio_timer_init(void);

#endif