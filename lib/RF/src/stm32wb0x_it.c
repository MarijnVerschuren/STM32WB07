#include "main.h"
#include "stm32wb0x_it.h"


void RADIO_TIM_CPU_WKUP_handler(void)
{
  /* USER CODE BEGIN RADIO_TIMER_CPU_WKUP_IRQn 0 */

  /* USER CODE END RADIO_TIMER_CPU_WKUP_IRQn 0 */
  HAL_RADIO_TIMER_CPU_WKUP_IRQHandler();
  /* USER CODE BEGIN RADIO_TIMER_CPU_WKUP_IRQn 1 */

  /* USER CODE END RADIO_TIMER_CPU_WKUP_IRQn 1 */
}

void RADIO_TIM_TXRX_WKUP_handler(void)
{
  /* USER CODE BEGIN RADIO_TIMER_TXRX_WKUP_IRQn 0 */

  /* USER CODE END RADIO_TIMER_TXRX_WKUP_IRQn 0 */
  HAL_RADIO_TIMER_TXRX_WKUP_IRQHandler();
  /* USER CODE BEGIN RADIO_TIMER_TXRX_WKUP_IRQn 1 */

  /* USER CODE END RADIO_TIMER_TXRX_WKUP_IRQn 1 */
}

void RADIO_TIM_error_handler(void)
{
  /* USER CODE BEGIN RADIO_TIMER_ERROR_IRQn 0 */

  /* USER CODE END RADIO_TIMER_ERROR_IRQn 0 */
  HAL_RADIO_TIMER_ERROR_IRQHandler();
  /* USER CODE BEGIN RADIO_TIMER_ERROR_IRQn 1 */

  /* USER CODE END RADIO_TIMER_ERROR_IRQn 1 */
}

void RADIO_TXRX_handler(void)
{
  /* USER CODE BEGIN RADIO_TXRX_IRQn 0 */

  /* USER CODE END RADIO_TXRX_IRQn 0 */
  HAL_RADIO_TXRX_IRQHandler();
  /* USER CODE BEGIN RADIO_TXRX_IRQn 1 */

  /* USER CODE END RADIO_TXRX_IRQn 1 */
}

void RADIO_TXRX_SEQ_handler(void)
{
  /* USER CODE BEGIN RADIO_TXRX_SEQ_IRQn 0 */

  /* USER CODE END RADIO_TXRX_SEQ_IRQn 0 */
  HAL_RADIO_TXRX_SEQ_IRQHandler();
  /* USER CODE BEGIN RADIO_TXRX_SEQ_IRQn 1 */

  /* USER CODE END RADIO_TXRX_SEQ_IRQn 1 */
}