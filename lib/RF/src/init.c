//
// Created by marijn on 11/13/24.
//
#include "init.h"
// HAL includes
#define STM32WB07
#include "main.h"

void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	for (;;);
	/* USER CODE END Error_Handler_Debug */
}

extern volatile uint64_t tick;
uint32_t HAL_GetTick(void) {
	return tick & 0xFFFFFFFFUL;
}

void MX_RADIO_Init(void) {
	/* USER CODE BEGIN RADIO_Init 0 */

	/* USER CODE END RADIO_Init 0 */

	RADIO_HandleTypeDef hradio = {0};

	/* USER CODE BEGIN RADIO_Init 1 */

	/* USER CODE END RADIO_Init 1 */
	hradio.Instance = RADIO;
	HAL_RADIO_Init(&hradio);
	/* USER CODE BEGIN RADIO_Init 2 */

	/* USER CODE END RADIO_Init 2 */

}

void MX_RADIO_TIMER_Init(void) {
	RADIO_TIMER_InitTypeDef RADIO_TIMER_InitStruct = {0};
	if (__HAL_RCC_RADIO_IS_CLK_DISABLED()) {
		/* Radio Peripheral reset */
		__HAL_RCC_RADIO_FORCE_RESET();
		__HAL_RCC_RADIO_RELEASE_RESET();

		/* Enable Radio peripheral clock */
		__HAL_RCC_RADIO_CLK_ENABLE();
	}
	/* Wait to be sure that the Radio Timer is active */
	while(LL_RADIO_TIMER_GetAbsoluteTime(WAKEUP) < 0x10);
	RADIO_TIMER_InitStruct.XTAL_StartupTime = 320;
	RADIO_TIMER_InitStruct.enableInitialCalibration = FALSE;
	RADIO_TIMER_InitStruct.periodicCalibrationInterval = 0;
	HAL_RADIO_TIMER_Init(&RADIO_TIMER_InitStruct);
}