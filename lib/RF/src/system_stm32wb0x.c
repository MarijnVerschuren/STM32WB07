
/**
  ******************************************************************************
  * @file    system_stm32wb0x.c
  * @author  GPM WBL Application Team
  * @brief   CMSIS Cortex-M0+ Device Peripheral Access Layer System Source File
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32wb0x.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *   After each device reset the HSI (64 MHz) is used as system clock source.
  *   Then SystemInit() function is called, in "startup_stm32wb0x.s" file, to
  *   configure the system clock before to branch to main program.
  *
  *   This file configures the system clock as follows:
  *=============================================================================
  *-----------------------------------------------------------------------------
  *        System Clock source                     | HSI
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                              | 16000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                                | 16000000
  *-----------------------------------------------------------------------------
  *=============================================================================
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup STM32WB0x_system
  * @{
  */

/** @addtogroup STM32WB0x_System_Private_Includes
  * @{
  */

#include "stm32wb0x.h"

/**
  * @}
  */

/** @addtogroup STM32WB0x_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32WB0x_System_Private_Defines
  * @{
  */
#if !defined (HSE_VALUE)
#define HSE_VALUE     (32000000U) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined (HSI_VALUE)
#define HSI_VALUE     (64000000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/* Note: Following vector table addresses must be defined in line with linker
         configuration. */
/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in Flash or Sram, else the vector table is kept at the automatic
     remap of boot address selected */
/* #define USER_VECT_TAB_ADDRESS */

#if defined(USER_VECT_TAB_ADDRESS)
/*!< Uncomment the following line if you need to relocate your vector Table
     in SRAM else user remap will be done in FLASH. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x100. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x100. */
#else
#define VECT_TAB_BASE_ADDRESS   NVM_BASE        /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x100. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x100. */
#endif /* VECT_TAB_SRAM */
#endif /* USER_VECT_TAB_ADDRESS */

/******************************************************************************/

/*!< HW TRIMMING Defines */
#define VALIDITY_TAG      0xFCBCECCC  /*!< TAG to validate the content of the 
					   trimming area content. */
#define VALIDITY_LOCATION 0x10001EF8  /*!< ROM address of the the validity trimming values content. */

/*!< SMPS Configuration Defines */
#if !defined(CFG_HW_SMPS)
#define CFG_HW_SMPS SMPS_ON
#endif
                                             
#if !defined(CFG_HW_SMPS_BOM)
#define CFG_HW_SMPS_BOM SMPS_BOM3 /*!< SMPS Inductor 10uH */
#endif

#if !defined(CFG_HW_SMPS_LOW_POWER)
#define CFG_HW_SMPS_LOW_POWER SMPS_LOW_POWER_OPEN
#endif

/**
  * @}
  */

/** @addtogroup STM32WB0x_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32WB0x_System_Private_Variables
  * @{
  */
  /* The SystemCoreClock variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
  uint32_t SystemCoreClock = 16000000U; /* The HSI (64MHz) is used as system clock source after startup from reset, configured at 16 MHz. */

  /* The RAM_VR variable is a mirroring in RAM of some registers information.
     It is a sort of virtual register in RAM.
  */
#if defined ( __ICCARM__ )
  #pragma location=".ram_vr"
  __root __no_init RAM_VR_TypeDef RAM_VR;
#else
#if defined ( __ARMCC_VERSION )
  __attribute__((section(".bss" ".ram_vr")))
#elif defined (  __GNUC__  )
  __attribute__((section(".ram_vr")))
#endif
  RAM_VR_TypeDef RAM_VR __attribute__((used));
#endif
/**
  * @}
  */

/** @addtogroup STM32WB0x_System_Private_FunctionPrototypes
  * @{
  */

void CPUcontextRestore(void);

/**
  * @}
  */

/** @addtogroup STM32WB0x_System_Private_Functions
  * @{
  */


/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint8_t directHSE_enabled;
  uint8_t divPrescaler;

  /* Get SYSCLK source HSE or HSI+PLL64MHz */
  directHSE_enabled = (RCC->CFGR & RCC_CFGR_HSESEL) >> RCC_CFGR_HSESEL_Pos;

#if defined(STM32WB06) || defined(STM32WB07)
  /* Get the clock divider */
    divPrescaler = (RCC->CFGR & RCC_CFGR_CLKSYSDIV) >> RCC_CFGR_CLKSYSDIV_Pos;
#else
  /* Get the clock divider */
  divPrescaler = (RCC->CFGR & RCC_CFGR_CLKSYSDIV_STATUS) >> RCC_CFGR_CLKSYSDIV_STATUS_Pos;
#endif

  if (directHSE_enabled)
  {
    SystemCoreClock = HSE_VALUE >> (divPrescaler - 1U);
  }
  else
  {
    SystemCoreClock = HSI_VALUE >> divPrescaler;
  }  
}

/**
  * @brief  Restores the saved CPU state before to enter in power save 
  *         by popping it from the stack 
  * @param  None
  * @retval None
  */
__WEAK void CPUcontextRestore(void)
{
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
