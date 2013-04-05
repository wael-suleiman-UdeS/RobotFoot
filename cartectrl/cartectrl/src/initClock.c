/**
  ******************************************************************************
  * @file    initClock.c
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version V1.0.0
  * @date    2013-03-21
  * @brief   Clock initialisation routine
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_flash.h"

/** @addtogroup Init
  * @{
  */

/** @defgroup Clock
  * @brief Clock initialisation
  * @{
  */

//------------------------------------------------------------------------------

/**
  * @brief  Initialise the different clocks inside the MCU specifically
  *         for the application.
  * @note   It initialises the SYSCLK and USB clocks as well as the differents
  *         prescalers for the different buses.
  * @note   Flash wait cycles are adjusted in the process so program code
  *         executes reliably; instruction/data caches as well as the prefetch
  *         buffer are enabled to increase performance.
  * @param  None
  * @retval None
  * @todo   It should return an error code, ideally, when everything is screwed
  *         up
  */
void initClock(void)
{
    enum {
        RCC_SYSCLK_HSI = 0,
        RCC_SYSCLK_HSE = 4,
        RCC_SYSCLK_PLL = 8
    };

    // Deinit the RCC registers
    RCC_DeInit();

    // Enable Prefetch Buffer and set Flash Latency
    FLASH_SetLatency(FLASH_Latency_6);
    FLASH_PrefetchBufferCmd(ENABLE);

    // Enable instruction and data cache
    FLASH_InstructionCacheCmd(ENABLE);
    FLASH_DataCacheCmd(ENABLE);

    // Put internal regulator to hi performance mode.
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1ENR_PWREN, DISABLE);
    PWR_MainRegulatorModeConfig(PWR_Regulator_Voltage_Scale1);

    // Enable HSE
    RCC_HSEConfig(RCC_HSE_ON);
    RCC_WaitForHSEStartUp(); // NOTE: return value discarded!

    // Configure PLL
    // TODO: put names to symbols.
    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
    RCC_PLLCmd(ENABLE);

    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == 0)
    {
        // wait until PLL clock is stable and can be used as system clock source
        // TODO: put a timout so it can error out.
    }

    // Set up clock division for the differents buses
    RCC_HCLKConfig(RCC_HCLK_Div1);      // APB1 = 42MHz
    RCC_PCLK1Config(RCC_HCLK_Div4);     // APB1 = 42MHz
    RCC_PCLK2Config(RCC_HCLK_Div2);     // APB2 = 84MHz

    // Use PLL clock as system clock
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    // Wait till PLL is used as system clock source
    // (Yeah, that's right. There's no define/enum for this.)
    // TODO: put a timout so it can error out.
    while(RCC_GetSYSCLKSource() != RCC_SYSCLK_PLL)
    {}
}
//------------------------------------------------------------------------------

/**
  * @}
  */

/**
  * @}
  */

//------------------------------------------------------------------------------
