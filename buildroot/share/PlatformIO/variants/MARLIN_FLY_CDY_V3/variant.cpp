/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "pins_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

// Digital PinName array
const PinName digitalPin[] = {
  PA_0,  // Digital pin 0
  PA_1,  // Digital pin 1
  PA_2,  // Digital pin 2
  PA_3,  // Digital pin 3
  PA_4,  // Digital pin 4
  PA_5,  // Digital pin 5
  PA_6,  // Digital pin 6
  PA_7,  // Digital pin 7
  PA_8,  // Digital pin 8
  PA_9,  // Digital pin 9
  PA_10, // Digital pin 10
  PA_11, // Digital pin 11
  PA_12, // Digital pin 12
  PA_13, // Digital pin 13
  PA_14, // Digital pin 14
  PA_15, // Digital pin 15

  PB_0,  // Digital pin 16
  PB_1,  // Digital pin 17
  PB_2,  // Digital pin 18
  PB_3,  // Digital pin 19
  PB_4,  // Digital pin 20
  PB_5,  // Digital pin 21
  PB_6,  // Digital pin 22
  PB_7,  // Digital pin 23
  PB_8,  // Digital pin 24
  PB_9,  // Digital pin 25
  PB_10, // Digital pin 26
  PB_11, // Digital pin 27
  PB_12, // Digital pin 28
  PB_13, // Digital pin 29
  PB_14, // Digital pin 30
  PB_15, // Digital pin 31

  PC_0,  // Digital pin 32
  PC_1,  // Digital pin 33
  PC_2,  // Digital pin 34
  PC_3,  // Digital pin 35
  PC_4,  // Digital pin 36
  PC_5,  // Digital pin 37
  PC_6,  // Digital pin 38
  PC_7,  // Digital pin 39
  PC_8,  // Digital pin 40
  PC_9,  // Digital pin 41
  PC_10, // Digital pin 42
  PC_11, // Digital pin 43
  PC_12, // Digital pin 44
  PC_13, // Digital pin 45
  PC_14, // Digital pin 46
  PC_15, // Digital pin 47

  PD_0,  // Digital pin 48
  PD_1,  // Digital pin 49
  PD_2,  // Digital pin 50
  PD_3,  // Digital pin 51
  PD_4,  // Digital pin 52
  PD_5,  // Digital pin 53
  PD_6,  // Digital pin 54
  PD_7,  // Digital pin 55
  PD_8,  // Digital pin 56
  PD_9,  // Digital pin 57
  PD_10, // Digital pin 58
  PD_11, // Digital pin 59
  PD_12, // Digital pin 60
  PD_13, // Digital pin 61
  PD_14, // Digital pin 62
  PD_15, // Digital pin 63

  PE_0,  // Digital pin 64
  PE_1,  // Digital pin 65
  PE_2,  // Digital pin 66
  PE_3,  // Digital pin 67
  PE_4,  // Digital pin 68
  PE_5,  // Digital pin 69
  PE_6,  // Digital pin 70
  PE_7,  // Digital pin 71
  PE_8,  // Digital pin 72
  PE_9,  // Digital pin 73
  PE_10, // Digital pin 74
  PE_11, // Digital pin 75
  PE_12, // Digital pin 76
  PE_13, // Digital pin 77
  PE_14, // Digital pin 78
  PE_15, // Digital pin 79

  PH_0,  // Digital pin 80, used by the external oscillator
  PH_1   // Digital pin 81, used by the external oscillator
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  0,  // A0,  PA0
  1,  // A1,  PA1
  2,  // A2,  PA2
  3,  // A3,  PA3
  4,  // A4,  PA4
  5,  // A5,  PA5
  6,  // A6,  PA6
  7,  // A7,  PA7
  16, // A8,  PB0
  17, // A9,  PB1
  32, // A10, PC0
  33, // A11, PC1
  34, // A12, PC2
  35, // A13, PC3
  36, // A14, PC4
  37  // A15, PC5
};

#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief  Configures the System clock source, PLL Multiplier and Divider factors,
 *               AHB/APBx prescalers and Flash settings
 * @note   This function should be called only once the RCC clock configuration
 *         is reset to the default reset state (done in SystemInit() function).
 * @param  None
 * @retval None
 */

/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
static uint8_t SetSysClock_PLL_HSE(uint8_t bypass)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* The voltage scaling allows optimizing the power consumption when the device is
  clocked below the maximum system frequency, to update the voltage scaling value
  regarding system frequency refer to product datasheet. */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSE oscillator and activate PLL with HSE as source
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  if (bypass == 0) {
    RCC_OscInitStruct.HSEState          = RCC_HSE_ON; // External 8 MHz xtal on OSC_IN/OSC_OUT
  } else {
    RCC_OscInitStruct.HSEState          = RCC_HSE_BYPASS; // External 8 MHz clock on OSC_IN
  }

  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM            = HSE_VALUE / 1000000L; // Expects an 8 MHz external clock by default. Redefine HSE_VALUE if not
  RCC_OscInitStruct.PLL.PLLN            = 336;                  // VCO output clock = 336 MHz (1 MHz * 336)
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;        // PLLCLK = 168 MHz (336 MHz / 2)
  RCC_OscInitStruct.PLL.PLLQ            = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    return 0; // FAIL
  }

  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;         // 168 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;           // 42 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;           // 84 MHz
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    return 0; // FAIL
  }

  /* Output clock on MCO1 pin(PA8) for debugging purpose */
  /*
  if (bypass == 0)
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_2); // 4 MHz
  else
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1); // 8 MHz
  */

  return 1; // OK
}

/******************************************************************************/
/*            PLL (clocked by HSI) used as System clock source                */
/******************************************************************************/
uint8_t SetSysClock_PLL_HSI(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* The voltage scaling allows optimizing the power consumption when the device is
    clocked below the maximum system frequency, to update the voltage scaling value
    regarding system frequency refer to product datasheet. */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSI oscillator and activate PLL with HSI as source
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSEState            = RCC_HSE_OFF;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 16;            // VCO input clock = 1 MHz (16 MHz / 16)
  RCC_OscInitStruct.PLL.PLLN            = 336;           // VCO output clock = 336 MHz (1 MHz * 336)
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2; // PLLCLK = 168 MHz (336 MHz / 2)
  RCC_OscInitStruct.PLL.PLLQ            = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    return 0; // FAIL
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;         // 168 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;           // 42 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;           // 84 MHz
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    return 0; // FAIL
  }

  /* Output clock on MCO1 pin(PA8) for debugging purpose */
  //HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1); // 16 MHz

  return 1; // OK
}

WEAK void SystemClock_Config(void)
{
  /* 1- If fail try to start with HSE and external xtal */
  if (SetSysClock_PLL_HSE(0) == 0) {
    /* 2- Try to start with HSE and external clock */
    if (SetSysClock_PLL_HSE(1) == 0) {
      /* 3- If fail start with HSI clock */
      if (SetSysClock_PLL_HSI() == 0) {
        Error_Handler();
      }
    }
  }

  /* Ensure CCM RAM clock is enabled */
  __HAL_RCC_CCMDATARAMEN_CLK_ENABLE();

  /* Output clock on MCO2 pin(PC9) for debugging purpose */
  //HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_4);
}

#ifdef __cplusplus
}
#endif
