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

// Pin number
const PinName digitalPin[] = {
  PA_0,  //D0
  PA_1,  //D1
  PA_2,  //D2
  PA_3,  //D3
  PA_4,  //D4
  PA_5,  //D5
  PA_6,  //D6
  PA_7,  //D7
  PA_8,  //D8
  PA_9,  //D9
  PA_10, //D10
  PA_11, //D11
  PA_12, //D12
  PA_13, //D13
  PA_14, //D14
  PA_15, //D15
  PB_0,  //D16
  PB_1,  //D17
  PB_2,  //D18
  PB_3,  //D19
  PB_4,  //D20
  PB_5,  //D21
  PB_6,  //D22
  PB_7,  //D23
  PB_8,  //D24
  PB_9,  //D25
  PB_10, //D26
  PB_11, //D27
  PB_12, //D28
  PB_13, //D29
  PB_14, //D30
  PB_15, //D31
  PC_0,  //D32
  PC_1,  //D33
  PC_2,  //D34
  PC_3,  //D35
  PC_4,  //D36
  PC_5,  //D37
  PC_6,  //D38
  PC_7,  //D39
  PC_8,  //D40
  PC_9,  //D41
  PC_10, //D42
  PC_11, //D43
  PC_12, //D44
  PC_13, //D45
  PC_14, //D46
  PC_15, //D47
  PD_0,  //D48
  PD_1,  //D49
  PD_2,  //D50
  PD_3,  //D51
  PD_4,  //D52
  PD_5,  //D53
  PD_6,  //D54
  PD_7,  //D55
  PD_8,  //D56
  PD_9,  //D57
  PD_10, //D58
  PD_11, //D59
  PD_12, //D60
  PD_13, //D61
  PD_14, //D62
  PD_15, //D63
  PE_0,  //D64
  PE_1,  //D65
  PE_2,  //D66
  PE_3,  //D67
  PE_4,  //D68
  PE_5,  //D69
  PE_6,  //D70
  PE_7,  //D71
  PE_8,  //D72
  PE_9,  //D73
  PE_10, //D74
  PE_11, //D75
  PE_12, //D76
  PE_13, //D77
  PE_14, //D78
  PE_15, //D79
  PF_0,  //D80
  PF_1,  //D81
  PF_2,  //D82
  PF_3,  //D83
  PF_4,  //D84
  PF_5,  //D85
  PF_6,  //D86
  PF_7,  //D87
  PF_8,  //D88
  PF_9,  //D89
  PF_10, //D90
  PF_11, //D91
  PF_12, //D92
  PF_13, //D93
  PF_14, //D94
  PF_15, //D95
  PG_0,  //D96
  PG_1,  //D97
  PG_2,  //D98
  PG_3,  //D99
  PG_4,  //D100
  PG_5,  //D101
  PG_6,  //D102
  PG_7,  //D103
  PG_8,  //D104
  PG_9,  //D105
  PG_10, //D106
  PG_11, //D107
  PG_12, //D108
  PG_13, //D109
  PG_14, //D110
  PG_15, //D111

  //Duplicated ADC Pins
  PA_3,  //D112/A0
  PA_4,  //D113/A1
  PC_0,  //D114/A2
  PC_1,  //D115/A3
  PC_2,  //D116/A4
  PC_3,  //D117/A5
  PC_4,  //D118/A6
  PF_3,  //D119/A16 - 1:FSMC_A3  2:ADC3_IN9
  PF_4,  //D120/A17 - 1:FSMC_A4  2:ADC3_IN14
  PF_5,  //D121/A18 - 1:FSMC_A5  2:ADC3_IN15
  PF_6,  //D122/A19 - 1:TIM10_CH1  2:ADC3_IN4
  PF_7,  //D123/A20 - 1:TIM11_CH1  2:ADC3_IN5
  PF_8,  //D124/A20 - 1:TIM11_CH1  2:ADC3_IN6
};

#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 12000000
  *            PLL_M                          = 6
  *            PLL_N                          = 180
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
WEAK void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;


  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

#ifdef HAL_PWR_MODULE_ENABLED
  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
#endif

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 6;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

#ifdef __cplusplus
}
#endif
