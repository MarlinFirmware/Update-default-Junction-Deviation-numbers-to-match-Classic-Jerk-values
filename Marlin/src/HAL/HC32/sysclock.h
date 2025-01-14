/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * HC32F460 system clock configuration.
 *
 * With the HC32 HAL, the various peripheral clocks (including the CPU clock) are derived
 * from the main PLL (MPLL-P) output (referred to at F_SYSTEM_CLOCK).
 *
 * F_SYSTEM_CLOCK is the target frequency of the main PLL, and the PLL is automatically configured
 * to achieve this frequency.
 *
 * The peripheral clocks are the result of integer division of F_SYSTEM_CLOCK.
 * Their target frequencies are defined here, and the required division factors are calculated automatically.
 * Note that the division factor must be a power of 2 between 1 and 64.
 * If the target frequency is not achievable, a compile-time error will be generated.
 *
 * Additionally, there are interdependencies between the peripheral clocks, which are described in
 * Section 4.4 "Working Clock Specifications" of the HC32F460 Reference Manual.
 * With Arduino core >= 1.2.0, these interdependencies are checked at compile time.
 * On earlier versions, you are on your own.
 *
 * For all clock frequencies, they can be checked at runtime by enabling the 'ALWAYS_VALIDATE_CLOCKS' define.
 * In MARLIN_DEV_MODE, they will also be printed to the serial console by 'MarlinHAL::HAL_clock_frequencies_dump'.
 *
 * See https://github.com/MarlinFirmware/Marlin/pull/27099 for more information.
 */

// Target peripheral clock frequencies, must be integer divisors of F_SYSTEM_CLOCK.
// Changing the frequency here will automagically update everything else.
#define F_HCLK  200000000UL  // 200 MHz; CPU
#define F_EXCLK (F_HCLK / 2) // 100 MHz; SDIO
#define F_PCLK0 (F_HCLK / 2) // 100 MHz; Timer6 (unused)
#define F_PCLK1 (F_HCLK / 4) //  50 MHz; USART, SPI, Timer0 (step + temp), TimerA (Servo)
#define F_PCLK2 (F_HCLK / 8) //  25 MHz; ADC Sampling
#define F_PCLK3 (F_HCLK / 8) //  25 MHz; I2C, WDT
#define F_PCLK4 (F_HCLK / 2) // 100 MHz; ADC Control

// MPLL-P clock target frequency. This must be >= the highest peripheral clock frequency.
// PLL config is automatically calculated based on this value.
#define F_SYSTEM_CLOCK F_HCLK

// The Peripheral clocks are only checked at runtime if this is enabled OR MARLIN_DEV_MODE is enabled.
// Compile time checks are always performed with Arduino core version >= 1.2.0.
#define ALWAYS_VALIDATE_CLOCKS 1
