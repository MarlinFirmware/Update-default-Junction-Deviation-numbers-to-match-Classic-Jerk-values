/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * MALYAN M200 pin assignments
 */

#if NOT_TARGET(__STM32F1__, STM32F1xx, STM32F0xx)
  #error "Oops! Select an STM32 board in your IDE."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Malyan M200"
#endif

// Prevents hanging from an extra watchdog init
#define DISABLE_WATCHDOG_INIT

// Assume Flash EEPROM
#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
#endif

// Based on PWM timer usage, we have to use these timers and soft PWM for the fans
// On STM32F103:
// PB3, PB6, PB7, and PB8 can be used with pwm, which rules out TIM2 and TIM4.
// On STM32F070, 16 and 17 are in use, but 1 and 3 are available.
#define STEP_TIMER  1
#define TEMP_TIMER  3

//
// Limit Switches
//
#define X_STOP_PIN                          PB4
#define Y_STOP_PIN                          PA15
#define Z_STOP_PIN                          PB5

//
// Steppers
//
// X & Y enable are the same
#define X_STEP_PIN                          PB14
#define X_DIR_PIN                           PB15
#define X_ENABLE_PIN                        PA8

#define Y_STEP_PIN                          PB12
#define Y_DIR_PIN                           PB13
#define Y_ENABLE_PIN                        PA8

#define Z_STEP_PIN                          PB10
#define Z_DIR_PIN                           PB2
#define Z_ENABLE_PIN                        PB11

#define E0_STEP_PIN                         PB0
#define E0_DIR_PIN                          PC13
#define E0_ENABLE_PIN                       PB1

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA0   // Analog Input (HOTEND0 thermistor)
#define TEMP_BED_PIN                        PA1   // Analog Input (BED thermistor)

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB6   // HOTEND0 MOSFET
#define HEATER_BED_PIN                      PB7   // BED MOSFET

#define MALYAN_FAN1_PIN                     PB8   // FAN1 header on board - PRINT FAN
#define MALYAN_FAN2_PIN                     PB3   // FAN2 header on board - CONTROLLER FAN

#define FAN1_PIN                 MALYAN_FAN2_PIN
