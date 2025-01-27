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

#if NOT_TARGET(__STM32F1__, STM32F1xx, STM32F0xx)
  #error "Oops! Select a 'Malyan M300' board in 'Tools > Board.'"
#endif

#define BOARD_INFO_NAME "Malyan M300"

//
// EEPROM Emulation
//
#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
  #ifndef MARLIN_EEPROM_SIZE
    #define MARLIN_EEPROM_SIZE            0x800U  // 2K
  #endif
#endif

//
// Timers
//
#define STEP_TIMER  6
#define TEMP_TIMER  7

//
// Limit Switches
//
#define X_STOP_PIN                          PC13
#define Y_STOP_PIN                          PC14
#define Z_STOP_PIN                          PC15

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PB7
#endif

//
// Steppers
//
#define X_STEP_PIN                          PB14
#define X_DIR_PIN                           PB13
#define X_ENABLE_PIN                        PB10

#define Y_STEP_PIN                          PB12
#define Y_DIR_PIN                           PB11
#define Y_ENABLE_PIN                        PB10

#define Z_STEP_PIN                          PB2
#define Z_DIR_PIN                           PB1
#define Z_ENABLE_PIN                        PB10

#define E0_STEP_PIN                         PA7
#define E0_DIR_PIN                          PA6
#define E0_ENABLE_PIN                       PB0

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA0   // Analog Input (HOTEND0 thermistor)
#define TEMP_BED_PIN                        PA4   // Analog Input (BED thermistor)

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA1   // HOTEND0 MOSFET
#define HEATER_BED_PIN                      PA5   // BED MOSFET

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                   PA8
#endif
