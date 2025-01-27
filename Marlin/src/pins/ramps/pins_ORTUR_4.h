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
 * Ortur 4 Arduino Mega based on RAMPS v1.4 pin assignments
 * ATmega2560
 */

#define BOARD_INFO_NAME "Ortur 4.3"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

//
// Servos
//
#define SERVO0_PIN                            29

//
// Limit Switches
//
#define X_MAX_PIN                             18
#define Z_MIN_PIN                             63

#define Z_MIN_PROBE_PIN                        2
#define FIL_RUNOUT_PIN                        59

//
// Steppers
//
#define E0_STEP_PIN                           36
#define E0_DIR_PIN                            34
#define E0_ENABLE_PIN                         30
#define E0_CS_PIN                             44

#define E1_STEP_PIN                           26
#define E1_DIR_PIN                            28
#define E1_ENABLE_PIN                         24
#define E1_CS_PIN                             42

//
// Temperature Sensors
//
#define TEMP_0_PIN                            15  // Analog Input
#define TEMP_1_PIN                            13  // Analog Input

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                     59
  #define X_SERIAL_RX_PIN                     63

  #define Y_SERIAL_TX_PIN                     64
  #define Y_SERIAL_RX_PIN                     40

  #define Z_SERIAL_TX_PIN                     44
  #define Z_SERIAL_RX_PIN                     42

  #define E0_SERIAL_TX_PIN                    66
  #define E0_SERIAL_RX_PIN                    65
#endif

//
// LCD / Controller
//

#if IS_RRD_FG_SC
  #define BEEPER_PIN                          35

  #define LCD_PINS_RS                         27
  #define LCD_PINS_EN                         23
  #define LCD_PINS_D4                         37

  #define LCD_SDSS_PIN                        53
  #define SD_DETECT_PIN                       49

  #define BTN_EN1                             29
  #define BTN_EN2                             25
  #define BTN_ENC                             16

  #define LCD_PINS_DEFINED

#endif

#include "pins_RAMPS.h"
