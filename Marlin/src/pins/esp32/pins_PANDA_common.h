/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Panda common pin assignments
 */

#include "env_validate.h"

#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

//
// Servos
//
#define SERVO0_PIN                             0

//
// Limit Switches
//
#define X_STOP_PIN                             4
#define Y_STOP_PIN                            35
#define Z_STOP_PIN                            21

//
// Steppers
//
#define X_STEP_PIN                           101
#define X_DIR_PIN                            100

#define Y_STEP_PIN                           103
#define Y_DIR_PIN                            102

#define Z_STEP_PIN                           105
#define Z_DIR_PIN                            104

#define E0_STEP_PIN                          107
#define E0_DIR_PIN                           106

//
// Temperature Sensors
//
#define TEMP_0_PIN                            39  // Analog Input
#define TEMP_BED_PIN                          36  // Analog Input

#if ENABLED(MAX31856_PANDAPI)
  #define TEMP_0_CLK_PIN                      29
  #define TEMP_0_MISO_PIN                     24
  #define TEMP_0_MOSI_PIN                     28
  #define TEMP_0_CS_PIN                       27
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                         108
#define HEATER_BED_PIN                       109
#define FAN0_PIN                             118  // FAN0
#define FAN1_PIN                             119  // FAN1

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                    120  // FAN2
#endif

/**            ------                              ------
 * (MISO 19?) | 1  2 | (18 SCK?)     (BEEPER) 129 | 1  2 | 12 (^ENC)
 *   (EN1) 33 | 3  4 | (5 SDSS?)     (EN)      26 | 3  4 | 27 (RS)
 *   (EN2) 32   5  6 | (23 MOSI?)    (D4)      14 | 5  6   --
 * (SDDET 2?) | 7  8 | (RESET)                 -- | 7  8 | --
 *         -- | 9 10 | --                     GND | 9 10 | 5V
 *             ------                              ------
 *              EXP2                                EXP1
 */
#define EXP1_01_PIN                          129
#define EXP1_02_PIN                           12
#define EXP1_03_PIN                           26
#define EXP1_04_PIN                           27
#define EXP1_05_PIN                           14

#define EXP2_01_PIN                           19  // ?
#define EXP2_02_PIN                           18  // ?
#define EXP2_03_PIN                           33
#define EXP2_04_PIN                            5  // ?
#define EXP2_05_PIN                           32
#define EXP2_06_PIN                           23  // ?
#define EXP2_07_PIN                            2  // ?

//
// SD Card
//
#if HAS_MEDIA
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_SS_PIN                  EXP2_04_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
#endif

#if HAS_WIRED_LCD
  #define BEEPER_PIN                 EXP1_01_PIN

  #define BTN_ENC                    EXP1_02_PIN
  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN

  #define LCD_PINS_RS                EXP1_04_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_D4                EXP1_05_PIN
#endif
