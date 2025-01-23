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
 * Arduino Mega with PICA pin assignments
 * Schematic: https://github.com/mjrice/PICA/blob/master/pica_schematic.pdf
 * ATmega2560
 *
 * PICA is Power, Interface, and Control Adapter and is open source hardware.
 * See https://github.com/mjrice/PICA for schematics etc.
 *
 * Applies to PICA, PICA_REVB
 */

#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "PICA supports up to 2 hotends / E steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "PICA"
#endif

//
// Servos
//
#define SERVO0_PIN                             3
#define SERVO1_PIN                             4
#define SERVO2_PIN                             5

//
// Limit Switches
//
#define X_MIN_PIN                             14
#define X_MAX_PIN                             15
#define Y_MIN_PIN                             16
#define Y_MAX_PIN                             17
#define Z_MIN_PIN                             23
#define Z_MAX_PIN                             22

//
// Steppers
//
#define X_STEP_PIN                            55
#define X_DIR_PIN                             54
#define X_ENABLE_PIN                          60

#define Y_STEP_PIN                            57
#define Y_DIR_PIN                             56
#define Y_ENABLE_PIN                          61

#define Z_STEP_PIN                            59
#define Z_DIR_PIN                             58
#define Z_ENABLE_PIN                          62

#define E0_STEP_PIN                           67
#define E0_DIR_PIN                            24
#define E0_ENABLE_PIN                         26

#define E1_STEP_PIN                           68
#define E1_DIR_PIN                            28
#define E1_ENABLE_PIN                         27

//
// Temperature Sensors
//
#define TEMP_0_PIN                             9  // Analog Input
#define TEMP_1_PIN                            10
#define TEMP_BED_PIN                          10
#define TEMP_2_PIN                            11
#define TEMP_3_PIN                            12

//
// Heaters / Fans
//
#ifndef HEATER_0_PIN
  #define HEATER_0_PIN                        10  // E0
#endif
#ifndef HEATER_1_PIN
  #define HEATER_1_PIN                         2  // E1
#endif
#define HEATER_BED_PIN                         8  // HEAT-BED

#ifndef FAN0_PIN
  #define FAN0_PIN                             9
#endif
#ifndef FAN_2_PIN
  #define FAN_2_PIN                            7
#endif

#define SDPOWER_PIN                           -1
#define LED_PIN                               -1
#define PS_ON_PIN                             -1
#define KILL_PIN                              -1

#define SSR_PIN                                6

// SPI for MAX Thermocouple
#if !HAS_MEDIA
  #define TEMP_0_CS_PIN                       66  // Don't use 53 if using Display/SD card
#else
  #define TEMP_0_CS_PIN                       66  // Don't use 49 (SD_DETECT_PIN)
#endif

//
// SD Support
//
#define SD_DETECT_PIN                EXP2_07_PIN
#define SD_SS_PIN                    EXP2_04_PIN

/**                      PICA Expansion Headers
 *               ------                            ------
 *    (BEEP) 29 | 1  2 | 31 (ENC)       (MISO) 50 | 1  2 | 52 (SCK)
 *  (LCD_EN) 30 | 3  4 | 33 (LCD_RS)     (EN1) 47 | 3  4 | 53 (SDSS)
 *  (LCD_D4) 35   5  6 | 32 (LCD_D5)     (EN2) 48   5  6 | 51 (MOSI)
 *  (LCD_D6) 37 | 7  8 | 36 (LCD_D7)   (SDDET) 49 | 7  8 | 41 (KILL)
 *          GND | 9 10 | 5V                    -- | 9 10 | --
 *               ------                            ------
 *                EXP1                              EXP2
 */
#define EXP1_01_PIN                           29  // BEEPER
#define EXP1_02_PIN                           31  // ENC
#define EXP1_03_PIN                           30  // LCD_EN
#define EXP1_04_PIN                           33  // LCD_RS
#define EXP1_05_PIN                           35  // LCD_D4
#define EXP1_06_PIN                           32  // LCD_D5
#define EXP1_07_PIN                           37  // LCD_D6
#define EXP1_08_PIN                           36  // LCD_D7

#define EXP2_01_PIN                           50  // MISO
#define EXP2_02_PIN                           52  // SCK
#define EXP2_03_PIN                           47  // EN1
#define EXP2_04_PIN                           53  // SDSS
#define EXP2_05_PIN                           48  // EN2
#define EXP2_06_PIN                           51  // MOSI
#define EXP2_07_PIN                           49  // SDDET
#define EXP2_08_PIN                           41  // KILL

//
// LCD / Controller
//
#define BEEPER_PIN                   EXP1_01_PIN

#if HAS_WIRED_LCD
  #define LCD_PINS_RS                EXP1_04_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_D4                EXP1_05_PIN
  #define LCD_PINS_D5                EXP1_06_PIN
  #define LCD_PINS_D6                EXP1_07_PIN
  #define LCD_PINS_D7                EXP1_08_PIN

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #define LCD_SDSS_PIN               EXP2_04_PIN
#endif
