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
 * Cheaptronic v2.0 pin assignments
 * Built and sold by Michal Dyntar - RRO
 *          www.reprapobchod.cz (DOES NOT EXIST ANYMORE)
 *          https://web.archive.org/web/20190306201523/reprapobchod.cz
 * ATmega2560
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Cheaptronic v2.0"

//
// Limit Switches
//
#define X_MIN_PIN                             30
#define X_MAX_PIN                             31
#define Y_MIN_PIN                             32
#define Y_MAX_PIN                             33
#define Z_MIN_PIN                             34
#define Z_MAX_PIN                             35

//
// Steppers
//
#define X_STEP_PIN                            17
#define X_DIR_PIN                             16
#define X_ENABLE_PIN                          48

#define Y_STEP_PIN                            54
#define Y_DIR_PIN                             47
#define Y_ENABLE_PIN                          55

#define Z_STEP_PIN                            57
#define Z_DIR_PIN                             56
#define Z_ENABLE_PIN                          62

#define E0_STEP_PIN                           23
#define E0_DIR_PIN                            22
#define E0_ENABLE_PIN                         24

#define E1_STEP_PIN                           26
#define E1_DIR_PIN                            25
#define E1_ENABLE_PIN                         27

#define E2_STEP_PIN                           29
#define E2_DIR_PIN                            28
#define E2_ENABLE_PIN                         39

//
// Temperature sensors
//
#define TEMP_0_PIN                            15
#define TEMP_1_PIN                            13
#define TEMP_2_PIN                            14
#define TEMP_3_PIN                            11  // should be used for chamber temperature control
#define TEMP_BED_PIN                          12

//
// Heaters / Fans
//
#define HEATER_0_PIN                           6
#define HEATER_1_PIN                           7
#define HEATER_2_PIN                           8
#define HEATER_BED_PIN                         9
#ifndef FAN0_PIN
  #define FAN0_PIN                             3
#endif
#define FAN2_PIN                              58  // additional fan or light control output

//
// Other board specific pins
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      37  // board input labeled as F-DET
#endif
#define Z_MIN_PROBE_PIN                       36  // additional external board input labeled as E-SENS (should be used for Z-probe)
#define LED_PIN                               13
#define SPINDLE_ENABLE_PIN                     4  // additional PWM pin 1 at JP1 connector - should be used for laser control too
#define EXT_2                                  5  // additional PWM pin 2 at JP1 connector
#define EXT_3                                  2  // additional PWM pin 3 at JP1 connector
#define PS_ON_PIN                             45
#define KILL_PIN                              46

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                        11  // shared with TEMP_3 analog input
#endif

//
// LCD / Controller
//
#define LCD_PINS_RS                           19
#define LCD_PINS_EN                           42
#define LCD_PINS_D4                           18
#define LCD_PINS_D5                           38
#define LCD_PINS_D6                           41
#define LCD_PINS_D7                           40

#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
  #define BTN_ENC_EN                 LCD_PINS_D7  // Detect the presence of the encoder
#endif

//
// Beeper, SD Card, Encoder
//
#define BEEPER_PIN                            44

#if HAS_MEDIA
  #define SDSS                                53
  #define SD_DETECT_PIN                       49
#endif

#if IS_NEWPANEL
  #define BTN_EN1                             11
  #define BTN_EN2                             12
  #define BTN_ENC                             43
#endif
