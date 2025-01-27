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
 * Intamsys Funmat HT V4.0 Mainboard
 * ATmega2560
 * 4988 Drivers Tested
 * 2208 version exists and may or may not work
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Intamsys 4.0"

//
// Servos
//
#define SERVO0_PIN                            12  // Uses High Temp Present Jumper Pin

//
// Limit Switches
//
#define X_STOP_PIN                            22
#define Y_STOP_PIN                            26
#define Z_MIN_PIN                             29
#define Z_MAX_PIN                             69

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     69
#endif

#define FIL_RUNOUT_PIN                        10

//
// Steppers
//
#define X_STEP_PIN                            25
#define X_DIR_PIN                             23
#define X_ENABLE_PIN                          27  // 44

#define Y_STEP_PIN                            32  // 33
#define Y_DIR_PIN                             33  // 31, 32
#define Y_ENABLE_PIN                          31  // 32

#define Z_STEP_PIN                            35  // 35
#define Z_DIR_PIN                             36
#define Z_ENABLE_PIN                          34  // 34

#define E0_STEP_PIN                           42
#define E0_DIR_PIN                            43
#define E0_ENABLE_PIN                         37

#define E1_STEP_PIN                           49
#define E1_DIR_PIN                            47
#define E1_ENABLE_PIN                         48

#define MOTOR_CURRENT_PWM_X_PIN               11
#define MOTOR_CURRENT_PWM_Y_PIN               44
#define MOTOR_CURRENT_PWM_Z_PIN               45
#define MOTOR_CURRENT_PWM_E_PIN               46

// Motor current PWM conversion, PWM value = MotorCurrentSetting * 255 / range
#ifndef MOTOR_CURRENT_PWM_RANGE
  #define MOTOR_CURRENT_PWM_RANGE            2000
#endif
#define DEFAULT_PWM_MOTOR_CURRENT  { 1300, 1300, 1250 }

//
// Temperature Sensors
//
#define TEMP_0_PIN                             8  // Analog Input D62
#define TEMP_BED_PIN                          10  // Analog Input D64

#define TEMP_CHAMBER_PIN                       9  // Analog Input D63

//
// Heaters / Fans
//
#define HEATER_0_PIN                           2  // PWM
#define HEATER_BED_PIN                         4  // PWM
#define HEATER_CHAMBER_PIN                     3  // PWM
#define FAN0_PIN                               7  // PWM

//
// Misc. Functions
//
#define SD_SS_PIN                             53
#define SD_DETECT_PIN                         39

#if ENABLED(CASE_LIGHT_ENABLE)
  #define CASE_LIGHT_PIN                       8
#endif

#if ENABLED(PSU_CONTROL)
  #define PS_ON_PIN                           38  // UPS Module
#endif

//
// LCD Controller
//

#define BEEPER_PIN                            18

#if HAS_WIRED_LCD
  #define LCD_PINS_RS                         20
  #define LCD_PINS_EN                         30
  #define LCD_PINS_D4                         14
  #define LCD_PINS_D5                         21
  #define LCD_PINS_D6                          5
  #define LCD_PINS_D7                          6
  #define BTN_EN1                             40
  #define BTN_EN2                             41
  #define BTN_ENC                             19
#endif

///////////////////// SPARE HEADERS //////////////

/**
 * J25
 * 1 D54
 * 2 D55
 * 3 D56
 * 4 D57
 * 5 D58
 * 6 D59
 * 7 D60
 * 8 D61

Hotend High Temp Connected : D12
*/
