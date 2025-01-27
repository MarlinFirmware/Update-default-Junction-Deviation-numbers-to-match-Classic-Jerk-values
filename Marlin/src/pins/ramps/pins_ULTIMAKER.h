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
 * Ultimaker pin assignments
 * ATmega2560, ATmega1280
 */

/**
 * Rev B   2 JAN 2017
 *
 *  Added pin definitions for:
 *    M3, M4 & M5 spindle control commands
 *    case light
 */

#include "env_validate.h"

#define BOARD_INFO_NAME         "Ultimaker"
#define DEFAULT_MACHINE_NAME    BOARD_INFO_NAME
#define DEFAULT_SOURCE_CODE_URL "github.com/Ultimaker/Marlin"

//
// Servos
//
#define SERVO0_PIN                            11

//
// Limit Switches
//
#define X_MIN_PIN                             22
#define X_MAX_PIN                             24
#define Y_MIN_PIN                             26
#define Y_MAX_PIN                             28
#define Z_MIN_PIN                             30
#define Z_MAX_PIN                             32

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     32
#endif

//
// Steppers
//
#define X_STEP_PIN                            25
#define X_DIR_PIN                             23
#define X_ENABLE_PIN                          27

#define Y_STEP_PIN                            31
#define Y_DIR_PIN                             33
#define Y_ENABLE_PIN                          29

#define Z_STEP_PIN                            37
#define Z_DIR_PIN                             39
#define Z_ENABLE_PIN                          35

#define E0_STEP_PIN                           43
#define E0_DIR_PIN                            45
#define E0_ENABLE_PIN                         41

#define E1_STEP_PIN                           49
#define E1_DIR_PIN                            47
#define E1_ENABLE_PIN                         48

//
// Temperature Sensors
//
#define TEMP_0_PIN                             8  // Analog Input
#define TEMP_1_PIN                             9  // Analog Input
#define TEMP_BED_PIN                          10  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           2
#define HEATER_1_PIN                           3
#define HEATER_BED_PIN                         4

#ifndef FAN0_PIN
  #define FAN0_PIN                             7
#endif

//
// Misc. Functions
//
#define SD_SS_PIN                             53
#define LED_PIN                               13
#define PS_ON_PIN                             12
#define SUICIDE_PIN                           54  // PIN that has to be turned on right after start, to keep power flowing.

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                       8
#endif

//
// LCD / Controller
//

#if HAS_WIRED_LCD

  #define BEEPER_PIN                          18

  #if IS_NEWPANEL

    #define LCD_PINS_RS                       20
    #define LCD_PINS_EN                       17
    #define LCD_PINS_D4                       16
    #define LCD_PINS_D5                       21
    #define LCD_PINS_D6                        5
    #define LCD_PINS_D7                        6

    // Buttons directly attached
    #define BTN_EN1                           40
    #define BTN_EN2                           42
    #define BTN_ENC                           19

    #define SD_DETECT_PIN                     38

  #else // !IS_NEWPANEL - Old style panel with shift register

    // Buttons attached to a shift register
    #define SHIFT_CLK_PIN                     38
    #define SHIFT_LD_PIN                      42
    #define SHIFT_OUT_PIN                     40
    #define SHIFT_EN_PIN                      17

    #define LCD_PINS_RS                       16
    #define LCD_PINS_EN                        5
    #define LCD_PINS_D4                        6
    #define LCD_PINS_D5                       21
    #define LCD_PINS_D6                       20
    #define LCD_PINS_D7                       19

    #define SD_DETECT_PIN                     -1

  #endif // !IS_NEWPANEL

#endif // HAS_WIRED_LCD

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #define SPINDLE_LASER_PWM_PIN                9  // Hardware PWM
  #define SPINDLE_LASER_ENA_PIN               10  // Pullup!
  #define SPINDLE_DIR_PIN                     11  // use the EXP3 PWM header
#endif
