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
 * Geeetech GT2560 3.0/3.1 pin assignments
 * Schematic (3.0): https://github.com/Geeetech3D/Diagram/blob/master/GT2560_V3.0_SCH.pdf
 * ATmega2560
 *
 * Also GT2560 RevB and GT2560 4.0/4.1
 */

#define ALLOW_MEGA1280
#include "env_validate.h"

#if HOTENDS > 3 || E_STEPPERS > 3
  #error "GT2560 supports up to 3 hotends / E steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "GT2560 3.x"
#endif

//
// Servos
//
#define SERVO0_PIN                            11  // 13 untested  3Dtouch

//
// Limit Switches
//
#ifndef X_STOP_PIN
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                         24
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                         22
  #endif
#endif
#ifndef Y_STOP_PIN
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                         28
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                         26
  #endif
#endif
#ifndef Z_STOP_PIN
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                         30
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                         32
  #endif
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     32
#endif

//
// Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      66
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                     67
#endif
#ifndef FIL_RUNOUT3_PIN
  #define FIL_RUNOUT3_PIN                     54
#endif

//
// Power Loss Detection
//
#define POWER_LOSS_PIN                        69  // Pin to detect power loss
#define POWER_LOSS_STATE                     LOW

//
// Steppers
//
#define X_STEP_PIN                            37
#define X_DIR_PIN                             39
#define X_ENABLE_PIN                          35

#define Y_STEP_PIN                            31
#define Y_DIR_PIN                             33
#define Y_ENABLE_PIN                          29

#define Z_STEP_PIN                            25
#define Z_DIR_PIN                             23
#define Z_ENABLE_PIN                          27

#define E0_STEP_PIN                           46
#define E0_DIR_PIN                            44
#define E0_ENABLE_PIN                         12

#define E1_STEP_PIN                           49
#define E1_DIR_PIN                            47
#define E1_ENABLE_PIN                         48

#define E2_STEP_PIN                           43
#define E2_DIR_PIN                            45
#define E2_ENABLE_PIN                         41

//
// Temperature Sensors
//
#define TEMP_0_PIN                            11  // Analog Input
#define TEMP_1_PIN                             9  // Analog Input
#define TEMP_2_PIN                             8  // Analog Input
#define TEMP_BED_PIN                          10  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                          10
#define HEATER_1_PIN                           3
#define HEATER_2_PIN                           2
#define HEATER_BED_PIN                         4
#define FAN0_PIN                               9
#define FAN1_PIN                               8
#define FAN2_PIN                               7

//
// Misc. Functions
//
#define SD_DETECT_PIN                         38
#define SD_SS_PIN                             53
#define LED_PIN                               13  // Use 6 (case light) for external LED. 13 is internal (yellow) LED.
#define PS_ON_PIN                             12

#if NUM_RUNOUT_SENSORS < 3
  #define SUICIDE_PIN                         54  // This pin must be enabled at boot to keep power flowing
#endif

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                       6  // 21
#endif

//
// LCD Controller
//
#define BEEPER_PIN                            18

#if ENABLED(YHCB2004)
  #ifndef YHCB2004_MOSI_PIN
    #define YHCB2004_MOSI_PIN                 21
  #endif
  #ifndef YHCB2004_MISO_PIN
    #define YHCB2004_MISO_PIN                 36
  #endif
  #ifndef YHCB2004_SCK_PIN
    #define YHCB2004_SCK_PIN                   5
  #endif
  #ifndef YHCB2004_SS_PIN
    #define YHCB2004_SS_PIN                   SS
  #endif
#elif HAS_WIRED_LCD
  #ifndef LCD_PINS_RS
    #define LCD_PINS_RS                       20
  #endif
  #ifndef LCD_PINS_EN
    #define LCD_PINS_EN                       17
  #endif
  #ifndef LCD_PINS_D4
    #define LCD_PINS_D4                       16
  #endif
  #ifndef LCD_PINS_D5
    #define LCD_PINS_D5                       21
  #endif
  #ifndef LCD_PINS_D6
    #define LCD_PINS_D6                        5
  #endif
  #ifndef LCD_PINS_D7
    #define LCD_PINS_D7                       36
  #endif
#endif

#if ENABLED(YHCB2004)
  #ifndef BTN_EN1
    #define BTN_EN1                           16
  #endif
  #ifndef BTN_EN2
    #define BTN_EN2                           17
  #endif
  #ifndef BTN_ENC
    #define BTN_ENC                           19
  #endif
#elif IS_NEWPANEL
  #ifndef BTN_EN1
    #define BTN_EN1                           42
  #endif
  #ifndef BTN_EN2
    #define BTN_EN2                           40
  #endif
  #ifndef BTN_ENC
    #define BTN_ENC                           19
  #endif
#endif
