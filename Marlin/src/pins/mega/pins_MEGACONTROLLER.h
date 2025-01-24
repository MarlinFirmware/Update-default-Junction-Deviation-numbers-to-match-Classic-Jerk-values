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
 * Mega controller pin assignments
 * Schematic: https://reprap.org/mediawiki/images/b/ba/Mega_controller.pdf
 * ATmega2560
 */

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Mega Controller supports up to 2 hotends / E steppers."
#endif

#include "env_validate.h"

#define BOARD_INFO_NAME "Mega Controller"

//
// Servos
//
#define SERVO0_PIN                            30
#define SERVO1_PIN                            31
#define SERVO2_PIN                            32
#define SERVO3_PIN                            33

//
// Limit Switches
//
#define X_MIN_PIN                             43
#define X_MAX_PIN                             42
#define Y_MIN_PIN                             38
#define Y_MAX_PIN                             41
#define Z_MIN_PIN                             40
#define Z_MAX_PIN                             37

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     37
#endif

//
// Steppers
//
#define X_STEP_PIN                            62  // A8
#define X_DIR_PIN                             63  // A9
#define X_ENABLE_PIN                          61  // A7

#define Y_STEP_PIN                            65  // A11
#define Y_DIR_PIN                             66  // A12
#define Y_ENABLE_PIN                          64  // A10

#define Z_STEP_PIN                            68  // A14
#define Z_DIR_PIN                             69  // A15
#define Z_ENABLE_PIN                          67  // A13

#define E0_STEP_PIN                           23
#define E0_DIR_PIN                            24
#define E0_ENABLE_PIN                         22

#define E1_STEP_PIN                           26
#define E1_DIR_PIN                            27
#define E1_ENABLE_PIN                         25

//
// Temperature Sensors
//
#if TEMP_SENSOR_0 == -1
  #define TEMP_0_PIN                           4  // Analog Input
#else
  #define TEMP_0_PIN                           0  // Analog Input
#endif

#if TEMP_SENSOR_1 == -1
  #define TEMP_1_PIN                           5  // Analog Input
#else
  #define TEMP_1_PIN                           2  // Analog Input
#endif

#define TEMP_2_PIN                             3  // Analog Input

#if TEMP_SENSOR_BED == -1
  #define TEMP_BED_PIN                         6  // Analog Input
#else
  #define TEMP_BED_PIN                         1  // Analog Input
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                          29
#define HEATER_1_PIN                          34
#define HEATER_BED_PIN                        28

#ifndef FAN0_PIN
  #define FAN0_PIN                            39
#endif
#define FAN1_PIN                              35
#define FAN2_PIN                              36

#ifndef CONTROLLER_FAN_PIN
  #define CONTROLLER_FAN_PIN            FAN2_PIN
#endif

#define FAN_SOFT_PWM_REQUIRED

//
// Misc. Functions
//
#define SD_SS_PIN                        MINI_06
#define LED_PIN                               13

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                       2
#endif

/**
 * MegaController LCD/SD Connector
 *
 *     SDD   MOSI  SDSS  --   RESET  --  LCDSS  A0    KILL  ENC
 *     49    51    53           |         45    47    12    10
 *     PL0   PB2   PB0          |         PL4   PL2   PB6   PB4
 *    ----------------------------------------------------------
 *   |  2     4     6     8    10    12    14    16    18    20 |
 *   |  1     3     5     7     9    11    13    15    17    19 |
 *    ----------------------------------------------------------
 *     |     PB3   PB1    |     |         PL5   PL3   PL1   PB5
 *     |     50    52     |     |         44    46    48    11
 *     5V    MISO  SCK   GND   3V3   --   BL    BEEP  EN1   EN2
 */
#define MINI_02                               49  // SD_DETECT
#define MINI_03                               50  // MISO
#define MINI_04                               51  // MOSI
#define MINI_05                               52  // SCK
#define MINI_06                               53  // SDSS
#define MINI_13                               44  // BACKLIGHT
#define MINI_14                               45  // LCDSS
#define MINI_15                               46  // BEEP
#define MINI_16                               47  // A0
#define MINI_17                               48  // EN1
#define MINI_18                               12  // KILL
#define MINI_19                               11  // EN2
#define MINI_20                               10  // ENC

//
// LCD / Controller
//

#if ENABLED(MINIPANEL)

  #define BEEPER_PIN                     MINI_15

  #define DOGLCD_A0                      MINI_16
  #define DOGLCD_CS                      MINI_14
  #define LCD_BACKLIGHT_PIN              MINI_13  // backlight LED on PA3

  #define KILL_PIN                       MINI_18

  #define BTN_ENC                        MINI_20
  #define BTN_EN1                        MINI_17
  #define BTN_EN2                        MINI_19

  #define SD_DETECT_PIN                  MINI_02

  //#define LCD_SCREEN_ROTATE                180  // 0, 90, 180, 270

#endif // MINIPANEL

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #define SPINDLE_LASER_PWM_PIN                6  // Hardware PWM
  #define SPINDLE_LASER_ENA_PIN                7  // Pullup!
  #define SPINDLE_DIR_PIN                      8
#endif
