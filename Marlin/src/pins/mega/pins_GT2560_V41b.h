/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Geeetech GT2560 V4.1b Pins
 * Schematic: https://www.geeetech.com/download.html?spm=a2g0s.imconversation.0.0.22d23e5fXlQBWv&download_id=45
 * ATmega2560
 */

#define ALLOW_MEGA1280
#include "env_validate.h"

#if HOTENDS > 3 || E_STEPPERS > 3
  #error "GT2560 supports up to 3 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "GT2560 4.1b"

/**   Limit Switches Connectors
 *    All have external 10k pull-up resistors
 *
 * X-Y-Z AXIS MAX LIMIT SWITCHES CONNECTOR (H6)
 *        ---------
 *       | 1  4  7 |  5V 32 Z0_MAX GND
 *       | 2  5  8 |  5V 26 Y_MAX  GND
 *       | 3  6  9 |  5V 22 X_MAX  GND
 *        ---------
 *           H6
 *
 *  X AXIS              Y AXIS               Z1 AXIS            Z0 AXIS
 *    ---                 ---                 ---                 ---
 *   | 1 | 5V            | 1 | 5V            | 1 | 5V            | 1 | 5V
 *   | 2 | 24 X_MIN      | 2 | 28 Y_MIN      | 2 | 80 Z1_MIN     | 2 | 30 Z0_MIN
 *   | 3 | GND           | 3 | GND           | 3 | GND           | 3 | GND
 *    ---                 ---                 ---                 ---
 *     J3                  J4                  J5                  J6
 *
 */

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
#ifndef Z2_STOP_PIN
  #define Z2_STOP_PIN                         80  // PE7 - Extended mega2560 pin
#endif

/**                   Filament Runout Sensors
 *
 *      Filament 1           Filament 2           Filament 3
 *         ---                  ---                  ---
 *        | 1 | 5V             | 1 | 5V             | 1 | 5V
 *        | 2 | 66 F_DET0      | 2 | 67 F_DET1      | 2 | 54 F_DET2
 *        | 3 | GND            | 3 | GND            | 3 | GND
 *         ---                  ---                  ---
 *         J12                  J13                  J14
 */

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

/**                  Printhead Connector
 *                         ------
 * (PWM8_FAN0) FAN_E0   9 | 1  2 | 9    FAN_E0 24V PWM FROM (PWM8_FAN0)
 *               (T0) A11 | 3  4 | A11  (T0) E0 Temp
 *                    GND | 5  6 | 30   Z_MIN1 same as (Z0_MIN)
 *                     5V | 7  8 | 11   (PB5) servo for BL_TOUCH/3D_TOUCH
 *      (PB4_HE2) HE2  19 | 9 10 | GND
 *      (PB4_HE2) HE2  19 |11 12 | 19   HE2 24V PWM out for E0 (PB4_HE2)
 *                    V24 |13 14 | V24
 *                    V24 |15 16 | V24
 *                         ------
 *                           H3
 */

#define SERVO0_PIN                            11  // BLTouch / 3DTouch

//
// Z Probe PIN6 Header H3 (Print head connector)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN              Z_MIN_PIN
#endif

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
#define HEATER_0_PIN                          10  // PWM out to E0
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

/**            LCD Connector
 *                ------
 *            5V | 1  2 | GND
 *   (LCM_D7) 36 | 3  4 |  5 (LCM_D6)
 *   (LCM_D5) 21 | 5  6 | GND
 *   (LCM_D4) 16 | 7  8 | 17 (LCM_EN)
 * (EC_PRESS) 19 | 9 10 | GND
 *       (RESET) |11 12 | 18 (BEEP)
 *                ------
 *                  H2
 */

//#define H2_01_PIN                           5V
//#define H2_02_PIN                          GND
#define H2_03_PIN                             36  // LCM_D7
#define H2_04_PIN                              5  // LCM_D6
#define H2_05_PIN                             21  // LCM_D5
//#define H2_06_PIN                          GND
#define H2_07_PIN                             16  // LCM_D4
#define H2_08_PIN                             17  // LCM_EN
#define H2_09_PIN                             19  // EC_PRESS
//#define H2_10_PIN                          GND
//#define H2_11_PIN                        RESET
#define H2_12_PIN                             18  // BEEP

#define LCM_RS                                20  // Pin named and connected to 10k pull-up resistor but unused

#if ENABLED(YHCB2004)
  #define YHCB2004_SS_PIN              H2_04_PIN
  #define YHCB2004_SCK_PIN             H2_05_PIN
  #define YHCB2004_MOSI_PIN            H2_03_PIN
  #define YHCB2004_MISO_PIN               LCM_RS  // Unused on V4.1b board
  #define BTN_EN1                      H2_07_PIN
  #define BTN_EN2                      H2_08_PIN
  #define BTN_ENC                      H2_09_PIN
  #define BEEPER_PIN                   H2_12_PIN
#elif ENABLED(CR10_STOCKDISPLAY)                  // Firmware compatible with stock GT 128x64 12pin LCD for the V41b
  #define LCD_PINS_RS                  H2_04_PIN  // DOGLCD_CS
  #define LCD_PINS_D4                  H2_05_PIN  // DOGLCD_SCK
  #define LCD_PINS_EN                  H2_03_PIN  // DOGLCD_MOSI
  #define BTN_EN1                      H2_07_PIN
  #define BTN_EN2                      H2_08_PIN
  #define BTN_ENC                      H2_09_PIN
  #define BEEPER_PIN                   H2_12_PIN
#elif HAS_WIRED_LCD
  #error "GT2560 V4.1b requires an adapter for common LCDs."
  /*  Cannot use because V4.1b board has not LCD_PINS_RS wired to display connector */
#endif
