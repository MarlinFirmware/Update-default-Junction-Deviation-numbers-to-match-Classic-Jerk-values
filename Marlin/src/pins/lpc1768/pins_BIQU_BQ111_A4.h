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
 * BIQU BQ111-A4 pin assignments
 *
 * Applies to the following boards:
 *
 *  BOARD_BIQU_BQ111_A4 (Hotend, Fan, Bed)
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "BIQU BQ111-A4"

//
// Limit Switches
//
#define X_MIN_PIN                          P1_24  // 10k pullup to 3.3V, 1K series
#define X_MAX_PIN                          P1_25  // 10k pullup to 3.3V, 1K series
#define Y_MIN_PIN                          P1_26  // 10k pullup to 3.3V, 1K series
#define Y_MAX_PIN                          P1_27  // 10k pullup to 3.3V, 1K series
#define Z_MIN_PIN                          P1_28  // 10k pullup to 3.3V, 1K series
#define Z_MAX_PIN                          P1_29  // 10k pullup to 3.3V, 1K series

//
// Steppers
//
#define X_STEP_PIN                         P2_00
#define X_DIR_PIN                          P0_05
#define X_ENABLE_PIN                       P0_04

#define Y_STEP_PIN                         P2_01
#define Y_DIR_PIN                          P0_11
#define Y_ENABLE_PIN                       P0_10

#define Z_STEP_PIN                         P2_02
#define Z_DIR_PIN                          P0_20
#define Z_ENABLE_PIN                       P0_19

#define E0_STEP_PIN                        P2_03
#define E0_DIR_PIN                         P0_22
#define E0_ENABLE_PIN                      P0_21

//
// Temperature Sensors
//  3.3V max when defined as an analog input
//
#define TEMP_0_PIN                      P0_23_A0  // A0 (T0)
#define TEMP_BED_PIN                    P0_24_A1  // A1 (T1)

//
// Heaters / Fans
//
#define HEATER_0_PIN                       P2_07
#define HEATER_BED_PIN                     P2_05
#ifndef FAN0_PIN
  #define FAN0_PIN                         P2_04
#endif

//
// Unused
//
//#define PIN_P2_10                        P2_10  // IBOOT-1
//#define PIN_P0_27                        P0_27  // Onboard SD Detect

/**
 * LCD / Controller
 *
 * REPRAP_DISCOUNT_SMART_CONTROLLER is not supported due to the lack of LCD_PINS_D5,
 * LCD_PINS_D6 or LCD_PINS_D7 in the EXP1 connector.
 *
 * A remote SD card is not supported as the pins routed to EXP2 are the same as used
 * for the onboard SD card, and a chip select signal is not provided for the remote
 * SD card.
 */
#if HAS_WIRED_LCD

  #define BEEPER_PIN                       P1_31  // EXP1-1

  #define BTN_EN1                          P3_26  // EXP2-3
  #define BTN_EN2                          P3_25  // EXP2-5
  #define BTN_ENC                          P1_30  // EXP1-2

  #define SD_DETECT_PIN                    P0_27  // EXP2-7
  #define LCD_PINS_RS                      P0_16  // EXP1-4
  #define LCD_PINS_EN                      P0_18  // (MOSI) EXP1-3
  #define LCD_PINS_D4                      P0_15  // (SCK)  EXP1-5

  #if ALL(HAS_MARLINUI_HD44780, IS_RRD_SC)
    #error "REPRAP_DISCOUNT_SMART_CONTROLLER displays aren't supported by the BIQU BQ111-A4"
  #endif

  #if HAS_MEDIA
    #error "SDSUPPORT is not supported by the BIQU BQ111-A4 when an LCD controller is used"
  #endif

#endif // HAS_WIRED_LCD

/**
 * SD Card Reader
 *
 * Software SPI is used to interface with a stand-alone SD card reader connected to EXP1.
 * Hardware SPI can't be used because P0_17 (MISO) is not brought out on this board.
 */
#if HAS_MEDIA
  #define SD_SCK_PIN                       P0_15  // EXP1-5
  #define SD_MISO_PIN                      P0_16  // EXP1-4
  #define SD_MOSI_PIN                      P0_18  // EXP1-3
  #define SD_SS_PIN                        P1_30  // EXP1-2
#endif

/**
 *  PWMS
 *
 *  There are 6 PWMS.  Each PWM can be assigned to one of two pins.
 *
 *  PWM1.1   P0_18   LCD_PINS_EN
 *  PWM1.1   P2_0    X_STEP_PIN
 *  PWM1.2   P1_20   <none>
 *  PWM1.2   P2_1    Y_STEP_PIN
 *  PWM1.3   P1_21   <none>
 *  PWM1.3   P2_2    Z_STEP_PIN
 *  PWM1.4   P1_23   <none>
 *  PWM1.4   P2_3    E0_STEP_PIN
 *  PWM1.5   P1_24   X_MIN_PIN
 *  PWM1.5   P2_4    FAN0_PIN
 *  PWM1.6   P1_26   Y_MIN_PIN
 *  PWM1.6   P2_5    HEATER_BED_PIN
 */
