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
 * Smoothieware Smoothieboard pin assignments
 * See https://smoothieware.github.io/Webif-pack/documentation/web/html/smoothieboard.html
 */

#define ALLOW_LPC1768_OR_9
#include "env_validate.h"

#define BOARD_INFO_NAME   "Smoothieboard"
#define BOARD_WEBSITE_URL "smoothieware.github.io"

//
// Servos
//
#define SERVO0_PIN                         P1_23

//
// Limit Switches
//
#define X_MIN_PIN                          P1_24
#define X_MAX_PIN                          P1_25
#define Y_MIN_PIN                          P1_26
#define Y_MAX_PIN                          P1_27
#define Z_MIN_PIN                          P1_28
#define Z_MAX_PIN                          P1_29

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

#define E1_STEP_PIN                        P2_08
#define E1_DIR_PIN                         P2_13
#define E1_ENABLE_PIN                      P4_29

//
// Temperature Sensors
// 3.3V max when defined as an analog input
//
#define TEMP_0_PIN                      P0_23_A0  // (T1)
#define TEMP_BED_PIN                    P0_24_A1  // (T2)
#define TEMP_1_PIN                      P0_25_A2  // (T3)
#define TEMP_2_PIN                      P0_26_A3  // (T4)

//
// Heaters / Fans
//
#define HEATER_BED_PIN                     P2_05
#define HEATER_0_PIN                       P2_07
#define HEATER_1_PIN                       P1_23
#ifndef FAN0_PIN
  #define FAN0_PIN                         P2_06
#endif
#define FAN1_PIN                           P2_04

//
// LCD / Controller
//

#if ANY(VIKI2, miniVIKI)

  #define BEEPER_PIN                       P1_31
  #define DOGLCD_A0                        P2_11
  #define DOGLCD_CS                        P0_16

  #define BTN_EN1                          P3_25
  #define BTN_EN2                          P3_26
  #define BTN_ENC                          P1_30

  #define SD_DETECT_PIN                    P1_18
  #define SD_SS_PIN                        P1_21

  #define STAT_LED_RED_PIN                 P1_19
  #define STAT_LED_BLUE_PIN                P1_20

#elif HAS_WIRED_LCD

  /**
   * SD Support
   *
   * For the RRD GLCD it CANNOT share the same SPI as the LCD so it must be
   * hooked up to the onboard SDCard SPI and use a spare pin for the SDCS.
   * Also note that an external SDCard sharing the SPI port with the
   * onboard/internal SDCard must be ejected before rebooting as the bootloader
   * does not like the external card. NOTE Smoothie will not boot if the external
   * sdcard is inserted in the RRD LCD sdcard slot at boot time, it must be
   * inserted after it has booted.
   */
  #define SD_DETECT_PIN                    P0_27  // EXP2 Pin 7 (SD_CD, SD_DET)

  #define SD_MISO_PIN                      P0_08  // EXP2 Pin 1 (PB3, SD_MISO)
  #define SD_SCK_PIN                       P0_07  // EXP2 Pin 2 (SD_SCK)
  #define SD_SS_PIN                        P0_28  // EXP2 Pin 4 (SD_CSEL, SD_CS)
  #define SD_MOSI_PIN                      P0_09  // EXP2 Pin 6 (PB2, SD_MOSI)

  /**
   * The Smoothieboard supports the REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER with either
   * a custom cable with breakouts to the pins indicated below or the RRD GLCD Adapter board.
   */
  #if IS_RRD_FG_SC
    //  EXP1 Pins
    #define BEEPER_PIN                     P1_31  // EXP1 Pin 1
    #define BTN_ENC                        P1_30  // EXP1 Pin 2
    #define LCD_PINS_EN                    P0_18  // EXP1 Pin 3 (MOSI)
    #define LCD_PINS_RS                    P0_16  // EXP1 Pin 4 (CS)
    #define LCD_PINS_D4                    P0_15  // EXP1 Pin 5 (SCK)
    //  EXP2 Pins
    #define BTN_EN2                        P3_26  // EXP2 Pin 3
    #define BTN_EN1                        P3_25  // EXP2 Pin 5

  #elif IS_TFTGLCD_PANEL

    #define SD_DETECT_PIN                  P0_27  // EXP2 Pin 7 (SD_CD, SD_DET)

    #if ENABLED(TFTGLCD_PANEL_SPI)
      #define TFTGLCD_CS                   P3_26  // EXP2 Pin 3
    #endif

  #else
    #error "Marlin's Smoothieboard support cannot drive your LCD."
  #endif

#endif

/**
 * I2C Digipots - MCP4451
 * Address 58 (2C << 1)
 * Set from 0 - 127 with stop bit.
 * (Ex. 3F << 1 | 1)
 */
#define DIGIPOTS_I2C_SCL                   P0_00
#define DIGIPOTS_I2C_SDA_X                 P0_04
#define DIGIPOTS_I2C_SDA_Y                 P0_10
#define DIGIPOTS_I2C_SDA_Z                 P0_19
#define DIGIPOTS_I2C_SDA_E0                P0_21
#define DIGIPOTS_I2C_SDA_E1                P4_29

#ifndef DIGIPOT_I2C_ADDRESS_A
  #define DIGIPOT_I2C_ADDRESS_A             0x2C  // unshifted slave address (58 <- 2C << 1)
#endif
