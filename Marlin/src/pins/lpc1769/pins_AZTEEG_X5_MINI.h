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
 * Azteeg X5 MINI pin assignments
 * Schematic: http://files.panucatt.com/datasheets/x5mini_design_files.zip
 */
#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Azteeg X5 MINI"
#endif
#define BOARD_WEBSITE_URL "panucatt.com"

//
// LED
//
#define LED_PIN                            P1_18
#define LED2_PIN                           P1_20
#define LED3_PIN                           P1_19
#define LED4_PIN                           P1_21

//
// Servos
//
#define SERVO0_PIN                         P1_29

//
// Limit Switches
//
#define X_STOP_PIN                         P1_24
#define Y_STOP_PIN                         P1_26
#define Z_STOP_PIN                         P1_28

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                _EXP1_09
#endif

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                  _EXP1_04  // Analog Input (P0_25)
#endif

//
// Steppers
//
#define X_STEP_PIN                         P2_01
#define X_DIR_PIN                          P0_11
#define X_ENABLE_PIN                       P0_10

#define Y_STEP_PIN                         P2_02
#define Y_DIR_PIN                          P0_20
#define Y_ENABLE_PIN                       P0_19

#define Z_STEP_PIN                         P2_03
#define Z_DIR_PIN                          P0_22
#define Z_ENABLE_PIN                       P0_21

#define E0_STEP_PIN                        P2_00
#define E0_DIR_PIN                         P0_05
#define E0_ENABLE_PIN                      P0_04

//
// DIGIPOT slave addresses (7-bit unshifted)
//
#ifndef DIGIPOT_I2C_ADDRESS_A
  #define DIGIPOT_I2C_ADDRESS_A             0x2C
#endif
#ifndef DIGIPOT_I2C_ADDRESS_B
  #define DIGIPOT_I2C_ADDRESS_B             0x2E
#endif

//
// Temperature Sensors
// 3.3V max when defined as an analog input
//
#define TEMP_BED_PIN                    P0_23_A0  // A0 (TH1)
#define TEMP_0_PIN                      P0_24_A1  // A1 (TH2)

//
// Heaters / Fans
//
#define HEATER_BED_PIN                     P2_07
#define HEATER_0_PIN                       P2_05
#ifndef FAN0_PIN
  #define FAN0_PIN                         P0_26
#endif
#define FAN1_PIN                           P1_25

//
// Headers V1.1 - V3.0
//
//#define _EXP1_01                         -1     // GND
#define _EXP1_02                           P1_03
//#define _EXP1_03                         -1     // 3.3V
#define _EXP1_04                        P0_25_A2
#define _EXP1_05                           P0_27  // SDA0
#define _EXP1_06                           P4_29
#define _EXP1_07                           P0_28  // SCL0
#define _EXP1_08                           P2_08
#define _EXP1_09                           P2_04
#define _EXP1_10                           P1_22

#define _EXP2_01                           P1_31
#define _EXP2_02                           P3_26
#define _EXP2_03                           P2_11
#define _EXP2_04                           P3_25
#define _EXP2_05                           P1_23
#define _EXP2_06                           P0_17
#define _EXP2_07                           P0_16
#define _EXP2_08                           P2_06
#define _EXP2_09                           P0_15
#define _EXP2_10                           P0_18

//
// Only V2.0
//
//#define _J7_01                           -1     // 3.3V
//#define _J7_02                           -1     // GND
#define _J7_03                             P1_16
#define _J7_04                             P1_17
#define _J7_05                             P1_15
#define _J7_06                             P0_14
#define _J7_07                             P1_09
#define _J7_08                             P1_10
#define _J7_09                             P1_08
#define _J7_10                             P1_04
#define _J7_11                             P1_00
#define _J7_12                             P1_01

//
// Display
//
#if HAS_WIRED_LCD

  #if ENABLED(CR10_STOCKDISPLAY)

    // Re-Arm can support Creality stock display without SD card reader and single cable on EXP3.
    // Re-Arm J3 pins 1 (p1.31) & 2 (P3.26) are not used. Stock cable will need to have one
    // 10-pin IDC connector trimmed or replaced with a 12-pin IDC connector to fit J3.
    // Requires REVERSE_ENCODER_DIRECTION in Configuration.h

    #define BEEPER_PIN                  _EXP2_03

    #define BTN_EN1                     _EXP2_07
    #define BTN_EN2                     _EXP2_05
    #define BTN_ENC                     _EXP2_04

    #define LCD_PINS_RS                 _EXP2_09
    #define LCD_PINS_EN                 _EXP2_10
    #define LCD_PINS_D4                 _EXP2_08

  #else

    #define BTN_EN1                     _EXP2_02
    #define BTN_EN2                     _EXP2_04
    #define BTN_ENC                     _EXP2_03

    #define SD_DETECT_PIN               _EXP2_01
    #define KILL_PIN                    _EXP1_10
    #define LCD_PINS_RS                 _EXP2_07
    #define LCD_SDSS_PIN                _EXP2_07
    #define LCD_BACKLIGHT_PIN           _EXP2_07
    #define LCD_PINS_EN                 _EXP2_10
    #define LCD_PINS_D4                 _EXP2_09

    #define DOGLCD_A0                   _EXP2_08

    #if IS_RRW_KEYPAD
      #define SHIFT_OUT_PIN             _EXP2_10
      #define SHIFT_CLK_PIN             _EXP2_09
      #define SHIFT_LD_PIN              _EXP2_01
    #elif !IS_NEWPANEL
      //#define SHIFT_OUT_PIN           _EXP2_03
      //#define SHIFT_CLK_PIN           _EXP2_02
      //#define SHIFT_LD_PIN            _EXP2_04
      //#define SHIFT_EN_PIN            _EXP1_10
    #endif

    #if ANY(VIKI2, miniVIKI)
      #define BEEPER_PIN                   P1_30
      #define DOGLCD_CS                    P0_26
      #define DOGLCD_SCK              SD_SCK_PIN
      #define DOGLCD_MOSI            SD_MOSI_PIN

      #define STAT_LED_BLUE_PIN            P0_26
      #define STAT_LED_RED_PIN             P1_21

    #else
      #if IS_ULTIPANEL
        #define LCD_PINS_D5                P1_17
        #define LCD_PINS_D6                P1_14
        #define LCD_PINS_D7                P1_10
      #endif
      #define BEEPER_PIN                   P1_30
      #define DOGLCD_CS                 _EXP2_07

      #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
        #define BTN_ENC_EN           LCD_PINS_D7  // Detect the presence of the encoder
      #endif

    #endif

    #if ANY(VIKI2, miniVIKI, MINIPANEL)
      //#define LCD_SCREEN_ROTATE            180  // 0, 90, 180, 270
    #endif

  #endif

#endif // HAS_WIRED_LCD

//
// SD Support
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(LCD)
  #define SD_SCK_PIN                    _EXP2_09
  #define SD_MISO_PIN                   _EXP2_06
  #define SD_MOSI_PIN                   _EXP2_10
  #define SD_SS_PIN                     _EXP2_05
#elif SD_CONNECTION_IS(ONBOARD)
  #undef SD_DETECT_PIN
  #define SD_SCK_PIN                       P0_07
  #define SD_MISO_PIN                      P0_08
  #define SD_MOSI_PIN                      P0_09
  #define ONBOARD_SD_CS_PIN                P0_06  // Chip select for "System" SD card
  #define SD_SS_PIN            ONBOARD_SD_CS_PIN
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "No custom SD drive cable defined for this board."
#endif
