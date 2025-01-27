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
 * Geeetech GT2560 Revision A board pin assignments, based on the work of
 * George Robles (https://georges3dprinters.com) and
 * Richard Smith <galorin@gmail.com>
 * Schematic: https://www.geeetech.com/wiki/images/9/90/GT2560_sch.pdf
 * ATmega2560
 */

#define ALLOW_MEGA1280
#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "GT2560 Rev.A"
#endif
#define DEFAULT_MACHINE_NAME "Prusa i3 Pro B"

//
// Limit Switches
//
#define X_MIN_PIN                             22
#define X_MAX_PIN                             24
#define Y_MIN_PIN                             26
#define Y_MAX_PIN                             28
#define Z_MIN_PIN                             30

#if ENABLED(BLTOUCH) && !defined(SERVO0_PIN)
  #define SERVO0_PIN                          32
#endif

#if SERVO0_PIN == 32
  #define Z_MAX_PIN                           -1
#else
  #define Z_MAX_PIN                           32
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
#define TEMP_0_PIN                             8
#define TEMP_1_PIN                             9
#define TEMP_BED_PIN                          10

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

// Power monitoring pins - set to 0 for main VIN, 1 for dedicated bed supply rail.
// Don't forget to enable POWER_MONITOR_VOLTAGE in Configuration_adv.h
// and set POWER_MONITOR_VOLTS_PER_VOLT to 0.090909.
#define POWER_MONITOR_VOLTAGE_PIN              0

/**              LCD                             SDCARD
 *              ------                           ------
 *    (TX1) 18 | 1  2 | 19 (RX1)      (MISO) 50 | 1  2 | 52 (SCK)
 *    (RX2) 17 | 3  4 | 20 (SDA)             42 | 3  4 | 53 (SS)
 *    (TX2) 16 | 5  6   21 (SCL)             40 | 5  6   51 (MOSI)
 *           5 | 7  8 |  6                   38 | 7  8 | RESET
 *         GND | 9 10 | 5V                  GND | 9 10 | 5V/3V3
 *              ------                           ------
 *               SV1                              SV3
 *
 * GT2560 LCD & SD headers follow typical EXP1 & EXP2 format.
 * SD header voltage pin set by link pads beneath the header; R25 for 5V, R44 for 3.3V (default)
 * Pins 20 (SDA) and 21 (SCL) have external 10K pull-ups on the board.
 */

#define EXP1_01_PIN                           18  // TX1 / BEEPER
#define EXP1_02_PIN                           19  // RX1 / ENC
#define EXP1_03_PIN                           17  // RX2 / CS
#define EXP1_04_PIN                           20  // SDA / A0
#define EXP1_05_PIN                           16  // TX2 / LCD_RS
#define EXP1_06_PIN                           21  // SCL / CS
#define EXP1_07_PIN                            5  // D6  / A0
#define EXP1_08_PIN                            6  // D7  / D4

#define EXP2_01_PIN                           50  // MISO
#define EXP2_02_PIN                           52  // SCK
#define EXP2_03_PIN                           42  // EN2
#define EXP2_04_PIN                           53  // SDSS
#define EXP2_05_PIN                           40  // EN1
#define EXP2_06_PIN                           51  // MOSI
#define EXP2_07_PIN                           38  // SD_DET
#define EXP2_08_PIN                           -1  // RESET

#define SD_SS_PIN                    EXP2_04_PIN
#define LED_PIN                               13

#if HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN

  #if IS_NEWPANEL

    #if ENABLED(MKS_MINI_12864)
      #define DOGLCD_CS              EXP1_06_PIN
      #define DOGLCD_A0              EXP1_07_PIN
      #define BTN_EN1                EXP2_05_PIN
      #define BTN_EN2                EXP2_03_PIN
    #elif ENABLED(FYSETC_MINI_12864)
      // Disconnect EXP2-1 and EXP2-2, otherwise future firmware upload won't work.
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_A0              EXP1_04_PIN

      #define NEOPIXEL_PIN           EXP1_06_PIN
      #define BTN_EN1                EXP2_03_PIN
      #define BTN_EN2                EXP2_05_PIN

      #define LCD_RESET_PIN          EXP1_05_PIN

      #define LCD_CONTRAST_INIT              220

      #define LCD_BACKLIGHT_PIN               -1
    #else
      #define LCD_PINS_RS            EXP1_04_PIN
      #define LCD_PINS_EN            EXP1_03_PIN
      #define LCD_PINS_D4            EXP1_05_PIN
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN
      #define BTN_EN1                EXP2_03_PIN
      #define BTN_EN2                EXP2_05_PIN
    #endif

    #define BTN_ENC                  EXP1_02_PIN
    #define SD_DETECT_PIN            EXP2_07_PIN

  #else // !IS_NEWPANEL

    #define SHIFT_CLK_PIN            EXP2_07_PIN
    #define SHIFT_LD_PIN             EXP2_03_PIN
    #define SHIFT_OUT_PIN            EXP2_05_PIN
    #define SHIFT_EN_PIN             EXP1_03_PIN

    #define LCD_PINS_RS              EXP1_05_PIN
    #define LCD_PINS_EN              EXP1_07_PIN
    #define LCD_PINS_D4              EXP1_08_PIN
    #define LCD_PINS_D5              EXP1_06_PIN
    #define LCD_PINS_D6              EXP1_04_PIN
    #define LCD_PINS_D7              EXP1_02_PIN

    #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
      #define BTN_ENC_EN             LCD_PINS_D7  // Detect the presence of the encoder
    #endif

    #define SD_DETECT_PIN                     -1

  #endif // !IS_NEWPANEL

#endif // HAS_WIRED_LCD
