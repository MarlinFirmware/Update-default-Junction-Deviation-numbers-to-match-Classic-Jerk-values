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
 * RAMPS-FD
 *
 * No EEPROM
 * Use 4k7 thermistor tables
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "RAMPS-FD v1"
#endif

#define INVERTED_HEATER_PINS
#define INVERTED_BED_PINS
#define INVERTED_FAN_PINS

//
// Servos
//
#define SERVO0_PIN                             7
#define SERVO1_PIN                             6
#define SERVO2_PIN                             5
#define SERVO3_PIN                             3

//
// Limit Switches
//
#define X_MIN_PIN                             22
#define X_MAX_PIN                             30
#define Y_MIN_PIN                             24
#define Y_MAX_PIN                             38
#define Z_MIN_PIN                             26
#define Z_MAX_PIN                             34

//
// Steppers
//
#define X_STEP_PIN                            63
#define X_DIR_PIN                             62
#define X_ENABLE_PIN                          48
#ifndef X_CS_PIN
  #define X_CS_PIN                            68
#endif

#define Y_STEP_PIN                            65
#define Y_DIR_PIN                             64
#define Y_ENABLE_PIN                          46
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            60
#endif

#define Z_STEP_PIN                            67
#define Z_DIR_PIN                             66
#define Z_ENABLE_PIN                          44
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            58
#endif

#define E0_STEP_PIN                           36
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         42
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           67
#endif

#define E1_STEP_PIN                           43
#define E1_DIR_PIN                            41
#define E1_ENABLE_PIN                         39
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           61
#endif

#define E2_STEP_PIN                           32
#define E2_DIR_PIN                            47
#define E2_ENABLE_PIN                         45
#ifndef E2_CS_PIN
  #define E2_CS_PIN                           59
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                             1  // Analog Input
#define TEMP_1_PIN                             2  // Analog Input
#define TEMP_2_PIN                             3  // Analog Input
#define TEMP_BED_PIN                           0  // Analog Input

// SPI for MAX Thermocouple
#if !HAS_MEDIA
  #define TEMP_0_CS_PIN                       53
#else
  #define TEMP_0_CS_PIN                       49
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                           9
#define HEATER_1_PIN                          10
#define HEATER_2_PIN                          11
#define HEATER_BED_PIN                         8

#ifndef FAN0_PIN
  #define FAN0_PIN                            12
#endif

//
// Misc. Functions
//
#define SD_SS_PIN                              4
#define LED_PIN                               13

/**
 *                RAMPS-FD LCD adapter
 *         ------                     ------
 *     37 | 1  2 | 35      (MISO) 50 | 1  2 | 76 (SCK)
 *     29 | 3  4 | 27       (EN2) 31 | 3  4 |  4 (SD_SS)
 *     25   5  6 | 23       (EN1) 33   5  6 | 75 (MOSI)
 *     16 | 7  8 | 17       (SDD) 49 | 7  8 | RESET
 *    GND | 9 10 | 5V            GND | 9 10 | --
 *         ------                     ------
 *          EXP1                       EXP2
 */
#define EXP1_01_PIN                           37
#define EXP1_02_PIN                           35
#define EXP1_03_PIN                           29
#define EXP1_04_PIN                           27
#define EXP1_05_PIN                           25
#define EXP1_06_PIN                           23
#define EXP1_07_PIN                           16
#define EXP1_08_PIN                           17

#define EXP2_01_PIN                           74
#define EXP2_02_PIN                           76
#define EXP2_03_PIN                           31
#define EXP2_04_PIN                            4
#define EXP2_05_PIN                           33
#define EXP2_06_PIN                           75
#define EXP2_07_PIN                           49
#define EXP2_08_PIN                           -1

//
// LCD / Controller
//

#if HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN

  #define BTN_ENC                    EXP1_02_PIN
  #define BTN_EN2                    EXP2_03_PIN
  #define BTN_EN1                    EXP2_05_PIN

  #define SD_DETECT_PIN              EXP2_07_PIN

  #if IS_NEWPANEL
    #define LCD_PINS_RS              EXP1_07_PIN
    #define LCD_PINS_EN              EXP1_08_PIN
  #endif

  #if ENABLED(FYSETC_MINI_12864)
    #define DOGLCD_CS                EXP1_08_PIN
    #define DOGLCD_A0                EXP1_07_PIN
    #define DOGLCD_SCK               EXP2_02_PIN
    #define DOGLCD_MOSI              EXP2_06_PIN

    //#define FORCE_SOFT_SPI                      // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

    #define LCD_RESET_PIN            EXP1_06_PIN  // Must be high or open for LCD to operate normally.

    #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
      #ifndef RGB_LED_R_PIN
        #define RGB_LED_R_PIN        EXP1_05_PIN
      #endif
      #ifndef RGB_LED_G_PIN
        #define RGB_LED_G_PIN        EXP1_04_PIN
      #endif
      #ifndef RGB_LED_B_PIN
        #define RGB_LED_B_PIN        EXP1_03_PIN
      #endif
    #elif ENABLED(FYSETC_MINI_12864_2_1)
      #define NEOPIXEL_PIN           EXP1_05_PIN
    #endif

  #elif IS_NEWPANEL

    #define LCD_PINS_D4              EXP1_06_PIN
    #define LCD_PINS_D5              EXP1_05_PIN
    #define LCD_PINS_D6              EXP1_04_PIN
    #define LCD_PINS_D7              EXP1_03_PIN

    #if ENABLED(MINIPANEL)
      #define DOGLCD_CS              EXP1_05_PIN
      #define DOGLCD_A0              EXP1_04_PIN
    #endif

  #endif

  #if ANY(VIKI2, miniVIKI)
    #define DOGLCD_A0                EXP1_07_PIN
    #define KILL_PIN                          51
    #define STAT_LED_BLUE_PIN        EXP1_03_PIN
    #define STAT_LED_RED_PIN         EXP1_06_PIN
    #define DOGLCD_CS                EXP1_08_PIN
    #define DOGLCD_SCK               EXP2_02_PIN  // SCK_PIN   - Required for DUE Hardware SPI
    #define DOGLCD_MOSI              EXP2_06_PIN  // MOSI_PIN
    #define DOGLCD_MISO              EXP2_01_PIN  // MISO_PIN
  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

#endif // HAS_WIRED_LCD

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E4_HARDWARE_SERIAL Serial1
#endif

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HOTENDS < 3 && HAS_CUTTER && !PIN_EXISTS(SPINDLE_LASER_ENA)
  #define SPINDLE_LASER_PWM_PIN               12  // Hardware PWM
  #define SPINDLE_LASER_ENA_PIN               45  // Use E2 ENA
  #define SPINDLE_DIR_PIN                     47  // Use E2 DIR
#endif
