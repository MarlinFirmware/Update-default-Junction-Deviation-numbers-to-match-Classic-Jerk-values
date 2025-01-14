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
 * Arduino Mega? or Due with RuRAMPS4DUE pin assignments
 * Ported by sys0724 & Vynt
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "RuRAMPS4Due v1.1"

//
// Servos
//
#define SERVO0_PIN                             5
#define SERVO1_PIN                             3

//
// Limit Switches
//
#define X_MIN_PIN                             45
#define X_MAX_PIN                             39
#define Y_MIN_PIN                             46
#define Y_MAX_PIN                             41
#define Z_MIN_PIN                             47
#define Z_MAX_PIN                             43

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     43
#endif

//
// Steppers
//
#define X_STEP_PIN                            37  // Support Extension Board
#define X_DIR_PIN                             36
#define X_ENABLE_PIN                          38
#ifndef X_CS_PIN
  #define X_CS_PIN                            -1
#endif

#define Y_STEP_PIN                            32  // Support Extension Board
#define Y_DIR_PIN                             35
#define Y_ENABLE_PIN                          34
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            -1
#endif

#define Z_STEP_PIN                            30  // Support Extension Board
#define Z_DIR_PIN                              2
#define Z_ENABLE_PIN                          33
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            -1
#endif

#define E0_STEP_PIN                           29
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         31
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           -1
#endif

#define E1_STEP_PIN                           22
#define E1_DIR_PIN                            24
#define E1_ENABLE_PIN                         26
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           -1
#endif

#define E2_STEP_PIN                           25
#define E2_DIR_PIN                            23
#define E2_ENABLE_PIN                         27
#ifndef E2_CS_PIN
  #define E2_CS_PIN                           -1
#endif

#define E3_STEP_PIN                           15  // Only For Extension Board
#define E3_DIR_PIN                            14
#define E3_ENABLE_PIN                         61
#ifndef E3_CS_PIN
  #define E3_CS_PIN                           -1
#endif

// For Future: Microstepping pins - Mapping not from fastio.h (?)
//#define E3_MS1_PIN         ?
//#define E3_MS2_PIN         ?
//#define E3_MS3_PIN         ?

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN               Y_MIN_PIN
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                          13
#define HEATER_1_PIN                          12
#define HEATER_2_PIN                          11
#define HEATER_BED_PIN                         7  // BED H1

#ifndef FAN0_PIN
  #define FAN0_PIN                             9
#endif
#define FAN1_PIN                               8
#define CONTROLLER_FAN_PIN                    -1

//
// Temperature Sensors
//
#define TEMP_0_PIN                             0  // ANALOG A0
#define TEMP_1_PIN                             1  // ANALOG A1
#define TEMP_2_PIN                             2  // ANALOG A2
#define TEMP_3_PIN                             3  // ANALOG A3
#define TEMP_BED_PIN                           4  // ANALOG A4

// The thermocouple uses Analog pins
#if ENABLED(VER_WITH_THERMOCOUPLE)                // Defined in Configuration.h
  #define TEMP_4_PIN                           5  // A5
  #define TEMP_5_PIN                           6  // A6 (Marlin 2.0 not support)
#endif

// SPI for MAX Thermocouple
/*
#if !HAS_MEDIA
  #define TEMP_0_CS_PIN              EXP1_08_PIN
#else
  #define TEMP_0_CS_PIN                       49
#endif
*/

//
// Misc. Functions
//
#define SDSS                                   4  // 4,10,52 if using HW SPI.
#define LED_PIN                               -1  // 13 - HEATER_0_PIN
#define PS_ON_PIN                             -1  // 65

// MKS TFT / Nextion Use internal USART-1
#define TFT_LCD_MODULE_COM                     1
#define TFT_LCD_MODULE_BAUDRATE              115600

#if ENABLED(WIFISUPPORT)
  // ESP WiFi Use internal USART-2
  #define ESP_WIFI_MODULE_COM                  2
  #define ESP_WIFI_MODULE_BAUDRATE           115600
  #define ESP_WIFI_MODULE_RESET_PIN           -1
  #define PIGGY_GPIO_PIN                      -1
#endif

//
// EEPROM
//
#define MARLIN_EEPROM_SIZE                0x8000  // 32K (24lc256)
#define I2C_EEPROM                                // EEPROM on I2C-0
//#define EEPROM_SD                               // EEPROM on SDCARD
//#define SPI_EEPROM                              // EEPROM on SPI-0
//#define SPI_CHAN_EEPROM1        ?
//#define SPI_EEPROM1_CS_PIN      ?
// 2K EEPROM
//#define SPI_EEPROM2_CS_PIN      ?
// 32Mb FLASH
//#define SPI_FLASH_CS_PIN        ?

/**
 *              ------                                ------
 * (BEEPER) 62 | 1  2 | 40 (BTN_ENC)  (MISO)      74 | 1  2 | 76 (SCK)
 * (LCD_EN) 64 | 3  4 | 63 (LCD_RS)   (BTN_EN1)   44 | 3  4 | 10 (SD_SS)
 * (LCD_D4) 48 | 5  6   50 (LCD_D5)   (BTN_EN2)   42 | 5  6   75 (MOSI)
 * (LCD_D6) 52 | 7  8 | 53 (LCD_D7)   (SD_DETECT) 51 | 7  8 | RESET
 *         GND | 9 10 | 5V                       GND | 9 10 | --
 *              ------                                ------
 *               EXP1                                  EXP2
 */
#define EXP1_01_PIN                           62  // BEEPER
#define EXP1_02_PIN                           40  // ENC
#define EXP1_03_PIN                           64  // LCD_EN
#define EXP1_04_PIN                           63  // LCD_RS
#define EXP1_05_PIN                           48  // LCD_D4 / RESET
#define EXP1_06_PIN                           50  // LCD_D5
#define EXP1_07_PIN                           52  // LCD_D6
#define EXP1_08_PIN                           53  // LCD_D7 / ENABLE

#define EXP2_01_PIN                           74  // MISO
#define EXP2_02_PIN                           76  // SCK
#define EXP2_03_PIN                           44  // EN1
#define EXP2_04_PIN                           10  // SDSS
#define EXP2_05_PIN                           42  // EN2
#define EXP2_06_PIN                           75  // MOSI
#define EXP2_07_PIN                           51  // SD DET
#define EXP2_08_PIN                           -1  // RESET

//
// LCD / Controller
//

#if HAS_WIRED_LCD

  #if ANY(RADDS_DISPLAY, IS_RRD_SC, IS_RRD_FG_SC)
    #define BEEPER_PIN               EXP1_01_PIN
    #define LCD_PINS_D4              EXP1_05_PIN
    #define LCD_PINS_D5              EXP1_06_PIN
    #define LCD_PINS_D6              EXP1_07_PIN
    #define LCD_PINS_D7              EXP1_08_PIN
    #define SD_DETECT_PIN            EXP2_07_PIN
  #endif

  #if ANY(RADDS_DISPLAY, IS_RRD_SC)

    #define LCD_PINS_RS              EXP1_04_PIN
    #define LCD_PINS_EN              EXP1_03_PIN

  #elif IS_RRD_FG_SC

    #define LCD_PINS_RS              EXP1_07_PIN
    #define LCD_PINS_EN              EXP1_08_PIN

  #elif HAS_U8GLIB_I2C_OLED

    #define BEEPER_PIN               EXP1_01_PIN
    #define LCD_SDSS                 EXP2_04_PIN
    #define SD_DETECT_PIN            EXP2_07_PIN

  #elif ENABLED(FYSETC_MINI_12864)

    #define BEEPER_PIN               EXP1_01_PIN
    #define DOGLCD_CS                EXP1_03_PIN
    #define DOGLCD_A0                EXP1_04_PIN

    //#define FORCE_SOFT_SPI                      // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

    #define LCD_RESET_PIN            EXP1_05_PIN  // Must be high or open for LCD to operate normally.

    #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
      #ifndef RGB_LED_R_PIN
        #define RGB_LED_R_PIN        EXP1_06_PIN  // D5
      #endif
      #ifndef RGB_LED_G_PIN
        #define RGB_LED_G_PIN        EXP1_07_PIN  // D6
      #endif
      #ifndef RGB_LED_B_PIN
        #define RGB_LED_B_PIN        EXP1_08_PIN  // D7
      #endif
    #elif ENABLED(FYSETC_MINI_12864_2_1)
      #define NEOPIXEL_PIN           EXP1_06_PIN  // D5
    #endif

  #elif ENABLED(SPARK_FULL_GRAPHICS)

    // https://sparklab-shop.de/elektronik/40/sparklcd-adapter
    #error "Oops! SPARK_FULL_GRAPHICS not supported with RURAMPS4D."
    //#define LCD_PINS_D4                     29  //?
    //#define LCD_PINS_EN                     27  //?
    //#define LCD_PINS_RS                     25  //?
    //#define BTN_EN1                         35  //?
    //#define BTN_EN2                         33  //?
    //#define BTN_ENC                         37  //?

  #endif // SPARK_FULL_GRAPHICS

  #if IS_NEWPANEL
    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN
    #define BTN_ENC                  EXP1_02_PIN
  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

#endif // HAS_WIRED_LCD
