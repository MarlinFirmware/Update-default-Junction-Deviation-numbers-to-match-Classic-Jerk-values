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

#include "env_validate.h"

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "STM32F401CCU6 boards support one hotend / E-steppers only."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "I3DBEE BP_01"
#endif

#define TEMP_TIMER 5

//#define DISABLE_DEBUG                         // DISABLE_(DEBUG|JTAG) is not supported for STM32F4.
//#define DISABLE_JTAG
//#define ALLOW_STM32F4
//#define BOARD_NO_NATIVE_USB

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
  #define FLASH_EEPROM_LEVELING
  #define FLASH_SECTOR          (FLASH_SECTOR_TOTAL - 1)
  #define FLASH_UNIT_SIZE               0x4000U  // 16K
  #define MARLIN_EEPROM_SIZE    FLASH_UNIT_SIZE
  #undef NO_EEPROM_SELECTED
#endif

/*
#if NO_EEPROM_SELECTED
  #define IIC_BL24CXX_EEPROM                      // EEPROM on I2C-0
  #define SDCARD_EEPROM_EMULATION
#endif
*/

//
// Limit Switches
//
#define X_STOP_PIN                          PB9
#define Y_STOP_PIN                          PB3
#define Z_STOP_PIN                          PA9

//
// Steppers
//
#define X_STEP_PIN                          PB7
#define X_DIR_PIN                           PB6
#define X_ENABLE_PIN                        PB8

#define Y_STEP_PIN                          PB5
#define Y_DIR_PIN                           PB4
#define Y_ENABLE_PIN                X_ENABLE_PIN

#define Z_STEP_PIN                          PA15
#define Z_DIR_PIN                           PA10
#define Z_ENABLE_PIN                X_ENABLE_PIN

#define E0_STEP_PIN                         PA8
#define E0_DIR_PIN                          PB15
#define E0_ENABLE_PIN               X_ENABLE_PIN

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PB1   // Analog Input (HOTEND thermistor)
#define TEMP_BED_PIN                        PB0   // Analog Input (BED thermistor)

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA2   // HOTEND MOSFET
#define HEATER_BED_PIN                      PA0   // BED MOSFET

#define FAN0_PIN                            PA1   // FAN1 header on board - PRINT FAN

//
// SD Card
//
#define SD_SS_PIN                           PA4
#define SD_DETECT_PIN                       -1
#define SD_SCK_PIN                          PA5
#define SD_MISO_PIN                         PA6
#define SD_MOSI_PIN                         PA7

/**
 *                ------                                    ------
 * (BEEPER) PB10 | 1  2 | PC13 (BTN_ENC)    (MISO)    PA6  | 1  2 | PA5 (SCK)
 * (LCD_EN) PB13 | 3  4 | PB12 (LCD_RS)     (BTN_EN1) PC15 | 3  4 | PA4 (SD_SS)
 * (LCD_D4) PB14   5  6 | PA3  (LCD_D5)     (BTN_EN2) PC14   5  6 | PA7 (MOSI)
 * (LCD_D6)    - | 7  8 | -    (LCD_D7)     (SD_DET)     - | 7  8 | RESET
 *           GND | 9 10 | 5V                           GND | 9 10 | -
 *                ------                                    ------
 *                 EXP1                                      EXP2
 */
#define EXP1_01_PIN                         PB10  // BEEPER
#define EXP1_02_PIN                         PC13  // ENC
#define EXP1_03_PIN                         PB13  // EN
#define EXP1_04_PIN                         PB12  // RS
#define EXP1_05_PIN                         PB14  // D4
#define EXP1_06_PIN                         PA3   // D5
#define EXP1_07_PIN                         -1
#define EXP1_08_PIN                         -1

#define EXP2_01_PIN                         PA6   // MISO
#define EXP2_02_PIN                         PA5   // SCK
#define EXP2_03_PIN                         PC15  // EN1
#define EXP2_04_PIN                         PA4   // SS
#define EXP2_05_PIN                         PC14  // EN2
#define EXP2_06_PIN                         PA7   // MOSI
#define EXP2_07_PIN                         -1
#define EXP2_08_PIN                         -1    // RESET
#define EXP2_10_PIN                         -1

//
// LCD / Controller
//

#if HAS_WIRED_LCD
  #define BEEPER_PIN                 EXP1_01_PIN

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #define LCD_PINS_RS                EXP1_04_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_D4                EXP1_05_PIN
  #define LCD_PINS_D5                EXP1_06_PIN

  #if ENABLED(FYSETC_MINI_12864)
    #define DOGLCD_CS                EXP1_03_PIN
    #define DOGLCD_A0                EXP1_04_PIN
    //#define LCD_BACKLIGHT_PIN             -1
    #define LCD_RESET_PIN            EXP1_05_PIN  // Must be high or open for LCD to operate normally.
    #if ENABLED(FYSETC_MINI_12864_2_1)
      #define NEOPIXEL_PIN           EXP1_06_PIN
    #endif
  #endif

  //
  // GLCD features
  //
  //#define LCD_CONTRAST_INIT                190

  //
  // Dcreen orientation
  //
  //#define LCD_SCREEN_ROT_90
  //#define LCD_SCREEN_ROT_180
  //#define LCD_SCREEN_ROT_270
#endif
