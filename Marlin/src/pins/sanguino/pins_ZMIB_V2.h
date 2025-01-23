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

#define ALLOW_MEGA644P
#include "env_validate.h"

#define BOARD_INFO_NAME   "Zonestar ZMIB_V2"
#define BOARD_WEBSITE_URL "www.aliexpress.com/item/32957490744.html"

#define IS_ZMIB_V2

/**
 * ZMIB pin assignments
 * Schematic: https://github.com/ZONESTAR3D/Control-Board/blob/main/8bit/ZMIB/ZMIB%20V2/ZMIB_V2_Schmatic.pdf
 *
 * The ZMIB board needs a bootloader installed before Marlin can be uploaded.
 * If you don't have a chip programmer you can use a spare Arduino plus a few
 * electronic components to write the bootloader.
 *
 * See https://www.instructables.com/Burn-Arduino-Bootloader-with-Arduino-MEGA/
 */

/**
 * PIN:   0   Port: B0   HEATER_0_PIN
 * PIN:   1   Port: B1   HEATER_BED_PIN
 * PIN:   2   Port: B2   EXP1_4(BTN_EN2)
 * PIN:   3   Port: B3   V1: SD_DETECT_PIN
 * PIN:   3   Port: B3   V2: EXP1_6
 * PIN:   4   Port: B4   SDSS
 * PIN:   4   Port: B4   V1: EXP1_6
 * PIN:   5   Port: B5   AVR_MOSI_PIN
 * .                     SD_MOSI_PIN
 * PIN:   6   Port: B6   AVR_MISO_PIN
 * .                     EXP1_9(SD_MISO_PIN)
 * PIN:   7   Port: B7   AVR_SCK_PIN
 * .                     EXP1_10(SD_SCK_PIN)
 * PIN:   8   Port: D0   RXD
 * PIN:   9   Port: D1   TXD
 * PIN:  10   Port: D2   EXP1_8
 * PIN:  11   Port: D3   EXP1_7
 * PIN:  12   Port: D4   EXP1_5(BTN_EN1)
 * PIN:  13   Port: D5   Z_MIN_PIN
 * PIN:  14   Port: D6   E1_DIR_PIN
 * PIN:  15   Port: D7   E1_STEP_PIN
 * PIN:  16   Port: C0   Z_DIR_PIN
 * PIN:  17   Port: C1   Z_STEP_PIN
 * PIN:  18   Port: C2   Y_MIN_PIN
 * PIN:  19   Port: C3   Y_DIR_PIN
 * PIN:  20   Port: C4   Y_STEP_PIN
 * PIN:  21   Port: C5   X_MIN_PIN
 * PIN:  22   Port: C6   X_DIR_PIN
 * PIN:  23   Port: C7   X_STEP_PIN
 * PIN:  24   Port: A7   X_ENABLE_PIN
 *                       Y_ENABLE_PIN
 *                       Z_ENABLE_PIN
 *                       E0_ENABLE_PIN
 *                       E1_ENABLE_PIN
 * PIN:  25   Port: A6   FIL_RUNOUT_PIN
 * PIN:  26   Port: A5   E0_DIR_PIN
 * PIN:  27   Port: A4   E0_STEP_PIN
 * PIN:  28   Port: A3   FAN0_PIN
 * PIN:  29   Port: A2   EXP1_3(BTN_ENC)
 *                       ADC_KEY_PIN
 * PIN:  30   Port: A1   TEMP_0_PIN
 * PIN:  31   Port: A0   TEMP_BED_PIN
 */

//
// Limit Switches
//
#define X_MIN_PIN                             21
#define Y_MIN_PIN                             18

#if ANY(Z6S_ZFAULT, Z6BS_ZFAULT)
  #define Z_MIN_PIN                           25
#else
  #define Z_MIN_PIN                           13
#endif

//
// Steppers
//
#define X_STEP_PIN                            23
#define X_DIR_PIN                             22
#define X_ENABLE_PIN                          24

#define Y_STEP_PIN                            20
#define Y_DIR_PIN                             19
#define Y_ENABLE_PIN                          24

#if ANY(Z6S_ZFAULT, Z6BS_ZFAULT)
  #define Z_STEP_PIN                          27
  #define Z_DIR_PIN                           26
#else
  #define Z_STEP_PIN                          17
  #define Z_DIR_PIN                           16
#endif

#define Z_ENABLE_PIN                          24

#if ANY(Z6S_ZFAULT, Z6BS_ZFAULT)
  #define E0_STEP_PIN                         15
  #define E0_DIR_PIN                          14
#else
  #define E0_STEP_PIN                         27
  #define E0_DIR_PIN                          26
#endif

#define E0_ENABLE_PIN                         24

#define E1_STEP_PIN                           15
#define E1_DIR_PIN                            14
#define E1_ENABLE_PIN                         24

//
// Temperature Sensors
//
#define TEMP_0_PIN                             1  // Analog Input
#define TEMP_BED_PIN                           0  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           0
#define HEATER_BED_PIN                         1
#define FAN0_PIN                              28
#define FAN1_PIN                              -1

//
// Filament Runout Sensor
//
#if ANY(Z6S_ZFAULT, Z6BS_ZFAULT)
  #define FIL_RUNOUT_PIN                      13
#else
  #define FIL_RUNOUT_PIN                      25  // Z-MIN
#endif

//
// SD card
//
#if HAS_MEDIA
  #define SD_SS_PIN                            4
#endif
#define SD_DETECT_PIN                         -1

/**             EXP1
 *             ------
 * (MOSI) D5  | 1  2 | D7       (SCK)
 * (CS)   D11 | 3  4 | D10      (DC/D4)
 * (EN2)  D12   5  6 | D4 or D3 (EN/RS)
 * (ENC)  D29 | 7  8 | D2       (EN1)
 *        GND | 9 10 | 5V
 *             ------
 */
#define EXP1_01_PIN                            5
#define EXP1_02_PIN                            7
#define EXP1_03_PIN                           11
#define EXP1_04_PIN                           10
#define EXP1_05_PIN                           12
#ifndef IS_ZMIB_V2
  #define EXP1_06_PIN                          4  // ZMIB V1
#else
  #define EXP1_06_PIN                          3  // ZMIB V2
#endif
#define EXP1_07_PIN                           29
#define EXP1_08_PIN                            2

#if ENABLED(ZONESTAR_12864LCD)
  //
  // LCD 128x64
  //
  #define LCDSCREEN_NAME "ZONESTAR_12864LCD"
  #define FORCE_SOFT_SPI
  //#define LCD_SDSS_PIN             EXP1_03_PIN
  #define LCD_PINS_RS                EXP1_03_PIN  // ST7920 CS  (LCD-4)
  #define LCD_PINS_EN                EXP1_06_PIN  // ST7920 DAT (LCD-5)
  #define LCD_PINS_D4                EXP1_04_PIN  // ST7920 CLK (LCD-6)

  #define BOARD_ST7920_DELAY_1       DELAY_2_NOP
  #define BOARD_ST7920_DELAY_2       DELAY_2_NOP
  #define BOARD_ST7920_DELAY_3       DELAY_2_NOP

#elif ANY(ZONESTAR_12864OLED, ZONESTAR_12864OLED_SSD1306)
  //
  // OLED 128x64
  //
  #define LCDSCREEN_NAME "ZONESTAR 12864OLED"
  #define FORCE_SOFT_SPI
  #define LCD_PINS_RS                EXP1_06_PIN
  #define LCD_PINS_DC                EXP1_04_PIN
  #define DOGLCD_CS                  EXP1_03_PIN

  #if ENABLED(OLED_HW_IIC)
    #error "Oops! can't choose HW IIC for ZMIB board!!"
  #else
    #define DOGLCD_A0                LCD_PINS_DC
    #if DISABLED(OLED_HW_SPI)
      #define DOGLCD_MOSI           AVR_MOSI_PIN  // Software SPI
      #define DOGLCD_SCK             AVR_SCK_PIN
    #endif
  #endif
#endif

//
// All the above are also RRDSC with rotary encoder
//
#if IS_RRD_SC
  #define BTN_EN1                    EXP1_08_PIN
  #define BTN_EN2                    EXP1_05_PIN
  #define BTN_ENC                    EXP1_07_PIN
  #define BEEPER_PIN                          -1
  #define KILL_PIN                            -1
#endif
