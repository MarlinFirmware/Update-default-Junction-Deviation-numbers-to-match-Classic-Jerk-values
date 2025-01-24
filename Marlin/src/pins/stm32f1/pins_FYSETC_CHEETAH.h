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

#include "env_validate.h"

#define BOARD_INFO_NAME   "FYSETC Cheetah"
#define BOARD_WEBSITE_URL "github.com/FYSETC/FYSETC-Cheetah"

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

#define BOARD_NO_NATIVE_USB
#define RESET_STEPPERS_ON_MEDIA_INSERT
#define DISABLE_JTAG

#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

//
// Servos
//
#define SERVO0_PIN                          PA0

//
// Limit Switches
//
#define X_STOP_PIN                          PA1
#define Y_STOP_PIN                          PB4
#define Z_STOP_PIN                          PA15

//
// Filament runout
//
#define FIL_RUNOUT_PIN                      PB5

//
// Steppers
//
#define X_STEP_PIN                          PB8
#define X_DIR_PIN                           PB9
#define X_ENABLE_PIN                        PA8

#define Y_STEP_PIN                          PB2
#define Y_DIR_PIN                           PB3
#define Y_ENABLE_PIN                        PB1

#define Z_STEP_PIN                          PC0
#define Z_DIR_PIN                           PC1
#define Z_ENABLE_PIN                        PC2

#define E0_STEP_PIN                         PC15
#define E0_DIR_PIN                          PC14
#define E0_ENABLE_PIN                       PC13

#if HAS_TMC_UART
  #define X_HARDWARE_SERIAL  MSerial2
  #define Y_HARDWARE_SERIAL  MSerial2
  #define Z_HARDWARE_SERIAL  MSerial2
  #define E0_HARDWARE_SERIAL MSerial2

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                    0
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                    1
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                    2
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                   3
  #endif
  static_assert(X_SLAVE_ADDRESS == 0, "X_SLAVE_ADDRESS must be 0 for BOARD_FYSETC_CHEETAH.");
  static_assert(Y_SLAVE_ADDRESS == 1, "Y_SLAVE_ADDRESS must be 1 for BOARD_FYSETC_CHEETAH.");
  static_assert(Z_SLAVE_ADDRESS == 2, "Z_SLAVE_ADDRESS must be 2 for BOARD_FYSETC_CHEETAH.");
  static_assert(E0_SLAVE_ADDRESS == 3, "E0_SLAVE_ADDRESS must be 3 for BOARD_FYSETC_CHEETAH.");
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC6
#define HEATER_BED_PIN                      PC7
#ifndef FAN0_PIN
  #define FAN0_PIN                          PC8
#endif

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PC5   // Analog Input
#define TEMP_0_PIN                          PC4   // Analog Input

//
// Misc. Functions
//
#define SD_SS_PIN                           PA4
#define SD_DETECT_PIN                       PC3

#ifndef RGB_LED_R_PIN
  #define RGB_LED_R_PIN                     PB0
#endif
#ifndef RGB_LED_G_PIN
  #define RGB_LED_G_PIN                     PB7
#endif
#ifndef RGB_LED_B_PIN
  #define RGB_LED_B_PIN                     PB6
#endif

/**
 * EXP1 pinout for the LCD according to FYSETC's Cheetah board schematic
 *                 ------
 * (BEEPER)  PC9  | 1  2 | PC12 (BTN_ENC)
 * (BTN_EN2) PC11 | 3  4 | PB14 (LCD_RS / MISO)
 * (BTN_EN1) PC10   5  6 | PB13 (SCK)
 * (LCD_EN)  PB12 | 7  8 | PB15 (MOSI)
 *            GND | 9 10 | 5V
 *                 ------
 *                  EXP1
 *
 * Notes:
 *  - The pin-numbers match the connector correctly and are not in reverse order like on the Ender-3 board.
 *  - Functionally the pins are assigned in the same order as on the Ender-3 board.
 *  - Pin 4 on the Cheetah board is assigned to an I/O, it is assigned to RESET on the Ender-3 board.
 */
#define EXP1_01_PIN                         PC9
#define EXP1_02_PIN                         PC12
#define EXP1_03_PIN                         PC11
#define EXP1_04_PIN                         PB14
#define EXP1_05_PIN                         PC10
#define EXP1_06_PIN                         PB13
#define EXP1_07_PIN                         PB12
#define EXP1_08_PIN                         PB15

#if HAS_WIRED_LCD
  #define BEEPER_PIN                 EXP1_01_PIN

  #if HAS_MARLINUI_U8GLIB
    #define DOGLCD_A0                EXP1_04_PIN
    #define DOGLCD_CS                EXP1_07_PIN
    #define DOGLCD_SCK               EXP1_06_PIN
    #define DOGLCD_MOSI              EXP1_08_PIN

    #if ANY(FYSETC_MINI_12864, U8GLIB_ST7920)
      #define FORCE_SOFT_SPI
    #endif
    //#define LCD_SCREEN_ROTATE              180  // 0, 90, 180, 270
  #endif

  #define LCD_PINS_RS                EXP1_07_PIN  // CS -- SOFT SPI for ENDER3 LCD
  #define LCD_PINS_D4                EXP1_06_PIN  // SCLK
  #define LCD_PINS_EN                EXP1_08_PIN  // DATA MOSI

  //#define LCD_CONTRAST_INIT                190

  #if IS_NEWPANEL
    #define BTN_EN1                  EXP1_05_PIN
    #define BTN_EN2                  EXP1_03_PIN
    #define BTN_ENC                  EXP1_02_PIN
  #endif
#endif

#if ENABLED(TOUCH_UI_FTDI_EVE)
  #define BEEPER_PIN                 EXP1_01_PIN
  #define CLCD_MOD_RESET             EXP1_03_PIN
  #define CLCD_SPI_CS                EXP1_07_PIN

  //#define CLCD_USE_SOFT_SPI                     // the Cheetah can use hardware-SPI so we do not really need this

  #if ENABLED(CLCD_USE_SOFT_SPI)
    #define CLCD_SOFT_SPI_MOSI       EXP1_08_PIN
    #define CLCD_SOFT_SPI_MISO       EXP1_04_PIN
    #define CLCD_SOFT_SPI_SCLK       EXP1_06_PIN
  #else
    #define CLCD_SPI_BUS                       2
  #endif
#endif
