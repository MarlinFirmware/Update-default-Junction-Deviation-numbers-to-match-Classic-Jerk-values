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

#define BOARD_INFO_NAME   "FLY_MINI"
#define BOARD_WEBSITE_URL "github.com/FLYmaker/FLY-MINI"
#define DISABLE_JTAG

//
// Flash EEPROM Emulation
//
#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     0x800 // 2K
  #define EEPROM_START_ADDRESS (0x8000000 + 256 * 1024 - 2 * EEPROM_PAGE_SIZE) // 256K firmware space
  #define MARLIN_EEPROM_SIZE   EEPROM_PAGE_SIZE
#endif

//
// Servos
//
#define SERVO0_PIN                          PA8

//
// Limit Switches
//
#define X_MIN_PIN                           PC12
#define X_MAX_PIN                           PC11
#define Y_MIN_PIN                           PC10
#define Y_MAX_PIN                           PA15
#define Z_MIN_PIN                           PA14
#define Z_MAX_PIN                           PA13

//
// Steppers
//
#define X_STEP_PIN                          PB1
#define X_DIR_PIN                           PB2
#define X_ENABLE_PIN                        PB10
#ifndef X_CS_PIN
  #define X_CS_PIN                          PB0
#endif

#define Y_STEP_PIN                          PA2
#define Y_DIR_PIN                           PC4
#define Y_ENABLE_PIN                        PC5
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PA7
#endif

#define Z_STEP_PIN                          PA3
#define Z_DIR_PIN                           PA5
#define Z_ENABLE_PIN                        PA6
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PA4
#endif

#define E0_STEP_PIN                         PA1
#define E0_DIR_PIN                          PC3
#define E0_ENABLE_PIN                       PA0
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PC2
#endif

#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI               EXP2_06_PIN
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO               EXP2_01_PIN
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                EXP2_02_PIN
#endif

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PB0
  #define Y_SERIAL_TX_PIN                   PA7
  #define Z_SERIAL_TX_PIN                   PA4
  #define E0_SERIAL_TX_PIN                  PC2
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC6
#define HEATER_BED_PIN                      PC7
#ifndef FAN0_PIN
  #define FAN0_PIN                          PC8
#endif
#define FAN1_PIN                            PC9

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PC0   // Analog Input
#define TEMP_0_PIN                          PC1   // Analog Input

/**               ------                                        ------
 * (BEEPER) PC14 | 1  2 | PC13 (BTN_ENC)      (MISO)      PB14 | 1  2 | PB13 (SD_SCK)
 * (LCD_EN) PB9  | 3  4 | PB8  (LCD_RS)       (BTN_EN1)   PB3  | 3  4 | PB12 (SD_CS2)
 * (LCD_D4) PB7    5  6 | PB6  (LCD_D5)       (BTN_EN2)   PD2    5  6 | PB15 (SD_MOSI)
 * (LCD_D6) PB5  | 7  8 | PB4  (LCD_D7)       (SD_DETECT) PB11 | 7  8 | RESET
 *           GND | 9 10 | 5V                               GND | 9 10 | --
 *                ------                                        ------
 *                 EXP1                                          EXP2
 */
#define EXP1_01_PIN                         PC14
#define EXP1_02_PIN                         PC13
#define EXP1_03_PIN                         PB9
#define EXP1_04_PIN                         PB8
#define EXP1_05_PIN                         PB7
#define EXP1_06_PIN                         PB6
#define EXP1_07_PIN                         PB5
#define EXP1_08_PIN                         PB4

#define EXP2_01_PIN                         PB14
#define EXP2_02_PIN                         PB13
#define EXP2_03_PIN                         PB3
#define EXP2_04_PIN                         PB12
#define EXP2_05_PIN                         PD2
#define EXP2_06_PIN                         PB15
#define EXP2_07_PIN                         PB11
#define EXP2_08_PIN                         -1    // RESET

//
// LCD / Controller
//

#if HAS_WIRED_LCD

  #define SPI_DEVICE                           2  // Maple
  #define SD_SS_PIN                  EXP2_04_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN

  #define SDSS                         SD_SS_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN

  #define BEEPER_PIN                 EXP1_01_PIN

  #define LCD_PINS_RS                EXP1_04_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_D4                EXP1_05_PIN
  #define LCD_PINS_D5                EXP1_06_PIN
  #define LCD_PINS_D6                EXP1_07_PIN
  #define LCD_PINS_D7                EXP1_08_PIN

  #define BTN_EN1                    EXP2_05_PIN
  #define BTN_EN2                    EXP2_03_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

  // Alter timing for graphical display
  #if IS_U8GLIB_ST7920
    #define BOARD_ST7920_DELAY_1              96
    #define BOARD_ST7920_DELAY_2              48
    #define BOARD_ST7920_DELAY_3             715
  #endif

#endif // HAS_WIRED_LCD
