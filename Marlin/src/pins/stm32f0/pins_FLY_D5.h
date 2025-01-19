/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

// #if NOT_TARGET(__STM32F1__, STM32F1xx, STM32F0xx)
//   #error "Oops! Select a 'Malyan M300' board in 'Tools > Board.'"
// #endif

#define BOARD_INFO_NAME "FLY_D5"

//
// EEPROM Emulation
//
#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
  #ifndef MARLIN_EEPROM_SIZE
    #define MARLIN_EEPROM_SIZE 0x800U             // 2K
  #endif
#endif

//
// Timers
//
#define STEP_TIMER  6
#define TEMP_TIMER  7

//
// Limit Switches
//
#define X_STOP_PIN                          PB4
#define Y_STOP_PIN                          PB3
#define Z_STOP_PIN                          PD2
//
// Steppers
//
#define X_STEP_PIN                          PC15
#define X_DIR_PIN                           PC14
#define X_ENABLE_PIN                        PC2
#ifndef X_CS_PIN
  #define X_CS_PIN                          PC13
#endif

#define Y_STEP_PIN                          PA1
#define Y_DIR_PIN                           PA0
#define Y_ENABLE_PIN                        PA2
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PC3
#endif

#define Z_STEP_PIN                          PA5
#define Z_DIR_PIN                           PA4
#define Z_ENABLE_PIN                        PA6
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PA3
#endif

#define E0_STEP_PIN                         PC5
#define E0_DIR_PIN                          PC4
#define E0_ENABLE_PIN                       PB0
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PA7
#endif

#define E1_STEP_PIN                         PB10
#define E1_DIR_PIN                          PB2
#define E1_ENABLE_PIN                       PB11
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PB1
#endif
//
// Trinamic SPI
//
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                EXP2_02_PIN
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO               EXP2_01_PIN
#endif
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI               EXP2_06_PIN
#endif

//
// Trinamic Software Serial
//

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PC13
  #define Y_SERIAL_TX_PIN                   PC3
  #define Z_SERIAL_TX_PIN                   PA3
  #define E0_SERIAL_TX_PIN                  PA7
  #define E1_SERIAL_TX_PIN                  PB1
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // Analog Input (HOTEND0 thermistor)
#define TEMP_BED_PIN                        PC0   // Analog Input (BED thermistor)

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC6   // HOTEND0 MOSFET
#define HEATER_BED_PIN                      PC7   // BED MOSFET

#ifndef FAN0_PIN
  #define FAN0_PIN                          PC8
#endif
#define FAN1_PIN                            PC9

/**
 *        ------                ------
 *  NC   | 1  2 | PA15    PB14 | 1  2 | PB13
 *  PC11 | 3  4 | PA14    PC12 | 3  4 | PB12
 *  PC10 | 5  6 | PA13    PB6  | 5  6 | PB15
 *  NC   | 7  8 | NC      PB7  | 7  8 | RESET
 *   GND | 9 10 | 5V       GND | 9 10 | --
 *        ------                ------
 *         EXP1                  EXP2
 */
#define EXP1_01_PIN                         -1
#define EXP1_02_PIN                         PA15
#define EXP1_03_PIN                         PC11
#define EXP1_04_PIN                         PA14
#define EXP1_05_PIN                         PC10
#define EXP1_06_PIN                         PA13
#define EXP1_07_PIN                         -1
#define EXP1_08_PIN                         -1

#define EXP2_01_PIN                         PB14
#define EXP2_02_PIN                         PB13
#define EXP2_03_PIN                         PC12
#define EXP2_04_PIN                         PB12
#define EXP2_05_PIN                         PB6
#define EXP2_06_PIN                         PB15
#define EXP2_07_PIN                         PB7
#define EXP2_08_PIN                         -1    // RESET

//
// Onboard SD support
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if ENABLED(SDSUPPORT)

  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SDSS                       EXP2_04_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN

#endif

//
// Filament runout
//

#define FIL_RUNOUT_PIN                      PA3

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1                96
  #define BOARD_ST7920_DELAY_2                48
  #define BOARD_ST7920_DELAY_3               715
#endif
