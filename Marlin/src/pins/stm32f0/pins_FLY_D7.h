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

#define BOARD_INFO_NAME "FLY_D7"

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
#define STEP_TIMER  1
#define TEMP_TIMER  3

//
// Limit Switches
//
#define X_STOP_PIN                          PB3
#define Y_STOP_PIN                          PD2
#define Z_STOP_PIN                          PA9
#define E_STOP_PIN                          PA10

//
// Steppers
//
#define X_STEP_PIN                          PC14
#define X_DIR_PIN                           PC13
#define X_ENABLE_PIN                        PC15
#ifndef X_CS_PIN
  #define X_CS_PIN                          PB7
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

#define E2_STEP_PIN                         PC12
#define E2_DIR_PIN                          PB5
#define E2_ENABLE_PIN                       PC11
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PB6
#endif

#define E3_STEP_PIN                         PA15
#define E3_DIR_PIN                          PA14
#define E3_ENABLE_PIN                       PA13
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PC10
#endif

//
// Trinamic SPI
//
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PB13
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PB14
#endif
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PB15
#endif

//
// Trinamic Software Serial
//

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PB7
  #define X_SERIAL_RX_PIN        X_SERIAL_TX_PIN

  #define Y_SERIAL_TX_PIN                   PC3
  #define Y_SERIAL_RX_PIN        Y_SERIAL_TX_PIN

  #define Z_SERIAL_TX_PIN                   PA3
  #define Z_SERIAL_RX_PIN        Z_SERIAL_TX_PIN

  #define E0_SERIAL_TX_PIN                  PA7
  #define E0_SERIAL_RX_PIN      E0_SERIAL_TX_PIN

  #define E1_SERIAL_TX_PIN                   PB1
  #define E1_SERIAL_RX_PIN      E1_SERIAL_TX_PIN

  #define E2_SERIAL_TX_PIN                   PB6
  #define E2_SERIAL_RX_PIN      E2_SERIAL_TX_PIN

  #define E3_SERIAL_TX_PIN                   PC10
  #define E3_SERIAL_RX_PIN      E3_SERIAL_TX_PIN
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // Analog Input (HOTEND0 thermistor)
#define TEMP_1_PIN                          PC2   // Analog Input (HOTEND1 thermistor)
#define TEMP_BED_PIN                        PC0   // Analog Input (BED thermistor)

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB12  // HOTEND0 MOSFET
#define HEATER_1_PIN                        PC6   // HOTEND0 MOSFET
#define HEATER_BED_PIN                      PC7   // BED MOSFET

#ifndef FAN0_PIN
  #define FAN0_PIN                          PC8
#endif
#define FAN1_PIN                            PC9
