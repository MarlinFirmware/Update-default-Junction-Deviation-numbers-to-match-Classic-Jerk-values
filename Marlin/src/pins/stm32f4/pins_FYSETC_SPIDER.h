/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "FYSETC SPIDER"
#endif
#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME BOARD_INFO_NAME
#endif

//
// EEPROM Emulation
//
#if NO_EEPROM_SELECTED
  #undef NO_EEPROM_SELECTED
  //#define FLASH_EEPROM_EMULATION
  //#define SRAM_EEPROM_EMULATION
  #define I2C_EEPROM
#endif

#if ENABLED(I2C_EEPROM)
  #define MARLIN_EEPROM_SIZE              0x1000  // 4K
#endif

//
// Steppers
//
#define X2_STEP_PIN                         PD12
#define X2_DIR_PIN                          PC4
#define X2_ENABLE_PIN                       PE8
#define X2_CS_PIN                           PA15

#define Z2_STEP_PIN                         PE1
#define Z2_DIR_PIN                          PE0
#define Z2_ENABLE_PIN                       PC5
#define Z2_CS_PIN                           PD11

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB15
#define HEATER_1_PIN                        PC8
#define HEATER_2_PIN                        PB3
#define HEATER_BED_PIN                      PB4

//
// Steppers
//
#define X_ENABLE_PIN                        PE9

//
// Servos
// Z_MAX_PIN only works in input mode
//
#define SERVO0_PIN                          PA2

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PE7
  #define X2_SERIAL_TX_PIN                  PA15
  #define Y_SERIAL_TX_PIN                   PE15
  #define Z_SERIAL_TX_PIN                   PD10
  #define Z2_SERIAL_TX_PIN                  PD11
  #define E0_SERIAL_TX_PIN                  PD7
  #define E1_SERIAL_TX_PIN                  PC14
  #define E2_SERIAL_TX_PIN                  PC15
#endif

//
// SPI pins for TMC2130 stepper drivers
//
#define TMC_USE_SW_SPI
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PE14
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PE13
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PE12
#endif

#if HOTENDS > 3 || E_STEPPERS > 3
  #error "FYSETC SPIDER supports up to 3 hotends / E steppers."
#else
  #include "pins_FYSETC_S6.h"
#endif
