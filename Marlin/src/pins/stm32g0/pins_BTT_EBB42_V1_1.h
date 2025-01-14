/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/** CAUTION **
 * This board definition is to facilitate support for a Filament Extrusion
 * devices, used to convert waste plastic into 3D printable filament.
 * This board is NOT a general 3D printing controller; it is NOT supported
 * as a toolboard via CANBUS (as it was originally designed) or any device
 * that requires kinematics.
 */

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "BTT EBB42 V1.1"
#endif
#define BOARD_WEBSITE_URL "github.com/bigtreetech/EBB/tree/master/EBB%20CAN%20V1.1%20(STM32G0B1)/EBB42%20CAN%20V1.1"

//
// EEPROM
//
#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #undef NO_EEPROM_SELECTED
  #ifndef FLASH_EEPROM_EMULATION
    #define FLASH_EEPROM_EMULATION
  #endif
  #define EEPROM_PAGE_SIZE      (0x800UL) // 2K
  #define EEPROM_START_ADDRESS  (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 1UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE
#endif

//#define USES_DIAG_JUMPERS

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

#define LED_PIN                             PA13

#define I2C_SDA_PIN                         PB4
#define I2C_SCL_PIN                         PB3

//
// Servos
//
#define SERVO0_PIN                          PB9   // SERVOS

//
// Limit Switches
//
#if !HAS_WIRED_LCD
  #define X_STOP_PIN                        PB6
  #define Y_STOP_PIN                        PB5
  #define Z_STOP_PIN                        PB7
#endif

//
// Z Probe must be this pin
//
#define Z_MIN_PROBE_PIN                     PB8   // PROBE

//
// Steppers
//

#define E0_ENABLE_PIN                       PD2
#define E0_STEP_PIN                         PD0
#define E0_DIR_PIN                          PD1

#if HAS_TMC_UART
  /**
   * TMC220x stepper drivers
   * Hardware serial communication ports
   */
  //#define E0_HARDWARE_SERIAL MSerial4

  // This is the stable default value after testing, but, higher UART rates could be configured, remeber to test the Steppers with the M122 command to check if everything works.
  //#define TMC_BAUD_RATE 250000

  #define E0_SERIAL_TX_PIN                  PA15

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

  // Default TMC slave addresses
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                0
  #endif
  static_assert(E0_SLAVE_ADDRESS == 0, "E0_SLAVE_ADDRESS must be 0 for BOARD_BTT_EBB42_V1_1.");
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA3   // Analog Input "TH0"

// SPI for MAX Thermocouple
// Uses a separate SPI bus

#define TEMP_0_CS_PIN                       PA4   // GTR K-TEMP
#define TEMP_0_SCK_PIN                      PA5   // SCK
#define TEMP_0_MISO_PIN                     PA6   // MISO
#define TEMP_0_MOSI_PIN                     PA7   // For MAX31865

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA2   // "HE" V1.1
#define HEATER_1_PIN                        PB13  // "HE" V1.2
#define FAN0_PIN                            PA0   // "FAN0"
#define FAN1_PIN                            PA1   // "FAN1"

//
// Default NEOPIXEL_PIN
//
#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN                PD3   // LED driving pin
#endif

//
// LCD / Controller
//

#if HAS_WIRED_LCD
  #define BTN_EN1                           PB7
  #define BTN_EN2                           PB5
  #define BTN_ENC                           PB6
#endif
