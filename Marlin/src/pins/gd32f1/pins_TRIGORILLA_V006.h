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

#define BOARD_INFO_NAME      "TRIGORILLA_V006"
#define DEFAULT_MACHINE_NAME "GD32F103"

#define BOARD_NO_NATIVE_USB

// Release PA13 from SWD for CASE_LIGHT_PIN
#define DISABLE_DEBUG
#define DISABLE_JTAG

//
// EEPROM
//
#define FLASH_EEPROM_EMULATION
#define MARLIN_EEPROM_SIZE               0x1000U  // 4K

//
// Limit Switches
//
#define X_MIN_PIN                           PA7
#define X_MAX_PIN                           PC6
#define Y_MIN_PIN                           PC5
#define Y_MAX_PIN                           -1
#define Z_MIN_PIN                           PB2
#define Z_MAX_PIN                           -1

//
// Steppers
//
#define X_ENABLE_PIN                        PC3
#define X_STEP_PIN                          PC2
#define X_DIR_PIN                           PB9

#define Y_ENABLE_PIN                        PC13
#define Y_STEP_PIN                          PB8
#define Y_DIR_PIN                           PB7

#define Z_ENABLE_PIN                        PC14
#define Z_STEP_PIN                          PB6
#define Z_DIR_PIN                           PB5

#define E0_ENABLE_PIN                       PA15
#define E0_STEP_PIN                         PB4
#define E0_DIR_PIN                          PB3

#define E1_ENABLE_PIN                       PC15
#define E1_STEP_PIN                         PC0
#define E1_DIR_PIN                          PC1

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC4   // T0
#define TEMP_1_PIN                          -1    // T1 PA6 used for power loss
#define TEMP_BED_PIN                        PB0   // TB

//
// Heaters
//
#define HEATER_0_PIN                        PA1   // H0
#define HEATER_1_PIN                        PA8   // H1
#define HEATER_BED_PIN                      PA4   // HB

//
// Fans
//
#define FAN0_PIN                            PA0   // FAN
#define FAN1_PIN                            PA14  // Connected to +24V
#define FAN2_PIN                            -1    // PB1, auto fan for E0
#define CONTROLLER_FAN_PIN              FAN1_PIN

//
// Misc
//
#define BEEPER_PIN                          PB15
#define LED_PIN                             -1
#define POWER_LOSS_PIN                      PA6
#define FIL_RUNOUT_PIN                      PA5
#define CASE_LIGHT_PIN                      PA13
#define POWER_MONITOR_VOLTAGE_PIN           PA6

#define AUTO_LEVEL_TX_PIN                   PB13
#define AUTO_LEVEL_RX_PIN                   PB12

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN      AUTO_LEVEL_RX_PIN
#endif

//
// SD Card
//
#define SD_DETECT_PIN                       PC7

#ifndef ONBOARD_SDIO
  #define ONBOARD_SDIO
#endif
#if ENABLED(ONBOARD_SDIO)
  //
  // SPI
  //
  #define SPI_DEVICE                        -1    // Maple
  #define SD_SCK_PIN                        -1
  #define SD_MISO_PIN                       -1
  #define SD_MOSI_PIN                       -1

  //
  // SDIO
  //
  #define SDIO_READ_RETRIES 16
  #define SDIO_D0_PIN                       PC8
  #define SDIO_D1_PIN                       PC9
  #define SDIO_D2_PIN                       PC10
  #define SDIO_D3_PIN                       PC11
  #define SDIO_CK_PIN                       PC12
  #define SDIO_CMD_PIN                      PD2

#else

  #define SD_SS_PIN                         PC11  // SDIO_D3_PIN
  #define SD_SCK_PIN                        PC12  // SDIO_CK_PIN
  #define SD_MISO_PIN                       PC8   // SDIO_D0_PIN
  #define SD_MOSI_PIN                       PD2   // SDIO_CMD_PIN
  #define SOFTWARE_SPI

#endif
