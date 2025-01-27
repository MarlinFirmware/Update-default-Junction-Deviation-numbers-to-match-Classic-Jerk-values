/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

//
// MKS Neptune 3
//

// Avoid conflict with TIMER_TONE
#define ALLOW_STM32F4
#include "env_validate.h"

#define BOARD_INFO_NAME "MKS Neptune 3"

//
// Release PB4 (Z_DIR_PIN) from JTAG NRST role
//
//#define DISABLE_DEBUG

// Use one of these or SDCard-based Emulation will be used
//#define SRAM_EEPROM_EMULATION                   // Use BackSRAM-based EEPROM emulation
//#define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation
#if EITHER(NO_EEPROM_SELECTED, I2C_EEPROM)
  #define I2C_EEPROM
  #define MARLIN_EEPROM_SIZE             0x1000U  // 4KB
  #define I2C_SCL_PIN                       PB6
  #define I2C_SDA_PIN                       PB7
#endif

//
// Servos
//
#define SERVO0_PIN                          PA8   // BLTOUCH

//
// Limit Switches
//
//#define ZNP_TEST
#ifdef ZNP_TEST
  #define X_DIAG_PIN                        PC14  // Z+
#else
  #define X_DIAG_PIN                        PA13  // X-
#endif

#define Y_DIAG_PIN                          PB8
#define Z_DIAG_PIN                          PC13

#define X_STOP_PIN                    X_DIAG_PIN
#define Y_STOP_PIN                    Y_DIAG_PIN
#define Z_STOP_PIN                    Z_DIAG_PIN

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PC14  // PB9
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PD2
#define X_STEP_PIN                          PC12
#define X_DIR_PIN                           PB3

#define Y_ENABLE_PIN                        PC10
#define Y_STEP_PIN                          PC11
#define Y_DIR_PIN                           PA15

#define Z_ENABLE_PIN                        PC8
#define Z_STEP_PIN                          PC7
#define Z_DIR_PIN                           PC9

#define E0_ENABLE_PIN                       PC6
#define E0_STEP_PIN                         PB10
#define E0_DIR_PIN                          PB1

#define E1_ENABLE_PIN                       PC5
#define E1_STEP_PIN                         PC4
#define E1_DIR_PIN                          PA4

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // TH1
#define TEMP_1_PIN                          PC2   // TH2
#define TEMP_BED_PIN                        PC0   // TB1

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA6   // HEATER1
#define HEATER_BED_PIN                      PA5   // HOT BED

#define FAN_PIN                             PB0   // FAN
//#define FAN1_PIN                          PA7   // FAN1

#if NEED_TOUCH_PINS
  #define TOUCH_CS_PIN                      PA7   // SPI2_NSS
  #define TOUCH_SCK_PIN                     PB13  // SPI2_SCK
  #define TOUCH_MISO_PIN                    PB14  // SPI2_MISO
  #define TOUCH_MOSI_PIN                    PB15  // SPI2_MOSI
#endif

//
// SD Support
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define ENABLE_SPI3
  #define SD_SS_PIN                         -1
  #define SDSS                              PB12
  #define SD_SCK_PIN                        PB13
  #define SD_MISO_PIN                       PB14
  #define SD_MOSI_PIN                       PB15
  #define SD_SPI_SPEED            SPI_HALF_SPEED
  #if ENABLED(NO_SD_HOST_DRIVE)
    // Detect pin doesn't work when NO_SD_HOST_DRIVE disabled
    #define SD_DETECT_PIN                   PC3
  #endif
#endif
