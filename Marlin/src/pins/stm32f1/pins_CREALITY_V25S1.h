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

/**
 * Creality v2.5.S1 (STM32F103RE / STM32F103RC) as found in the CR-10 Smart Pro
 */

#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "Creality V2.5.S1 only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME      "Creality v2.5.S1"
#define DEFAULT_MACHINE_NAME "Creality3D"

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
#define DISABLE_DEBUG

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #define IIC_BL24CXX_EEPROM                      // EEPROM on I2C-0
  //#define SDCARD_EEPROM_EMULATION
  //#define FLASH_EEPROM_EMULATION
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
  #define IIC_EEPROM_SDA                    PA11
  #define IIC_EEPROM_SCL                    PA12
  #define MARLIN_EEPROM_SIZE              0x800U  // 2K (24C16)
#elif ANY(SDCARD_EEPROM_EMULATION, FLASH_EEPROM_EMULATION)
  #define MARLIN_EEPROM_SIZE              0x800U  // 2K
#endif

//
// Limit Switches
//
#define X_STOP_PIN                          PC4
#define Y_STOP_PIN                          PC5
#define Z_STOP_PIN                          PC15

//
// Probe
//
#if ENABLED(BLTOUCH)
  #define SERVO0_PIN                        PC14  // BLTouch OUT PIN
#elif ENABLED(PROBE_ACTIVATION_SWITCH)
  #define PROBE_TARE_PIN                    PC14
  #define PROBE_ACTIVATION_SWITCH_PIN       PB2
#elif ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN                  PC14
#endif

//
// Filament Runout Sensor
//
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FIL_RUNOUT_PIN                    PA15  // "Pulled-high"
#endif

//
// Steppers
//
#define X_STEP_PIN                          PB8
#define X_DIR_PIN                           PB7
#define X_ENABLE_PIN                        PC3

#define Y_STEP_PIN                          PB6
#define Y_DIR_PIN                           PB5
#define Y_ENABLE_PIN                X_ENABLE_PIN

#define Z_STEP_PIN                          PB4
#define Z_DIR_PIN                           PB3
#define Z_ENABLE_PIN                X_ENABLE_PIN

#define E0_STEP_PIN                         PC2
#define E0_DIR_PIN                          PB9
#define E0_ENABLE_PIN               X_ENABLE_PIN

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PB1   // TH1
#define TEMP_BED_PIN                        PB0   // TB1

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB14  // HEATER1
#define HEATER_BED_PIN                      PB13  // HOT BED

#define FAN0_PIN                            PB15  // FAN
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                   PC13  // FAN
#endif
#define FAN_SOFT_PWM_REQUIRED

//
// SD Card
//
#define SD_DETECT_PIN                       PC7
#define SDCARD_CONNECTION                ONBOARD
#define ON_BOARD_SPI_DEVICE                    1
#define ONBOARD_SD_CS_PIN                   PC12  // SDSS
#define ONBOARD_SDIO
#define NO_SD_HOST_DRIVE                          // This board's SD is only seen by the printer

//
// Misc. Functions
//
#define CASE_LIGHT_PIN                      PA7

//
// Suicide Power
//
#define PS_ON_PIN                           PA0
#define MOTOR_CIRCUIT_PIN                   PA1

//
// Motor Protect
//
#define MOTOR_PROTECT_PIN                   PC0

//
// WiFi Reset
//
#define RESET_WIFI_PIN                      PB12
