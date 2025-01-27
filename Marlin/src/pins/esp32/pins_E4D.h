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

/**
 * E4d@box pin assignments
 *
 * Small factor 3D printer control board based on the ESP32 microcontroller for Laser, CNC and 3D printers.
 * More info at https://atbox.tech/ and the E4d@box Facebook page.
 */

#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "E4d@box only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME       "E4d@box"
#define BOARD_WEBSITE_URL     "github.com/Exilaus/E4d@box"
#define DEFAULT_MACHINE_NAME  BOARD_INFO_NAME

//
// Redefine I2S for ESP32
//
#undef I2S_WS
#undef I2S_BCK
#undef I2S_DATA
#define I2S_WS                                23
#define I2S_BCK                               22
#define I2S_DATA                              21

//
// Limit Switches
//
#define X_STOP_PIN                            34
#define Y_STOP_PIN                            35
#define Z_STOP_PIN                            16

//
// Steppers
//
#define X_STEP_PIN                            12
#define X_DIR_PIN                             13
#define X_ENABLE_PIN                          17
//#define X_CS_PIN                             0

#define Y_STEP_PIN                            32
#define Y_DIR_PIN                             33
#define Y_ENABLE_PIN                X_ENABLE_PIN
//#define Y_CS_PIN                            13

#define Z_STEP_PIN                            25
#define Z_DIR_PIN                             26
#define Z_ENABLE_PIN                X_ENABLE_PIN
//#define Z_CS_PIN                             5  // SS_PIN

#define E0_STEP_PIN                           27
#define E0_DIR_PIN                            14
#define E0_ENABLE_PIN               X_ENABLE_PIN
//#define E0_CS_PIN                           21

//
// Temperature Sensors
//
#define TEMP_0_PIN                            36  // Analog Input
#define TEMP_BED_PIN                          39  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           2
#define FAN0_PIN                               0
#define HEATER_BED_PIN                        15

//
// MicroSD card on SPI
//
#define SD_MOSI_PIN                           23
#define SD_MISO_PIN                           19
#define SD_SCK_PIN                            18
#define SD_SS_PIN                              5
#define USES_SHARED_SPI                           // SPI is shared by SD card with TMC SPI drivers
