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
 * Einstart-S pin assignments
 * ATmega2560, ATmega1280
 * PCB Silkscreen: 3DPrinterCon_v3.5
 */

#define ALLOW_MEGA1280
#include "env_validate.h"

#define BOARD_INFO_NAME "Einstart-S"

//
// Limit Switches
//
#define X_STOP_PIN                            44
#define Y_STOP_PIN                            43
#define Z_STOP_PIN                            42

//
// Steppers
//
#define X_STEP_PIN                            76
#define X_DIR_PIN                             75
#define X_ENABLE_PIN                          73

#define Y_STEP_PIN                            31
#define Y_DIR_PIN                             32
#define Y_ENABLE_PIN                          72

#define Z_STEP_PIN                            34
#define Z_DIR_PIN                             35
#define Z_ENABLE_PIN                          33

#define E0_STEP_PIN                           36
#define E0_DIR_PIN                            37
#define E0_ENABLE_PIN                         30

//
// Temperature Sensors
//
#define TEMP_0_PIN                             0  // Analog Input
#define TEMP_BED_PIN                           1  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                          83
#define HEATER_BED_PIN                        38

#define FAN0_PIN                              82

//
// Misc. Functions
//
#define SD_SS_PIN                             53
#define LED_PIN                                4

//
// LCD / Controller
//

// Requires #define U8GLIB_SH1106_EINSTART in Configuration.h
// u8glib constructor
// U8GLIB_SH1106_128X64 u8g(DOGLCD_SCK, DOGLCD_MOSI, DOGLCD_CS, LCD_PINS_DC, LCD_PINS_RS);

#define LCD_PINS_DC                           78
#define LCD_PINS_RS                           79
// DOGM SPI LCD Support
#define DOGLCD_CS                              3
#define DOGLCD_MOSI                            2
#define DOGLCD_SCK                             5
#define DOGLCD_A0                              2

//
// LCD Display input pins
//
#define BTN_UP                                25
#define BTN_DOWN                              26
#define BTN_LEFT                              27
#define BTN_RIGHT                             28

// 'OK' button
#define BTN_ENC                               29

// Set Kill to right arrow, same as RIGID_PANEL
#define KILL_PIN                              28
