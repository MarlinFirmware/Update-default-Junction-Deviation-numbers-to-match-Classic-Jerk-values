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
 * VERTEX NANO Arduino Mega with RAMPS EFB v1.4 pin assignments.
 * ATmega2560, ATmega1280
 */

#if HAS_MULTI_HOTEND
  #error "K8600 only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME      "K8600"
#define DEFAULT_MACHINE_NAME "Vertex Nano"

//
// Limit Switches
//
#define X_MIN_PIN                              3
#define Y_MAX_PIN                             14
#define Z_STOP_PIN                            18

//
// Steppers
//
#define Z_ENABLE_PIN                          63

//
// Heaters / Fans
//
#define HEATER_BED_PIN                        -1
#define FAN0_PIN                               8

//
// Misc. Functions
//
#define SD_SS_PIN                             25
#define CASE_LIGHT_PIN                         7

//
// LCD / Controller
//

#if HAS_WIRED_LCD && IS_NEWPANEL

  #define LCD_PINS_RS                         27
  #define LCD_PINS_EN                         29
  #define LCD_PINS_D4                         37
  #define LCD_PINS_D5                         35
  #define LCD_PINS_D6                         33
  #define LCD_PINS_D7                         31

  // Buttons
  #define BTN_EN1                             17
  #define BTN_EN2                             16
  #define BTN_ENC                             23

  #define LCD_PINS_DEFINED

#else

  #define BEEPER_PIN                          33

#endif

//
// Other RAMPS pins
//
#include "pins_RAMPS.h"
