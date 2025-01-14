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

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Dagoma3D D6 supports up to 2 hotends / E-steppers."
#endif

#define BOARD_INFO_NAME "Dagoma3D D6"

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                            43
#define Y_DIAG_PIN                            41
#define Z_DIAG_PIN                            47
#define E0_DIAG_PIN                           21
#define E1_DIAG_PIN                           20

//
// Endstops
//
#define X_STOP_PIN                             2
#define Y_STOP_PIN                             3
#define Z_STOP_PIN                            15

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      39
#endif
#if EXTRUDERS > 1 && !defined(FIL_RUNOUT2_PIN)
  #define FIL_RUNOUT2_PIN                     14
#endif

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #ifndef BOARD_ST7920_DELAY_1
    #define BOARD_ST7920_DELAY_1               0
  #endif
  #ifndef BOARD_ST7920_DELAY_2
    #define BOARD_ST7920_DELAY_2             250
  #endif
  #ifndef BOARD_ST7920_DELAY_3
    #define BOARD_ST7920_DELAY_3             250
  #endif
#endif

#define KILL_PIN                              -1  // NC

#define LCD_CONTRAST_DEFAULT                 255

//
// Sensorless homing DIAG pin is not directly connected to the MCU. Close
// the jumper next to the limit switch socket when using sensorless homing.
//
#if HAS_TMC_UART
  //
  // TMC2208/TMC2209 stepper drivers
  //
  #define X_SERIAL_TX_PIN                     73
  #define Y_SERIAL_TX_PIN                     73
  #define Z_SERIAL_TX_PIN                     73
  #define E0_SERIAL_TX_PIN                    73
  #define E1_SERIAL_TX_PIN                    12

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                    0
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                    1
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                    2
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                   3
  #endif
  #ifndef E1_SLAVE_ADDRESS
    #define E1_SLAVE_ADDRESS                   3
  #endif
  static_assert(X_SLAVE_ADDRESS == 0, "X_SLAVE_ADDRESS must be 0 for BOARD_DAGOMA_D6.");
  static_assert(Y_SLAVE_ADDRESS == 1, "Y_SLAVE_ADDRESS must be 1 for BOARD_DAGOMA_D6.");
  static_assert(Z_SLAVE_ADDRESS == 2, "Z_SLAVE_ADDRESS must be 2 for BOARD_DAGOMA_D6.");
  static_assert(E0_SLAVE_ADDRESS == 3, "E0_SLAVE_ADDRESS must be 3 for BOARD_DAGOMA_D6.");
  static_assert(E1_SLAVE_ADDRESS == 3, "E1_SLAVE_ADDRESS must be 3 for BOARD_DAGOMA_D6.");

#endif

//
// Import default RAMPS 1.4 pins
//
#include "pins_RAMPS.h"
