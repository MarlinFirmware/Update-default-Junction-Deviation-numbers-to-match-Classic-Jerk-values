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
 * Longer3D LK1/LK4/LK5 Pro board pin assignments
 * ATmega2560
 */

#define REQUIRE_MEGA2560
#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "Longer3D LGT KIT V1.0 only supports 1 hotend / E stepper."
#endif

#if SERIAL_PORT == 1 || SERIAL_PORT_2 == 1 || SERIAL_PORT_3 == 1
  #warning "Serial 1 is originally reserved for DGUS LCD."
#endif
#if SERIAL_PORT == 2 || SERIAL_PORT_2 == 2 || SERIAL_PORT_3 == 2 || LCD_SERIAL_PORT == 2
  #warning "Serial 2 has no connector. Hardware changes may be required to use it."
#endif
#if SERIAL_PORT == 3 || SERIAL_PORT_2 == 3 || SERIAL_PORT_3 == 3 || LCD_SERIAL_PORT == 3
  #warning "Serial 3 is originally reserved for Y limit switches. Hardware changes are required to use it."
  #define Y_STOP_PIN                          37
  #if MB(LONGER3D_LKx_PRO)
    #define Z_STOP_PIN                        35
  #endif
#endif

#define BOARD_INFO_NAME "LGT KIT V1.0"
#if ENABLED(LONGER_LK5)
  #define DEFAULT_MACHINE_NAME "LONGER LK5"
#else
  #define DEFAULT_MACHINE_NAME "LONGER 3D Printer"
#endif

//
// Servos
//
#if MB(LONGER3D_LKx_PRO)
  #define SERVO0_PIN                           7
#endif
#define SERVO1_PIN                            -1
#define SERVO2_PIN                            -1
#define SERVO3_PIN                            -1

//
// Limit Switches
//
#if ENABLED(LONGER_LK5)
  #define X_MIN_PIN                            3
  #define X_MAX_PIN                            2
#else
  #define X_STOP_PIN                           3
#endif

#if !ANY_PIN(Y_MIN, Y_MAX, Y_STOP)
  #if ENABLED(LONGER_LK5)
    #define Y_STOP_PIN                        14
  #else
    #define Y_MIN_PIN                         14
    #define Y_MAX_PIN                         15
  #endif
#endif

#if !ANY_PIN(Z_MIN, Z_MAX, Z_STOP)
  #if MB(LONGER3D_LKx_PRO)
    #define Z_MIN_PIN                         35
  #else
    #define Z_MIN_PIN                         11
  #endif
  #define Z_MAX_PIN                           37
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#define Z_MIN_PROBE_PIN                       -1

//
// Steppers - No E1 pins
//
#define E1_STEP_PIN                           -1
#define E1_DIR_PIN                            -1
#define E1_ENABLE_PIN                         -1
#define E1_CS_PIN                             -1

//
// Temperature Sensors
//
#define TEMP_1_PIN                            -1

//
// Průša i3 MK2 Multiplexer Support
//
#if HAS_PRUSA_MMU1
  #define E_MUX2_PIN                          -1
#endif

//
// Misc. Functions
//
#define SD_DETECT_PIN                         49
#define FIL_RUNOUT_PIN                         2

//          ------------------        ----------------        ---------------        -------------
//    Aux-1 | D19 D18 GND 5V |    J21 | D4 D5 D6 GND |    J17 | D11 GND 24V |    J18 | D7 GND 5V |
//          ------------------        ----------------        ---------------        -------------

#if ALL(CR10_STOCKDISPLAY, LONGER_LK5)
  /**       CR-10 Stock Display
   *             ------
   * BEEPER D11 | 1  2 | D15 ENC
   *    EN1 D18 | 3  4 | GND
   *    EN2 D19   5  6 | D6  LCD_D4
   * LCD_RS D5  | 7  8 | D4  LCD_ENABLE
   *        GND | 9 10 | 5V
   *             ------
   *      Connected via provided custom cable to:
   *      Aux-1, J21, J17 and Y-Max.
   */

  // Migrated to pins/lcd

  #define EXP1_01_PIN                         11
  #define EXP1_02_PIN                         15
  #define EXP1_03_PIN                         18
  #define EXP1_05_PIN                         19
  #define EXP1_06_PIN                          6
  #define EXP1_07_PIN                          5
  #define EXP1_08_PIN                          4

  #define SDCARD_CONNECTION              ONBOARD
#endif

//
// Other RAMPS 1.3 pins
//
#include "pins_RAMPS_13.h" // ... pins_RAMPS.h
