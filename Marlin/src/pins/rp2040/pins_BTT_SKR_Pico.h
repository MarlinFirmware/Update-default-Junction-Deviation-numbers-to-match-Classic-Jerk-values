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
 * BigTreeTech SKR Pico
 * https://github.com/bigtreetech/SKR-Pico
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "BTT SKR Pico"
#define DEFAULT_MACHINE_NAME "BIQU 3D Printer"

#define USBCON

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE             0x2000U  // 8K
#endif

//
// Servos
//
#define SERVO0_PIN                            29

//
// Limit Switches
//
#define X_DIAG_PIN                             4
#define Y_DIAG_PIN                             3
#define Z_DIAG_PIN                            25  // probe:z_virtual_endstop

#define X_STOP_PIN                    X_DIAG_PIN
#define Y_STOP_PIN                    Y_DIAG_PIN
#define Z_STOP_PIN                    Z_DIAG_PIN

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      16  // E0-STOP
#endif

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     22
#endif

//
// Steppers
//
#define X_STEP_PIN                            11
#define X_DIR_PIN                             10
#define X_ENABLE_PIN                          12

#define Y_STEP_PIN                             6
#define Y_DIR_PIN                              5
#define Y_ENABLE_PIN                           7

#define Z_STEP_PIN                            19
#define Z_DIR_PIN                             28
#define Z_ENABLE_PIN                           2

#define E0_STEP_PIN                           14
#define E0_DIR_PIN                            13
#define E0_ENABLE_PIN                         15

//
// Temperature Sensors
//
#define TEMP_0_PIN                            27  // Analog Input
#define TEMP_BED_PIN                          26  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                          23
#define HEATER_BED_PIN                        21

#define FAN0_PIN                              17
#define FAN1_PIN                              18

#if HAS_CUTTER
  #define SPINDLE_LASER_ENA_PIN                0
  #define SPINDLE_LASER_PWM_PIN                1
#else
  #define FAN2_PIN                            20
#endif

//
// Misc. Functions
//
#define NEOPIXEL_PIN                          24

/**
 * TMC2208/TMC2209 stepper drivers
 */
#if HAS_TMC_UART
  /**
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL MSerial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E4_HARDWARE_SERIAL Serial1

  /**
   * Software serial
   */
  #ifndef X_SERIAL_TX_PIN
    #define X_SERIAL_TX_PIN                    8
  #endif
  #ifndef X_SERIAL_RX_PIN
    #define X_SERIAL_RX_PIN                    9
  #endif
  #ifndef Y_SERIAL_TX_PIN
    #define Y_SERIAL_TX_PIN                    8
  #endif
  #ifndef Y_SERIAL_RX_PIN
    #define Y_SERIAL_RX_PIN                    9
  #endif
  #ifndef Z_SERIAL_TX_PIN
    #define Z_SERIAL_TX_PIN                    8
  #endif
  #ifndef Z_SERIAL_RX_PIN
    #define Z_SERIAL_RX_PIN                    9
  #endif
  #ifndef E0_SERIAL_TX_PIN
    #define E0_SERIAL_TX_PIN                   8
  #endif
  #ifndef E0_SERIAL_RX_PIN
    #define E0_SERIAL_RX_PIN                   9
  #endif
#endif
