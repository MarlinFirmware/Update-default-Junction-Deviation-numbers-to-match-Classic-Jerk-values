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
 * Espressif ESP32 (Tensilica Xtensa LX6) on ESPNP by enwi
 * Website https://github.com/enwi/ESPNP
 */

#include "env_validate.h"

#define BOARD_INFO_NAME      "ESPNP"
#define DEFAULT_MACHINE_NAME "ESP Pick and Place"

//
// I2S (steppers & other output-only pins)
//
#ifndef I2S_STEPPER_STREAM
  #define I2S_STEPPER_STREAM
#endif
#if ENABLED(I2S_STEPPER_STREAM)
  #define I2S_WS                              17
  #define I2S_BCK                             22
  #define I2S_DATA                            21
#endif

//
// Servos
//
#define SERVO0_PIN                            26
#define SERVO1_PIN                            25

//
// Limit Switches
//
#define X_STOP_PIN                            36
#define Y_STOP_PIN                            39
#define Z_STOP_PIN                            34
#define I_STOP_PIN                            35
#define J_STOP_PIN                            05
#define K_STOP_PIN                            16

//
// Steppers
//
#define X_ENABLE_PIN                         136
#define X_DIR_PIN                            137
#define X_STEP_PIN                           138
#define X_CS_PIN                             139

#define Y_ENABLE_PIN                         140
#define Y_DIR_PIN                            141
#define Y_STEP_PIN                           142
#define Y_CS_PIN                             143

#define Z_ENABLE_PIN                         144
#define Z_DIR_PIN                            145
#define Z_STEP_PIN                           146
#define Z_CS_PIN                             147

#define I_ENABLE_PIN                         148
#define I_DIR_PIN                            149
#define I_STEP_PIN                           150
#define I_CS_PIN                             151

#define J_ENABLE_PIN                         152
#define J_DIR_PIN                            153
#define J_STEP_PIN                           154
#define J_CS_PIN                             155

#define K_ENABLE_PIN                         156
#define K_DIR_PIN                            157
#define K_STEP_PIN                           158
#define K_CS_PIN                             159

// Reduce baud rate to improve software serial reliability
#ifndef TMC_BAUD_RATE
  #define TMC_BAUD_RATE                    19200
#endif

//
// Temperature Sensors
//
//#define TEMP_0_PIN                          36  // Analog Input
//#define TEMP_BED_PIN                        39  // Analog Input

// General use mosfets, useful for things like pumps and solenoids
// Shift register pins 128, 129, 130 and 131 are broken out and can be used
#define FAN0_PIN                             132
#define FAN1_PIN                             134
#define FAN2_PIN                             135
#define FAN3_PIN                             136

// #define FAN_SOFT_PWM_REQUIRED // check if needed

//
// NeoPixel Rings
//
#define BOARD_NEOPIXEL_PIN                    14
#define NEOPIXEL2_PIN                         27
