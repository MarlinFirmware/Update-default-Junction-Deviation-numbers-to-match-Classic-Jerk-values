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

/****************************************************************************************
* Teensy 4.1 (IMXRT1062) Breadboard pin assignments
* Requires the Teensyduino software with Teensy 4.1 selected in Arduino IDE!
* https://www.pjrc.com/teensy/teensyduino.html
****************************************************************************************/

#include "env_validate.h"

#define BOARD_INFO_NAME "Teensy4.1"

/**
 * Plan for Teensy 4.0 and Teensy 4.1:
 *                                            USB
 *                                 GND |-----#####-----| VIN (3.65 TO 5.5V)
 *  X_STEP_PIN     CS1     RX1  PWM  0 |     #####     | GND
 *  X_DIR_PIN      MISO1   TX1  PWM  1 |               | 3.3V
 *  Y_STEP_PIN                  PWM  2 |               | 23  A9 PWM              SERVO1_PIN
 *  Y_DIR_PIN                   PWM  3 |               | 22  A8 PWM              SERVO0_PIN
 *  Z_STEP_PIN                  PWM  4 |               | 21  A7      RX5
 *  Z_DIR_PIN                   PWM  5 |               | 20  A6      TX5         FILWIDTH_PIN
 *  X_ENABLE_PIN                PWM  6 |               | 19  A5 PWM        SCL0
 *  Y_ENABLE_PIN           RX2  PWM  7 |               | 18  A4 PWM        SDA0  HEATER_1_PIN
 *  Z_ENABLE_PIN           TX2  PWM  8 |               | 17  A3      RX4   SDA1
 *  E0_STEP_PIN                 PWM  9 |               | 16  A2      TX4   SCL1  TEMP_0_PIN
 *  E0_DIR_PIN                  PWM 10 |               | 15  A1 PWM  RX3         TEMP_BED_PIN
 *  MOSI_PIN       MOSI0        PWM 11 |               | 14  A0 PWM  TX3         TEMP_1_PIN
 *  MISO_PIN       MISO0        PWM 12 |               | 13 LED PWM  SCK0        SCK_PIN
 *                                3.3V |               | GND
 *  Z_STOP_PIN                  PWM 24 |               | 41 A17
 *  E0_ENABLE_PIN               PWM 25 |               | 40 A16
 *  FAN0_PIN       MOSI1            26 |               | 39 A15      MISO1       X_STOP_PIN
 *  Z-PROBE PWR    SCK1             27 | *  *  *  *  * | 38 A14                  Y_STOP_PIN
 *  SOL1_PIN               RX7  PWM 28 |               | 37     PWM              HEATER_0_PIN
 *  FAN0_PIN               TX7  PWM 29 |               | 36     PWM              HEATER_BED_PIN
 *  X_CS_PIN                        30 |               | 35          TX8         E1_ENABLE_PIN
 *  y_CS_PIN                        31 |    SDCARD     | 34          RX8         E1_DIR_PIN
 *  Z_CS_PIN                        32 |_______________| 33     PWM              E1_STEP_PIN
 */

//
// Servos
//
#define SERVO0_PIN                            22
#define SERVO1_PIN                            23

//
// Limit Switches
//
#define X_STOP_PIN                            39
#define Y_STOP_PIN                            38
#define Z_STOP_PIN                            24

//
// Steppers
//
#define X_STEP_PIN                             0
#define X_DIR_PIN                              1
#define X_ENABLE_PIN                           6
//#define X_CS_PIN                            30

#define Y_STEP_PIN                             2
#define Y_DIR_PIN                              3
#define Y_ENABLE_PIN                           7
//#define Y_CS_PIN                            31

#define Z_STEP_PIN                             4
#define Z_DIR_PIN                              5
#define Z_ENABLE_PIN                           8
//#define Z_CS_PIN                            32

#define E0_STEP_PIN                            9
#define E0_DIR_PIN                            10
#define E0_ENABLE_PIN                         25

#define E1_STEP_PIN                           33
#define E1_DIR_PIN                            34
#define E1_ENABLE_PIN                         35

//
// Heaters / Fans
//
#define HEATER_0_PIN                          37
#define HEATER_1_PIN                          18
#define HEATER_BED_PIN                        36
#ifndef FAN0_PIN
  #define FAN0_PIN                            29
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                             2  // Extruder / Analog pin numbering: 2 => A2
#define TEMP_1_PIN                             0
#define TEMP_BED_PIN                           1  // Bed / Analog pin numbering

//
// Misc. Functions
//
#define LED_PIN                               13
#define SOL0_PIN                              28
//#define PS_ON_PIN                            1
//#define FILWIDTH_PIN                         6  // A6

#ifndef VOLUME0
  #define VOLUME0                        ONBOARD
#endif
