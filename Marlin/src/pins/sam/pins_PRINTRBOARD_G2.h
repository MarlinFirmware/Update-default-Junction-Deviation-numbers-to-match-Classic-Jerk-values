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
 * PRINTRBOARD_G2
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Printrboard G2"
#endif

//
// Servos
//
//#define SERVO0_PIN                          -1
//#define SERVO1_PIN                          -1

//
// Limit Switches
//
#define X_STOP_PIN                            22  // PB26
#define Y_STOP_PIN                            18  // PA11
#define Z_STOP_PIN                            19  // PA10

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     22
#endif

#ifndef FIL_RUNOUT_PIN
  //#define FIL_RUNOUT_PIN                    57  // PA22
#endif
#ifndef FIL_RUNOUT2_PIN
  //#define FIL_RUNOUT2_PIN                   21  // PB13
#endif

//
// LED defines
//
//#define NEOPIXEL_TYPE                 NEO_GRBW  // NEO_GRBW / NEO_GRB - four/three channel driver type (defined in Adafruit_NeoPixel.h)
//#define BOARD_NEOPIXEL_PIN                  20  // LED driving pin on motherboard
//#define NEOPIXEL_PIXELS                      3  // Number of LEDs in the strip
//#define SDA0                                20  // PB12 NeoPixel pin I2C data
//#define SCL0                                21  // PB13              I2C clock

// D0_12 #REF! (INDICATOR_LED)
// B28 JTAG-CLK
// B31 JTAG_TMS /SWD_DIO
//A18 INTERRUPT_OUT
//A12 USART_RX not used
//A13 USART_TX not used
//A14 UART_RTS
//A15 UART_CTS
//PB2 Unassigned
//PB4 to PB9 Unassigned
//#define UART_RX_PIN                          0  // PA8    "RX0"
//#define UART_TX_PIN                          1  // PA9    "TX0"
//#define UART_RTS_PIN                        23  // PA14
//#define UART_CTS_PIN                        24  // PA15

//
// Steppers
//
#define Z_STEP_PIN                            73  // PA21      MOTOR 1
#define Z_DIR_PIN                             75  // PA26
#define Z_ENABLE_PIN                          74  // PA25

#define X_STEP_PIN                            66  // PB15      MOTOR 2
#define X_DIR_PIN                             54  // PA16
#define X_ENABLE_PIN                          67  // PB16

#define Y_STEP_PIN                            34  // PA29      MOTOR 3
#define Y_DIR_PIN                             35  // PB1
#define Y_ENABLE_PIN                          36  // PB0

#define E0_STEP_PIN                           53  // PB14      MOTOR 4
#define E0_DIR_PIN                            78  // PB23
#define E0_ENABLE_PIN                         37  // PB22

// Microstepping mode pins
#define Z_MS1_PIN                             52  // PB21 MODE0     MOTOR 1
#define Z_MS2_PIN                             52  // PB21 MODE1
#define Z_MS3_PIN                             65  // PB20 MODE2

#define X_MS1_PIN                             43  // PA20 MODE0     MOTOR 2
#define X_MS2_PIN                             43  // PA20 MODE1
#define X_MS3_PIN                             42  // PA19 MODE2

#define Y_MS1_PIN                             77  // PA28 MODE0     MOTOR 3
#define Y_MS2_PIN                             77  // PA28 MODE1
#define Y_MS3_PIN                             76  // PA27 MODE2

#define E0_MS1_PIN                            38  // PB11 MODE0     MOTOR 4
#define E0_MS2_PIN                            38  // PB11 MODE1
#define E0_MS3_PIN                            39  // PB10 MODE2

// Motor current PWM pins
#define MOTOR_CURRENT_PWM_X_PIN               62  // PB17        MOTOR 1
#define MOTOR_CURRENT_PWM_Z_PIN               63  // PB18        MOTOR 2
#define MOTOR_CURRENT_PWM_Y_PIN               64  // PB19        MOTOR 3
#define MOTOR_CURRENT_PWM_E_PIN               61  // PA2         MOTOR 4

#define DEFAULT_PWM_MOTOR_CURRENT { 300, 400, 1000}  // XY Z E0, 1000 = 1000mAh

//
// Temperature Sensors
//
#define TEMP_0_PIN                             2  // digital 56 PA23
#define TEMP_BED_PIN                           5  // digital 59 PA4

//
// Heaters / Fans
//
#define HEATER_0_PIN                          40  // PA5
#define HEATER_BED_PIN                        41  // PB24

#ifndef FAN0_PIN
  #define FAN0_PIN                            13  //  PB27 Fan1A
#endif
#define FAN1_PIN                              58  //  PA6  Fan1B

#define FET_SAFETY_PIN                        31  // PA7  must be pulsed low every 50 mS or FETs are turned off
#define FET_SAFETY_DELAY                      50  // 50 mS delay between pulses
#define FET_SAFETY_INVERTED                 true  // true - negative going pulse of 2 uS

/////////////////////////////////////////////////////////

#define SD_MISO_PIN                           68  // set to unused pins for now
#define SD_MOSI_PIN                           69  // set to unused pins for now
#define SD_SCK_PIN                            70  // set to unused pins for now
#define SD_SS_PIN                             71  // set to unused pins for now

/**
 * G2 uses 8 pins that are not available in the DUE environment:
 *   34 PA29 - Y_STEP_PIN
 *   35 PB1  - Y_DIR_PIN
 *   36 PB0  - Y_ENABLE_PIN
 *   37 PB22 - E0_ENABLE_PIN
 *   38 PB11 - E0_MS1_PIN - normally used by the USB native port
 *   39 PB10 - E0_MS3_PIN - normally used by the USB native port
 *   40 PA5  - HEATER_0_PIN
 *   41 PB24 - HEATER_BED_PIN
 *
 * None of these are in the arduino_due_x variant so digitalWrite and digitalRead can't be used on them.
 *
 * They can be accessed via FASTIO functions WRITE, READ, OUT_WRITE, OUTPUT, ...
 */
