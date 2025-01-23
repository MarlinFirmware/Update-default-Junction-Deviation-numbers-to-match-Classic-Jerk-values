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
 *  Rev B  2 JUN 2017
 *
 *  Converted to Arduino pin numbering
 */

/**
 *  There are two Arduino IDE extensions that are compatible with this board
 *  and with the mainstream Marlin software.
 *
 *  Teensyduino - https://www.pjrc.com/teensy/teensyduino.html
 *    Select Teensy++ 2.0 in Arduino IDE from the 'Tools > Board' menu
 *
 *    Installation instructions are at the above URL.  Don't bother loading the
 *    libraries - they are not used with the Marlin software.
 *
 *  Printrboard - https://github.com/scwimbush/Printrboard-HID-Arduino-IDE-Support
 *
 *    Installation:
 *
 *       1. Go to the above URL, click on the "Clone or Download" button and then
 *          click on "Download ZIP" button.
 *       2. Unzip the file, find the "printrboard" directory and then copy it to the
 *          hardware directory in Arduino.  The Arduino hardware directory will probably
 *          be located in a path similar to this: C:\Program Files (x86)\Arduino\hardware.
 *       3. Restart Arduino.
 *       4. Select "Printrboard" from the 'Tools > Board' menu.
 *
 *  Teensyduino is the most popular option. Printrboard is used if your board doesn't have
 *  the Teensyduino bootloader on it.
 */

/**
 *  To burn the bootloader that comes with Printrboard:
 *
 *   1. Connect your programmer to the board.
 *   2. In the Arduino IDE select "Printrboard" and then select the programmer.
 *   3. In the Arduino IDE click on "burn bootloader". Don't worry about the "verify failed at 1F000" error message.
 *   4. The programmer is no longer needed. Remove it.
 */

/**
 * Teensy++ 2.0 Breadboard pin assignments (AT90USB1286)
 * Requires the Teensyduino software with Teensy++ 2.0 selected in Arduino IDE!
 * https://www.pjrc.com/teensy/teensyduino.html
 * See https://reprap.org/wiki/Printrboard for more info
 *
 * CLI build: HARDWARE_MOTHERBOARD=84 make
 *
 *  DaveX plan for Teensylu/printrboard-type pinouts for a TeensyBreadboard:
 *  (ref teensylu & sprinter)
 *
 *                               USB
 *           GND       GND |-----#####-----| +5V              ATX +5SB
 *     ATX PS_ON    PWM 27 |b7   #####   b6| 26    PWM*       Stepper Enable
 *                  PWM  0 |d0           b5| 25    PWM*
 *                  PWM  1 |d1           b4| 24    PWM
 *         X_MIN         2 |d2           b3| 23               MISO_PIN
 *         Y_MIN         3 |d3           b2| 22               MOSI_PIN
 *         Z_MIN         4 |d4  * *      b1| 21               SCK_PIN
 *                       5 |d5  e e      b0| 20               SDSS
 *                LED    6 |d6  5 4      e7| 19
 *                       7 |d7           e6| 18
 *       LCD  RS         8 |e0             | GND
 *       LCD  EN         9 |e1   a4 a0    R| AREF
 *       LCD  D4        10 |c0   a5 a1   f0| 38 A0            ENC_1
 *       LCD  D5        11 |c1   a6 a2   f1| 39 A1            ENC_2
 *       LCD  D6        12 |c2   a7 a3   f2| 40 A2            ENC_CLK
 *       LCD  D6        13 |c3           f3| 41 A3
 *      Bed Heat    PWM 14 |c4   V G R   f4| 42 A4
 * Extruder Heat    PWM 15 |c5   c n S   f5| 43 A5
 *           Fan    PWM 16 |c6   c d T   f6| 44 A6            Bed TC
 *                      17 |c7   * * *   f7| 45 A7            Extruder TC * 4.7k * +5
 *                         -----------------
 *
 *      Interior E4: 36, INT4
 *      Interior E5: 37, INT5
 *      Interior PA0-7: 28-35  -- Printrboard and Teensylu use these pins for step & direction:
 *             T++ PA Signal  Marlin
 *
 *       Z STEP  32 a4  a0 28 X STEP
 *       Z DIR   33 a5  a1 29 X DIR
 *       E STEP  34 a6  a2 30 Y STEP
 *       E DIR   35 a7  a3 31 Y DIR
 */

#include "env_validate.h"

// https://reprap.org/wiki/Teensy_Breadboard
#define BOARD_INFO_NAME "Teensy++2.0"

//
// Limit Switches
//
#define X_STOP_PIN                             2  // D2
#define Y_STOP_PIN                             3  // D3
#define Z_STOP_PIN                             4  // D4

//
// Steppers
//
#define X_STEP_PIN                            28  // A0 Marlin
#define X_DIR_PIN                             29  // A1 Marlin
#define X_ENABLE_PIN                          26  // B6

#define Y_STEP_PIN                            30  // A2 Marlin
#define Y_DIR_PIN                             31  // A3
#define Y_ENABLE_PIN                          26  // B6 Shared w/x

#define Z_STEP_PIN                            32  // A4
#define Z_DIR_PIN                             33  // A5
#define Z_ENABLE_PIN                          26  // B6 Shared w/x

#define E0_STEP_PIN                           34  // A6
#define E0_DIR_PIN                            35  // A7
#define E0_ENABLE_PIN                         26  // B6 Shared w/x

//
// Temperature Sensors
//
#define TEMP_0_PIN                             7  // F7 Analog Input (Extruder)
#define TEMP_BED_PIN                           6  // F6 Analog Input (Bed)

//
// Heaters / Fans
//
#define HEATER_0_PIN                          15  // C5 PWM3B  Extruder
#define HEATER_BED_PIN                        14  // C4 PWM3C
#ifndef FAN0_PIN
  #define FAN0_PIN                            16  // C6 PWM3A  Fan
#endif

//
// Misc. Functions
//
#define SD_SS_PIN                             20  // B0
#define LED_PIN                                6  // D6
#define PS_ON_PIN                             27  // B7

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                       1  // D1 PWM2B  MUST BE HARDWARE PWM
#endif

//
// LCD / Controller
//

#if IS_ULTIPANEL
  #define LCD_PINS_RS                          8  // E0
  #define LCD_PINS_EN                          9  // E1
  #define LCD_PINS_D4                         10  // C0
  #define LCD_PINS_D5                         11  // C1
  #define LCD_PINS_D6                         12  // C2
  #define LCD_PINS_D7                         13  // C3
  #define BTN_EN1                             38  // F0
  #define BTN_EN2                             39  // F1
  #define BTN_ENC                             40  // F2
#endif

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #define SPINDLE_LASER_PWM_PIN                0  // D0 PWM0B   MUST BE HARDWARE PWM
  #define SPINDLE_LASER_ENA_PIN                5  // D5  Pin should have a pullup!
  #define SPINDLE_DIR_PIN                      7  // D7
#endif
