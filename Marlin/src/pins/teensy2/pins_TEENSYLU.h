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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.   See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.   If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

/**
 *  Rev C  2 JUN 2017
 *
 *  Converted to Arduino pin numbering
 *
 *  Schematic (1.0): https://raw.githubusercontent.com/StephS/Teensylu/master/working/Teensylu-1.0.sch
 *  (*) Other versions are discouraged by creator.
 */

/**
 *  There are two Arduino IDE extensions that are compatible with this board
 *  and with the mainstream Marlin software.  All have been used with Arduino 1.6.12
 *
 *  Teensyduino - https://www.pjrc.com/teensy/teensyduino.html
 *    Select Teensy++ 2.0 in Arduino IDE from the 'Tools > Board' menu
 *
 *    Installation instructions are at the above URL.  Don't bother loading the
 *    libraries - they are not used with the Marlin software.
 *
 *  Printrboard - https://github.com/scwimbush/Printrboard-HID-Arduino-IDE-Support
 *    This is basically Teensyduino but with a bootloader that can handle image sizes
 *    larger than 64K.
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
  *  SILKSCREEN ERROR
  *
  *  The silkscreen for the endstops do NOT match the schematic.  The silkscreen SHOULD
  *  read (from left to right) X-STOP, Y-STOP, Z-STOP & E-STOP.  The silkscreen actually
  *  reads                     E-STOP, X-STOP, Y-STOP & Z-STOP.
  *
  *  The pin assignments in this file match the silkscreen.
  */

#define ALLOW_AT90USB1286P
#include "env_validate.h"

#if NOT_TARGET(__AVR_AT90USB1286__, __AVR_AT90USB1286P__)
  #error "Oops! Select 'Teensy++ 2.0' or 'Printrboard' in 'Tools > Board.'"
#endif

#define BOARD_INFO_NAME "Teensylu"

//
// Limit Switch definitions that match the SCHEMATIC
//
//#define X_STOP_PIN                          25  // B5
//#define Y_STOP_PIN                          26  // B6
//#define Z_STOP_PIN                          27  // B7
//#define E_STOP_PIN                          36  // E4

//
// Limit Switch definitions that match the SILKSCREEN
//
#define X_STOP_PIN                            26  // B6
#define Y_STOP_PIN                            27  // B7
#define Z_STOP_PIN                            36  // E4
//#define E_STOP_PIN                          25  // B5

//
// Steppers
//
#define X_STEP_PIN                            28  // A0
#define X_DIR_PIN                             29  // A1
#define X_ENABLE_PIN                          19  // E7

#define Y_STEP_PIN                            30  // A2
#define Y_DIR_PIN                             31  // A3
#define Y_ENABLE_PIN                          18  // E6

#define Z_STEP_PIN                            32  // A4
#define Z_DIR_PIN                             33  // A5
#define Z_ENABLE_PIN                          17  // C7

#define E0_STEP_PIN                           34  // A6
#define E0_DIR_PIN                            35  // A7
#define E0_ENABLE_PIN                         13  // C3

//
// Temperature Sensors
//
#define TEMP_0_PIN                             7  // Analog Input (Extruder)
#define TEMP_BED_PIN                           6  // Analog Input (Bed)

//
// Heaters / Fans
//
#define HEATER_0_PIN                          15  // C5 PWM3B - Extruder
#define HEATER_BED_PIN                        14  // C4 PWM3C

#ifndef FAN0_PIN
  #define FAN0_PIN                            16  // C6 PWM3A
#endif

//
// Misc. Functions
//
#define SD_SS_PIN                             20  // B0 JP31-6

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                       0  // D0 IO-14  PWM0B
#endif

//
// LCD / Controller
//

#if HAS_WIRED_LCD && IS_NEWPANEL

  #define BEEPER_PIN                          -1

  #if ENABLED(LCD_I2C_PANELOLU2)
    #define BTN_EN1                            3  // D3 IO-8
    #define BTN_EN2                            2  // D2 IO-10
    #define BTN_ENC                           41  // F3 IO-7
    #define SD_SS_PIN                         38  // F0 IO-13 use SD card on Panelolu2
  #endif

  #define SD_DETECT_PIN                       -1

#endif // HAS_WIRED_LCD && IS_NEWPANEL

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #define SPINDLE_LASER_PWM_PIN               24  // B4 IO-3 PWM2A - MUST BE HARDWARE PWM
  #define SPINDLE_LASER_ENA_PIN               39  // F1 IO-11 - Pin should have a pullup!
  #define SPINDLE_DIR_PIN                     40  // F2 IO-9
#endif
