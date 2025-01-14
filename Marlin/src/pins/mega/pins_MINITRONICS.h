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
 * Minitronics v1.0/1.1 pin assignments
 * Schematic (1.0): https://reprap.org/wiki/File:MinitronicsPDF.zip
 * Schematic (1.1): https://reprapworld.nl/documentation/datasheet%20minitronics%20v1.1.pdf
 * ATmega1281
 */

/**
 * Rev B   2 JAN 2017
 *
 *  Added pin definitions for M3, M4 & M5 spindle control commands
 */

#if NOT_TARGET(__AVR_ATmega1281__)
  #error "Oops! Select 'Minitronics' in 'Tools > Board.'"
#elif HOTENDS > 2 || E_STEPPERS > 2
  #error "Minitronics supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "Minitronics v1.0/1.1"
//
// Limit Switches
//
#define X_STOP_PIN                             5
#define Y_STOP_PIN                             2
#define Z_STOP_PIN                             6

//
// Steppers
//
#define X_STEP_PIN                            48
#define X_DIR_PIN                             47
#define X_ENABLE_PIN                          49

#define Y_STEP_PIN                            39
#define Y_DIR_PIN                             40
#define Y_ENABLE_PIN                          38

#define Z_STEP_PIN                            42
#define Z_DIR_PIN                             43
#define Z_ENABLE_PIN                          41

#define E0_STEP_PIN                           45
#define E0_DIR_PIN                            44
#define E0_ENABLE_PIN                         27

#define E1_STEP_PIN                           36
#define E1_DIR_PIN                            35
#define E1_ENABLE_PIN                         37

//
// Temperature Sensors
//
#define TEMP_0_PIN                             7  // Analog Input
#define TEMP_1_PIN                             6  // Analog Input
#define TEMP_BED_PIN                           6  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           7  // EXTRUDER 1
#define HEATER_1_PIN                           8  // EXTRUDER 2
#define HEATER_BED_PIN                         3  // BED

#ifndef FAN0_PIN
  #define FAN0_PIN                             9
#endif

//
// Misc. Functions
//
#define SDSS                                  16
#define LED_PIN                               46

//
// LCD / Controller
//
#define BEEPER_PIN                            -1

#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

  #define LCD_PINS_RS                         15  // CS chip select /SS chip slave select
  #define LCD_PINS_EN                         11  // SID (MOSI)
  #define LCD_PINS_D4                         10  // SCK (CLK) clock

  #define BTN_EN1                             18
  #define BTN_EN2                             17
  #define BTN_ENC                             25

  #define SD_DETECT_PIN                       30

#else

  #define LCD_PINS_RS                         -1
  #define LCD_PINS_EN                         -1

  // Buttons are directly attached using keypad
  #define BTN_EN1                             -1
  #define BTN_EN2                             -1
  #define BTN_ENC                             -1

  #define SD_DETECT_PIN                       -1  // Minitronics doesn't use this
#endif

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER                                    // assumes we're only doing CNC work (no 3D printing)
  #undef HEATER_BED_PIN
  #undef TEMP_BED_PIN                             // need to free up some pins but also need to
  #undef TEMP_0_PIN                               // re-assign them (to unused pins) because Marlin
  #undef TEMP_1_PIN                               // requires the presence of certain pins or else it
  #define HEATER_BED_PIN                       4  // won't compile
  #define TEMP_BED_PIN                        50
  #define TEMP_0_PIN                          51
  #define SPINDLE_LASER_PWM_PIN                3  // WARNING - LED & resistor pull up to +12/+24V stepper voltage
  #define SPINDLE_LASER_ENA_PIN               52  // using A6 because it already has a pullup
  #define SPINDLE_DIR_PIN                     53
#endif
