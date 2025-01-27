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
 * Arduino Mega with RAMPS-S v1.2 by Sakul.cz pin assignments
 * Written by Michal Rábek <rabek33@gmail.com>
 * ATmega2560
 *
 * Applies to the following boards:
 *
 *  BOARD_RAMPS_S_12_EEFB  Ramps S 1.2 (Hotend0, Hotend1, Fan, Bed)
 *  BOARD_RAMPS_S_12_EEEB  Ramps S 1.2 (Hotend0, Hotend1, Hotend2, Bed)
 *  BOARD_RAMPS_S_12_EFFB  Ramps S 1.2 (Hotend, Fan0, Fan1, Bed)
 *
 * Other pins_MYBOARD.h files may override these defaults
 */

#include "env_validate.h"

// Custom flags and defines for the build
//#define BOARD_CUSTOM_BUILD_FLAGS -D__FOO__

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "RAMPS S 1.2"
#endif

//
// Servos
//
#ifndef SERVO0_PIN
  #define SERVO0_PIN                          10
#endif
#ifndef SERVO1_PIN
  #define SERVO1_PIN                          11
#endif
#ifndef SERVO2_PIN
  #define SERVO2_PIN                          12
#endif
#ifndef SERVO3_PIN
  #define SERVO3_PIN                          44
#endif

//
// Limit Switches
//
#ifndef X_STOP_PIN
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                         37
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                         36
  #endif
#endif
#ifndef Y_STOP_PIN
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                         35
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                         34
  #endif
#endif
#ifndef Z_STOP_PIN
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                         33
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                         32
  #endif
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                      5
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      44  // RAMPS_S S3 on the servos connector
#endif

//
// Steppers
//
#define X_STEP_PIN                            17
#define X_DIR_PIN                             16
#define X_ENABLE_PIN                          48

#define Y_STEP_PIN                            54
#define Y_DIR_PIN                             47
#define Y_ENABLE_PIN                          55

#ifndef Z_STEP_PIN
  #define Z_STEP_PIN                          57
#endif
#define Z_DIR_PIN                             56
#define Z_ENABLE_PIN                          62

#define E0_STEP_PIN                           23
#define E0_DIR_PIN                            22
#define E0_ENABLE_PIN                         24

#define E1_STEP_PIN                           26
#define E1_DIR_PIN                            25
#define E1_ENABLE_PIN                         27

#define E2_STEP_PIN                           29
#define E2_DIR_PIN                            28
#define E2_ENABLE_PIN                         39

//
// Temperature Sensors
//
#ifndef TEMP_0_PIN
  #define TEMP_0_PIN                          15  // Analog Input
#endif
#ifndef TEMP_1_PIN
  #define TEMP_1_PIN                          14  // Analog Input
#endif
#ifndef TEMP_2_PIN
  #define TEMP_2_PIN                          13  // Analog Input
#endif
#ifndef TEMP_3_PIN
  #define TEMP_3_PIN                          12  // Analog Input
#endif
#ifndef TEMP_BED_PIN
  #define TEMP_BED_PIN                        11  // Analog Input
#endif

//
// Heaters / Fans
//
#ifndef MOSFET_D_PIN
  #define MOSFET_D_PIN                        -1
#endif
#ifndef RAMPS_S_HE_0
  #define RAMPS_S_HE_0                         2
#endif
#ifndef RAMPS_S_HE_1
  #define RAMPS_S_HE_1                         3
#endif
#ifndef RAMPS_S_HE_2
  #define RAMPS_S_HE_2                         6
#endif

#define HEATER_BED_PIN                         9

#define HEATER_0_PIN                RAMPS_S_HE_0

#if MB(RAMPS_S_12_EEFB)                           // Hotend0, Hotend1, Fan, Bed
  #define HEATER_1_PIN              RAMPS_S_HE_1
  #define FAN0_PIN                  RAMPS_S_HE_2
#elif MB(RAMPS_S_12_EEEB)                         // Hotend0, Hotend1, Hotend2, Bed
  #define HEATER_1_PIN              RAMPS_S_HE_1
  #define HEATER_2_PIN              RAMPS_S_HE_2
#elif MB(RAMPS_S_12_EFFB)                         // Hotend, Fan0, Fan1, Bed
  #define FAN0_PIN                  RAMPS_S_HE_1
  #define FAN1_PIN                  RAMPS_S_HE_2
#endif

//
// Misc. Functions
//
#define SD_SS_PIN                             53
#define LED_PIN                               13

#ifndef KILL_PIN
  #define KILL_PIN                            46
#endif

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                        60  // Analog Input on EXTEND
#endif

#ifndef PS_ON_PIN
  #define PS_ON_PIN                           12  // RAMPS_S S2 on the servos connector
#endif

#if ENABLED(CASE_LIGHT_ENABLE) && !defined(CASE_LIGHT_PIN) && !defined(SPINDLE_LASER_ENA_PIN)
  #if NUM_SERVOS <= 1                             // Prefer the servo connector
    #define CASE_LIGHT_PIN                    12  // Hardware PWM (RAMPS_S S1 on the servos connector)
  #elif HAS_FREE_AUX2_PINS
    #define CASE_LIGHT_PIN                    44  // Hardware PWM
  #endif
#endif

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER && !defined(SPINDLE_LASER_ENA_PIN)
  #define SPINDLE_LASER_PWM_PIN                6  // Hardware PWM
  #define SPINDLE_LASER_ENA_PIN                4  // Pullup or pulldown!
  #define SPINDLE_DIR_PIN                      5
#endif

//
// TMC software SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                        51
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                        50
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                         53
#endif

//
// Průša i3 MK2 Multiplexer Support
//
#if HAS_PRUSA_MMU1
  #ifndef E_MUX0_PIN
    #define E_MUX0_PIN                        29  // E2_STEP_PIN
  #endif
  #ifndef E_MUX1_PIN
    #define E_MUX1_PIN                        28  // E2_DIR_PIN
  #endif
  #ifndef E_MUX2_PIN
    #define E_MUX2_PIN                        39  // E2_ENABLE_PIN
  #endif
#endif

//
// LCD / Controller
//

#if HAS_WIRED_LCD
  #define BEEPER_PIN                          45
  #define LCD_PINS_RS                         19
  #define LCD_PINS_EN                         49
  #define LCD_PINS_D4                         18
  #define LCD_PINS_D5                         30
  #define LCD_PINS_D6                         41
  #define LCD_PINS_D7                         31
  #ifndef SD_DETECT_PIN
    #define SD_DETECT_PIN                     38
  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

#endif

//
// LCD Display input pins
//
#if IS_NEWPANEL
  #define BTN_EN1                             40
  #define BTN_EN2                             42
  #define BTN_ENC                             43
#endif
