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
 * RADDS
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "RADDS"

//
// EEPROM
//
#if ANY(NO_EEPROM_SELECTED, I2C_EEPROM)
  #define I2C_EEPROM
  #define MARLIN_EEPROM_SIZE              0x2000  // 8K
#endif

//
// Servos
//
#if !HAS_CUTTER
  #define SERVO0_PIN                           5
#endif
#define SERVO1_PIN                             6
#define SERVO2_PIN                            39
#define SERVO3_PIN                            40

//
// Limit Switches
//
#define X_MIN_PIN                             28
#define X_MAX_PIN                             34
#define Y_MIN_PIN                             30
#define Y_MAX_PIN                             36
#define Z_MIN_PIN                             32
#define Z_MAX_PIN                             38

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     38
#endif

//
// Steppers
//
#define X_STEP_PIN                            24
#define X_DIR_PIN                             23
#define X_ENABLE_PIN                          26
#ifndef X_CS_PIN
  #define X_CS_PIN                            25
#endif

#define Y_STEP_PIN                            17
#define Y_DIR_PIN                             16
#define Y_ENABLE_PIN                          22
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            27
#endif

#define Z_STEP_PIN                             2
#define Z_DIR_PIN                              3
#define Z_ENABLE_PIN                          15
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            29
#endif

#define E0_STEP_PIN                           61
#define E0_DIR_PIN                            60
#define E0_ENABLE_PIN                         62
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           31
#endif

#define E1_STEP_PIN                           64
#define E1_DIR_PIN                            63
#define E1_ENABLE_PIN                         65
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           33
#endif

#define E2_STEP_PIN                           51
#define E2_DIR_PIN                            53
#define E2_ENABLE_PIN                         49
#ifndef E2_CS_PIN
  #define E2_CS_PIN                           35
#endif

/**
 * RADDS Extension Board V2 / V3
 * https://web.archive.org/web/20210627142010/doku.radds.org/dokumentation/extension-board/
 */
//#define RADDS_EXTENSION                      2
#if RADDS_EXTENSION >= 2
  #define E3_DIR_PIN                          33
  #define E3_STEP_PIN                         35
  #define E3_ENABLE_PIN                       37
  #ifndef E3_CS_PIN
    #define E3_CS_PIN                          6
  #endif

  #if RADDS_EXTENSION == 3

    #define E4_DIR_PIN                        27
    #define E4_STEP_PIN                       29
    #define E4_ENABLE_PIN                     31
    #ifndef E4_CS_PIN
      #define E4_CS_PIN                       39
    #endif

    #define E5_DIR_PIN                        66
    #define E5_STEP_PIN                       67
    #define E5_ENABLE_PIN                     68
    #ifndef E5_CS_PIN
      #define E5_CS_PIN                        6
    #endif

    #define RADDS_EXT_MSI_PIN                 69

    #define BOARD_INIT() OUT_WRITE(RADDS_EXT_VDD_PIN, HIGH)

  #else

    #define E4_DIR_PIN                        27
    #define E4_STEP_PIN                       29
    #define E4_ENABLE_PIN                     31
    #ifndef E4_CS_PIN
      #define E4_CS_PIN                       39
    #endif

    // E3 and E4 share the same MSx pins
    #define E3_MS1_PIN                        67
    #define E4_MS1_PIN                        67
    #define E3_MS2_PIN                        68
    #define E4_MS2_PIN                        68
    #define E3_MS3_PIN                        69
    #define E4_MS3_PIN                        69

    #define RADDS_EXT_VDD2_PIN                66

    #define BOARD_INIT() do{ OUT_WRITE(RADDS_EXT_VDD_PIN, HIGH); OUT_WRITE(RADDS_EXT_VDD2_PIN, HIGH); }while(0)

  #endif

  #define RADDS_EXT_VDD_PIN                   25

#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                             0  // Analog Input
#define TEMP_1_PIN                             1  // Analog Input
#define TEMP_2_PIN                             2  // Analog Input
#define TEMP_3_PIN                             3  // Analog Input
#define TEMP_4_PIN                             5  // dummy so will compile when PINS_DEBUGGING is enabled
#define TEMP_BED_PIN                           4  // Analog Input

// SPI for MAX Thermocouple
#if !HAS_MEDIA
  #define TEMP_0_CS_PIN                       53
#else
  #define TEMP_0_CS_PIN                       49
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                          13
#define HEATER_1_PIN                          12
#define HEATER_2_PIN                          11
#if !HAS_CUTTER
  #define HEATER_BED_PIN                       7  // BED
#endif

#ifndef FAN0_PIN
  #define FAN0_PIN                             9
#endif
#define FAN1_PIN                               8

//
// Misc. Functions
//
#define SD_DETECT_PIN                         14
#define PS_ON_PIN                             40  // SERVO3_PIN

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      39  // SERVO2_PIN
#endif

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #if !NUM_SERVOS
    #define SPINDLE_LASER_PWM_PIN              5  // SERVO0_PIN
  #endif
  #define SPINDLE_LASER_ENA_PIN                7  // HEATER_BED_PIN - Pullup/down!
#endif

//
// LCD / Controller
//

#if HAS_WIRED_LCD

  #if ENABLED(RADDS_DISPLAY)

    #define LCD_PINS_RS                       42
    #define LCD_PINS_EN                       43
    #define LCD_PINS_D4                       44
    #define LCD_PINS_D5                       45
    #define LCD_PINS_D6                       46
    #define LCD_PINS_D7                       47

    #define BEEPER_PIN                        41

    #define BTN_EN1                           50
    #define BTN_EN2                           52
    #define BTN_ENC                           48

    #define BTN_BACK                          71

    #define SDSS                              10
    #define SD_DETECT_PIN                     14

  #elif IS_RRD_FG_SC

    // The REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER requires
    // an adapter such as https://www.thingiverse.com/thing:1740725

    #define LCD_PINS_RS                       42
    #define LCD_PINS_EN                       43
    #define LCD_PINS_D4                       44

    #define BEEPER_PIN                        41

    #define BTN_EN1                           50
    #define BTN_EN2                           52
    #define BTN_ENC                           48

    #define SDSS                              10
    #define SD_DETECT_PIN                     14

  #elif HAS_U8GLIB_I2C_OLED

    #define BTN_EN1                           50
    #define BTN_EN2                           52
    #define BTN_ENC                           48
    #define BEEPER_PIN                        41
    #define LCD_SDSS                          10
    #define SD_DETECT_PIN                     14

  #elif ENABLED(SPARK_FULL_GRAPHICS)

    #define LCD_PINS_D4                       29
    #define LCD_PINS_EN                       27
    #define LCD_PINS_RS                       25

    #define BTN_EN1                           35
    #define BTN_EN2                           33
    #define BTN_ENC                           37

  #endif // SPARK_FULL_GRAPHICS

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN                        47  // Detect the presence of the encoder
  #endif

#endif // HAS_WIRED_LCD

#ifndef SDSS
  #define SDSS                                 4
#endif
