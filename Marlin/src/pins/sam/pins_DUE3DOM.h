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
 * DUE3DOM pin assignments
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "DUE3DOM"

//
// Servos
//
#define SERVO0_PIN                             5
#define SERVO1_PIN                             6
#define SERVO2_PIN                            13
#define SERVO3_PIN                            -1

//
// Limit Switches
//
#define X_MIN_PIN                             38
#define X_MAX_PIN                             36
#define Y_MIN_PIN                             34
#define Y_MAX_PIN                             32
#define Z_MIN_PIN                             30
#define Z_MAX_PIN                             28

//
// Steppers
//
#define X_STEP_PIN                             2
#define X_DIR_PIN                              3
#define X_ENABLE_PIN                          22

#define Y_STEP_PIN                            17
#define Y_DIR_PIN                             16
#define Y_ENABLE_PIN                          26

#define Z_STEP_PIN                            61  // Z1 STP
#define Z_DIR_PIN                             60  // Z1 DIR
#define Z_ENABLE_PIN                          15  // Z1 ENA

#define E0_STEP_PIN                           64  // Z2 STP
#define E0_DIR_PIN                            63  // Z2 DIR
#define E0_ENABLE_PIN                         62  // Z2 ENA

#define E1_STEP_PIN                           51  // E1 STP
#define E1_DIR_PIN                            53  // E1 DIR
#define E1_ENABLE_PIN                         65  // E1 ENA

#define E2_STEP_PIN                           24  // E2 STP
#define E2_DIR_PIN                            23  // E2 DIR
#define E2_ENABLE_PIN                         49  // E2 ENA

//
// Temperature Sensors
//
#define TEMP_0_PIN                             0  // Analog Input (HOTEND0 thermistor)
#define TEMP_1_PIN                             2  // Analog Input (HOTEND1 thermistor)
#define TEMP_2_PIN                             5  // Analog Input (unused)
#define TEMP_BED_PIN                           1  // Analog Input (BED thermistor)

// SPI for MAX Thermocouple
#if !HAS_MEDIA
  #define TEMP_0_CS_PIN                       -1
#else
  #define TEMP_0_CS_PIN                       -1
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                           7  // HOTEND0 MOSFET
#define HEATER_1_PIN                           8  // HOTEND1 MOSFET
#define HEATER_BED_PIN                        39  // BED MOSFET

#ifndef FAN0_PIN
  #define FAN0_PIN                            11  // FAN1 header on board - PRINT FAN
#endif
#define FAN1_PIN                               9  // FAN2 header on board - CONTROLLER FAN
#define FAN2_PIN                              12  // FAN3 header on board - EXTRUDER0 FAN

//
// Misc. Functions
//
#define SD_SS_PIN                              4
#define PS_ON_PIN                             40

//
// LCD / Controller
//

#if HAS_WIRED_LCD

  #define LCD_PINS_RS                         42
  #define LCD_PINS_EN                         43
  #define LCD_PINS_D4                         44
  #define LCD_PINS_D5                         45
  #define LCD_PINS_D6                         46
  #define LCD_PINS_D7                         47

  #if IS_RRD_SC

    #define BEEPER_PIN                        41

    #define BTN_EN1                           50
    #define BTN_EN2                           52
    #define BTN_ENC                           48

    #define SD_DETECT_PIN                     14

  #elif ENABLED(RADDS_DISPLAY)

    #define BEEPER_PIN                        41

    #define BTN_EN1                           50
    #define BTN_EN2                           52
    #define BTN_ENC                           48

    #define BTN_BACK                          71

    #define SD_DETECT_PIN                     14

  #elif HAS_U8GLIB_I2C_OLED

    #define BTN_EN1                           50
    #define BTN_EN2                           52
    #define BTN_ENC                           48
    #define BEEPER_PIN                        41
    #define LCD_SDSS_PIN                       4
    #define SD_DETECT_PIN                     14

  #elif ENABLED(SPARK_FULL_GRAPHICS)

    #define LCD_PINS_D4                       29
    #define LCD_PINS_EN                       27
    #define LCD_PINS_RS                       25

    #define BTN_EN1                           35
    #define BTN_EN2                           33
    #define BTN_ENC                           37

    #define BEEPER_PIN                        -1
  #endif // SPARK_FULL_GRAPHICS

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

#endif // HAS_WIRED_LCD
