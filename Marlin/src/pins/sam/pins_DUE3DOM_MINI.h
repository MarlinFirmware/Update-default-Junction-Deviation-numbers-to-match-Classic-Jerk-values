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
 * DUE3DOM MINI pin assignments
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "DUE3DOM MINI"

//
// Servos
//
#define SERVO0_PIN                             5
#define SERVO1_PIN                             6
#define SERVO2_PIN                             8  // 4-pin header FAN0
#define SERVO3_PIN                            -1

//
// Limit Switches
//
#define X_STOP_PIN                            38
#define Y_STOP_PIN                            34
#define Z_STOP_PIN                            30

//
// Steppers
//
#define X_STEP_PIN                            17
#define X_DIR_PIN                             16
#define X_ENABLE_PIN                          22

#define Y_STEP_PIN                             2
#define Y_DIR_PIN                              3
#define Y_ENABLE_PIN                          26

#define Z_STEP_PIN                            64
#define Z_DIR_PIN                             63
#define Z_ENABLE_PIN                          15

#define E0_STEP_PIN                           61
#define E0_DIR_PIN                            60
#define E0_ENABLE_PIN                         62

//
// Temperature Sensors
//
#define TEMP_0_PIN                             0  // Analog Input (HOTEND0 thermistor)
#define TEMP_1_PIN                             2  // Analog Input (unused)
#define TEMP_BED_PIN                           1  // Analog Input (BED thermistor)

#ifndef TEMP_BOARD_PIN
  #define TEMP_BOARD_PIN                       5  // Analog Input (OnBoard thermistor beta 3950)
#endif

// SPI for MAX Thermocouple
#if !HAS_MEDIA
  #define TEMP_0_CS_PIN                       53
#else
  #define TEMP_0_CS_PIN                       53
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                          13  // HOTEND0 MOSFET
#define HEATER_BED_PIN                         7  // BED MOSFET

#ifndef FAN0_PIN
  #define FAN0_PIN                            11  // FAN1 header on board - PRINT FAN
#endif
#define FAN1_PIN                              12  // FAN2 header on board - CONTROLLER FAN
#define FAN2_PIN                               9  // FAN3 header on board - EXTRUDER0 FAN
//#define FAN3_PIN                             8  // FAN0 4-pin header on board

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

    #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
      #define BTN_ENC_EN             LCD_PINS_D7  // Detect the presence of the encoder
    #endif

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
    #define LCD_SDSS_PIN               SD_SS_PIN
    #define SD_DETECT_PIN                     14

  #elif ENABLED(SPARK_FULL_GRAPHICS)

    #define LCD_PINS_D4                       29
    #define LCD_PINS_EN                       27
    #define LCD_PINS_RS                       25

    #define BTN_EN1                           35
    #define BTN_EN2                           33
    #define BTN_ENC                           37

    #define BEEPER_PIN                        -1

  #elif ENABLED(MINIPANEL)

    #define BTN_EN1                           52
    #define BTN_EN2                           50
    #define BTN_ENC                           48
    #define LCD_SDSS_PIN               SD_SS_PIN
    #define SD_DETECT_PIN                     14
    #define BEEPER_PIN                        41
    #define DOGLCD_A0                         46
    #define DOGLCD_CS                         45

  #endif

#endif // HAS_WIRED_LCD
