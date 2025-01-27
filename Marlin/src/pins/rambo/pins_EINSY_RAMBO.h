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
 * Einsy-Rambo pin assignments
 * Schematic: https://github.com/ultimachine/Einsy-Rambo/blob/1.1a/board/Project%20Outputs/Schematic%20Prints_Einsy%20Rambo_1.1a.PDF
 */

#include "env_validate.h"

#define BOARD_INFO_NAME       "Einsy Rambo"
#define DEFAULT_MACHINE_NAME  "Prusa MK3"

//#define MK3_FAN_PINS

//
// TMC2130 Configuration_adv defaults for EinsyRambo
//
#if (HAS_X_AXIS && !AXIS_DRIVER_TYPE_X(TMC2130)) || (HAS_Y_AXIS && !AXIS_DRIVER_TYPE_Y(TMC2130)) || (HAS_Z_AXIS && !AXIS_DRIVER_TYPE_Z(TMC2130)) || (HAS_EXTRUDERS && !AXIS_DRIVER_TYPE_E0(TMC2130))
  #error "For EinsyRambo you must set all *_DRIVER_TYPE to TMC2130 in Configuration.h."
#endif

// TMC2130 Diag Pins (currently just for reference)
#define X_DIAG_PIN                            64
#define Y_DIAG_PIN                            69
#define Z_DIAG_PIN                            68
#define E0_DIAG_PIN                           65

//
// Limit Switches
//
// Only use Diag Pins when SENSORLESS_HOMING is enabled for the TMC2130 drivers.
// Otherwise use a physical endstop based configuration.
//
// SERVO0_PIN and Z_MIN_PIN configuration for BLTOUCH sensor when combined with SENSORLESS_HOMING.
//

#if DISABLED(SENSORLESS_HOMING)

  #define X_STOP_PIN                          12
  #define Y_STOP_PIN                          11
  #define Z_STOP_PIN                          10

#else

  #define X_STOP_PIN                  X_DIAG_PIN
  #define Y_STOP_PIN                  Y_DIAG_PIN

  #if ENABLED(BLTOUCH)
    #define Z_STOP_PIN                        11  // Y-MIN
    #define SERVO0_PIN                        10  // Z-MIN
  #else
    #define Z_STOP_PIN                        10
  #endif

#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     10
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      62
#endif

//
// Steppers
//
#define X_STEP_PIN                            37
#define X_DIR_PIN                             49
#define X_ENABLE_PIN                          29
#define X_CS_PIN                              41

#define Y_STEP_PIN                            36
#define Y_DIR_PIN                             48
#define Y_ENABLE_PIN                          28
#define Y_CS_PIN                              39

#define Z_STEP_PIN                            35
#define Z_DIR_PIN                             47
#define Z_ENABLE_PIN                          27
#define Z_CS_PIN                              67

#define E0_STEP_PIN                           34
#define E0_DIR_PIN                            43
#define E0_ENABLE_PIN                         26
#define E0_CS_PIN                             66

//
// Temperature Sensors
//
#define TEMP_0_PIN                             0  // Analog Input, Header J2
#define TEMP_1_PIN                             1  // Analog Input, Header J3
#define TEMP_BED_PIN                           2  // Analog Input, Header J6
#define TEMP_PROBE_PIN                         3  // Analog Input, Header J15

#ifndef TEMP_BOARD_PIN
  #define TEMP_BOARD_PIN                      91  // Onboard thermistor, 100k TDK NTCG104LH104JT1
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                           3
#define HEATER_BED_PIN                         4

#ifndef FAN0_PIN
  #ifdef MK3_FAN_PINS
    #define FAN0_PIN                           6
  #else
    #define FAN0_PIN                           8
  #endif
#endif

#ifndef FAN1_PIN
  #ifdef MK3_FAN_PINS
    #define FAN1_PIN                          -1
  #else
    #define FAN1_PIN                           6
  #endif
#endif

/**
 *             ------                           ------                           ------
 *     84 PH2 | 1  2 | PH6  9          50 MISO | 1  2 | SCK  52          62 PK0 | 1  2 | PJ5 76
 *     61 PF7 | 3  4 | PD5 82          72  PJ2 | 3  4 | SDSS 77          20 SDA | 3  4 | GND
 *     59 PF5 | 5  6   PG4 70          14  TX3 | 5  6   MOSI 51          21 SCL | 5  6   RX2 16
 *     85 PH7 | 7  8 | PG3 71          15  RX3 | 7  8 | RESET               GND | 7  8 | TX2 17
 *        GND | 9 10 | 5V                  GND | 9 10 | PE3   5              5V | 9 10 | 5V
 *             ------                           ------                           ------
 *               P1                               P2                               P3
 */

#define EXP1_01_PIN                           84
#define EXP1_02_PIN                            9
#define EXP1_03_PIN                           61
#define EXP1_04_PIN                           82
#define EXP1_05_PIN                           59
#define EXP1_06_PIN                           70
#define EXP1_07_PIN                           85
#define EXP1_08_PIN                           71

#define EXP2_01_PIN                           50
#define EXP2_02_PIN                           52
#define EXP2_03_PIN                           72
#define EXP2_04_PIN                           77
#define EXP2_05_PIN                           14
#define EXP2_06_PIN                           51
#define EXP2_07_PIN                           15
#define EXP2_08_PIN                           -1

//
// Misc. Functions
//
#define SD_SS_PIN                    EXP2_04_PIN
#define LED_PIN                               13

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN             EXP1_02_PIN
#endif

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  // Use P1 connector for spindle pins
  #define SPINDLE_LASER_PWM_PIN      EXP1_02_PIN  // Hardware PWM
  #define SPINDLE_LASER_ENA_PIN               18  // Pullup!
  #define SPINDLE_DIR_PIN                     19
#endif

//
// Průša i3 MK2 Multiplexer Support
//
#if HAS_PRUSA_MMU1
  #define E_MUX0_PIN                          17
  #define E_MUX1_PIN                          16
  #define E_MUX2_PIN                          78  // 84 in MK2 Firmware, with BEEPER as 78
#endif

//
// LCD / Controller
//

#if HAS_WIRED_LCD || TOUCH_UI_ULTIPANEL

  #define KILL_PIN                            32

  #if IS_ULTIPANEL || TOUCH_UI_ULTIPANEL

    #if ENABLED(CR10_STOCKDISPLAY)
      #define LCD_PINS_RS            EXP1_07_PIN
      #define LCD_PINS_EN            EXP1_08_PIN
      #define LCD_PINS_D4            EXP1_06_PIN
      #define BTN_EN1                EXP1_03_PIN
      #define BTN_EN2                EXP1_05_PIN
    #else
      #define LCD_PINS_RS            EXP1_04_PIN
      #define LCD_PINS_EN            EXP1_03_PIN
      #define LCD_PINS_D4            EXP1_05_PIN
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN
      #define BTN_EN1                EXP2_05_PIN
      #define BTN_EN2                EXP2_03_PIN

      #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
        #define BTN_ENC_EN           LCD_PINS_D7  // Detect the presence of the encoder
      #endif

    #endif

    #define BTN_ENC                  EXP1_02_PIN  // P1
    #define BEEPER_PIN               EXP1_01_PIN  // P1
    #define SD_DETECT_PIN            EXP2_07_PIN

  #endif // IS_ULTIPANEL || TOUCH_UI_ULTIPANEL
#endif // HAS_WIRED_LCD

#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1                 0
  #define BOARD_ST7920_DELAY_2               250
  #define BOARD_ST7920_DELAY_3                 0
#endif

#undef MK3_FAN_PINS
