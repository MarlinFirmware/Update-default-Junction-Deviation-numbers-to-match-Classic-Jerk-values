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
 * Einsy-Retro pin assignments
 * Schematic (1.0b): https://github.com/ultimachine/EinsyRetro/blob/master/board/Project%20Outputs/Schematic%20Prints_EinsyRetro_1.0b.PDF
 * Schematic (1.0c): https://github.com/ultimachine/EinsyRetro/blob/master/board/Project%20Outputs/Schematic%20Prints_EinsyRetro_1.0c.PDF
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Einsy Retro"

//
// TMC2130 Configuration_adv defaults for EinsyRetro
//
#if !AXIS_DRIVER_TYPE_X(TMC2130) || !AXIS_DRIVER_TYPE_Y(TMC2130) || !AXIS_DRIVER_TYPE_Z(TMC2130) || !AXIS_DRIVER_TYPE_E0(TMC2130)
  #error "You must set ([XYZ]|E0)_DRIVER_TYPE to TMC2130 in Configuration.h for EinsyRetro."
#endif

// TMC2130 Diag Pins
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

  #define X_MIN_PIN                           12  // X-
  #define Y_MIN_PIN                           11  // Y-
  #define X_MAX_PIN                           81  // X+
  #define Y_MAX_PIN                           57  // Y+

#else

  #if X_HOME_TO_MIN
    #define X_MIN_PIN                 X_DIAG_PIN
    #define X_MAX_PIN                         81  // X+
  #else
    #define X_MIN_PIN                         12  // X-
    #define X_MAX_PIN                 X_DIAG_PIN
  #endif

  #if Y_HOME_TO_MIN
    #define Y_MIN_PIN                 Y_DIAG_PIN
    #define Y_MAX_PIN                         57  // Y+
  #else
    #define Y_MIN_PIN                         11  // Y-
    #define Y_MAX_PIN                 Y_DIAG_PIN
  #endif

  #if ENABLED(BLTOUCH)
    #define Z_MIN_PIN                         11  // Y-
    #define SERVO0_PIN                        10  // Z-
  #endif

#endif

#define Z_MAX_PIN                              7
#ifndef Z_MIN_PIN
  #define Z_MIN_PIN                           10  // Z-
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     10
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
#define TEMP_0_PIN                             0  // Analog Input
#define TEMP_1_PIN                             1  // Analog Input
#define TEMP_BED_PIN                           2  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           3
#define HEATER_BED_PIN                         4

#ifndef FAN0_PIN
  #define FAN0_PIN                             8
#endif
#define FAN1_PIN                               6

//
// Misc. Functions
//
#define SD_SS_PIN                             53
#define LED_PIN                               13

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                       9
#endif

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  // Use P1 connector for spindle pins
  #define SPINDLE_LASER_PWM_PIN                9  // Hardware PWM
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
// EXP Headers
//
#define EXP1_01_PIN                           84  // PH2
#define EXP1_02_PIN                            9  // PH6
#define EXP1_03_PIN                           18  // TX1
#define EXP1_04_PIN                           82  // PD5
#define EXP1_05_PIN                           19  // RX1
#define EXP1_06_PIN                           70  // PG4
#define EXP1_07_PIN                           85  // PH7
#define EXP1_08_PIN                           71  // PG3

#define EXP2_01_PIN                           50  // MISO
#define EXP2_02_PIN                           52  // SCK
#define EXP2_03_PIN                           72  // PJ2
#define EXP2_04_PIN                           53  // SDSS
#define EXP2_05_PIN                           14  // TX3
#define EXP2_06_PIN                           51  // MOSI
#define EXP2_07_PIN                           15  // RX3
#define EXP2_08_PIN                           -1  // RESET

#define EXP3_01_PIN                           62  // PK0 (A8)
#define EXP3_02_PIN                           76  // PJ5
#define EXP3_03_PIN                           20  // SDA
#define EXP3_04_PIN                           -1  // GND
#define EXP3_05_PIN                           21  // SCL
#define EXP3_06_PIN                           16  // RX2
#define EXP3_07_PIN                           -1  // GND
#define EXP3_08_PIN                           17  // TX2

//
// LCD / Controller
//

#if ANY(HAS_WIRED_LCD, TOUCH_UI_ULTIPANEL, TOUCH_UI_FTDI_EVE)

  #define KILL_PIN                            32

  #if ANY(IS_ULTIPANEL, TOUCH_UI_ULTIPANEL, TOUCH_UI_FTDI_EVE)

    #if ENABLED(CR10_STOCKDISPLAY)
      #define LCD_PINS_RS            EXP1_07_PIN
      #define LCD_PINS_EN            EXP1_08_PIN
      #define LCD_PINS_D4            EXP1_06_PIN
      #define BTN_EN1                EXP1_03_PIN
      #define BTN_EN2                EXP1_05_PIN
    #else
      #define LCD_PINS_RS            EXP1_04_PIN
      #define LCD_PINS_EN            EXP1_03_PIN  // On 0.6b, use 61
      #define LCD_PINS_D4            EXP1_05_PIN  // On 0.6b, use 59
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN
      #define BTN_EN1                EXP2_05_PIN
      #define BTN_EN2                EXP2_03_PIN
    #endif

    #define BTN_ENC                  EXP1_02_PIN  // AUX-2
    #define BEEPER_PIN               EXP1_01_PIN  // AUX-4

    #define SD_DETECT_PIN            EXP2_07_PIN

    #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
      #define BTN_ENC_EN             LCD_PINS_D7  // Detect the presence of the encoder
    #endif

  #endif // IS_ULTIPANEL || TOUCH_UI_ULTIPANEL || TOUCH_UI_FTDI_EVE

#endif // HAS_WIRED_LCD || TOUCH_UI_ULTIPANEL || TOUCH_UI_FTDI_EVE

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1                 0
  #define BOARD_ST7920_DELAY_2               250
  #define BOARD_ST7920_DELAY_3                 0
#endif
