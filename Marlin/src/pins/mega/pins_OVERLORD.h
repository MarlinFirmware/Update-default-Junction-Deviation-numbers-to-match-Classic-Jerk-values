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
 * Dreammaker Overlord v1.1 pin assignments
 * Schematic: https://github.com/jdpiercy/Overlord-Pro/blob/master/Motherboard/Schematic.pdf
 * ATmega2560
 */

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Overlord supports up to 2 hotends / E steppers."
#endif

#include "env_validate.h"

#define BOARD_INFO_NAME         "OVERLORD"
#define DEFAULT_MACHINE_NAME    BOARD_INFO_NAME

//
// Limit Switches
//
#define X_STOP_PIN                            24
#define Y_STOP_PIN                            28
#define Z_MIN_PIN                             46
#define Z_MAX_PIN                             32

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     46  // JP4, Tfeed1
#endif

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      44  // JP3, Tfeed2
#endif

//
// Steppers
//
#define X_STEP_PIN                            25
#define X_DIR_PIN                             23
#define X_ENABLE_PIN                          27

#define Y_STEP_PIN                            31
#define Y_DIR_PIN                             33
#define Y_ENABLE_PIN                          29

#define Z_STEP_PIN                            37
#define Z_DIR_PIN                             39
#define Z_ENABLE_PIN                          35

#define E0_STEP_PIN                           43
#define E0_DIR_PIN                            45
#define E0_ENABLE_PIN                         41

#define E1_STEP_PIN                           49
#define E1_DIR_PIN                            47
#define E1_ENABLE_PIN                         48

//
// Temperature Sensors
//
#define TEMP_0_PIN                             8  // Analog Input
#define TEMP_1_PIN                             9  // Analog Input - Redundant temp sensor
#define TEMP_2_PIN                            12  // Analog Input
#define TEMP_3_PIN                            14  // Analog Input
#define TEMP_BED_PIN                          10  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           2
#define HEATER_1_PIN                           3
#define HEATER_BED_PIN                         4

#define FAN0_PIN                               7  // material cooling fan

//
// SD Card
//
#define SD_SS_PIN                             53
#define SD_DETECT_PIN                         38

//
// Misc. Functions
//
#define LED_PIN                               13  // On PCB status led
#define PS_ON_PIN                             12  // For stepper/heater/fan power. Active HIGH.
#define POWER_LOSS_PIN                        34  // Power check - whether hotends/steppers/fans have power

#if ENABLED(BATTERY_STATUS_AVAILABLE)
  #undef BATTERY_STATUS_PIN
  #define BATTERY_STATUS_PIN                  26  // Status of power loss battery, whether it is charged (low) or charging (high)
#endif
#if ENABLED(INPUT_VOLTAGE_AVAILABLE)
  #undef VOLTAGE_DETECTION_PIN
  #define VOLTAGE_DETECTION_PIN               11  // Analog Input - ADC Voltage level of main input
#endif

//
// LCD / Controller
//

#if HAS_WIRED_LCD
  // OVERLORD OLED pins
  #define LCD_PINS_RS                         20
  #define LCD_PINS_D5                         21
  #define LCD_PINS_EN                         15
  #define LCD_PINS_D4                         14
  #define LCD_PINS_D6                          5
  #define LCD_PINS_D7                          6
  #ifndef LCD_RESET_PIN
    #define LCD_RESET_PIN                      5  // LCD_PINS_D6
  #endif
#endif

#if IS_NEWPANEL
  #define BTN_ENC                             16  // Enter Pin
  #define BTN_UP                              19  // Button UP Pin
  #define BTN_DOWN                            17  // Button DOWN Pin
#endif

// Additional connectors/pins on the Overlord V1.X board
#define PCB_VERSION_PIN                       22
#define APPROACH_PIN                          11  // JP7, Tpd
#define GATE_PIN                              36  // Threshold, JP6, Tg
