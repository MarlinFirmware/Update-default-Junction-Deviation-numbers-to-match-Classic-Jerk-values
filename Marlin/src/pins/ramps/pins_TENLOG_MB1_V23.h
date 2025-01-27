/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Tenlog MB1 V2.3 pin assignments
 * ATmega2560
 */

#define REQUIRE_MEGA2560
#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Tenlog supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME      "Tenlog MB1 V2.3"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

//
// Limit Switches
//
#define X_MIN_PIN                              3
#define X_MAX_PIN                              2
#define Y_MIN_PIN                             14
//#define Y_MAX_PIN                           15  // Connected to "DJ" plug on extruder heads
#define Z_MIN_PIN                             18
#if ENABLED(BLTOUCH)
  #define SERVO0_PIN                          19
#else
  #define Z_MAX_PIN                           19
#endif

//
// Steppers
//
#define X_STEP_PIN                            54
#define X_DIR_PIN                             55
#define X_ENABLE_PIN                          38

#define X2_STEP_PIN                           36
#define X2_DIR_PIN                            34
#define X2_ENABLE_PIN                         30

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56

#define Z_STEP_PIN                            46
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          62

#define Z2_STEP_PIN                           65
#define Z2_DIR_PIN                            66
#define Z2_ENABLE_PIN                         64

#define E0_STEP_PIN                           57
#define E0_DIR_PIN                            58
#define E0_ENABLE_PIN                         59

#define E1_STEP_PIN                           26
#define E1_DIR_PIN                            28
#define E1_ENABLE_PIN                         24

//
// Temperature Sensors
//
#define TEMP_0_PIN                            15  // Analog Input
#define TEMP_1_PIN                            13  // Analog Input
#define TEMP_BED_PIN                          14  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                          11
#define HEATER_1_PIN                          10
#define HEATER_BED_PIN                         8

#define FAN0_PIN                               9
#define FAN2_PIN                               5  // Normally this would be a servo pin

//#define NUM_RUNOUT_SENSORS                   0
#define FIL_RUNOUT_PIN                        15
//#define FIL_RUNOUT2_PIN                     21

//
// PSU and Powerloss Recovery
//
#if ENABLED(PSU_CONTROL)
  #define PS_ON_PIN                           40  // The M80/M81 PSU pin for boards v2.1-2.3
#endif

//
// Misc. Functions
//
//#define CASE_LIGHT_PIN                       5
//#ifndef LED_PIN
//  #define LED_PIN                           13
//#endif

#if HAS_CUTTER
  //#define SPINDLE_LASER_PWM_PIN             -1  // Hardware PWM
  //#define SPINDLE_LASER_ENA_PIN              4  // Pullup!
#endif

// Use the RAMPS 1.4 Analog input 5 on the AUX2 connector
//#define FILWIDTH_PIN                         5  // Analog Input

#define SD_SS_PIN                             53
#define SD_DETECT_PIN                         49

//
// LCD / Controller
//

//#if IS_RRD_SC

//#ifndef BEEPER_PIN
//  #define BEEPER_PIN                        -1
//#endif

#define LCD_PINS_RS                           -1
#define LCD_PINS_EN                           -1
#define LCD_PINS_D4                           -1
#define LCD_PINS_D5                           -1
#define LCD_PINS_D6                           -1
#define LCD_PINS_D7                           -1

//#define BTN_EN1                             31
//#define BTN_EN2                             33
//#define BTN_ENC                             35

//#ifndef KILL_PIN
//  #define KILL_PIN                          41
//#endif

//#endif // IS_RRD_SC
