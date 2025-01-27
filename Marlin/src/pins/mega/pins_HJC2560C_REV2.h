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
 * Geeetech HJC2560-C Rev 2.x board pin assignments
 * ATmega2560
 */

#include "env_validate.h"

#define DEFAULT_MACHINE_NAME "ADIMLab Gantry v2"
#define BOARD_INFO_NAME      "HJC2560-C"

//
// Servos
//
//#ifndef SERVO0_PIN
//  #define SERVO0_PIN       11
//#endif

//
// Limit Switches
//
#define X_STOP_PIN                            22
#define Y_STOP_PIN                            26
#define Z_STOP_PIN                            29
//#define EXP_STOP_PIN                        28

//
// Steppers
//
#define X_STEP_PIN                            25
#define X_DIR_PIN                             23
#define X_ENABLE_PIN                          27

#define Y_STEP_PIN                            32
#define Y_DIR_PIN                             33
#define Y_ENABLE_PIN                          31

#define Z_STEP_PIN                            35
#define Z_DIR_PIN                             36
#define Z_ENABLE_PIN                          34

#define E0_STEP_PIN                           42
#define E0_DIR_PIN                            43
#define E0_ENABLE_PIN                         37

#define E1_STEP_PIN                           49
#define E1_DIR_PIN                            47
#define E1_ENABLE_PIN                         48

#define MOTOR_CURRENT_PWM_XY_PIN              44
#define MOTOR_CURRENT_PWM_Z_PIN               45
#define MOTOR_CURRENT_PWM_E_PIN               46
// Motor current PWM conversion, PWM value = MotorCurrentSetting * 255 / range
#ifndef MOTOR_CURRENT_PWM_RANGE
  #define MOTOR_CURRENT_PWM_RANGE            2000
#endif
#define DEFAULT_PWM_MOTOR_CURRENT  { 1300, 1300, 1250 }

//
// Temperature Sensors
//
#define TEMP_0_PIN                             8  // Analog Input
#define TEMP_1_PIN                             9  // Analog Input
#define TEMP_BED_PIN                          10  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           2
#define HEATER_1_PIN                           3
#define HEATER_BED_PIN                         4

#ifndef FAN0_PIN
  #define FAN0_PIN                             7  //默认不使用PWM_FAN冷却喷嘴，如果需要，则取消注释
#endif

//
// Misc. Functions
//
#define SD_SS_PIN                             53
#define SD_DETECT_PIN                         39
//#define LED_PIN                              8

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                       8  // 8 默认挤出机风扇作为Case LED，如果需要PWM FAN,则需要将FAN_PIN置为7，LED_PIN置为8
#endif

//#define SAFETY_TRIGGERED_PIN                28  // PIN to detect the safety circuit has triggered
//#define MAIN_VOLTAGE_MEASURE_PIN            14  // ANALOG PIN to measure the main voltage, with a 100k - 4k7 resitor divider.

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #define SPINDLE_LASER_PWM_PIN                9  // Hardware PWM
  #define SPINDLE_LASER_ENA_PIN               17  // Pin should have a pullup!
  #define SPINDLE_DIR_PIN                     16
#endif

//
// LCD / Controller
//

#if HAS_WIRED_LCD

  #define BEEPER_PIN                          18

  #if IS_NEWPANEL

    #define LCD_PINS_RS                       20  // LCD_CS
    #define LCD_PINS_EN                       15  // LCD_SDA
    #define LCD_PINS_D4                       14  // LCD_SCK

    #if ENABLED(HJC_LCD_SMART_CONTROLLER)
      #define LCD_BACKLIGHT_PIN                5  // LCD_Backlight
      //#ifndef LCD_CONTRAST_PIN
      //  #define LCD_CONTRAST_PIN             5  // LCD_Contrast
      //#endif
      #ifndef FIL_RUNOUT_PIN
        #define FIL_RUNOUT_PIN                24  // Filament runout
      #endif
    #else
      #define LCD_PINS_D5                     21
      #define LCD_PINS_D6                      5
      #define LCD_PINS_D7                      6
    #endif

    #define BTN_EN1                           41
    #define BTN_EN2                           40
    #define BTN_ENC                           19

    #define SD_DETECT_PIN                     39

  #else

    // Buttons attached to a shift register
    #define SHIFT_CLK_PIN                     38
    #define SHIFT_LD_PIN                      42
    #define SHIFT_OUT_PIN                     40
    #define SHIFT_EN_PIN                      17

    #define LCD_PINS_RS                       16
    #define LCD_PINS_EN                        5
    #define LCD_PINS_D4                        6
    #define LCD_PINS_D5                       21
    #define LCD_PINS_D6                       20
    #define LCD_PINS_D7                       19

  #endif // !IS_NEWPANEL

#endif // HAS_WIRED_LCD
