/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * JOKER pin assignments
 */

#include "env_validate.h"

#define BOARD_INFO_NAME      "JOKER"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

//
// Expansion Boards for more axes, sensors, heaters
// Only enable one of these options.
//
//#define JOKER_PLUS_2
//#define JOKER_PLUS_5

#if ENABLED(JOKER_PLUS_2)
  #define MAX_EXTRUDERS 2
  #if E_STEPPERS > 2
    #error "JOKER with +2 expansion supports up to 2 E steppers."
  #elif HOTENDS > 2
    #error "JOKER with +2 expansion supports up to 2 hotends."
  #endif
#elif ENABLED(JOKER_PLUS_5)
  #define MAX_EXTRUDERS 3
  #if E_STEPPERS > 3
    #error "JOKER with +5 expansion supports up to 3 E steppers."
  #elif HOTENDS > 3
    #error "JOKER with +5 expansion supports up to 3 hotends."
  #endif
#else
  #if E_STEPPERS > 1
    #error "JOKER without expansion only supports 1 E stepper."
  #elif HAS_MULTI_HOTEND
    #error "JOKER without expansion only supports 1 hotend."
  #endif
#endif

//
// Servos
//
#define SERVO0_PIN                            14

//
// Limit Switches
//
#define X_STOP_PIN                            15
#define Y_STOP_PIN                            16
#define Z_STOP_PIN                            17

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      13
#endif

#ifndef Z_MIN_PROBE_PIN
  //#define Z_MIN_PROBE_PIN                   35  // Pin 35 is the RAMPS default
#endif

//
// Enable I2S stepper stream
//
#ifndef I2S_STEPPER_STREAM
  #define I2S_STEPPER_STREAM
#endif
#if ENABLED(I2S_STEPPER_STREAM)
  #define I2S_WS                              26
  #define I2S_BCK                             25
  #define I2S_DATA                            27
  #if ENABLED(LIN_ADVANCE)
    #error "I2S stream is currently incompatible with LIN_ADVANCE."
  #endif
#endif

//
// Steppers
//
#define X_STEP_PIN                           128
#define X_DIR_PIN                            129
#define X_ENABLE_PIN                         136

#define Y_STEP_PIN                           130
#define Y_DIR_PIN                            131
#define Y_ENABLE_PIN                X_ENABLE_PIN

#define Z_STEP_PIN                           132
#define Z_DIR_PIN                            133
#define Z_ENABLE_PIN                         137

#define E0_STEP_PIN                          134
#define E0_DIR_PIN                           135
#define E0_ENABLE_PIN                        138

#if ENABLED(JOKER_PLUS_2)

  #define E1_STEP_PIN                        144
  #define E1_DIR_PIN                         145
  #define E1_ENABLE_PIN                      146

  #define E2_STEP_PIN                        147
  #define E2_DIR_PIN                         148
  #define E2_ENABLE_PIN                      149

#elif ENABLED(JOKER_PLUS_5)

  #define E1_STEP_PIN                        144
  #define E1_DIR_PIN                         145
  #define E1_ENABLE_PIN                      146

  #define E2_STEP_PIN                        153
  #define E2_DIR_PIN                         154
  #define E2_ENABLE_PIN                      155

  #define E3_STEP_PIN                        156
  #define E3_DIR_PIN                         157
  #define E3_ENABLE_PIN                      158

  // Are these preferred for X2, Y2, Z2, Z3 ?
  #define E4_STEP_PIN                        147
  #define E4_DIR_PIN                         148
  #define E4_ENABLE_PIN                      149

  #define E5_STEP_PIN                        150
  #define E5_DIR_PIN                         151
  #define E5_ENABLE_PIN                      152

#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                            36  // Analog Input
#define TEMP_BED_PIN                          39  // Analog Input

#if ENABLED(JOKER_PLUS_2)
  #define TEMP_1_PIN                          34  // Analog Input
#elif ENABLED(JOKER_PLUS_5)
  #define TEMP_1_PIN                          32  // Analog Input
  #define TEMP_2_PIN                          33  // Analog Input
  #define TEMP_3_PIN                          34  // Analog Input
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                         141
#define HEATER_BED_PIN                       140
#define FAN0_PIN                             143

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                    142  // Enabled in Configuration_adv.h
#endif

#ifndef CONTROLLER_FAN_PIN
  //#define CONTROLLER_FAN_PIN                -1
#endif

#if ENABLED(JOKER_PLUS_2)

  #define HEATER_1_PIN                       150
  #define FAN1_PIN                           151

#elif ENABLED(JOKER_PLUS_5)

  #define HEATER_1_PIN                       159
  #define HEATER_2_PIN                       160
  #define HEATER_3_PIN                       161

  #define FAN1_PIN                           162
  #define FAN2_PIN                           163
  #define FAN3_PIN                           164

  #ifndef E1_AUTO_FAN_PIN
    #define E1_AUTO_FAN_PIN                  165
  #endif
  #ifndef E2_AUTO_FAN_PIN
    #define E2_AUTO_FAN_PIN                  166
  #endif
  #ifndef E3_AUTO_FAN_PIN
    #define E3_AUTO_FAN_PIN                  167
  #endif

#endif

//
// SD Card
//
#define SD_MOSI_PIN                           23
#define SD_MISO_PIN                           19
#define SD_SCK_PIN                            18
#define SD_SS_PIN                              5
#define USES_SHARED_SPI                           // SPI is shared by SD card with TMC SPI drivers

//
// LCD / Controller
//

#if HAS_WIRED_LCD

  //#define LCD_PINS_RS                       13
  //#define LCD_PINS_EN                       17
  //#define LCD_PINS_D4                       16

  #if ENABLED(CR10_STOCKDISPLAY)

    #define BEEPER_PIN                       151

  #elif IS_RRD_FG_SC

    #define BEEPER_PIN                       151

    //#define LCD_PINS_D5                     -1
    //#define LCD_PINS_D6                     -1
    //#define LCD_PINS_D7                     -1

  #endif

  #define BTN_EN1                              2
  #define BTN_EN2                              4
  #define BTN_ENC                             12
  #define BEEPER_PIN                         139

#endif // HAS_WIRED_LCD

/**
 * Hardware Serial
 * Add the following to Configuration.h or Configuration_adv.h to assign
 * specific pins to hardware Serial1 and Serial2.
 * Note: Serial2 can be defined using HARDWARE_SERIAL2_RX and HARDWARE_SERIAL2_TX but
 * JOKER does not have enough spare pins for such reassignment.
 */
//#define HARDWARE_SERIAL1_RX                 21
//#define HARDWARE_SERIAL1_TX                 22
//#define HARDWARE_SERIAL2_RX                  2
//#define HARDWARE_SERIAL2_TX                  4

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #define SPINDLE_LASER_ENA_PIN               -1  // FET 1
  #define SPINDLE_LASER_PWM_PIN               33  // Bed FET
  //#define SPINDLE_DIR_PIN                   -1  // FET 3
#endif
