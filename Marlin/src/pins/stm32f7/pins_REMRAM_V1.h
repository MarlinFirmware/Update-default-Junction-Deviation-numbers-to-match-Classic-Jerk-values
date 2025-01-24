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

#if NOT_TARGET(STM32F7xx)
  #error "Oops! Select an STM32F7 board in 'Tools > Board.'"
#endif

#define BOARD_INFO_NAME      "RemRam v1"
#define DEFAULT_MACHINE_NAME "RemRam"

#if NO_EEPROM_SELECTED
  #define SRAM_EEPROM_EMULATION                   // Emulate the EEPROM using Backup SRAM
#endif

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "RemRam only supports 1 hotend / E stepper."
#endif

//
// Limit Switches
//
#if DISABLED(SENSORLESS_HOMING)
  #define X_MIN_PIN                           58
  #define X_MAX_PIN                           59
  #define Y_MIN_PIN                           60
  #define Y_MAX_PIN                           61
  #define Z_MAX_PIN                           63
#else
  #define X_STOP_PIN                          36
  #define Y_STOP_PIN                          39
  #define Z_MAX_PIN                           42
#endif
#define Z_MIN_PIN                             62

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     26  // EXT_D1
#endif

//
// Steppers
//
#define X_STEP_PIN                            22
#define X_DIR_PIN                             35
#define X_ENABLE_PIN                          34
#define X_CS_PIN                              14

#define Y_STEP_PIN                            23
#define Y_DIR_PIN                             38
#define Y_ENABLE_PIN                          37
#define Y_CS_PIN                              15

#define Z_STEP_PIN                            24
#define Z_DIR_PIN                             41
#define Z_ENABLE_PIN                          40
#define Z_CS_PIN                              16

#define E0_STEP_PIN                           25
#define E0_DIR_PIN                            44
#define E0_ENABLE_PIN                         43
#define E0_CS_PIN                             10

//
// Temperature Sensors
//
#define TEMP_0_PIN                            64  // THERM_1
#define TEMP_1_PIN                            65  // THERM_2
#define TEMP_BED_PIN                          66  // THERM_3

//
// Heaters / Fans
//
#define HEATER_0_PIN                          33
#define HEATER_BED_PIN                        31

#ifndef FAN0_PIN
  #define FAN0_PIN                            30  // "FAN1"
#endif
#define FAN1_PIN                              32  // "FAN2"

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                     32
#endif

//
// Servos
//
#define SERVO0_PIN                            26  // PWM_EXT1
#define SERVO1_PIN                            27  // PWM_EXT2

#define SD_SS_PIN                             57  // Onboard SD card reader
//#define SD_SS_PIN                            9  // LCD SD card reader
#define LED_PIN                               21  // STATUS_LED

//
// LCD / Controller
//
#define SD_DETECT_PIN                         56  // SD_CARD_DET
#define BEEPER_PIN                            46  // LCD_BEEPER
#define LCD_PINS_RS                           49  // LCD_RS
#define LCD_PINS_EN                           48  // LCD_EN
#define LCD_PINS_D4                           50  // LCD_D4
#define LCD_PINS_D5                           51  // LCD_D5
#define LCD_PINS_D6                           52  // LCD_D6
#define LCD_PINS_D7                           53  // LCD_D7
#define BTN_EN1                               54  // BTN_EN1
#define BTN_EN2                               55  // BTN_EN2
#define BTN_ENC                               47  // BTN_ENC

//
// Timers
//

#define STEP_TIMER  2
