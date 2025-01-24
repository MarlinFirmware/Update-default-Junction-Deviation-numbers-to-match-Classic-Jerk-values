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
 * Alligator Board R2
 * https://reprap.org/wiki/Alligator_Board
 */

#include "env_validate.h"

#define BOARD_INFO_NAME    "Alligator Board R2"

//
// Servos
//
#define SERVO0_PIN                            36
#define SERVO1_PIN                            40
#define SERVO2_PIN                            41
#define SERVO3_PIN                            -1

//
// Limit Switches
//
#define X_MIN_PIN                             33  // PC1
#define X_MAX_PIN                             34  // PC2
#define Y_MIN_PIN                             35  // PC3
#define Y_MAX_PIN                             37  // PC5
#define Z_MIN_PIN                             38  // PC6
#define Z_MAX_PIN                             39  // PC7

//
// Steppers
//
#define X_STEP_PIN                            96  // PB24
#define X_DIR_PIN                              2  // PB25
#define X_ENABLE_PIN                          24  // PA15, motor RESET pin

#define Y_STEP_PIN                            94  // PB22
#define Y_DIR_PIN                             95  // PB23
#define Y_ENABLE_PIN                          24  // PA15, motor RESET pin

#define Z_STEP_PIN                            98  // PC27
#define Z_DIR_PIN                              3  // PC28
#define Z_ENABLE_PIN                          24  // PA15, motor RESET pin

#define E0_STEP_PIN                            5  // PC25
#define E0_DIR_PIN                             4  // PC26
#define E0_ENABLE_PIN                         24  // PA15, motor RESET pin

#define E1_STEP_PIN                           28  // PD3 on piggy
#define E1_DIR_PIN                            27  // PD2 on piggy
#define E1_ENABLE_PIN                         24  // PA15, motor RESET pin

#define E2_STEP_PIN                           11  // PD7 on piggy
#define E2_DIR_PIN                            29  // PD6 on piggy
#define E2_ENABLE_PIN                         24  // PA15, motor RESET pin

#define E3_STEP_PIN                           30  // PD9 on piggy
#define E3_DIR_PIN                            12  // PD8 on piggy
#define E3_ENABLE_PIN                         24  // PA15, motor RESET pin

// Microstepping pins - Mapping not from fastio.h (?)
#define X_MS1_PIN                             99  // PC10
#define Y_MS1_PIN                             10  // PC29
#define Z_MS1_PIN                             44  // PC19
#define E0_MS1_PIN                            45  // PC18

#ifndef MICROSTEP16
  #define MICROSTEP16 LOW,LOW,LOW
#endif
#ifndef MICROSTEP32
  #define MICROSTEP32 HIGH,HIGH,LOW
#endif

//#define MOTOR_FAULT_PIN                     22  // PB26 , motor X-Y-Z-E0 motor FAULT

//
// Temperature Sensors
//
#define TEMP_0_PIN                             1  // Analog Input (PA24)
#define TEMP_1_PIN                             2  // Analog Input (PA23 on piggy)
#define TEMP_2_PIN                             3  // Analog Input (PA22 on piggy)
#define TEMP_3_PIN                             4  // Analog Input (PA6 on piggy)
#define TEMP_BED_PIN                           0  // Analog Input (PA16)

//
// Heaters / Fans
//
// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN                          68  // PA1
#define HEATER_1_PIN                           8  // PC22 on piggy
#define HEATER_2_PIN                           9  // PC21 on piggy
#define HEATER_3_PIN                          97  // PC20 on piggy
#define HEATER_BED_PIN                        69  // PA0

#ifndef FAN0_PIN
  #define FAN0_PIN                            92  // PA5
#endif
#define FAN1_PIN                              31  // PA7

//
// Misc. Functions
//
#define SD_SS_PIN                             77  // PA28
#define SD_DETECT_PIN                         87  // PA29
#define LED_RED_PIN                           40  // PC8
#define LED_GREEN_PIN                         41  // PC9

#define EXP_VOLTAGE_LEVEL_PIN                 65

#define SPI_CHAN_DAC                           1

#define DAC0_SYNC_PIN                         53  // PB14
#define DAC1_SYNC_PIN                          6  // PC24

// 64K SPI EEPROM
#define SPI_EEPROM
#define SPI_CHAN_EEPROM1                       2
#define SPI_EEPROM1_CS_PIN                    25  // PD0

// 2K SPI EEPROM
#define SPI_EEPROM2_CS_PIN                    26  // PD1

// FLASH SPI
// 32Mb
#define SPI_FLASH_CS_PIN                      23  // PA14

//
// LCD / Controller
//

#if IS_RRD_FG_SC
  #define LCD_PINS_RS                         18
  #define LCD_PINS_EN                         15
  #define LCD_PINS_D4                         19
  #define BEEPER_PIN                          64
  #define UI_VOLTAGE_LEVEL                     1
#endif

#if IS_NEWPANEL
  #define BTN_EN1                             14
  #define BTN_EN2                             16
  #define BTN_ENC                             17
#endif
