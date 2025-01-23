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
 * ReprapWorld ULTRATRONICS v1.0
 * https://reprapworld.com/documentation/datasheet_ultratronics10_05.pdf
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Ultratronics v1.0"

//
// Servos
//
#if NUM_SERVOS > 0
  #define SERVO0_PIN                          11
  #if NUM_SERVOS > 1
    #define SERVO1_PIN                        12
  #endif
#endif

//
// Limit Switches
//
#define X_MIN_PIN                             31
#define X_MAX_PIN                             30
#define Y_MIN_PIN                             12
#define Y_MAX_PIN                             11
#define Z_MIN_PIN                             29
#define Z_MAX_PIN                             28

//
// Steppers
//
#define X_STEP_PIN                            35
#define X_DIR_PIN                             34
#define X_ENABLE_PIN                          37
#ifndef X_CS_PIN
  #define X_CS_PIN                            18
#endif

#define Y_STEP_PIN                            22
#define Y_DIR_PIN                             23
#define Y_ENABLE_PIN                          33
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            19
#endif

#define Z_STEP_PIN                            25
#define Z_DIR_PIN                             26
#define Z_ENABLE_PIN                          24
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            16
#endif

#define E0_STEP_PIN                           47
#define E0_DIR_PIN                            46
#define E0_ENABLE_PIN                         48
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           17
#endif

#define E1_STEP_PIN                           44
#define E1_DIR_PIN                            36
#define E1_ENABLE_PIN                         45
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           -1
#endif

#define E2_STEP_PIN                           42
#define E2_DIR_PIN                            41
#define E2_ENABLE_PIN                         43
#ifndef E2_CS_PIN
  #define E2_CS_PIN                           -1
#endif

#define E3_STEP_PIN                           39
#define E3_DIR_PIN                            38
#define E3_ENABLE_PIN                         40
#ifndef E3_CS_PIN
  #define E3_CS_PIN                           -1
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                             0  // Analog Input
#define TEMP_1_PIN                             2  // Analog Input
#define TEMP_2_PIN                             3  // Analog Input
#define TEMP_3_PIN                             4  // Analog Input
#define TEMP_BED_PIN                           1  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           3
#define HEATER_1_PIN                           8
#define HEATER_2_PIN                           7
#define HEATER_3_PIN                           9
#define HEATER_BED_PIN                         2

#ifndef FAN0_PIN
  #define FAN0_PIN                             6
#endif
#define FAN2_PIN                               5

//
// Misc. Functions
//
#define SD_SS_PIN                             59
#define SD_DETECT_PIN                         60
#define LED_PIN                               13
#define PS_ON_PIN                             32

//
// SPI Buses
//

#define DAC0_SYNC_PIN                         53  // PB14
#define SPI_CHAN_DAC                           1

#define SPI_CHAN_EEPROM1                      -1
#define SPI_EEPROM1_CS_PIN                    -1
#define SPI_EEPROM2_CS_PIN                    -1
#define SPI_FLASH_CS_PIN                      -1

#define SD_SCK_PIN                            76
#define SD_MISO_PIN                           74
#define SD_MOSI_PIN                           75

// SPI for MAX Thermocouple
#define TEMP_0_CS_PIN                         65
#define TEMP_1_CS_PIN                         52
#define TEMP_2_CS_PIN                         50
#define TEMP_3_CS_PIN                         51  // Not yet supported

#define ENC424_SS                             61

//
// LCD / Controller
//

#define BEEPER_PIN                            27

#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

  #define LCD_PINS_RS                         A8  // CS chip select / SS chip slave select
  #define LCD_PINS_EN                       MOSI  // SID (MOSI)
  #define LCD_PINS_D4                        SCK  // SCK (CLK) clock

  #define BTN_EN1                             20
  #define BTN_EN2                             21
  #define BTN_ENC                             64

#endif // REPRAPWORLD_GRAPHICAL_LCD
