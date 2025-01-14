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
 * FLYmaker FLY-CDY pin assignments
 * Schematic: https://github.com/Mellow-3D/FLY-CDY/blob/master/Motherboard%20information/FLY_CDY%20SCH.pdf
 */

#include "env_validate.h"

#define BOARD_INFO_NAME   "FLY-CDY"
#define BOARD_WEBSITE_URL "github.com/FLYmaker/FLY-CDY"

//
// Servos
//
#define SERVO0_PIN                         P1_26

//
// Limit Switches
//

#define X_MIN_PIN                          P1_29  // X-
#define X_MAX_PIN                          P1_28  // X+
#define Y_MIN_PIN                          P1_27  // Y-
#define Y_MAX_PIN                          P1_25  // Y+
#define Z_MIN_PIN                          P1_22  // Z-
#define Z_MAX_PIN                          P0_27  // Z+

//
// Steppers
//
#define X_STEP_PIN                         P2_00
#define X_DIR_PIN                          P1_01
#define X_ENABLE_PIN                       P1_00
#ifndef X_CS_PIN
  #define X_CS_PIN                         P1_04
#endif

#define Y_STEP_PIN                         P2_01
#define Y_DIR_PIN                          P1_09
#define Y_ENABLE_PIN                       P1_08
#ifndef Y_CS_PIN
  #define Y_CS_PIN                         P1_10
#endif

#define Z_STEP_PIN                         P2_02
#define Z_DIR_PIN                          P1_15
#define Z_ENABLE_PIN                       P1_14
#ifndef Z_CS_PIN
  #define Z_CS_PIN                         P1_16
#endif

#define E0_STEP_PIN                        P2_03
#define E0_DIR_PIN                         P4_29
#define E0_ENABLE_PIN                      P1_17
#ifndef E0_CS_PIN
  #define E0_CS_PIN                        P4_28
#endif

#define E1_STEP_PIN                        P2_04
#define E1_DIR_PIN                         P2_11
#define E1_ENABLE_PIN                      P0_04
#ifndef E1_CS_PIN
  #define E1_CS_PIN                        P2_12
#endif

#define E2_STEP_PIN                        P2_05
#define E2_DIR_PIN                         P0_11
#define E2_ENABLE_PIN                      P2_13
#ifndef E2_CS_PIN
  #define E2_CS_PIN                        P0_10
#endif

//
// Default pins for TMC software SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                     P0_20
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                     P0_19
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                      P0_21
#endif

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                  P1_04
  #define Y_SERIAL_TX_PIN                  P1_10
  #define Z_SERIAL_TX_PIN                  P1_16
  #define E0_SERIAL_TX_PIN                 P4_28
  #define E1_SERIAL_TX_PIN                 P2_12
  #define E2_SERIAL_TX_PIN                 P0_10

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
//
#define TEMP_0_PIN                      P0_26_A3  // (T4)
#define TEMP_1_PIN                      P0_25_A2  // (T3)
#define TEMP_2_PIN                      P0_24_A1  // (T2)
#define TEMP_BED_PIN                    P0_23_A0  // (T1)

//
// Heaters / Fans
//
#define HEATER_BED_PIN                     P3_26
#define HEATER_0_PIN                       P3_25
#define HEATER_1_PIN                       P1_20
#define HEATER_2_PIN                       P1_23
#ifndef FAN0_PIN
  #define FAN0_PIN                         P1_18
#endif
#define FAN1_PIN                           P1_21
#define FAN2_PIN                           P1_24

//
// LCD / Controller
//
#define BEEPER_PIN                         P2_07
#define LCD_PINS_RS                        P2_10
#define LCD_PINS_EN                        P0_22
#define LCD_PINS_D4                        P1_19
#define LCD_PINS_D5                        P2_08
#define LCD_PINS_D6                        P1_30
#define LCD_PINS_D7                        P1_31
#define BTN_EN1                            P0_00
#define BTN_EN2                            P0_01
#define BTN_ENC                            P0_28

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
    #define SD_SS_PIN                      P0_06
    #define SD_SCK_PIN                     P0_07
    #define SD_MISO_PIN                    P0_08
    #define SD_MOSI_PIN                    P0_09
    #define SD_DETECT_PIN                  P0_05
#elif SD_CONNECTION_IS(LCD)
  #define SD_SCK_PIN                       P0_15
  #define SD_MISO_PIN                      P0_17
  #define SD_MOSI_PIN                      P0_18
  #define SD_SS_PIN                        P0_16
  #define SD_DETECT_PIN                    P2_06
#endif
