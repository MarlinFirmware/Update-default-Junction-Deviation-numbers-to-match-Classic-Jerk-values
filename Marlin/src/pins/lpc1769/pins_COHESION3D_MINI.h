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
 * Cohesion3D Mini pin assignments
 * Schematic: https://lasergods.com/cohesion3d-mini-pinout-diagram/
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Cohesion3D Mini"

//
// Servos
//
#define SERVO0_PIN                         P1_23

//
// Limit Switches
//
#define X_MIN_PIN                          P1_24  // 10k pullup to 3.3V
#define X_MAX_PIN                          P1_25  // 10k pullup to 3.3V
#define Y_MIN_PIN                          P1_26  // 10k pullup to 3.3V
#define Y_MAX_PIN                          P1_27  // 10k pullup to 3.3V
#define Z_MIN_PIN                          P1_28  // 10k pullup to 3.3V
#define Z_MAX_PIN                          P1_29  // 10k pullup to 3.3V

//
// Steppers
//
#define X_STEP_PIN                         P2_00
#define X_DIR_PIN                          P0_05
#define X_ENABLE_PIN                       P0_04
#define X_CS_PIN                           P1_10  // Ethernet Expansion - Pin 9

#define Y_STEP_PIN                         P2_01
#define Y_DIR_PIN                          P0_11
#define Y_ENABLE_PIN                       P0_10
#define Y_CS_PIN                           P1_09  // Ethernet Expansion - Pin 10

#define Z_STEP_PIN                         P2_02
#define Z_DIR_PIN                          P0_20
#define Z_ENABLE_PIN                       P0_19
#define Z_CS_PIN                           P1_00  // Ethernet Expansion - Pin 11

#define E0_STEP_PIN                        P2_03
#define E0_DIR_PIN                         P0_22
#define E0_ENABLE_PIN                      P0_21
#define E0_CS_PIN                          P1_04  // Ethernet Expansion - Pin 12

//
// Default pins for TMC software SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                     P1_16  // Ethernet Expansion - Pin 5
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                     P1_17  // Ethernet Expansion - Pin 6
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                      P1_08  // Ethernet Expansion - Pin 7
#endif

//
// Analog Inputs
//  3.3V max when defined as an analog input
//
#define TEMP_0_PIN                      P0_23_A0  // P0_23
#define TEMP_BED_PIN                    P0_24_A1  // P0_24

//
// Heaters / Fans
//
#define HEATER_BED_PIN                     P2_05
#define HEATER_0_PIN                       P2_07  // FET 1
#ifndef FAN0_PIN
  #define FAN0_PIN                         P2_06  // FET 3
#endif

//
// Auto fans
//
#define AUTO_FAN_PIN                       P2_04  // FET 4
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E1_AUTO_FAN_PIN
  #define E1_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E2_AUTO_FAN_PIN
  #define E2_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif

//
// Misc. Functions
//
#define LED_PIN                            P4_28  // Play LED

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #undef HEATER_0_PIN
  #define SPINDLE_LASER_ENA_PIN            P2_07  // FET 1
  #undef HEATER_BED_PIN
  #define SPINDLE_LASER_PWM_PIN            P2_05  // Bed FET
  #undef FAN0_PIN
  #define SPINDLE_DIR_PIN                  P2_06  // FET 3
#endif

//
// LCD / Controller
//
// LCD_PINS_D5, D6, and D7 are not present in the EXP1 connector, and will need to be
// defined to use the REPRAP_DISCOUNT_SMART_CONTROLLER.
//
// A remote SD card is currently not supported because the pins routed to the EXP2
// connector are shared with the onboard SD card, and Marlin does not support reading
// G-code files from the onboard SD card.
//
#if HAS_WIRED_LCD

  #define BEEPER_PIN                       P0_27  // EXP2-7 - open drain

  #define BTN_EN1                          P3_26  // EXP2-5
  #define BTN_EN2                          P3_25  // EXP2-3
  #define BTN_ENC                          P1_30  // EXP1-2

  #define LCD_PINS_RS                      P0_16  // EXP1-4
  #define LCD_SDSS_PIN                     P0_28  // EXP2-4
  #define LCD_PINS_EN                      P0_18  // EXP1-3
  #define LCD_PINS_D4                      P0_15  // EXP1-5

  #define KILL_PIN                         P2_11  // EXP2-10

  #if HAS_MEDIA
    #error "SDSUPPORT is not currently supported by the Cohesion3D boards"
  #endif

#endif // HAS_WIRED_LCD

//
// Ethernet pins
//
#define ENET_MDIO                          P1_17
#define ENET_RX_ER                         P1_14
#define ENET_RXD1                          P1_10
#define ENET_MOC                           P1_16
#define REF_CLK                            P1_15
#define ENET_RXD0                          P1_09
#define ENET_CRS                           P1_08
#define ENET_TX_EN                         P1_04
#define ENET_TXD0                          P1_00
#define ENET_TXD1                          P1_01
