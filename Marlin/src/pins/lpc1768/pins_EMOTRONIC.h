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
 * eMotion-Tech eMotronic pin assignments
 * Schematic: https://data.emotion-tech.com/ftp/Datasheets_et_sources/Sources/eMotronic_brd_sources_1.0.4.zip
 *
 * Board pins<->features assignments are based on the
 * Micro-Delta Rework printer default connections.
 */

#include "env_validate.h"

#define BOARD_INFO_NAME   "eMotronic"
#define BOARD_WEBSITE_URL "www.reprap-france.com/article/lemotronic-quesaco"

//
// Limit Switches
//
#define X_STOP_PIN                         P1_22  // S0
#define Y_STOP_PIN                         P1_23  // S1
#define Z_STOP_PIN                         P1_24  // S2
#define I_STOP_PIN                         P1_25  // S3

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                  P0_26  // S4 (opto)
#endif

//
// Steppers
//
#define X_STEP_PIN                         P2_02  // M3
#define X_DIR_PIN                          P4_28
#define X_ENABLE_PIN                       P4_29

#define Y_STEP_PIN                         P2_01  // M2
#define Y_DIR_PIN                          P0_22
#define Y_ENABLE_PIN                       P0_21

#define Z_STEP_PIN                         P2_00  // M1
#define Z_DIR_PIN                          P3_26
#define Z_ENABLE_PIN                       P3_25

#define E0_STEP_PIN                        P2_03  // M4
#define E0_DIR_PIN                         P0_04
#define E0_ENABLE_PIN                      P0_05  // Correct!

#define E1_STEP_PIN                        P2_08  // M5
#define E1_DIR_PIN                         P0_20
#define E1_ENABLE_PIN                      P0_19

//
// Temperature Sensors
// 3.3V max when defined as an analog input
//
#define TEMP_0_PIN                      P0_24_A1  // A1 (TH1)
#define TEMP_1_PIN                      P0_23_A0  // A0 (TH0)
#define TEMP_BED_PIN                    P0_25_A2  // A2 (TH2)

//
// Heaters / Fans
//
#define _H0_PIN                            P2_04  // (H0: 15A)

#define HEATER_BED_PIN                     P2_05  // (H1: 10A shared)
#define HEATER_0_PIN                       P2_06  // (H2: 10A shared)
#define HEATER_1_PIN                       P2_07  // (H3: 10A shared)

#define FAN0_PIN                           P2_11  // (FAN0: 1A)
#define FAN1_PIN                           P2_13  // (FAN1: 1A)

//
// Extension ports
//
#define EXP1_01_PIN                        P0_03  // RX0
#define EXP1_02_PIN                        P0_02  // TX0
#define EXP1_03_PIN                        P1_30
#define EXP1_04_PIN                        P1_31
#define EXP1_05_PIN                        P0_17  // MISO0
#define EXP1_06_PIN                        P0_18  // MOSI0
#define EXP1_07_PIN                        P0_15  // SCK0
#define EXP1_08_PIN                        P0_16  // SSEL0
#define EXP1_09_PIN                        P0_27  // SDA0
#define EXP1_10_PIN                        P0_28  // SCL0

#define EXP2_01_PIN                        P0_10
#define EXP2_02_PIN                        P0_11
#define EXP2_03_PIN                        P0_00  // SDA1
#define EXP2_04_PIN                        P0_01  // SCL1
#define EXP2_05_PIN                        P1_28
#define EXP2_06_PIN                        P1_29
#define EXP2_07_PIN                        P1_26
#define EXP2_08_PIN                        P1_27

//
// SD Support
//
#ifndef VOLUME0
  #define VOLUME0                        ONBOARD
#endif

#if ANY_VOLUME_IS(ONBOARD)
  #define SD_SCK_PIN                       P0_07
  #define SD_MISO_PIN                      P0_08
  #define SD_MOSI_PIN                      P0_09
  #define SD_SS_PIN                        P0_06
#elif ANY_VOLUME_IS(LCD)
  #define SD_SCK_PIN                 EXP1_07_PIN
  #define SD_MISO_PIN                EXP1_05_PIN
  #define SD_MOSI_PIN                EXP1_06_PIN
  #define SD_SS_PIN                  EXP1_08_PIN
  #define SD_DETECT_PIN              EXP1_04_PIN
#elif ANY_VOLUME_IS(CUSTOM)
  #error "No custom SD pins defined for this board."
#endif

//
// LCD / Controller
//

#if ENABLED(EMOTION_TECH_LCD)
  #define BEEPER_PIN                 EXP2_01_PIN
  #define DOGLCD_A0                  EXP2_06_PIN
  #define DOGLCD_CS                  EXP2_04_PIN
  #define DOGLCD_SCK                 EXP1_07_PIN
  #define DOGLCD_MOSI                EXP1_06_PIN

  #define BTN_EN1                    EXP2_07_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define BTN_ENC                    EXP2_02_PIN
#endif
