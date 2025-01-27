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
 * Arduino Mega with RAMPS v1.4 adjusted pin assignments
 * Schematic (1.4): https://github.com/makerbase-mks/MKS-GEN/blob/master/hardware/MKS%20GEN%20V1.4_004/MKS%20GEN%20V1.4_004%20SCH.pdf
 * ATmega2560, ATmega1280
 *
 *  MKS GEN v1.3  (Extruder, Fan, Bed)
 *  MKS GEN v1.3  (Extruder, Extruder, Fan, Bed)
 *  MKS GEN v1.4  (Extruder, Fan, Bed)
 *  MKS GEN v1.4  (Extruder, Extruder, Fan, Bed)
 */

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS GEN 1.3/1.4 supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "MKS GEN >= v1.3"

//
// Heaters / Fans
//
#define MOSFET_B_PIN                           7
#define FAN0_PIN                               9

//
// PSU / SERVO
//
// If PSU_CONTROL is specified, always hijack Servo 3
//
#if ENABLED(PSU_CONTROL)
  #define SERVO3_PIN                          -1
  #define PS_ON_PIN                            4
#endif

#include "pins_RAMPS.h"

#undef EXP2_08_PIN
#define EXP2_08_PIN                           -1  // RESET

//
// LCD / Controller
//

#if ANY(VIKI2, miniVIKI)
  /**
   * VIKI2 Has two groups of wires with...
   *
   * +Vin     + Input supply, requires 120ma for LCD and mSD card
   * GND      Ground Pin
   * MOSI     Data input for LCD and SD
   * MISO     Data output for SD
   * SCK      Clock for LCD and SD
   * AO       Reg. Sel for LCD
   * LCS      Chip Select for LCD
   * SDCS     Chip Select for SD
   * SDCD     Card Detect pin for SD
   * ENCA     Encoder output A
   * ENCB     Encoder output B
   * ENCBTN   Encoder button switch
   *
   * BTN      Panel mounted button switch
   * BUZZER   Piezo buzzer
   * BLUE-LED Blue LED ring pin (3 to 5v, mosfet buffered)
   * RED-LED  Red LED ring pin (3 to 5v, mosfet buffered)
   *
   * This configuration uses the following arrangement:
   *
   *             ------                       ------
   *       ENCB | 1  2 |  ENCA          MISO | 1  2 | SCK
   *   BLUE_LED | 3  4 |  RED_LED     ENCBTN | 3  4 | SDCS
   *       KILL   5  6 |  BEEPER               5  6 | MOSI
   *         A0 | 7  8 |  LCD_CS        SDCD | 7  8 |
   *        GND | 9 10 | 5V              GND | 9 10 | --
   *             ------                       ------
   *              EXP1                         EXP2
   */
  #undef SD_DETECT_PIN
  #undef BTN_EN1
  #undef BTN_EN2
  #undef BTN_ENC
  #undef DOGLCD_A0
  #undef DOGLCD_CS
  #undef BEEPER_PIN
  #undef KILL_PIN
  #undef STAT_LED_RED_PIN
  #undef STAT_LED_BLUE_PIN

  //
  // VIKI2 12-wire lead
  //
  #define SD_DETECT_PIN              EXP2_07_PIN  // SDCD     orange/white
  #define BTN_EN1                    EXP1_02_PIN  // ENCA     white
  #define BTN_EN2                    EXP1_01_PIN  // ENCB     green
  #define BTN_ENC                    EXP2_03_PIN  // ENCBTN   purple
  #define DOGLCD_A0                  EXP1_07_PIN  // A0       brown
  #define DOGLCD_CS                  EXP1_08_PIN  // LCS      green/white

  // EXP2_01_PIN  gray   MISO
  // EXP2_06_PIN  yellow MOSI
  // EXP2_02_PIN  orange SCK

  //#define SD_SS_PIN                EXP2_04_PIN  // SDCS     blue

  //
  // VIKI2 4-wire lead
  //
  #define KILL_PIN                   EXP1_05_PIN  // BTN      blue
  #define BEEPER_PIN                 EXP1_06_PIN  // BUZZER   green
  #define STAT_LED_RED_PIN           EXP1_04_PIN  // RED-LED  yellow
  #define STAT_LED_BLUE_PIN          EXP1_03_PIN  // BLUE-LED white
#endif
