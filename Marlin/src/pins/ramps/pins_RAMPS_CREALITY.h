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

// ATmega2560

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Creality RAMPS supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "Creality3D RAMPS"

//#define CR2020_INDUSTRIAL_SERIES                // Use layout specific to CR2020

//
// 4-pin expansion header
//
#define EXP1_PIN                              65  // A11 - Used by CR2020 Industrial series for case
#define EXP2_PIN                              66  // A12
#define EXP3_PIN                              11  // RAMPS: SERVO0_PIN
#define EXP4_PIN                              12  // RAMPS: PS_ON_PIN

//
// Servos
//
#define SERVO1_PIN                            12

//
// Heaters / Fans
//
#define MOSFET_B_PIN                           7
#define FAN0_PIN                               9

//
// Filament Runout Sensor
//
#define FIL_RUNOUT_PIN                         2
#if NUM_RUNOUT_SENSORS >= 2
  #define FIL_RUNOUT2_PIN                     15  // Creality CR-X can use dual runout sensors
#endif

//
// Misc. Functions
//
#ifdef CR2020_INDUSTRIAL_SERIES
  #if ENABLED(PSU_CONTROL) && !defined(PS_ON_PIN)
    #define PS_ON_PIN                         40  // Used by CR2020 Industrial series
  #endif
  #ifndef SUICIDE_PIN
    #define SUICIDE_PIN                       12  // Used by CR2020 Industrial series
  #endif
  #ifndef SUICIDE_PIN_STATE
    #define SUICIDE_PIN_STATE               HIGH
  #endif
#endif

#if ENABLED(CASE_LIGHT_ENABLE) && !defined(CASE_LIGHT_PIN)
  #define CASE_LIGHT_PIN                      65
#endif

//
// SD Card
//
#ifndef SD_DETECT_PIN
  #if ANY_VOLUME_IS(ONBOARD)
    //#define HAS_ONBOARD_SD_DETECT               // If the SD_DETECT_PIN is wired up
  #endif
  #if ENABLED(HAS_ONBOARD_SD_DETECT) || !ANY_VOLUME_IS(ONBOARD)
    #define SD_DETECT_PIN                     49
  #endif
#endif

//
// Based on RAMPS 1.4
//
#include "pins_RAMPS.h"

//
// LCD / Controller
//
#ifndef BEEPER_PIN
  #define BEEPER_PIN                          37  // Always define beeper pin so Play Tone works with ExtUI
#endif
