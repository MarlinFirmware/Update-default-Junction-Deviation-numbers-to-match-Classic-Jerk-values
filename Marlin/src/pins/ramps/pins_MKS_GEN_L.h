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
 * MKS GEN L – Arduino Mega2560 with RAMPS v1.4 pin assignments
 * Schematic: https://github.com/makerbase-mks/MKS-GEN_L/blob/master/hardware/MKS%20Gen_L%20V1.0_008/MKS%20Gen_L%20V1.0_008%20SCH.pdf
 * Pin Diagram: https://github.com/makerbase-mks/MKS-GEN_L/blob/master/hardware/MKS%20Gen_L%20V1.0_008/MKS%20Gen_L%20V1.0_008%20PIN.pdf
 * Note: Schematic contains pin & connector errors, correct pin mappings are found on the pin diagram
 * ATmega2560
 */

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS GEN L supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "MKS GEN L"

//
// Heaters / Fans
//

#ifndef MOSFET_A_PIN
  #define MOSFET_A_PIN                        10  // HE0
#endif
#ifndef MOSFET_B_PIN
  #define MOSFET_B_PIN                         7  // HE1 or FAN Hotend Cooling
#endif
#ifndef MOSFET_C_PIN
  #define MOSFET_C_PIN                         8  // H-BED
#endif
#ifndef MOSFET_D_PIN
  #define MOSFET_D_PIN                         9  // FAN Part Cooling
#endif

// Always assign these to their labeled counterparts
#define HEATER_0_PIN                MOSFET_A_PIN
#if ANY(FET_ORDER_EEF, FET_ORDER_EEB)
  #define HEATER_1_PIN              MOSFET_B_PIN
#endif
#define HEATER_BED_PIN              MOSFET_C_PIN

#define FAN0_PIN                    MOSFET_D_PIN
#if ENABLED(FET_ORDER_EFF)
  #define FAN1_PIN                  MOSFET_B_PIN
#endif

//
// CS Pins wired to avoid conflict with the LCD
// See https://www.thingiverse.com/asset:66604
//

#ifndef X_CS_PIN
  #define X_CS_PIN                            59
#endif
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            63
#endif

#include "pins_RAMPS.h"
