/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * feature/cooler.cpp
 */

#include "../inc/MarlinConfig.h"

#if ANY(HAS_COOLER, LASER_COOLANT_FLOW_METER)

#include "cooler.h"
Cooler cooler;

#if HAS_COOLER
  uint8_t Cooler::mode = 0;
  uint16_t Cooler::capacity;
  uint16_t Cooler::load;
  bool Cooler::enabled = false;
#endif

#if ENABLED(LASER_COOLANT_FLOW_METER)
  bool Cooler::flowmeter = false;
  millis_t Cooler::flowmeter_next_ms; // = 0
  volatile uint16_t Cooler::flowpulses;
  float Cooler::flowrate;
  #if ENABLED(FLOWMETER_SAFETY)
    bool Cooler::flowsafety_enabled = true;
    bool Cooler::flowfault = false;
  #endif
#endif

#endif // HAS_COOLER || LASER_COOLANT_FLOW_METER
