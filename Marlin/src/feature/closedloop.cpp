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

/**
 * feature/closedloop.cpp
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)

#if !PIN_EXISTS(CLOSED_LOOP_ENABLE) || !PIN_EXISTS(CLOSED_LOOP_MOVE_COMPLETE)
  #error "CLOSED_LOOP_ENABLE_PIN and CLOSED_LOOP_MOVE_COMPLETE_PIN are required for EXTERNAL_CLOSED_LOOP_CONTROLLER."
#endif

#include "closedloop.h"

ClosedLoop closedloop;

void ClosedLoop::init() {
  OUT_WRITE(CLOSED_LOOP_ENABLE_PIN, LOW);
  SET_INPUT_PULLUP(CLOSED_LOOP_MOVE_COMPLETE_PIN);
}

void ClosedLoop::set(const byte val) {
  OUT_WRITE(CLOSED_LOOP_ENABLE_PIN, val);
}

#endif // EXTERNAL_CLOSED_LOOP_CONTROLLER
