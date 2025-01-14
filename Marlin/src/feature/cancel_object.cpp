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

#include "../inc/MarlinConfig.h"

#if ENABLED(CANCEL_OBJECTS)

#include "cancel_object.h"
#include "../gcode/gcode.h"
#include "../lcd/marlinui.h"

CancelObject cancelable;

cancel_state_t CancelObject::state;

void CancelObject::set_active_object(const int8_t obj) {
  state.active_object = obj;
  if (WITHIN(obj, 0, 31)) {
    if (obj >= state.object_count) state.object_count = obj + 1;
    state.skipping = TEST(state.canceled, obj);
  }
  else
    state.skipping = false;

  #if ALL(HAS_STATUS_MESSAGE, CANCEL_OBJECTS_REPORTING)
    if (state.active_object >= 0)
      ui.set_status(MString<30>(GET_TEXT_F(MSG_PRINTING_OBJECT), ' ', state.active_object));
    else
      ui.reset_status();
  #endif
}

void CancelObject::cancel_object(const int8_t obj) {
  if (WITHIN(obj, 0, 31)) {
    SBI(state.canceled, obj);
    if (obj == state.active_object) state.skipping = true;
  }
}

void CancelObject::uncancel_object(const int8_t obj) {
  if (WITHIN(obj, 0, 31)) {
    CBI(state.canceled, obj);
    if (obj == state.active_object) state.skipping = false;
  }
}

void CancelObject::report() {
  if (state.active_object >= 0)
    SERIAL_ECHO_MSG("Active Object: ", state.active_object);

  if (state.canceled == 0x0000) return;

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM("Canceled:");
  for (int i = 0; i < state.object_count; i++)
    if (TEST(state.canceled, i)) { SERIAL_CHAR(' '); SERIAL_ECHO(i); }
  SERIAL_EOL();
}

#endif // CANCEL_OBJECTS
