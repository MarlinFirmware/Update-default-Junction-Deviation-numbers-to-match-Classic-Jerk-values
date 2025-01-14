/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../module/endstops.h"

// One ISR for all EXT-Interrupts
void endstop_ISR() { endstops.update(); }

void setup_endstop_interrupts() {
  #define _ATTACH(P) attachInterrupt(digitalPinToInterrupt(P), endstop_ISR, CHANGE)
  TERN_(USE_X_MAX, _ATTACH(X_MAX_PIN));
  TERN_(USE_X_MIN, _ATTACH(X_MIN_PIN));
  TERN_(USE_Y_MAX, _ATTACH(Y_MAX_PIN));
  TERN_(USE_Y_MIN, _ATTACH(Y_MIN_PIN));
  TERN_(USE_Z_MAX, _ATTACH(Z_MAX_PIN));
  TERN_(HAS_Z_MIN_PIN, _ATTACH(Z_MIN_PIN));
  TERN_(USE_X2_MAX, _ATTACH(X2_MAX_PIN));
  TERN_(USE_X2_MIN, _ATTACH(X2_MIN_PIN));
  TERN_(USE_Y2_MAX, _ATTACH(Y2_MAX_PIN));
  TERN_(USE_Y2_MIN, _ATTACH(Y2_MIN_PIN));
  TERN_(USE_Z2_MAX, _ATTACH(Z2_MAX_PIN));
  TERN_(USE_Z2_MIN, _ATTACH(Z2_MIN_PIN));
  TERN_(USE_Z3_MAX, _ATTACH(Z3_MAX_PIN));
  TERN_(USE_Z3_MIN, _ATTACH(Z3_MIN_PIN));
  TERN_(USE_Z4_MAX, _ATTACH(Z4_MAX_PIN));
  TERN_(USE_Z4_MIN, _ATTACH(Z4_MIN_PIN));
  TERN_(USE_Z_MIN_PROBE_PIN, _ATTACH(Z_MIN_PROBE_PIN));
  TERN_(USE_I_MAX, _ATTACH(I_MAX_PIN));
  TERN_(USE_I_MIN, _ATTACH(I_MIN_PIN));
  TERN_(USE_J_MAX, _ATTACH(J_MAX_PIN));
  TERN_(USE_J_MIN, _ATTACH(J_MIN_PIN));
  TERN_(USE_K_MAX, _ATTACH(K_MAX_PIN));
  TERN_(USE_K_MIN, _ATTACH(K_MIN_PIN));
  TERN_(USE_U_MAX, _ATTACH(U_MAX_PIN));
  TERN_(USE_U_MIN, _ATTACH(U_MIN_PIN));
  TERN_(USE_V_MAX, _ATTACH(V_MAX_PIN));
  TERN_(USE_V_MIN, _ATTACH(V_MIN_PIN));
  TERN_(USE_W_MAX, _ATTACH(W_MAX_PIN));
  TERN_(USE_W_MIN, _ATTACH(W_MIN_PIN));
}
