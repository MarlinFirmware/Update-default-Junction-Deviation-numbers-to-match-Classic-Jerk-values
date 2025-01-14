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

/**
 * mmu2_marlin_macros.h
 */

// This file will not be the same on Marlin1 and Marlin2.
// Its purpose is to unify different macros in either of Marlin incarnations.

#ifdef __AVR__
  #include "../../MarlinCore.h"
  // brings _O and _T macros into MMU
  #include "../../core/language.h"
  #include "../../gcode/gcode.h"
  // we don't have these in Marlin 2.x so just define them here again
  #define _O(x)                             x
  #define _T(x)                             x
  #define MARLIN_KEEPALIVE_STATE_IN_PROCESS KEEPALIVE_STATE(IN_PROCESS)
#elif defined(UNITTEST)
  #define _O(x)                             x
  #define _T(x)                             x
  #define MARLIN_KEEPALIVE_STATE_IN_PROCESS /*KEEPALIVE_STATE(IN_PROCESS) TODO*/
#else
  #include "../../gcode/gcode.h"
  #define _O(x)                             x
  #define _T(x)                             x
  #define MARLIN_KEEPALIVE_STATE_IN_PROCESS KEEPALIVE_STATE(IN_PROCESS)
#endif
