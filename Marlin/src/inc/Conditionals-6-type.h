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
 * Conditionals-6-type.h
 * Internal defines that depend on Configurations and Pins but are not user-editable.
 * Define conditionals in this file if they depend on core/types.h.
 */

//========================================================
// Get requirements for the benefit of IntelliSense, etc.
//
#include "MarlinConfigPre-6-type.h"
//========================================================

#ifdef GITHUB_ACTIONS
  // Extras for CI testing
#endif

// If an axis's Homing Current differs from standard current...
#define HAS_CURRENT_HOME(N) (N##_CURRENT_HOME > 0 && N##_CURRENT_HOME != N##_CURRENT)

// Does any axis have homing current?
#define _OR_HAS_CURR_HOME(N) HAS_CURRENT_HOME(N) ||
#if MAIN_AXIS_MAP(_OR_HAS_CURR_HOME) MAP(_OR_HAS_CURR_HOME, X2, Y2, Z2, Z3, Z4) 0
  #define HAS_HOMING_CURRENT 1
#endif
#undef _OR_HAS_CURR_HOME
