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
 * feature/tramming.cpp
 */

#include "../inc/MarlinConfigPre.h"

#if ENABLED(ASSISTED_TRAMMING)

#include "tramming.h"

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../core/debug_out.h"

#define _TRAM_NAME_DEF(N) PGMSTR(point_name_##N, TRAMMING_POINT_NAME_##N);
#define _TRAM_NAME_ITEM(N) point_name_##N
REPEAT_1(_NR_TRAM_NAMES, _TRAM_NAME_DEF)

PGM_P const tramming_point_name[] PROGMEM = { REPLIST_1(_NR_TRAM_NAMES, _TRAM_NAME_ITEM) };

#ifdef ASSISTED_TRAMMING_WAIT_POSITION

  // Move to the defined wait position
  void move_to_tramming_wait_pos() {
    constexpr xyz_pos_t wait_pos = ASSISTED_TRAMMING_WAIT_POSITION;
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Moving away");
    do_blocking_move_to(wait_pos, XY_PROBE_FEEDRATE_MM_S);
  }

#endif

#endif // ASSISTED_TRAMMING
