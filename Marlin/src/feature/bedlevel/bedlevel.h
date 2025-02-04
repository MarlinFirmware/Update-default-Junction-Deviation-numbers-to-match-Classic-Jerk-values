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

#include "../../inc/MarlinConfigPre.h"

#if ANY(RESTORE_LEVELING_AFTER_G28, ENABLE_LEVELING_AFTER_G28)
  #define CAN_SET_LEVELING_AFTER_G28 1
#endif

#if ENABLED(PROBE_MANUALLY)
  extern bool g29_in_progress;
#else
  constexpr bool g29_in_progress = false;
#endif

bool leveling_is_valid();
void set_bed_leveling_enabled(const bool enable=true);
void reset_bed_level();

#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
  void set_z_fade_height(const_float_t zfh, const bool do_report=true);
#endif

#if ANY(MESH_BED_LEVELING, PROBE_MANUALLY)
  void _manual_goto_xy(const xy_pos_t &pos);
#endif

/**
 * A class to save and change the bed leveling state,
 * then restore it when it goes out of scope.
 */
class TemporaryBedLevelingState {
  bool saved;
  public:
    TemporaryBedLevelingState(const bool enable);
    ~TemporaryBedLevelingState() { set_bed_leveling_enabled(saved); }
};
#define TEMPORARY_BED_LEVELING_STATE(enable) const TemporaryBedLevelingState tbls(enable)

#if HAS_MESH

  typedef float bed_mesh_t[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    #include "abl/bbl.h"
  #elif ENABLED(AUTO_BED_LEVELING_UBL)
    #include "ubl/ubl.h"
  #elif ENABLED(MESH_BED_LEVELING)
    #include "mbl/mesh_bed_leveling.h"
  #endif

  #if ANY(AUTO_BED_LEVELING_BILINEAR, MESH_BED_LEVELING)

    #include <stdint.h>

    typedef float (*element_2d_fn)(const uint_fast8_t, const uint_fast8_t);

    /**
     * Print calibration results for plotting or manual frame adjustment.
     */
    void print_2d_array(const uint_fast8_t sx, const uint_fast8_t sy, const uint_fast8_t precision, const float *values);

  #endif

  struct mesh_index_pair {
    xy_int8_t pos;
    float distance;   // When populated, the distance from the search location
    void invalidate() { pos = -1; }
    bool valid() const { return pos.x >= 0 && pos.y >= 0; }
    #if ENABLED(AUTO_BED_LEVELING_UBL)
      xy_pos_t meshpos() {
        return { bedlevel.get_mesh_x(pos.x), bedlevel.get_mesh_y(pos.y) };
      }
    #endif
    operator xy_int8_t&() { return pos; }
    operator const xy_int8_t&() const { return pos; }
  };

#endif
