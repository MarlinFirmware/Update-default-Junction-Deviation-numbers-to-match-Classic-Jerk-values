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

//#define UBL_DEVEL_DEBUGGING

#include "../../../module/motion.h"

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../../core/debug_out.h"

#define UBL_VERSION "1.01"
#define UBL_OK false
#define UBL_ERR true

enum MeshPointType : char { INVALID, REAL, SET_IN_BITMAP, CLOSEST };

// External references

struct mesh_index_pair;

#define MESH_X_DIST (float((MESH_MAX_X) - (MESH_MIN_X)) / (GRID_MAX_CELLS_X))
#define MESH_Y_DIST (float((MESH_MAX_Y) - (MESH_MIN_Y)) / (GRID_MAX_CELLS_Y))

#if ENABLED(OPTIMIZED_MESH_STORAGE)
  typedef int16_t mesh_store_t[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
#endif

typedef struct {
  bool      C_seen;
  int8_t    KLS_storage_slot;
  grid_count_t R_repetition;
  uint8_t   V_verbosity,
            P_phase,
            T_map_type;
  float     B_shim_thickness,
            C_constant;
  xy_pos_t  XY_pos;
  xy_bool_t XY_seen;
  #if HAS_BED_PROBE
    uint8_t J_grid_size;
  #endif
} G29_parameters_t;

class unified_bed_leveling {
private:

  static G29_parameters_t param;

  #if IS_NEWPANEL
    static void move_z_with_encoder(const_float_t multiplier);
    static float measure_point_with_encoder();
    static float measure_business_card_thickness();
    static void manually_probe_remaining_mesh(const xy_pos_t&, const_float_t , const_float_t , const bool) __O0;
    static void fine_tune_mesh(const xy_pos_t &pos, const bool do_ubl_mesh_map) __O0;
  #endif

  static bool G29_parse_parameters() __O0;
  static void shift_mesh_height();
  static void probe_entire_mesh(const xy_pos_t &near, const bool do_ubl_mesh_map, const bool stow_probe, const bool do_furthest) __O0;
  static void tilt_mesh_based_on_probed_grid(const bool do_ubl_mesh_map);
  static bool smart_fill_one(const uint8_t x, const uint8_t y, const int8_t xdir, const int8_t ydir);
  static bool smart_fill_one(const xy_uint8_t &pos, const xy_uint8_t &dir) {
    return smart_fill_one(pos.x, pos.y, dir.x, dir.y);
  }

  #if ENABLED(UBL_DEVEL_DEBUGGING)
    static void g29_what_command();
    static void g29_eeprom_dump();
    static void g29_compare_current_mesh_to_stored_mesh();
  #endif

public:

  static void echo_name();
  static void report_current_mesh();
  static void report_state();
  static void save_ubl_active_state_and_disable();
  static void restore_ubl_active_state(const bool is_done=true);
  static void display_map(const uint8_t) __O0;
  static mesh_index_pair find_closest_mesh_point_of_type(const MeshPointType, const xy_pos_t&, const bool=false, MeshFlags *done_flags=nullptr) __O0;
  static mesh_index_pair find_furthest_invalid_mesh_point() __O0;
  static void reset();
  static void invalidate();
  static void set_all_mesh_points_to_value(const_float_t value);
  static void adjust_mesh_to_mean(const bool cflag, const_float_t value);
  static bool sanity_check();
  static void smart_fill_mesh();

  static void G29() __O0;                           // O0 for no optimization
  static void smart_fill_wlsf(const_float_t ) __O2; // O2 gives smaller code than Os on A2560

  static int8_t storage_slot;
  static bed_mesh_t z_values;

  #if ENABLED(OPTIMIZED_MESH_STORAGE)
    static void set_store_from_mesh(const bed_mesh_t &in_values, mesh_store_t &stored_values);
    static void set_mesh_from_store(const mesh_store_t &stored_values, bed_mesh_t &out_values);
  #endif

  #if DISABLED(PROUI_MESH_EDIT)
    static const float _mesh_index_to_xpos[GRID_MAX_POINTS_X],
                       _mesh_index_to_ypos[GRID_MAX_POINTS_Y];
  #endif

  #if HAS_MARLINUI_MENU
    static bool lcd_map_control;
    static void steppers_were_disabled();
  #else
    static void steppers_were_disabled() {}
  #endif

  static volatile int16_t encoder_diff; // Volatile because buttons may change it at interrupt time

  unified_bed_leveling();

  FORCE_INLINE static void set_z(const int8_t px, const int8_t py, const_float_t z) { z_values[px][py] = z; }

  static int8_t cell_index_x_raw(const_float_t x) {
    return FLOOR((x - (MESH_MIN_X)) * RECIPROCAL(MESH_X_DIST));
  }

  static int8_t cell_index_y_raw(const_float_t y) {
    return FLOOR((y - (MESH_MIN_Y)) * RECIPROCAL(MESH_Y_DIST));
  }

  static bool cell_index_x_valid(const_float_t x) {
    return WITHIN(cell_index_x_raw(x), 0, GRID_MAX_CELLS_X - 1);
  }

  static bool cell_index_y_valid(const_float_t y) {
    return WITHIN(cell_index_y_raw(y), 0, GRID_MAX_CELLS_Y - 1);
  }

  static uint8_t cell_index_x(const_float_t x) {
    return constrain(cell_index_x_raw(x), 0, GRID_MAX_CELLS_X - 1);
  }

  static uint8_t cell_index_y(const_float_t y) {
    return constrain(cell_index_y_raw(y), 0, GRID_MAX_CELLS_Y - 1);
  }

  static xy_uint8_t cell_indexes(const_float_t x, const_float_t y) {
    return { cell_index_x(x), cell_index_y(y) };
  }
  static xy_uint8_t cell_indexes(const xy_pos_t &xy) { return cell_indexes(xy.x, xy.y); }

  static int8_t closest_x_index(const_float_t x) {
    const int8_t px = (x - (MESH_MIN_X) + (MESH_X_DIST) * 0.5) * RECIPROCAL(MESH_X_DIST);
    return WITHIN(px, 0, (GRID_MAX_POINTS_X) - 1) ? px : -1;
  }
  static int8_t closest_y_index(const_float_t y) {
    const int8_t py = (y - (MESH_MIN_Y) + (MESH_Y_DIST) * 0.5) * RECIPROCAL(MESH_Y_DIST);
    return WITHIN(py, 0, (GRID_MAX_POINTS_Y) - 1) ? py : -1;
  }
  static xy_int8_t closest_indexes(const xy_pos_t &xy) {
    return { closest_x_index(xy.x), closest_y_index(xy.y) };
  }

  /**
   *                           z2   --|
   *                 z0        |      |
   *                  |        |      + (z2-z1)
   *   z1             |        |      |
   * ---+-------------+--------+--  --|
   *   a1            a0        a2
   *    |<---delta_a---------->|
   *
   *  calc_z0 is the basis for all the Mesh Based correction. It is used to
   *  find the expected Z Height at a position between two known Z-Height locations.
   *
   *  It is fairly expensive with its 4 floating point additions and 2 floating point
   *  multiplications.
   */
  FORCE_INLINE static float calc_z0(const_float_t a0, const_float_t a1, const_float_t z1, const_float_t a2, const_float_t z2) {
    return z1 + (z2 - z1) * (a0 - a1) / (a2 - a1);
  }

  #ifdef UBL_Z_RAISE_WHEN_OFF_MESH
    #define _UBL_OUTER_Z_RAISE UBL_Z_RAISE_WHEN_OFF_MESH
  #else
    #define _UBL_OUTER_Z_RAISE NAN
  #endif

  /**
   * z_correction_for_x_on_horizontal_mesh_line is an optimization for
   * the case where the printer is making a vertical line that only crosses horizontal mesh lines.
   */
  static float z_correction_for_x_on_horizontal_mesh_line(const_float_t rx0, const int x1_i, const int yi) {
    if (!WITHIN(x1_i, 0, (GRID_MAX_POINTS_X) - 1) || !WITHIN(yi, 0, (GRID_MAX_POINTS_Y) - 1)) {

      if (DEBUGGING(LEVELING)) {
        if (WITHIN(x1_i, 0, (GRID_MAX_POINTS_X) - 1)) DEBUG_ECHOPGM("yi"); else DEBUG_ECHOPGM("x1_i");
        DEBUG_ECHOLNPGM(" out of bounds in z_correction_for_x_on_horizontal_mesh_line(rx0=", rx0, ",x1_i=", x1_i, ",yi=", yi, ")");
      }

      // The requested location is off the mesh. Return UBL_Z_RAISE_WHEN_OFF_MESH or NAN.
      return _UBL_OUTER_Z_RAISE;
    }

    const float xratio = (rx0 - get_mesh_x(x1_i)) * RECIPROCAL(MESH_X_DIST),
                z1 = z_values[x1_i][yi];

    return z1 + xratio * (z_values[_MIN(x1_i, (GRID_MAX_POINTS_X) - 2) + 1][yi] - z1);  // Don't allow x1_i+1 to be past the end of the array
                                                                                        // If it is, it is clamped to the last element of the
                                                                                        // z_values[][] array and no correction is applied.
  }

  //
  // See comments above for z_correction_for_x_on_horizontal_mesh_line
  //
  static float z_correction_for_y_on_vertical_mesh_line(const_float_t ry0, const int xi, const int y1_i) {
    if (!WITHIN(xi, 0, (GRID_MAX_POINTS_X) - 1) || !WITHIN(y1_i, 0, (GRID_MAX_POINTS_Y) - 1)) {

      if (DEBUGGING(LEVELING)) {
        if (WITHIN(xi, 0, (GRID_MAX_POINTS_X) - 1)) DEBUG_ECHOPGM("y1_i"); else DEBUG_ECHOPGM("xi");
        DEBUG_ECHOLNPGM(" out of bounds in z_correction_for_y_on_vertical_mesh_line(ry0=", ry0, ", xi=", xi, ", y1_i=", y1_i, ")");
      }

      // The requested location is off the mesh. Return UBL_Z_RAISE_WHEN_OFF_MESH or NAN.
      return _UBL_OUTER_Z_RAISE;
    }

    const float yratio = (ry0 - get_mesh_y(y1_i)) * RECIPROCAL(MESH_Y_DIST),
                z1 = z_values[xi][y1_i];

    return z1 + yratio * (z_values[xi][_MIN(y1_i, (GRID_MAX_POINTS_Y) - 2) + 1] - z1);  // Don't allow y1_i+1 to be past the end of the array
                                                                                        // If it is, it is clamped to the last element of the
                                                                                        // z_values[][] array and no correction is applied.
  }

  /**
   * This is the generic Z-Correction. It works anywhere within a Mesh Cell. It first
   * does a linear interpolation along both of the bounding X-Mesh-Lines to find the
   * Z-Height at both ends. Then it does a linear interpolation of these heights based
   * on the Y position within the cell.
   */
  static float get_z_correction(const_float_t rx0, const_float_t ry0) {
    const int8_t cx = cell_index_x(rx0), cy = cell_index_y(ry0); // return values are clamped

    /**
     * Check if the requested location is off the mesh.  If so, and
     * UBL_Z_RAISE_WHEN_OFF_MESH is specified, that value is returned.
     */
    #ifdef UBL_Z_RAISE_WHEN_OFF_MESH
      if (!WITHIN(rx0, MESH_MIN_X, MESH_MAX_X) || !WITHIN(ry0, MESH_MIN_Y, MESH_MAX_Y))
        return UBL_Z_RAISE_WHEN_OFF_MESH;
    #endif

    const uint8_t mx = _MIN(cx, (GRID_MAX_POINTS_X) - 2) + 1, my = _MIN(cy, (GRID_MAX_POINTS_Y) - 2) + 1;
    const float x0 = get_mesh_x(cx), x1 = get_mesh_x(cx + 1),
                z1 = calc_z0(rx0, x0, z_values[cx][cy], x1, z_values[mx][cy]),
                z2 = calc_z0(rx0, x0, z_values[cx][my], x1, z_values[mx][my]);
    float z0 = calc_z0(ry0, get_mesh_y(cy), z1, get_mesh_y(cy + 1), z2);

    if (isnan(z0)) { // If part of the Mesh is undefined, it will show up as NAN
      z0 = 0.0;      // in z_values[][] and propagate through the calculations.
                     // If our correction is NAN, we throw it out because part of
                     // the Mesh is undefined and we don't have the information
                     // needed to complete the height correction.

      if (DEBUGGING(MESH_ADJUST)) DEBUG_ECHOLNPGM("??? Yikes! NAN in ");
    }

    if (DEBUGGING(MESH_ADJUST))
      DEBUG_ECHOLN(F("get_z_correction("), rx0, F(", "), ry0, F(") => "), p_float_t(z0, 6));

    return z0;
  }
  static float get_z_correction(const xy_pos_t &pos) { return get_z_correction(pos.x, pos.y); }

  static constexpr float get_z_offset() { return 0.0f; }

  #if ENABLED(PROUI_MESH_EDIT)
    static float get_mesh_x(const uint8_t i) {
      return MESH_MIN_X + i * (MESH_X_DIST);
    }
    static float get_mesh_y(const uint8_t j) {
      return MESH_MIN_Y + j * (MESH_Y_DIST);
    }
  #else
    static float get_mesh_x(const uint8_t i) {
      return i < (GRID_MAX_POINTS_X) ? pgm_read_float(&_mesh_index_to_xpos[i]) : MESH_MIN_X + i * (MESH_X_DIST);
    }
    static float get_mesh_y(const uint8_t j) {
      return j < (GRID_MAX_POINTS_Y) ? pgm_read_float(&_mesh_index_to_ypos[j]) : MESH_MIN_Y + j * (MESH_Y_DIST);
    }
  #endif

  #if UBL_SEGMENTED
    static bool line_to_destination_segmented(const_feedRate_t scaled_fr_mm_s);
  #else
    static void line_to_destination_cartesian(const_feedRate_t scaled_fr_mm_s, const uint8_t e);
  #endif

  static bool mesh_is_valid() {
    GRID_LOOP(x, y) if (isnan(z_values[x][y])) return false;
    return true;
  }

}; // class unified_bed_leveling

extern unified_bed_leveling bedlevel;

// Prevent debugging propagating to other files
#include "../../../core/debug_out.h"
