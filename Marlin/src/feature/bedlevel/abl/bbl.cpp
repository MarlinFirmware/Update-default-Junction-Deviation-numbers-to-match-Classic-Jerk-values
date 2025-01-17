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

#include "../../../inc/MarlinConfig.h"

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

#include "../bedlevel.h"

#include "../../../module/motion.h"

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../../core/debug_out.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../../../lcd/extui/ui_api.h"
#endif

LevelingBilinear bedlevel;

xy_pos_t LevelingBilinear::grid_spacing,
         LevelingBilinear::grid_start;
xy_float_t LevelingBilinear::grid_factor;
bed_mesh_t LevelingBilinear::z_values;
xy_pos_t LevelingBilinear::cached_rel;
xy_int8_t LevelingBilinear::cached_g;

/**
 * Extrapolate a single point from its neighbors
 */
void LevelingBilinear::extrapolate_one_point(const uint8_t x, const uint8_t y, const int8_t xdir, const int8_t ydir) {
  if (!isnan(z_values[x][y])) return;
  if (DEBUGGING(LEVELING)) {
    DEBUG_ECHOPGM("Extrapolate [");
    if (x < 10) DEBUG_CHAR(' ');
    DEBUG_ECHO(x);
    DEBUG_CHAR(xdir ? (xdir > 0 ? '+' : '-') : ' ');
    DEBUG_CHAR(' ');
    if (y < 10) DEBUG_CHAR(' ');
    DEBUG_ECHO(y);
    DEBUG_CHAR(ydir ? (ydir > 0 ? '+' : '-') : ' ');
    DEBUG_ECHOLNPGM("]");
  }

  // Get X neighbors, Y neighbors, and XY neighbors
  const uint8_t x1 = x + xdir, y1 = y + ydir, x2 = x1 + xdir, y2 = y1 + ydir;
  float a1 = z_values[x1][y ], a2 = z_values[x2][y ],
        b1 = z_values[x ][y1], b2 = z_values[x ][y2],
        c1 = z_values[x1][y1], c2 = z_values[x2][y2];

  // Treat far unprobed points as zero, near as equal to far
  if (isnan(a2)) a2 = 0.0;
  if (isnan(a1)) a1 = a2;
  if (isnan(b2)) b2 = 0.0;
  if (isnan(b1)) b1 = b2;
  if (isnan(c2)) c2 = 0.0;
  if (isnan(c1)) c1 = c2;

  const float a = 2 * a1 - a2, b = 2 * b1 - b2, c = 2 * c1 - c2;

  // Take the average instead of the median
  z_values[x][y] = (a + b + c) / 3.0;
  TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, z_values[x][y]));

  // Median is robust (ignores outliers).
  // z_values[x][y] = (a < b) ? ((b < c) ? b : (c < a) ? a : c)
  //                                : ((c < b) ? b : (a < c) ? a : c);
}

//Enable this if your SCARA uses 180° of total area
//#define EXTRAPOLATE_FROM_EDGE

#if ENABLED(EXTRAPOLATE_FROM_EDGE)
  #if (GRID_MAX_POINTS_X) < (GRID_MAX_POINTS_Y)
    #define HALF_IN_X
  #elif (GRID_MAX_POINTS_Y) < (GRID_MAX_POINTS_X)
    #define HALF_IN_Y
  #endif
#endif

void LevelingBilinear::reset() {
  grid_start.reset();
  grid_spacing.reset();
  GRID_LOOP(x, y) {
    z_values[x][y] = NAN;
    TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, 0));
  }
}

void LevelingBilinear::set_grid(const xy_pos_t& _grid_spacing, const xy_pos_t& _grid_start) {
  grid_spacing = _grid_spacing;
  grid_start = _grid_start;
  grid_factor = grid_spacing.reciprocal();
}

/**
 * Fill in the unprobed points (corners of circular print surface)
 * using linear extrapolation, away from the center.
 */
void LevelingBilinear::extrapolate_unprobed_bed_level() {
  #ifdef HALF_IN_X
    constexpr uint8_t ctrx2 = 0, xend = GRID_MAX_POINTS_X - 1;
  #else
    constexpr uint8_t ctrx1 = (GRID_MAX_CELLS_X) / 2, // left-of-center
                      ctrx2 = (GRID_MAX_POINTS_X) / 2,  // right-of-center
                      xend = ctrx1;
  #endif

  #ifdef HALF_IN_Y
    constexpr uint8_t ctry2 = 0, yend = GRID_MAX_POINTS_Y - 1;
  #else
    constexpr uint8_t ctry1 = (GRID_MAX_CELLS_Y) / 2, // top-of-center
                      ctry2 = (GRID_MAX_POINTS_Y) / 2,  // bottom-of-center
                      yend = ctry1;
  #endif

  for (uint_fast8_t xo = 0; xo <= xend; ++xo)
    for (uint_fast8_t yo = 0; yo <= yend; ++yo) {
      uint8_t x2 = ctrx2 + xo, y2 = ctry2 + yo;
      #ifndef HALF_IN_X
        const uint8_t x1 = ctrx1 - xo;
      #endif
      #ifndef HALF_IN_Y
        const uint8_t y1 = ctry1 - yo;
        #ifndef HALF_IN_X
          extrapolate_one_point(x1, y1, +1, +1);   //  left-below + +
        #endif
        extrapolate_one_point(x2, y1, -1, +1);     // right-below - +
      #endif
      #ifndef HALF_IN_X
        extrapolate_one_point(x1, y2, +1, -1);     //  left-above + -
      #endif
      extrapolate_one_point(x2, y2, -1, -1);       // right-above - -
    }
}

void LevelingBilinear::print_leveling_grid(const bed_mesh_t* _z_values/*=nullptr*/) {
  // print internal grid(s) or just the one passed as a parameter
  SERIAL_ECHOLNPGM("Bilinear Leveling Grid:");
  print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 3, _z_values ? *_z_values[0] : z_values[0]);

  #if ENABLED(ABL_BILINEAR_SUBDIVISION)
    if (!_z_values) {
      SERIAL_ECHOLNPGM("Subdivided with CATMULL ROM Leveling Grid:");
      print_2d_array(ABL_GRID_POINTS_VIRT_X, ABL_GRID_POINTS_VIRT_Y, 5, z_values_virt[0]);
    }
  #endif
}

#if ENABLED(ABL_BILINEAR_SUBDIVISION)

  #define ABL_TEMP_POINTS_X (GRID_MAX_POINTS_X + 2)
  #define ABL_TEMP_POINTS_Y (GRID_MAX_POINTS_Y + 2)
  float LevelingBilinear::z_values_virt[ABL_GRID_POINTS_VIRT_X][ABL_GRID_POINTS_VIRT_Y];
  xy_pos_t LevelingBilinear::grid_spacing_virt;
  xy_float_t LevelingBilinear::grid_factor_virt;

  #define LINEAR_EXTRAPOLATION(E, I) ((E) * 2 - (I))
  float LevelingBilinear::virt_coord(const uint8_t x, const uint8_t y) {
    uint8_t ep = 0, ip = 1;
    if (x > (GRID_MAX_POINTS_X) + 1 || y > (GRID_MAX_POINTS_Y) + 1) {
      // The requested point requires extrapolating two points beyond the mesh.
      // These values are only requested for the edges of the mesh, which are always an actual mesh point,
      // and do not require interpolation. When interpolation is not needed, this "Mesh + 2" point is
      // cancelled out in virt_cmr and does not impact the result. Return 0.0 rather than
      // making this function more complex by extrapolating two points.
      return 0.0;
    }
    if (!x || x == ABL_TEMP_POINTS_X - 1) {
      if (x) {
        ep = (GRID_MAX_POINTS_X) - 1;
        ip = GRID_MAX_CELLS_X - 1;
      }
      if (WITHIN(y, 1, ABL_TEMP_POINTS_Y - 2))
        return LINEAR_EXTRAPOLATION(
          z_values[ep][y - 1],
          z_values[ip][y - 1]
        );
      else
        return LINEAR_EXTRAPOLATION(
          virt_coord(ep + 1, y),
          virt_coord(ip + 1, y)
        );
    }
    if (!y || y == ABL_TEMP_POINTS_Y - 1) {
      if (y) {
        ep = (GRID_MAX_POINTS_Y) - 1;
        ip = GRID_MAX_CELLS_Y - 1;
      }
      if (WITHIN(x, 1, ABL_TEMP_POINTS_X - 2))
        return LINEAR_EXTRAPOLATION(
          z_values[x - 1][ep],
          z_values[x - 1][ip]
        );
      else
        return LINEAR_EXTRAPOLATION(
          virt_coord(x, ep + 1),
          virt_coord(x, ip + 1)
        );
    }
    return z_values[x - 1][y - 1];
  }

  float LevelingBilinear::virt_cmr(const float p[4], const uint8_t i, const float t) {
    return (
        p[i-1] * -t * sq(1 - t)
      + p[i]   * (2 - 5 * sq(t) + 3 * t * sq(t))
      + p[i+1] * t * (1 + 4 * t - 3 * sq(t))
      - p[i+2] * sq(t) * (1 - t)
    ) * 0.5f;
  }

  float LevelingBilinear::virt_2cmr(const uint8_t x, const uint8_t y, const_float_t tx, const_float_t ty) {
    float row[4], column[4];
    for (uint_fast8_t i = 0; i < 4; ++i) {
      for (uint_fast8_t j = 0; j < 4; ++j) {
        column[j] = virt_coord(i + x - 1, j + y - 1);
      }
      row[i] = virt_cmr(column, 1, ty);
    }
    return virt_cmr(row, 1, tx);
  }

  void LevelingBilinear::subdivide_mesh() {
    grid_spacing_virt = grid_spacing / (BILINEAR_SUBDIVISIONS);
    grid_factor_virt = grid_spacing_virt.reciprocal();
    for (uint_fast8_t y = 0; y < GRID_MAX_POINTS_Y; ++y)
      for (uint_fast8_t x = 0; x < GRID_MAX_POINTS_X; ++x)
        for (uint_fast8_t ty = 0; ty < BILINEAR_SUBDIVISIONS; ++ty)
          for (uint_fast8_t tx = 0; tx < BILINEAR_SUBDIVISIONS; ++tx) {
            if ((ty && y == (GRID_MAX_POINTS_Y) - 1) || (tx && x == (GRID_MAX_POINTS_X) - 1))
              continue;
            z_values_virt[x * (BILINEAR_SUBDIVISIONS) + tx][y * (BILINEAR_SUBDIVISIONS) + ty] =
              virt_2cmr(x + 1, y + 1, (float)tx / (BILINEAR_SUBDIVISIONS), (float)ty / (BILINEAR_SUBDIVISIONS));
          }
  }

#endif // ABL_BILINEAR_SUBDIVISION

// Refresh after other values have been updated
void LevelingBilinear::refresh_bed_level() {
  TERN_(ABL_BILINEAR_SUBDIVISION, subdivide_mesh());
  cached_rel.x = cached_rel.y = -999.999;
  cached_g.x = cached_g.y = -99;
}

#if ENABLED(ABL_BILINEAR_SUBDIVISION)
  #define ABL_BG_SPACING(A) grid_spacing_virt.A
  #define ABL_BG_FACTOR(A)  grid_factor_virt.A
  #define ABL_BG_POINTS_X   ABL_GRID_POINTS_VIRT_X
  #define ABL_BG_POINTS_Y   ABL_GRID_POINTS_VIRT_Y
  #define ABL_BG_GRID(X,Y)  z_values_virt[X][Y]
#else
  #define ABL_BG_SPACING(A) grid_spacing.A
  #define ABL_BG_FACTOR(A)  grid_factor.A
  #define ABL_BG_POINTS_X   GRID_MAX_POINTS_X
  #define ABL_BG_POINTS_Y   GRID_MAX_POINTS_Y
  #define ABL_BG_GRID(X,Y)  z_values[X][Y]
#endif

// Get the Z adjustment for non-linear bed leveling
float LevelingBilinear::get_z_correction(const xy_pos_t &raw) {

  static float z1, d2, z3, d4, L, D;

  static xy_pos_t ratio;

  // Whole units for the grid line indices. Constrained within bounds.
  static xy_int8_t thisg, nextg;

  // XY relative to the probed area
  xy_pos_t rel = raw - grid_start.asFloat();

  #if ENABLED(EXTRAPOLATE_BEYOND_GRID)
    #define FAR_EDGE_OR_BOX 2   // Keep using the last grid box
  #else
    #define FAR_EDGE_OR_BOX 1   // Just use the grid far edge
  #endif

  if (cached_rel.x != rel.x) {
    cached_rel.x = rel.x;
    ratio.x = rel.x * ABL_BG_FACTOR(x);
    const float gx = constrain(FLOOR(ratio.x), 0, ABL_BG_POINTS_X - (FAR_EDGE_OR_BOX));
    ratio.x -= gx;      // Subtract whole to get the ratio within the grid box

    #if DISABLED(EXTRAPOLATE_BEYOND_GRID)
      // Beyond the grid maintain height at grid edges
      NOLESS(ratio.x, 0); // Never <0 (>1 is ok when nextg.x==thisg.x)
    #endif

    thisg.x = gx;
    nextg.x = _MIN(thisg.x + 1, ABL_BG_POINTS_X - 1);
  }

  if (cached_rel.y != rel.y || cached_g.x != thisg.x) {

    if (cached_rel.y != rel.y) {
      cached_rel.y = rel.y;
      ratio.y = rel.y * ABL_BG_FACTOR(y);
      const float gy = constrain(FLOOR(ratio.y), 0, ABL_BG_POINTS_Y - (FAR_EDGE_OR_BOX));
      ratio.y -= gy;

      #if DISABLED(EXTRAPOLATE_BEYOND_GRID)
        // Beyond the grid maintain height at grid edges
        NOLESS(ratio.y, 0); // Never < 0.0. (> 1.0 is ok when nextg.y==thisg.y.)
      #endif

      thisg.y = gy;
      nextg.y = _MIN(thisg.y + 1, ABL_BG_POINTS_Y - 1);
    }

    if (cached_g != thisg) {
      cached_g = thisg;
      // Z at the box corners
      z1 = ABL_BG_GRID(thisg.x, thisg.y);       // left-front
      d2 = ABL_BG_GRID(thisg.x, nextg.y) - z1;  // left-back (delta)
      z3 = ABL_BG_GRID(nextg.x, thisg.y);       // right-front
      d4 = ABL_BG_GRID(nextg.x, nextg.y) - z3;  // right-back (delta)
    }

    // Bilinear interpolate. Needed since rel.y or thisg.x has changed.
                L = z1 + d2 * ratio.y;   // Linear interp. LF -> LB
    const float R = z3 + d4 * ratio.y;   // Linear interp. RF -> RB

    D = R - L;
  }

  const float offset = L + ratio.x * D;   // the offset almost always changes

  /*
  static float last_offset = 0;
  if (ABS(last_offset - offset) > 0.2) {
    SERIAL_ECHOLNPGM("Sudden Shift at x=", rel.x, " / ", grid_spacing.x, " -> thisg.x=", thisg.x);
    SERIAL_ECHOLNPGM(" y=", rel.y, " / ", grid_spacing.y, " -> thisg.y=", thisg.y);
    SERIAL_ECHOLNPGM(" ratio.x=", ratio.x, " ratio.y=", ratio.y);
    SERIAL_ECHOLNPGM(" z1=", z1, " z2=", z2, " z3=", z3, " z4=", z4);
    SERIAL_ECHOLNPGM(" L=", L, " R=", R, " offset=", offset);
  }
  last_offset = offset;
  //*/

  return offset;
}

#if IS_CARTESIAN && DISABLED(SEGMENT_LEVELED_MOVES)

  #define CELL_INDEX(A,V) ((V - grid_start.A) * ABL_BG_FACTOR(A))

  /**
   * Prepare a bilinear-leveled linear move on Cartesian,
   * splitting the move where it crosses grid borders.
   */
  void LevelingBilinear::line_to_destination(const_feedRate_t scaled_fr_mm_s, uint16_t x_splits, uint16_t y_splits) {
    // Get current and destination cells for this line
    xy_int_t c1 { CELL_INDEX(x, current_position.x), CELL_INDEX(y, current_position.y) },
             c2 { CELL_INDEX(x, destination.x), CELL_INDEX(y, destination.y) };
    LIMIT(c1.x, 0, ABL_BG_POINTS_X - 2);
    LIMIT(c1.y, 0, ABL_BG_POINTS_Y - 2);
    LIMIT(c2.x, 0, ABL_BG_POINTS_X - 2);
    LIMIT(c2.y, 0, ABL_BG_POINTS_Y - 2);

    // Start and end in the same cell? No split needed.
    if (c1 == c2) {
      current_position = destination;
      line_to_current_position(scaled_fr_mm_s);
      return;
    }

    #define LINE_SEGMENT_END(A) (current_position.A + (destination.A - current_position.A) * normalized_dist)

    float normalized_dist;
    xyze_pos_t end;
    const xy_int8_t gc { _MAX(c1.x, c2.x), _MAX(c1.y, c2.y) };

    // Crosses on the X and not already split on this X?
    // The x_splits flags are insurance against rounding errors.
    if (c2.x != c1.x && TEST(x_splits, gc.x)) {
      // Split on the X grid line
      CBI(x_splits, gc.x);
      end = destination;
      destination.x = grid_start.x + ABL_BG_SPACING(x) * gc.x;
      normalized_dist = (destination.x - current_position.x) / (end.x - current_position.x);
      destination.y = LINE_SEGMENT_END(y);
    }
    // Crosses on the Y and not already split on this Y?
    else if (c2.y != c1.y && TEST(y_splits, gc.y)) {
      // Split on the Y grid line
      CBI(y_splits, gc.y);
      end = destination;
      destination.y = grid_start.y + ABL_BG_SPACING(y) * gc.y;
      normalized_dist = (destination.y - current_position.y) / (end.y - current_position.y);
      destination.x = LINE_SEGMENT_END(x);
    }
    else {
      // Must already have been split on these border(s)
      // This should be a rare case.
      current_position = destination;
      line_to_current_position(scaled_fr_mm_s);
      return;
    }

    destination.z = LINE_SEGMENT_END(z);
    destination.e = LINE_SEGMENT_END(e);

    // Do the split and look for more borders
    line_to_destination(scaled_fr_mm_s, x_splits, y_splits);

    // Restore destination from stack
    destination = end;
    line_to_destination(scaled_fr_mm_s, x_splits, y_splits);
  }

#endif // IS_CARTESIAN && !SEGMENT_LEVELED_MOVES

#endif // AUTO_BED_LEVELING_BILINEAR
