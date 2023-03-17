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

#if ANY(NOZZLE_CLEAN_FEATURE, NOZZLE_PARK_FEATURE)

#include "nozzle.h"

Nozzle nozzle;

#include "../MarlinCore.h"
#include "../module/motion.h"

#if NOZZLE_CLEAN_MIN_TEMP > 20
  #include "../module/temperature.h"
#endif

#if ENABLED(NOZZLE_CLEAN_FEATURE)

  #if ENABLED(NOZZLE_CLEAN_PATTERN_LINE)
    /**
     * @brief Stroke clean pattern
     * @details Wipes the nozzle back and forth in a linear movement
     *
     * @param start xyz_pos_t defining the starting point
     * @param end xyz_pos_t defining the ending point
     * @param strokes number of strokes to execute
     */
    void Nozzle::stroke(const xyz_pos_t &start, const xyz_pos_t &end, const uint8_t strokes) {
      #if ENABLED(NOZZLE_CLEAN_GOBACK)
        const xyz_pos_t oldpos = current_position;
      #endif

      // Move to the starting point
      #if ENABLED(NOZZLE_CLEAN_NO_Z)
        #if ENABLED(NOZZLE_CLEAN_NO_Y)
          do_blocking_move_to_x(start.x);
        #else
          do_blocking_move_to_xy(start);
        #endif
      #else
        do_blocking_move_to(start);
      #endif

      // Start the stroke pattern
      for (uint_fast8_t i = 0; i < strokes >> 1; ++i) {
        #if ENABLED(NOZZLE_CLEAN_NO_Y)
          do_blocking_move_to_x(end.x);
          do_blocking_move_to_x(start.x);
        #else
          do_blocking_move_to_xy(end);
          do_blocking_move_to_xy(start);
        #endif
      }

      TERN_(NOZZLE_CLEAN_GOBACK, do_blocking_move_to(oldpos));
    }
  #endif

  #if ENABLED(NOZZLE_CLEAN_PATTERN_ZIGZAG)
    /**
     * @brief Zig-zag clean pattern
     * @details Apply a zig-zag cleaning pattern
     *
     * @param start xyz_pos_t defining the starting point
     * @param end xyz_pos_t defining the ending point
     * @param strokes number of strokes to execute
     * @param objects number of triangles to do
     */
    void Nozzle::zigzag(const xyz_pos_t &start, const xyz_pos_t &end, const uint8_t strokes, const uint8_t objects) {
      const xy_pos_t diff = end - start;
      if (!diff.x || !diff.y) return;

      #if ENABLED(NOZZLE_CLEAN_GOBACK)
        const xyz_pos_t back = current_position;
      #endif

      #if ENABLED(NOZZLE_CLEAN_NO_Z)
        do_blocking_move_to_xy(start);
      #else
        do_blocking_move_to(start);
      #endif

      const uint8_t zigs = objects << 1;
      const bool horiz = ABS(diff.x) >= ABS(diff.y);    // Do a horizontal wipe?
      const float P = (horiz ? diff.x : diff.y) / zigs; // Period of each zig / zag
      const xyz_pos_t *side;
      for (uint_fast8_t j = 0; j < strokes; ++j) {
        for (int8_t i = 0; i < zigs; i++) {
          side = (i & 1) ? &end : &start;
          if (horiz)
            do_blocking_move_to_xy(start.x + i * P, side->y);
          else
            do_blocking_move_to_xy(side->x, start.y + i * P);
        }
        for (int8_t i = zigs; i >= 0; i--) {
          side = (i & 1) ? &end : &start;
          if (horiz)
            do_blocking_move_to_xy(start.x + i * P, side->y);
          else
            do_blocking_move_to_xy(side->x, start.y + i * P);
        }
      }

      TERN_(NOZZLE_CLEAN_GOBACK, do_blocking_move_to(back));
    }
  #endif

  #if ENABLED(NOZZLE_CLEAN_PATTERN_CIRCLE)
    /**
     * @brief Circular clean pattern
     * @details Apply a circular cleaning pattern
     *
     * @param start xyz_pos_t defining the middle of circle
     * @param strokes number of strokes to execute
     * @param radius radius of circle
     */
    void Nozzle::circle(const xyz_pos_t &start, const xyz_pos_t &middle, const uint8_t strokes, const_float_t radius) {
      if (strokes == 0) return;

      #if ENABLED(NOZZLE_CLEAN_GOBACK)
        const xyz_pos_t back = current_position;
      #endif
      TERN(NOZZLE_CLEAN_NO_Z, do_blocking_move_to_xy, do_blocking_move_to)(start);

      for (uint_fast8_t s = 0; s < strokes; ++s)
        for (uint_fast8_t i = 0; i < NOZZLE_CLEAN_CIRCLE_FN; ++i)
          do_blocking_move_to_xy(
            middle.x + sin((RADIANS(360) / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius,
            middle.y + cos((RADIANS(360) / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius
          );

      // Let's be safe
      do_blocking_move_to_xy(start);

      TERN_(NOZZLE_CLEAN_GOBACK, do_blocking_move_to(back));
    }
  #endif

  /**
   * @brief Clean the nozzle
   * @details Starts the selected clean procedure pattern
   *
   * @param pattern one of the available patterns
   * @param argument depends on the cleaning pattern
   */
  void Nozzle::clean(const uint8_t pattern, const uint8_t strokes, const_float_t radius, const uint8_t objects, const uint8_t cleans) {
    xyz_pos_t start[HOTENDS] = NOZZLE_CLEAN_START_POINT, end[HOTENDS] = NOZZLE_CLEAN_END_POINT;
    #if ENABLED(NOZZLE_CLEAN_PATTERN_CIRCLE)
      xyz_pos_t middle[HOTENDS] = NOZZLE_CLEAN_CIRCLE_MIDDLE;
    #endif

    const uint8_t arrPos = ANY(SINGLENOZZLE, MIXING_EXTRUDER) ? 0 : active_extruder;

    switch (pattern) {
      #if DISABLED(NOZZLE_CLEAN_PATTERN_LINE)
        case 0: SERIAL_ECHOLNPGM("Pattern ", F("LINE"), " not enabled."); return;
      #endif
      #if DISABLED(NOZZLE_CLEAN_PATTERN_ZIGZAG)
        case 1: SERIAL_ECHOLNPGM("Pattern ", F("ZIGZAG"), " not enabled."); return;
      #endif
      #if DISABLED(NOZZLE_CLEAN_PATTERN_CIRCLE)
        case 2: SERIAL_ECHOLNPGM("Pattern ", F("CIRCULAR"), " not enabled."); return;
      #endif
    }

    #if NOZZLE_CLEAN_MIN_TEMP > 20
      if (thermalManager.degTargetHotend(arrPos) < NOZZLE_CLEAN_MIN_TEMP) {
        #if ENABLED(NOZZLE_CLEAN_HEATUP)
          SERIAL_ECHOLNPGM("Nozzle too Cold - Heating");
          thermalManager.setTargetHotend(NOZZLE_CLEAN_MIN_TEMP, arrPos);
          thermalManager.wait_for_hotend(arrPos);
        #else
          SERIAL_ECHOLNPGM("Nozzle too cold - Skipping wipe");
          return;
        #endif
      }
    #endif

    #if HAS_SOFTWARE_ENDSTOPS

      #define LIMIT_AXIS(A) do{ \
                                           LIMIT( start[arrPos].A, soft_endstop.min.A, soft_endstop.max.A); \
                                           LIMIT(   end[arrPos].A, soft_endstop.min.A, soft_endstop.max.A); \
        TERN_(NOZZLE_CLEAN_PATTERN_CIRCLE, LIMIT(middle[arrPos].A, soft_endstop.min.A, soft_endstop.max.A)); \
      }while(0)

      if (soft_endstop.enabled()) {
        LIMIT_AXIS(x); LIMIT_AXIS(y); LIMIT_AXIS(z);
        #if ENABLED(NOZZLE_CLEAN_PATTERN_CIRCLE)
          if (pattern == 2 && !(WITHIN(middle[arrPos].x, soft_endstop.min.x + radius, soft_endstop.max.x - radius)
                             && WITHIN(middle[arrPos].y, soft_endstop.min.y + radius, soft_endstop.max.y - radius))
          ) {
            SERIAL_ECHOLNPGM("Warning: Radius Out of Range"); return;
          }
        #endif
      }

    #endif

    #if ENABLED(NOZZLE_CLEAN_PATTERN_CIRCLE)
      if (pattern == 2 && !(cleans & (_BV(X_AXIS) | _BV(Y_AXIS)))) {
        SERIAL_ECHOLNPGM("Warning: Clean Circle requires XY"); return;
      }
    #endif

    if (TERN1(NOZZLE_CLEAN_PATTERN_CIRCLE, pattern != 2)) {
      if (!TEST(cleans, X_AXIS)) start[arrPos].x = end[arrPos].x = current_position.x;
      if (!TEST(cleans, Y_AXIS)) start[arrPos].y = end[arrPos].y = current_position.y;
    }
    if (!TEST(cleans, Z_AXIS)) start[arrPos].z = end[arrPos].z = current_position.z;

    switch (pattern) { // no default clause as pattern is already validated
      #if ENABLED(NOZZLE_CLEAN_PATTERN_LINE)
        case 0: stroke(start[arrPos], end[arrPos], strokes); break;
      #endif
      #if ENABLED(NOZZLE_CLEAN_PATTERN_ZIGZAG)
        case 1: zigzag(start[arrPos], end[arrPos], strokes, objects); break;
      #endif
      #if ENABLED(NOZZLE_CLEAN_PATTERN_CIRCLE)
        case 2: circle(start[arrPos], middle[arrPos], strokes, radius); break;
      #endif
    }
  }

#endif // NOZZLE_CLEAN_FEATURE

#if ENABLED(NOZZLE_PARK_FEATURE)

  #if HAS_Z_AXIS
    float Nozzle::park_mode_0_height(const_float_t park_z) {
      // Apply a minimum raise, if specified. Use park.z as a minimum height instead.
      return _MAX(park_z,                       // Minimum height over 0 based on input
        _MIN(Z_MAX_POS,                         // Maximum height is fixed
          #ifdef NOZZLE_PARK_Z_RAISE_MIN
            NOZZLE_PARK_Z_RAISE_MIN +           // Minimum raise...
          #endif
          current_position.z                    // ...over current position
        )
      );
    }
  #endif // HAS_Z_AXIS

  void Nozzle::park(const uint8_t z_action, const xyz_pos_t &park/*=NOZZLE_PARK_POINT*/) {
    #if HAS_Z_AXIS
      constexpr feedRate_t fr_z = NOZZLE_PARK_Z_FEEDRATE;

      switch (z_action) {
        case 1:   // Go to Z-park height
          do_blocking_move_to_z(park.z, fr_z);
          break;

        case 2:   // Raise by Z-park height
          do_blocking_move_to_z(_MIN(current_position.z + park.z, Z_MAX_POS), fr_z);
          break;

        case 3: { // Raise by NOZZLE_PARK_Z_RAISE_MIN, bypass XY-park position
          do_blocking_move_to_z(park_mode_0_height(0), fr_z);
          goto SKIP_XY_MOVE;
        } break;

        case 4:   // Skip Z raise, go to XY position
          break;

        default:  // Raise by NOZZLE_PARK_Z_RAISE_MIN, use park.z as a minimum height
          do_blocking_move_to_z(park_mode_0_height(park.z), fr_z);
          break;
      }
    #endif // HAS_Z_AXIS

    {
      #ifndef NOZZLE_PARK_MOVE
        #define NOZZLE_PARK_MOVE 0
      #endif
      constexpr feedRate_t fr_xy = NOZZLE_PARK_XY_FEEDRATE;
      switch (NOZZLE_PARK_MOVE) {
        case 0: do_blocking_move_to_xy(park, fr_xy); break;
        case 1: do_blocking_move_to_x(park.x, fr_xy); break;
        case 2: do_blocking_move_to_y(park.y, fr_xy); break;
        case 3: do_blocking_move_to_x(park.x, fr_xy);
                do_blocking_move_to_y(park.y, fr_xy); break;
        case 4: do_blocking_move_to_y(park.y, fr_xy);
                do_blocking_move_to_x(park.x, fr_xy); break;
      }
    }

    SKIP_XY_MOVE:
    report_current_position();
  }

#endif // NOZZLE_PARK_FEATURE

#endif // NOZZLE_CLEAN_FEATURE || NOZZLE_PARK_FEATURE
