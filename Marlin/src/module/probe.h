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

/**
 * module/probe.h - Move, deploy, enable, etc.
 */

#include "../inc/MarlinConfig.h"

#include "motion.h"

#if ENABLED(BLTOUCH)
  #include "../feature/bltouch.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../core/debug_out.h"

#if HAS_BED_PROBE
  enum ProbePtRaise : uint8_t {
    PROBE_PT_NONE,      // No raise or stow after run_z_probe
    PROBE_PT_STOW,      // Do a complete stow after run_z_probe
    PROBE_PT_LAST_STOW, // Stow for sure, even in BLTouch HS mode
    PROBE_PT_RAISE      // Raise to "between" clearance after run_z_probe
  };
#endif

#if ENABLED(BD_SENSOR)
  #define PROBE_READ() bdp_state
#elif USE_Z_MIN_PROBE
  #define PROBE_READ() READ(Z_MIN_PROBE_PIN)
#else
  #define PROBE_READ() READ(Z_MIN_PIN)
#endif
#if USE_Z_MIN_PROBE
  #define PROBE_HIT_STATE Z_MIN_PROBE_ENDSTOP_HIT_STATE
#else
  #define PROBE_HIT_STATE Z_MIN_ENDSTOP_HIT_STATE
#endif
#define PROBE_TRIGGERED() (PROBE_READ() == PROBE_HIT_STATE)

// In BLTOUCH HS mode, the probe travels in a deployed state.
#define Z_TWEEN_SAFE_CLEARANCE SUM_TERN(BLTOUCH, Z_CLEARANCE_BETWEEN_PROBES, bltouch.z_extra_clearance())

#if ENABLED(PREHEAT_BEFORE_LEVELING)
  #ifndef LEVELING_NOZZLE_TEMP
    #define LEVELING_NOZZLE_TEMP 0
  #endif
  #ifndef LEVELING_BED_TEMP
    #define LEVELING_BED_TEMP 0
  #endif
#endif

#if ENABLED(SENSORLESS_PROBING)
  extern abc_float_t offset_sensorless_adj;
#endif

class Probe {
public:

  #if ENABLED(SENSORLESS_PROBING)
    typedef struct { bool x:1, y:1, z:1; } sense_bool_t;
    static sense_bool_t test_sensitivity;
  #endif

  #if HAS_BED_PROBE

    static xyz_pos_t offset;

    #if ANY(PREHEAT_BEFORE_PROBING, PREHEAT_BEFORE_LEVELING)
      static void preheat_for_probing(const celsius_t hotend_temp, const celsius_t bed_temp, const bool early=false);
    #endif

    static void probe_error_stop();

    static bool set_deployed(const bool deploy, const bool no_return=false);

    #if IS_KINEMATIC

      #if HAS_PROBE_XY_OFFSET
        // Return true if the both nozzle and the probe can reach the given point.
        // Note: This won't work on SCARA since the probe offset rotates with the arm.
        static bool can_reach(const_float_t rx, const_float_t ry, const bool probe_relative=true) {
          if (probe_relative) {
            return position_is_reachable(rx - offset_xy.x, ry - offset_xy.y) // The nozzle can go where it needs to go?
                && position_is_reachable(rx, ry, PROBING_MARGIN);            // Can the probe also go near there?
          }
          else {
            return position_is_reachable(rx, ry)
                && position_is_reachable(rx + offset_xy.x, ry + offset_xy.y, PROBING_MARGIN);
          }
        }
      #else
        static bool can_reach(const_float_t rx, const_float_t ry, const bool=true) {
          return position_is_reachable(rx, ry)
              && position_is_reachable(rx, ry, PROBING_MARGIN);
        }
      #endif

    #else // !IS_KINEMATIC

      static bool obstacle_check(const_float_t rx, const_float_t ry) {
        #if ENABLED(AVOID_OBSTACLES)
          #ifdef OBSTACLE1
            constexpr float obst1[] = OBSTACLE1;
            static_assert(COUNT(obst1) == 4, "OBSTACLE1 must define a rectangle in the form { X1, Y1, X2, Y2 }.");
            if (WITHIN(rx, obst1[0], obst1[2]) && WITHIN(ry, obst1[1], obst1[3])) return false;
          #endif
          #ifdef OBSTACLE2
            constexpr float obst2[] = OBSTACLE2;
            static_assert(COUNT(obst2) == 4, "OBSTACLE2 must define a rectangle in the form { X1, Y1, X2, Y2 }.");
            if (WITHIN(rx, obst2[0], obst2[2]) && WITHIN(ry, obst2[1], obst2[3])) return false;
          #endif
          #ifdef OBSTACLE3
            constexpr float obst3[] = OBSTACLE3;
            static_assert(COUNT(obst3) == 4, "OBSTACLE3 must define a rectangle in the form { X1, Y1, X2, Y2 }.");
            if (WITHIN(rx, obst3[0], obst3[2]) && WITHIN(ry, obst3[1], obst3[3])) return false;
          #endif
          #ifdef OBSTACLE4
            constexpr float obst4[] = OBSTACLE4;
            static_assert(COUNT(obst4) == 4, "OBSTACLE4 must define a rectangle in the form { X1, Y1, X2, Y2 }.");
            if (WITHIN(rx, obst4[0], obst4[2]) && WITHIN(ry, obst4[1], obst4[3])) return false;
          #endif
        #endif
        return true;
      }

      /**
       * Return whether the given position is within the bed, and whether the nozzle
       * can reach the position required to put the probe at the given position.
       *
       * Example: For a probe offset of -10,+10, then for the probe to reach 0,0 the
       *          nozzle must be be able to reach +10,-10.
       */
      static bool can_reach(const_float_t rx, const_float_t ry, const bool probe_relative=true) {
        if (probe_relative) {
          return position_is_reachable(rx - offset_xy.x, ry - offset_xy.y)
              && COORDINATE_OKAY(rx, min_x() - fslop, max_x() + fslop)
              && COORDINATE_OKAY(ry, min_y() - fslop, max_y() + fslop)
              && obstacle_check(rx, ry)
              && obstacle_check(rx - offset_xy.x, ry - offset_xy.y);
        }
        else {
          return position_is_reachable(rx, ry)
              && COORDINATE_OKAY(rx + offset_xy.x, min_x() - fslop, max_x() + fslop)
              && COORDINATE_OKAY(ry + offset_xy.y, min_y() - fslop, max_y() + fslop)
              && obstacle_check(rx, ry)
              && obstacle_check(rx + offset_xy.x, ry + offset_xy.y);
        }
      }

    #endif // !IS_KINEMATIC

    static float probe_at_point(
      const_float_t      rx,
      const_float_t      ry,
      const ProbePtRaise raise_after        = PROBE_PT_NONE,
      const uint8_t      verbose_level      = 0,
      const bool         probe_relative     = true,
      const bool         sanity_check       = true,
      const_float_t      z_min_point        = Z_PROBE_LOW_POINT,
      const_float_t      z_clearance        = Z_TWEEN_SAFE_CLEARANCE,
      const bool         raise_after_is_rel = false
    );

    static float probe_at_point(
      const xy_pos_t     &pos,
      const ProbePtRaise raise_after        = PROBE_PT_NONE,
      const uint8_t      verbose_level      = 0,
      const bool         probe_relative     = true,
      const bool         sanity_check       = true,
      const_float_t      z_min_point        = Z_PROBE_LOW_POINT,
      const_float_t      z_clearance        = Z_TWEEN_SAFE_CLEARANCE,
      const bool         raise_after_is_rel = false
    ) {
      return probe_at_point(pos.x, pos.y, raise_after, verbose_level, probe_relative, sanity_check, z_min_point, z_clearance, raise_after_is_rel);
    }

  #else // !HAS_BED_PROBE

    static constexpr xyz_pos_t offset = xyz_pos_t(NUM_AXIS_ARRAY_1(0)); // See #16767

    static bool set_deployed(const bool, const bool=false) { return false; }

    static bool can_reach(const_float_t rx, const_float_t ry, const bool=true) { return position_is_reachable(TERN_(HAS_X_AXIS, rx) OPTARG(HAS_Y_AXIS, ry)); }

  #endif // !HAS_BED_PROBE

  static void use_probing_tool(const bool=true) IF_DISABLED(DO_TOOLCHANGE_FOR_PROBING, {});

  static void move_z_after_probing() {
    DEBUG_SECTION(mzah, "move_z_after_probing", DEBUGGING(LEVELING));
    #ifdef Z_AFTER_PROBING
      do_z_clearance(Z_AFTER_PROBING, true, true); // Move down still permitted
    #endif
  }

  static bool can_reach(const xy_pos_t &pos, const bool probe_relative=true) { return can_reach(pos.x, pos.y, probe_relative); }

  static bool good_bounds(const xy_pos_t &lf, const xy_pos_t &rb) {
    return (
      #if IS_KINEMATIC
        can_reach(lf.x, 0) && can_reach(rb.x, 0) && can_reach(0, lf.y) && can_reach(0, rb.y)
      #else
        can_reach(lf) && can_reach(rb)
      #endif
    );
  }

  // Use offset_xy for read only access
  // More optimal the XY offset is known to always be zero.
  #if HAS_PROBE_XY_OFFSET
    static const xy_pos_t &offset_xy;
  #else
    static constexpr xy_pos_t offset_xy = xy_pos_t({ 0, 0 });   // See #16767
  #endif

  static bool deploy(const bool no_return=false) { return set_deployed(true, no_return); }
  static bool stow(const bool no_return=false)   { return set_deployed(false, no_return); }

  #if HAS_BED_PROBE || HAS_LEVELING
    #if IS_KINEMATIC
      static constexpr float probe_radius(const xy_pos_t &probe_offset_xy=offset_xy) {
        return float(PRINTABLE_RADIUS) - _MAX(PROBING_MARGIN, HYPOT(probe_offset_xy.x, probe_offset_xy.y));
      }
    #endif

    /**
     * The nozzle is only able to move within the physical bounds of the machine.
     * If the PROBE has an OFFSET Marlin may need to apply additional limits so
     * the probe can be prevented from going to unreachable points.
     *
     * e.g., If the PROBE is to the LEFT of the NOZZLE, it will be limited in how
     * close it can get the RIGHT edge of the bed (unless the nozzle is able move
     * far enough past the right edge).
     */
    static constexpr float _min_x(const xy_pos_t &probe_offset_xy=offset_xy) {
      return TERN(IS_KINEMATIC,
        (X_CENTER) - probe_radius(probe_offset_xy),
        _MAX((X_MIN_BED) + (PROBING_MARGIN_LEFT), (X_MIN_POS) + probe_offset_xy.x)
      );
    }
    static constexpr float _max_x(const xy_pos_t &probe_offset_xy=offset_xy) {
      return TERN(IS_KINEMATIC,
        (X_CENTER) + probe_radius(probe_offset_xy),
        _MIN((X_MAX_BED) - (PROBING_MARGIN_RIGHT), (X_MAX_POS) + probe_offset_xy.x)
      );
    }
    static constexpr float _min_y(const xy_pos_t &probe_offset_xy=offset_xy) {
      return TERN(IS_KINEMATIC,
        (Y_CENTER) - probe_radius(probe_offset_xy),
        _MAX((Y_MIN_BED) + (PROBING_MARGIN_FRONT), (Y_MIN_POS) + probe_offset_xy.y)
      );
    }
    static constexpr float _max_y(const xy_pos_t &probe_offset_xy=offset_xy) {
      return TERN(IS_KINEMATIC,
        (Y_CENTER) + probe_radius(probe_offset_xy),
        _MIN((Y_MAX_BED) - (PROBING_MARGIN_BACK), (Y_MAX_POS) + probe_offset_xy.y)
      );
    }

    static float min_x() { return _min_x() TERN_(NOZZLE_AS_PROBE, TERN_(HAS_HOME_OFFSET, - home_offset.x)); }
    static float max_x() { return _max_x() TERN_(NOZZLE_AS_PROBE, TERN_(HAS_HOME_OFFSET, - home_offset.x)); }
    static float min_y() { return _min_y() TERN_(NOZZLE_AS_PROBE, TERN_(HAS_HOME_OFFSET, - home_offset.y)); }
    static float max_y() { return _max_y() TERN_(NOZZLE_AS_PROBE, TERN_(HAS_HOME_OFFSET, - home_offset.y)); }

    // constexpr helpers used in build-time static_asserts, relying on default probe offsets.
    class build_time {
      static constexpr xyz_pos_t default_probe_xyz_offset = xyz_pos_t(
        #if HAS_BED_PROBE
          NOZZLE_TO_PROBE_OFFSET
        #else
          { 0 }
        #endif
      );
      static constexpr xy_pos_t default_probe_xy_offset = xy_pos_t({ default_probe_xyz_offset.x,  default_probe_xyz_offset.y });

    public:
      static constexpr bool can_reach(float x, float y) {
        #if IS_KINEMATIC
          return HYPOT2(x, y) <= sq(probe_radius(default_probe_xy_offset));
        #else
          return COORDINATE_OKAY(x, _min_x(default_probe_xy_offset) - fslop, _max_x(default_probe_xy_offset) + fslop)
              && COORDINATE_OKAY(y, _min_y(default_probe_xy_offset) - fslop, _max_y(default_probe_xy_offset) + fslop);
        #endif
      }

      static constexpr bool can_reach(const xy_pos_t &point) { return can_reach(point.x, point.y); }
    };

    #if NEEDS_THREE_PROBE_POINTS
      // Retrieve three points to probe the bed. Any type exposing set(X,Y) may be used.
      template <typename T>
      static void get_three_points(T points[3]) {
        #if HAS_FIXED_3POINT
          #define VALIDATE_PROBE_PT(N) static_assert(Probe::build_time::can_reach(xy_pos_t(PROBE_PT_##N)), \
            "PROBE_PT_" STRINGIFY(N) " is unreachable using default NOZZLE_TO_PROBE_OFFSET and PROBING_MARGIN.");
          VALIDATE_PROBE_PT(1); VALIDATE_PROBE_PT(2); VALIDATE_PROBE_PT(3);
          points[0] = xy_float_t(PROBE_PT_1);
          points[1] = xy_float_t(PROBE_PT_2);
          points[2] = xy_float_t(PROBE_PT_3);
        #else
          #if IS_KINEMATIC
            constexpr float SIN0 = 0.0, SIN120 = 0.866025, SIN240 = -0.866025,
                            COS0 = 1.0, COS120 = -0.5    , COS240 = -0.5;
            points[0] = xy_float_t({ (X_CENTER) + probe_radius() * COS0,   (Y_CENTER) + probe_radius() * SIN0 });
            points[1] = xy_float_t({ (X_CENTER) + probe_radius() * COS120, (Y_CENTER) + probe_radius() * SIN120 });
            points[2] = xy_float_t({ (X_CENTER) + probe_radius() * COS240, (Y_CENTER) + probe_radius() * SIN240 });
          #elif ENABLED(AUTO_BED_LEVELING_UBL)
            points[0] = xy_float_t({ _MAX(float(MESH_MIN_X), min_x()), _MAX(float(MESH_MIN_Y), min_y()) });
            points[1] = xy_float_t({ _MIN(float(MESH_MAX_X), max_x()), _MAX(float(MESH_MIN_Y), min_y()) });
            points[2] = xy_float_t({ (_MAX(float(MESH_MIN_X), min_x()) + _MIN(float(MESH_MAX_X), max_x())) / 2, _MIN(float(MESH_MAX_Y), max_y()) });
          #else
            points[0] = xy_float_t({ min_x(), min_y() });
            points[1] = xy_float_t({ max_x(), min_y() });
            points[2] = xy_float_t({ (min_x() + max_x()) / 2, max_y() });
          #endif
        #endif
      }
    #endif

  #endif // HAS_BED_PROBE

  #if HAS_Z_SERVO_PROBE
    static void servo_probe_init();
  #endif

  #if HAS_QUIET_PROBING
    static void set_probing_paused(const bool p);
  #endif

  #if ENABLED(PROBE_TARE)
    static void tare_init();
    static bool tare();
  #endif

  // Basic functions for Sensorless Homing and Probing
  #if HAS_DELTA_SENSORLESS_PROBING
    static void set_offset_sensorless_adj(const_float_t sz);
    static void refresh_largest_sensorless_adj();
  #endif

private:
  #if HAS_BED_PROBE
    static bool probe_down_to_z(const_float_t z, const_feedRate_t fr_mm_s);
    static float run_z_probe(const bool sanity_check=true, const_float_t z_min_point=Z_PROBE_LOW_POINT, const_float_t z_clearance=Z_TWEEN_SAFE_CLEARANCE);
  #endif
};

extern Probe probe;
