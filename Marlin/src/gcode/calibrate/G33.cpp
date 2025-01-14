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

#include "../../inc/MarlinConfig.h"

#if ENABLED(DELTA_AUTO_CALIBRATION)

#include "../gcode.h"
#include "../../module/delta.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/endstops.h"
#include "../../lcd/marlinui.h"

#if HAS_BED_PROBE
  #include "../../module/probe.h"
#endif

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

constexpr uint8_t _7P_STEP = 1,              // 7-point step - to change number of calibration points
                  _4P_STEP = _7P_STEP * 2,   // 4-point step
                  NPP      = _7P_STEP * 6;   // number of calibration points on the radius
enum CalEnum : char {                        // the 7 main calibration points - add definitions if needed
  CEN      = 0,
  __A      = 1,
  _AB      = __A + _7P_STEP,
  __B      = _AB + _7P_STEP,
  _BC      = __B + _7P_STEP,
  __C      = _BC + _7P_STEP,
  _CA      = __C + _7P_STEP,
};

#define LOOP_CAL_PT(VAR, S, N) for (uint8_t VAR=S; VAR<=NPP; VAR+=N)
#define F_LOOP_CAL_PT(VAR, S, N) for (float VAR=S; VAR<NPP+0.9999; VAR+=N)
#define I_LOOP_CAL_PT(VAR, S, N) for (float VAR=S; VAR>CEN+0.9999; VAR-=N)
#define LOOP_CAL_ALL(VAR) LOOP_CAL_PT(VAR, CEN, 1)
#define LOOP_CAL_RAD(VAR) LOOP_CAL_PT(VAR, __A, _7P_STEP)
#define LOOP_CAL_ACT(VAR, _4P, _OP) LOOP_CAL_PT(VAR, _OP ? _AB : __A, _4P ? _4P_STEP : _7P_STEP)

float lcd_probe_pt(const xy_pos_t &xy);

void ac_home() {
  endstops.enable(true);
  TERN_(IMPROVE_HOMING_RELIABILITY, planner.enable_stall_prevention(true));
  home_delta();
  TERN_(IMPROVE_HOMING_RELIABILITY, planner.enable_stall_prevention(false));
  endstops.not_homing();
}

void ac_setup(const bool reset_bed) {
  TERN_(HAS_BED_PROBE, probe.use_probing_tool());

  planner.synchronize();
  remember_feedrate_scaling_off();

  #if HAS_LEVELING
    if (reset_bed) reset_bed_level(); // After full calibration bed-level data is no longer valid
  #endif
}

void ac_cleanup() {
  TERN_(DELTA_HOME_TO_SAFE_ZONE, do_blocking_move_to_z(delta_clip_start_height));
  TERN_(HAS_BED_PROBE, probe.stow());
  restore_feedrate_and_scaling();
  TERN_(HAS_BED_PROBE, probe.use_probing_tool(false));
}

void print_signed_float(FSTR_P const prefix, const_float_t f) {
  SERIAL_ECHO(F("  "), prefix, C(':'));
  serial_offset(f);
}

/**
 * - Print the delta settings
 */
static void print_calibration_settings(const bool end_stops, const bool tower_angles) {
  SERIAL_ECHOPGM(".Height:", delta_height);
  if (end_stops) {
    print_signed_float(F("Ex"), delta_endstop_adj.a);
    print_signed_float(F("Ey"), delta_endstop_adj.b);
    print_signed_float(F("Ez"), delta_endstop_adj.c);
  }
  if (end_stops && tower_angles) {
    SERIAL_ECHOLNPGM("  Radius:", delta_radius);
    SERIAL_CHAR('.');
    SERIAL_ECHO_SP(13);
  }
  if (tower_angles) {
    print_signed_float(F("Tx"), delta_tower_angle_trim.a);
    print_signed_float(F("Ty"), delta_tower_angle_trim.b);
    print_signed_float(F("Tz"), delta_tower_angle_trim.c);
  }
  if (end_stops != tower_angles)
    SERIAL_ECHOPGM("  Radius:", delta_radius);

  SERIAL_EOL();
}

/**
 * - Print the probe results
 */
static void print_calibration_results(const float z_pt[NPP + 1], const bool tower_points, const bool opposite_points) {
  SERIAL_ECHOPGM(".    ");
  print_signed_float(F("c"), z_pt[CEN]);
  if (tower_points) {
    print_signed_float(F(" x"), z_pt[__A]);
    print_signed_float(F(" y"), z_pt[__B]);
    print_signed_float(F(" z"), z_pt[__C]);
  }
  if (tower_points && opposite_points) {
    SERIAL_EOL();
    SERIAL_CHAR('.');
    SERIAL_ECHO_SP(13);
  }
  if (opposite_points) {
    print_signed_float(F("yz"), z_pt[_BC]);
    print_signed_float(F("zx"), z_pt[_CA]);
    print_signed_float(F("xy"), z_pt[_AB]);
  }
  SERIAL_EOL();
}

/**
 * - Calculate the standard deviation from the zero plane
 */
static float std_dev_points(float z_pt[NPP + 1], const bool _0p_cal, const bool _1p_cal, const bool _4p_cal, const bool _4p_opp) {
  if (!_0p_cal) {
    float S2 = sq(z_pt[CEN]);
    int16_t N = 1;
    if (!_1p_cal) { // std dev from zero plane
      LOOP_CAL_ACT(rad, _4p_cal, _4p_opp) {
        S2 += sq(z_pt[rad]);
        N++;
      }
      return LROUND(SQRT(S2 / N) * 1000.0f) / 1000.0f + 0.00001f;
    }
  }
  return 0.00001f;
}

/**
 * - Probe a point
 */
static float calibration_probe(const xy_pos_t &xy, const bool stow, const bool probe_at_offset) {
  #if HAS_BED_PROBE
    return probe.probe_at_point(xy, stow ? PROBE_PT_STOW : PROBE_PT_RAISE, 0, probe_at_offset, false, Z_PROBE_LOW_POINT, Z_TWEEN_SAFE_CLEARANCE, true);
  #else
    UNUSED(stow);
    return lcd_probe_pt(xy);
  #endif
}

/**
 * - Probe a grid
 */
static bool probe_calibration_points(float z_pt[NPP + 1], const int8_t probe_points, const float dcr, const bool towers_set, const bool stow_after_each, const bool probe_at_offset) {
  const bool _0p_calibration      = probe_points == 0,
             _1p_calibration      = probe_points == 1 || probe_points == -1,
             _4p_calibration      = probe_points == 2,
             _4p_opposite_points  = _4p_calibration && !towers_set,
             _7p_calibration      = probe_points >= 3,
             _7p_no_intermediates = probe_points == 3,
             _7p_1_intermediates  = probe_points == 4,
             _7p_2_intermediates  = probe_points == 5,
             _7p_4_intermediates  = probe_points == 6,
             _7p_6_intermediates  = probe_points == 7,
             _7p_8_intermediates  = probe_points == 8,
             _7p_11_intermediates = probe_points == 9,
             _7p_14_intermediates = probe_points == 10,
             _7p_intermed_points  = probe_points >= 4,
             _7p_6_center         = probe_points >= 5 && probe_points <= 7,
             _7p_9_center         = probe_points >= 8;

  LOOP_CAL_ALL(rad) z_pt[rad] = 0.0f;

  if (!_0p_calibration) {

    if (!_7p_no_intermediates && !_7p_4_intermediates && !_7p_11_intermediates) { // probe the center
      const xy_pos_t center{0};
      z_pt[CEN] += calibration_probe(center, stow_after_each, probe_at_offset);
      if (isnan(z_pt[CEN])) return false;
    }

    if (_7p_calibration) { // probe extra center points
      const float start  = _7p_9_center ? float(_CA) + _7P_STEP / 3.0f : _7p_6_center ? float(_CA) : float(__C),
                  steps  = _7p_9_center ? _4P_STEP / 3.0f : _7p_6_center ? _7P_STEP : _4P_STEP;
      I_LOOP_CAL_PT(rad, start, steps) {
        const float a = RADIANS(210 + (360 / NPP) *  (rad - 1)),
                    r = dcr * 0.1;
        const xy_pos_t vec = { cos(a), sin(a) };
        z_pt[CEN] += calibration_probe(vec * r, stow_after_each, probe_at_offset);
        if (isnan(z_pt[CEN])) return false;
     }
      z_pt[CEN] /= float(_7p_2_intermediates ? 7 : probe_points);
    }

    if (!_1p_calibration) {  // probe the radius
      const CalEnum start  = _4p_opposite_points ? _AB : __A;
      const float   steps  = _7p_14_intermediates ? _7P_STEP / 15.0f : // 15r * 6 + 10c = 100
                             _7p_11_intermediates ? _7P_STEP / 12.0f : // 12r * 6 +  9c = 81
                             _7p_8_intermediates  ? _7P_STEP /  9.0f : //  9r * 6 + 10c = 64
                             _7p_6_intermediates  ? _7P_STEP /  7.0f : //  7r * 6 +  7c = 49
                             _7p_4_intermediates  ? _7P_STEP /  5.0f : //  5r * 6 +  6c = 36
                             _7p_2_intermediates  ? _7P_STEP /  3.0f : //  3r * 6 +  7c = 25
                             _7p_1_intermediates  ? _7P_STEP /  2.0f : //  2r * 6 +  4c = 16
                             _7p_no_intermediates ? _7P_STEP :        //  1r * 6 +  3c = 9
                             _4P_STEP;                                // .5r * 6 +  1c = 4
      bool zig_zag = true;
      F_LOOP_CAL_PT(rad, start, _7p_9_center ? steps * 3 : steps) {
        const int8_t offset = _7p_9_center ? 2 : 0;
        for (int8_t circle = 0; circle <= offset; circle++) {
          const float a = RADIANS(210 + (360 / NPP) *  (rad - 1)),
                      r = dcr * (1 - 0.1 * (zig_zag ? offset - circle : circle)),
                      interpol = FMOD(rad, 1);
          const xy_pos_t vec = { cos(a), sin(a) };
          const float z_temp = calibration_probe(vec * r, stow_after_each, probe_at_offset);
          if (isnan(z_temp)) return false;
          // split probe point to neighbouring calibration points
          z_pt[uint8_t(LROUND(rad - interpol + NPP - 1)) % NPP + 1] += z_temp * sq(cos(RADIANS(interpol * 90)));
          z_pt[uint8_t(LROUND(rad - interpol))           % NPP + 1] += z_temp * sq(sin(RADIANS(interpol * 90)));
        }
        zig_zag = !zig_zag;
      }
      if (_7p_intermed_points)
        LOOP_CAL_RAD(rad)
          z_pt[rad] /= _7P_STEP / steps;

      do_blocking_move_to_xy(0.0f, 0.0f);
    }
  }
  return true;
}

/**
 * kinematics routines and auto tune matrix scaling parameters:
 * see https://github.com/LVD-AC/Marlin-AC/tree/1.1.x-AC/documentation for
 *  - formulae for approximative forward kinematics in the end-stop displacement matrix
 *  - definition of the matrix scaling parameters
 */
static void reverse_kinematics_probe_points(float z_pt[NPP + 1], abc_float_t mm_at_pt_axis[NPP + 1], const float dcr) {
  xyz_pos_t pos{0};

  LOOP_CAL_ALL(rad) {
    const float a = RADIANS(210 + (360 / NPP) *  (rad - 1)),
                r = (rad == CEN ? 0.0f : dcr);
    pos.set(cos(a) * r, sin(a) * r, z_pt[rad]);
    inverse_kinematics(pos);
    mm_at_pt_axis[rad] = delta;
  }
}

static void forward_kinematics_probe_points(abc_float_t mm_at_pt_axis[NPP + 1], float z_pt[NPP + 1], const float dcr) {
  const float r_quot = dcr / delta_radius;

  #define ZPP(N,I,A) (((1.0f + r_quot * (N)) / 3.0f) * mm_at_pt_axis[I].A)
  #define Z00(I, A) ZPP( 0, I, A)
  #define Zp1(I, A) ZPP(+1, I, A)
  #define Zm1(I, A) ZPP(-1, I, A)
  #define Zp2(I, A) ZPP(+2, I, A)
  #define Zm2(I, A) ZPP(-2, I, A)

  z_pt[CEN] = Z00(CEN, a) + Z00(CEN, b) + Z00(CEN, c);
  z_pt[__A] = Zp2(__A, a) + Zm1(__A, b) + Zm1(__A, c);
  z_pt[__B] = Zm1(__B, a) + Zp2(__B, b) + Zm1(__B, c);
  z_pt[__C] = Zm1(__C, a) + Zm1(__C, b) + Zp2(__C, c);
  z_pt[_BC] = Zm2(_BC, a) + Zp1(_BC, b) + Zp1(_BC, c);
  z_pt[_CA] = Zp1(_CA, a) + Zm2(_CA, b) + Zp1(_CA, c);
  z_pt[_AB] = Zp1(_AB, a) + Zp1(_AB, b) + Zm2(_AB, c);
}

static void calc_kinematics_diff_probe_points(float z_pt[NPP + 1], const float dcr, abc_float_t delta_e, const float delta_r, abc_float_t delta_t) {
  const float z_center = z_pt[CEN];
  abc_float_t diff_mm_at_pt_axis[NPP + 1], new_mm_at_pt_axis[NPP + 1];

  reverse_kinematics_probe_points(z_pt, diff_mm_at_pt_axis, dcr);

  delta_radius += delta_r;
  delta_tower_angle_trim += delta_t;
  recalc_delta_settings();
  reverse_kinematics_probe_points(z_pt, new_mm_at_pt_axis, dcr);

  LOOP_CAL_ALL(rad) diff_mm_at_pt_axis[rad] -= new_mm_at_pt_axis[rad] + delta_e;
  forward_kinematics_probe_points(diff_mm_at_pt_axis, z_pt, dcr);

  LOOP_CAL_RAD(rad) z_pt[rad] -= z_pt[CEN] - z_center;
  z_pt[CEN] = z_center;

  delta_radius -= delta_r;
  delta_tower_angle_trim -= delta_t;
  recalc_delta_settings();
}

static float auto_tune_h(const float dcr) {
  const float r_quot = dcr / delta_radius;
  return RECIPROCAL(r_quot / (2.0f / 3.0f));  // (2/3)/CR
}

static float auto_tune_r(const float dcr) {
  constexpr float diff = 0.01f, delta_r = diff;
  float r_fac = 0.0f, z_pt[NPP + 1] = { 0.0f };
  abc_float_t delta_e = { 0.0f }, delta_t = { 0.0f };

  calc_kinematics_diff_probe_points(z_pt, dcr, delta_e, delta_r, delta_t);
  r_fac = -(z_pt[__A] + z_pt[__B] + z_pt[__C] + z_pt[_BC] + z_pt[_CA] + z_pt[_AB]) / 6.0f;
  r_fac = diff / r_fac / 3.0f; // 1/(3*delta_Z)
  return r_fac;
}

static float auto_tune_a(const float dcr) {
  constexpr float diff = 0.01f, delta_r = 0.0f;
  float a_fac = 0.0f, z_pt[NPP + 1] = { 0.0f };
  abc_float_t delta_e = { 0.0f }, delta_t = { 0.0f };

  delta_t.reset();
  LOOP_NUM_AXES(axis) {
    delta_t[axis] = diff;
    calc_kinematics_diff_probe_points(z_pt, dcr, delta_e, delta_r, delta_t);
    delta_t[axis] = 0;
    a_fac += z_pt[uint8_t((axis * _4P_STEP) - _7P_STEP + NPP) % NPP + 1] / 6.0f;
    a_fac -= z_pt[uint8_t((axis * _4P_STEP) + 1 + _7P_STEP)] / 6.0f;
  }
  a_fac = diff / a_fac / 3.0f; // 1/(3*delta_Z)
  return a_fac;
}

/**
 * G33 - Delta '1-4-7-point' Auto-Calibration
 *       Calibrate height, z_offset, endstops, delta radius, and tower angles.
 *
 * Parameters:
 *
 *   Pn  Number of probe points:
 *      P0       Normalizes calibration.
 *      P1       Calibrates height only with center probe.
 *      P2       Probe center and towers. Calibrate height, endstops and delta radius.
 *      P3       Probe all positions: center, towers and opposite towers. Calibrate all.
 *      P4-P10   Probe all positions at different intermediate locations and average them.
 *
 *   Rn.nn  Temporary reduce the probe grid by the specified amount (mm)
 *
 *   T   Don't calibrate tower angle corrections
 *
 *   Cn.nn  Calibration precision; when omitted calibrates to maximum precision
 *
 *   Fn  Force to run at least n iterations and take the best result
 *
 *   Vn  Verbose level:
 *      V0  Dry-run mode. Report settings and probe results. No calibration.
 *      V1  Report start and end settings only
 *      V2  Report settings at each iteration
 *      V3  Report settings and probe results
 *
 *   E   Engage the probe for each point
 *
 *   O   Probe at offsetted probe positions (this is wrong but it seems to work)
 *
 * With SENSORLESS_PROBING:
 *   Use these flags to calibrate stall sensitivity: (e.g., `G33 P1 Y Z` to calibrate X only.)
 *   X   Don't activate stallguard on X.
 *   Y   Don't activate stallguard on Y.
 *   Z   Don't activate stallguard on Z.
 *
 *   S   Save offset_sensorless_adj
 */
void GcodeSuite::G33() {

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_PROBE));

  const int8_t probe_points = parser.intval('P', DELTA_CALIBRATION_DEFAULT_POINTS);
  if (!WITHIN(probe_points, 0, 10)) {
    SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(P)oints implausible (0-10)."));
    return;
  }

  const bool probe_at_offset = TERN0(HAS_PROBE_XY_OFFSET, parser.seen_test('O')),
                  towers_set = !parser.seen_test('T');

  // The calibration radius is set to a calculated value
  float dcr = probe_at_offset ? PRINTABLE_RADIUS : PRINTABLE_RADIUS - PROBING_MARGIN;
  #if HAS_PROBE_XY_OFFSET
    const float total_offset = HYPOT(probe.offset_xy.x, probe.offset_xy.y);
    dcr -= probe_at_offset ? _MAX(total_offset, PROBING_MARGIN) : total_offset;
  #endif
  NOMORE(dcr, PRINTABLE_RADIUS);
  if (parser.seenval('R')) dcr -= _MAX(parser.value_float(), 0.0f);
  TERN_(HAS_DELTA_SENSORLESS_PROBING, dcr *= sensorless_radius_factor);

  const float calibration_precision = parser.floatval('C', 0.0f);
  if (calibration_precision < 0) {
    SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(C)alibration precision implausible (>=0)."));
    return;
  }

  const int8_t force_iterations = parser.intval('F', 0);
  if (!WITHIN(force_iterations, 0, 30)) {
    SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(F)orce iteration implausible (0-30)."));
    return;
  }

  const int8_t verbose_level = parser.byteval('V', 1);
  if (!WITHIN(verbose_level, 0, 3)) {
    SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(V)erbose level implausible (0-3)."));
    return;
  }

  const bool stow_after_each = parser.seen_test('E');

  #if HAS_DELTA_SENSORLESS_PROBING
    probe.test_sensitivity = { !parser.seen_test('X'), !parser.seen_test('Y'), !parser.seen_test('Z') };
    const bool do_save_offset_adj = parser.seen_test('S');
  #endif

  const bool _0p_calibration      = probe_points == 0,
             _1p_calibration      = probe_points == 1 || probe_points == -1,
             _4p_calibration      = probe_points == 2,
             _4p_opposite_points  = _4p_calibration && !towers_set,
             _7p_9_center         = probe_points >= 8,
             _tower_results       = (_4p_calibration && towers_set) || probe_points >= 3,
             _opposite_results    = (_4p_calibration && !towers_set) || probe_points >= 3,
             _endstop_results     = probe_points != 1 && probe_points != -1 && probe_points != 0,
             _angle_results       = probe_points >= 3 && towers_set;
  int8_t iterations = 0;
  float test_precision,
        zero_std_dev = (verbose_level ? 999.0f : 0.0f), // 0.0 in dry-run mode : forced end
        zero_std_dev_min = zero_std_dev,
        zero_std_dev_old = zero_std_dev,
        h_factor, r_factor, a_factor,
        r_old = delta_radius,
        h_old = delta_height;

  abc_pos_t e_old = delta_endstop_adj, a_old = delta_tower_angle_trim;

  SERIAL_ECHOLNPGM("G33 Auto Calibrate");

  // Report settings
  FSTR_P const checkingac = F("Checking... AC");
  SERIAL_ECHO(checkingac, F(" at radius:"), dcr);
  if (verbose_level == 0) SERIAL_ECHOPGM(" (DRY-RUN)");
  SERIAL_EOL();
  ui.set_status(checkingac);

  print_calibration_settings(_endstop_results, _angle_results);

  ac_setup(!_0p_calibration && !_1p_calibration);

  if (!_0p_calibration) ac_home();

  #if HAS_DELTA_SENSORLESS_PROBING
    if (verbose_level > 0 && do_save_offset_adj) {
      offset_sensorless_adj.reset();
      auto caltower = [&](Probe::sense_bool_t s) {
        float z_at_pt[NPP + 1];
        LOOP_CAL_ALL(rad) z_at_pt[rad] = 0.0f;
        probe.test_sensitivity = s;
        if (probe_calibration_points(z_at_pt, 1, dcr, false, false, probe_at_offset))
          probe.set_offset_sensorless_adj(z_at_pt[CEN]);
      };
      caltower({ true, false, false }); // A
      caltower({ false, true, false }); // B
      caltower({ false, false, true }); // C

      probe.test_sensitivity = { true, true, true }; // reset to all
    }
  #endif

  do { // start iterations

    float z_at_pt[NPP + 1] = { 0.0f };

    test_precision = zero_std_dev_old != 999.0f ? (zero_std_dev + zero_std_dev_old) / 2.0f : zero_std_dev;
    iterations++;

    // Probe the points
    zero_std_dev_old = zero_std_dev;
    if (!probe_calibration_points(z_at_pt, probe_points, dcr, towers_set, stow_after_each, probe_at_offset)) {
      SERIAL_ECHOLNPGM("Correct delta settings with M665 and M666");
      return ac_cleanup();
    }
    zero_std_dev = std_dev_points(z_at_pt, _0p_calibration, _1p_calibration, _4p_calibration, _4p_opposite_points);

    // Solve matrices

    if ((zero_std_dev < test_precision || iterations <= force_iterations) && zero_std_dev > calibration_precision) {

      #if !HAS_BED_PROBE
        test_precision = 0.0f; // forced end
      #endif

      if (zero_std_dev < zero_std_dev_min) {
        // set roll-back point
        e_old = delta_endstop_adj;
        r_old = delta_radius;
        h_old = delta_height;
        a_old = delta_tower_angle_trim;
      }

      abc_float_t e_delta = { 0.0f }, t_delta = { 0.0f };
      float r_delta = 0.0f;

      /**
       * convergence matrices:
       * see https://github.com/LVD-AC/Marlin-AC/tree/1.1.x-AC/documentation for
       *  - definition of the matrix scaling parameters
       *  - matrices for 4 and 7 point calibration
       */
      #define ZP(N,I) ((N) * z_at_pt[I] / 4.0f) // 4.0 = divider to normalize to integers
      #define Z12(I) ZP(12, I)
      #define Z4(I) ZP(4, I)
      #define Z2(I) ZP(2, I)
      #define Z1(I) ZP(1, I)
      #define Z0(I) ZP(0, I)

      // calculate factors
      if (_7p_9_center) dcr *= 0.9f;
      h_factor = auto_tune_h(dcr);
      r_factor = auto_tune_r(dcr);
      a_factor = auto_tune_a(dcr);
      if (_7p_9_center) dcr /= 0.9f;

      switch (probe_points) {
        case 0:
          test_precision = 0.0f; // forced end
          break;

        case 1:
          test_precision = 0.0f; // forced end
          LOOP_NUM_AXES(axis) e_delta[axis] = +Z4(CEN);
          break;

        case 2:
          if (towers_set) { // see 4 point calibration (towers) matrix
            e_delta.set((+Z4(__A) -Z2(__B) -Z2(__C)) * h_factor  +Z4(CEN),
                        (-Z2(__A) +Z4(__B) -Z2(__C)) * h_factor  +Z4(CEN),
                        (-Z2(__A) -Z2(__B) +Z4(__C)) * h_factor  +Z4(CEN));
            r_delta   = (+Z4(__A) +Z4(__B) +Z4(__C) -Z12(CEN)) * r_factor;
          }
          else { // see 4 point calibration (opposites) matrix
            e_delta.set((-Z4(_BC) +Z2(_CA) +Z2(_AB)) * h_factor  +Z4(CEN),
                        (+Z2(_BC) -Z4(_CA) +Z2(_AB)) * h_factor  +Z4(CEN),
                        (+Z2(_BC) +Z2(_CA) -Z4(_AB)) * h_factor  +Z4(CEN));
            r_delta   = (+Z4(_BC) +Z4(_CA) +Z4(_AB) -Z12(CEN)) * r_factor;
          }
          break;

        default: // see 7 point calibration (towers & opposites) matrix
          e_delta.set((+Z2(__A) -Z1(__B) -Z1(__C) -Z2(_BC) +Z1(_CA) +Z1(_AB)) * h_factor  +Z4(CEN),
                      (-Z1(__A) +Z2(__B) -Z1(__C) +Z1(_BC) -Z2(_CA) +Z1(_AB)) * h_factor  +Z4(CEN),
                      (-Z1(__A) -Z1(__B) +Z2(__C) +Z1(_BC) +Z1(_CA) -Z2(_AB)) * h_factor  +Z4(CEN));
          r_delta   = (+Z2(__A) +Z2(__B) +Z2(__C) +Z2(_BC) +Z2(_CA) +Z2(_AB) -Z12(CEN)) * r_factor;

          if (towers_set) { // see 7 point tower angle calibration (towers & opposites) matrix
            t_delta.set((+Z0(__A) -Z4(__B) +Z4(__C) +Z0(_BC) -Z4(_CA) +Z4(_AB) +Z0(CEN)) * a_factor,
                        (+Z4(__A) +Z0(__B) -Z4(__C) +Z4(_BC) +Z0(_CA) -Z4(_AB) +Z0(CEN)) * a_factor,
                        (-Z4(__A) +Z4(__B) +Z0(__C) -Z4(_BC) +Z4(_CA) +Z0(_AB) +Z0(CEN)) * a_factor);
          }
          break;
      }
      delta_endstop_adj += e_delta;
      delta_radius += r_delta;
      delta_tower_angle_trim += t_delta;
    }
    else if (zero_std_dev >= test_precision) {
      // roll back
      delta_endstop_adj = e_old;
      delta_radius = r_old;
      delta_height = h_old;
      delta_tower_angle_trim = a_old;
    }

    if (verbose_level != 0) {                                    // !dry run

      // Normalize angles to least-squares
      if (_angle_results) {
        float a_sum = 0.0f;
        LOOP_NUM_AXES(axis) a_sum += delta_tower_angle_trim[axis];
        LOOP_NUM_AXES(axis) delta_tower_angle_trim[axis] -= a_sum / 3.0f;
      }

      // adjust delta_height and endstops by the max amount
      const float z_temp = _MAX(delta_endstop_adj.a, delta_endstop_adj.b, delta_endstop_adj.c);
      delta_height -= z_temp;
      LOOP_NUM_AXES(axis) delta_endstop_adj[axis] -= z_temp;
    }
    recalc_delta_settings();
    NOMORE(zero_std_dev_min, zero_std_dev);

    // print report

    if (verbose_level == 3 || verbose_level == 0) {
      print_calibration_results(z_at_pt, _tower_results, _opposite_results);
      #if HAS_DELTA_SENSORLESS_PROBING
        if (verbose_level == 0 && probe_points == 1) {
          if (do_save_offset_adj)
            probe.set_offset_sensorless_adj(z_at_pt[CEN]);
          else
            probe.refresh_largest_sensorless_adj();
        }
      #endif
    }

    if (verbose_level != 0) { // !dry run
      if ((zero_std_dev >= test_precision && iterations > force_iterations) || zero_std_dev <= calibration_precision) { // end iterations
        SERIAL_ECHOPGM("Calibration OK");
        SERIAL_ECHO_SP(32);
        #if HAS_BED_PROBE
          if (zero_std_dev >= test_precision && !_1p_calibration && !_0p_calibration)
            SERIAL_ECHOPGM("rolling back.");
          else
        #endif
          {
            SERIAL_ECHOPGM("std dev:", p_float_t(zero_std_dev_min, 3));
          }
        SERIAL_EOL();

        MString<21> msg(F("Calibration sd:"));
        if (zero_std_dev_min < 1)
          msg.appendf(F("0.%03i"), (int)LROUND(zero_std_dev_min * 1000.0f));
        else
          msg.appendf(F("%03i.x"), (int)LROUND(zero_std_dev_min));
        ui.set_status(msg);
        print_calibration_settings(_endstop_results, _angle_results);
        SERIAL_ECHOLNPGM("Save with M500 and/or copy to Configuration.h");
      }
      else { // !end iterations
        SString<15> msg;
        if (iterations < 31)
          msg.setf(F("Iteration : %02i"), (unsigned int)iterations);
        else
          msg.set(F("No convergence"));
        msg.echo();
        SERIAL_ECHO_SP(32);
        SERIAL_ECHOLNPGM("std dev:", p_float_t(zero_std_dev, 3));
        ui.set_status(msg);
        if (verbose_level > 1)
          print_calibration_settings(_endstop_results, _angle_results);
      }
    }
    else { // dry run
      FSTR_P const enddryrun = F("End DRY-RUN");
      SERIAL_ECHO(enddryrun);
      SERIAL_ECHO_SP(35);
      SERIAL_ECHOLNPGM("std dev:", p_float_t(zero_std_dev, 3));
      MString<30> msg(enddryrun, F(" sd:"));
      if (zero_std_dev < 1)
        msg.appendf(F("0.%03i"), (int)LROUND(zero_std_dev * 1000.0f));
      else
        msg.appendf(F("%03i.x"), (int)LROUND(zero_std_dev));
      ui.set_status(msg);
    }
    ac_home();
  }
  while (((zero_std_dev < test_precision && iterations < 31) || iterations <= force_iterations) && zero_std_dev > calibration_precision);

  ac_cleanup();

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_IDLE));
  #if HAS_DELTA_SENSORLESS_PROBING
    probe.test_sensitivity = { true, true, true };
  #endif
}

#endif // DELTA_AUTO_CALIBRATION
