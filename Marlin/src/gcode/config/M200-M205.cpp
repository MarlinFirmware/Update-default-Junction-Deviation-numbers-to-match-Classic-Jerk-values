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

#include "../gcode.h"
#include "../../MarlinCore.h"
#include "../../module/planner.h"

#if DISABLED(NO_VOLUMETRICS)

  /**
   * M200: Set filament diameter and set E axis units to cubic units
   *
   *    T<extruder> - Optional extruder number. Current extruder if omitted.
   *    D<linear>   - Set filament diameter and enable. D0 disables volumetric.
   *    S<bool>     - Turn volumetric ON or OFF.
   *
   * With VOLUMETRIC_EXTRUDER_LIMIT:
   *
   *    L<float>    - Volumetric extruder limit (in mm^3/sec). L0 disables the limit.
   */
  void GcodeSuite::M200() {
    if (!parser.seen("DST" TERN_(VOLUMETRIC_EXTRUDER_LIMIT, "L")))
      return M200_report();

    const int_fast8_t target_extruder = get_target_extruder_from_command();
    if (target_extruder < 0) return;

    bool vol_enable = parser.volumetric_enabled,
         can_enable = true;

    if (parser.seenval('D')) {
      const float dval = parser.value_linear_units();
      if (dval) { // Set filament size for volumetric calculation
        planner.set_filament_size(target_extruder, dval);
        vol_enable = true;    // Dn = enable for compatibility
      }
      else
        can_enable = false;   // D0 = disable for compatibility
    }

    // Enable or disable with S1 / S0
    parser.volumetric_enabled = can_enable && parser.boolval('S', vol_enable);

    #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
      if (parser.seenval('L')) {
        // Set volumetric limit (in mm^3/sec)
        const float lval = parser.value_float();
        if (WITHIN(lval, 0, VOLUMETRIC_EXTRUDER_LIMIT_MAX))
          planner.set_volumetric_extruder_limit(target_extruder, lval);
        else
          SERIAL_ECHOLNPGM(GCODE_ERR_MSG("L value out of range (0-" STRINGIFY(VOLUMETRIC_EXTRUDER_LIMIT_MAX) ")."));
      }
    #endif

    planner.calculate_volumetric_multipliers();
  }

  void GcodeSuite::M200_report(const bool forReplay/*=true*/) {
    TERN_(MARLIN_SMALL_BUILD, return);

    if (!forReplay) {
      report_heading(forReplay, F(STR_FILAMENT_SETTINGS), false);
      if (!parser.volumetric_enabled) SERIAL_ECHOPGM(" (Disabled):");
      SERIAL_EOL();
      report_echo_start(forReplay);
    }

    #if EXTRUDERS == 1
    {
      SERIAL_ECHOLNPGM(
        "  M200 S", parser.volumetric_enabled, " D", LINEAR_UNIT(planner.filament_size[0])
        #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
          , " L", LINEAR_UNIT(planner.volumetric_extruder_limit[0])
        #endif
      );
    }
    #else
      SERIAL_ECHOLNPGM("  M200 S", parser.volumetric_enabled);
      EXTRUDER_LOOP() {
        report_echo_start(forReplay);
        SERIAL_ECHOLNPGM(
          "  M200 T", e, " D", LINEAR_UNIT(planner.filament_size[e])
          #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
            , " L", LINEAR_UNIT(planner.volumetric_extruder_limit[e])
          #endif
        );
      }
    #endif
  }

#endif // !NO_VOLUMETRICS

/**
 * M201: Set max acceleration in units/s^2 for print moves.
 *
 *  X<accel> : Max Acceleration for X
 *  Y<accel> : Max Acceleration for Y
 *  Z<accel> : Max Acceleration for Z
 *       ... : etc
 *  E<accel> : Max Acceleration for Extruder
 *  T<index> : Extruder index to set
 *
 * With XY_FREQUENCY_LIMIT:
 *  F<Hz>      : Frequency limit for XY...IJKUVW
 *  S<percent> : Speed factor percentage.
 */
void GcodeSuite::M201() {
  if (!parser.seen("T" STR_AXES_LOGICAL TERN_(XY_FREQUENCY_LIMIT, "FS")))
    return M201_report();

  const int_fast8_t target_extruder = get_target_extruder_from_command();
  if (target_extruder < 0) return;

  #ifdef XY_FREQUENCY_LIMIT
    if (parser.seenval('F')) planner.set_frequency_limit(parser.value_byte());
    if (parser.seenval('S')) planner.xy_freq_min_speed_factor = constrain(parser.value_float(), 1, 100) / 100;
  #endif

  LOOP_LOGICAL_AXES(i) {
    if (parser.seenval(AXIS_CHAR(i))) {
      const AxisEnum a = TERN(HAS_EXTRUDERS, (i == E_AXIS ? E_AXIS_N(target_extruder) : (AxisEnum)i), (AxisEnum)i);
      planner.set_max_acceleration(a, parser.value_axis_units(a));
    }
  }
}

void GcodeSuite::M201_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F(STR_MAX_ACCELERATION));
  #if NUM_AXES
    SERIAL_ECHOPGM_P(
      LIST_N(DOUBLE(NUM_AXES),
        PSTR("  M201 X"), LINEAR_UNIT(planner.settings.max_acceleration_mm_per_s2[X_AXIS]),
        SP_Y_STR, LINEAR_UNIT(planner.settings.max_acceleration_mm_per_s2[Y_AXIS]),
        SP_Z_STR, LINEAR_UNIT(planner.settings.max_acceleration_mm_per_s2[Z_AXIS]),
        SP_I_STR, I_AXIS_UNIT(planner.settings.max_acceleration_mm_per_s2[I_AXIS]),
        SP_J_STR, J_AXIS_UNIT(planner.settings.max_acceleration_mm_per_s2[J_AXIS]),
        SP_K_STR, K_AXIS_UNIT(planner.settings.max_acceleration_mm_per_s2[K_AXIS]),
        SP_U_STR, U_AXIS_UNIT(planner.settings.max_acceleration_mm_per_s2[U_AXIS]),
        SP_V_STR, V_AXIS_UNIT(planner.settings.max_acceleration_mm_per_s2[V_AXIS]),
        SP_W_STR, W_AXIS_UNIT(planner.settings.max_acceleration_mm_per_s2[W_AXIS])
      )
    );
  #endif

  #if HAS_EXTRUDERS && DISABLED(DISTINCT_E_FACTORS)
    SERIAL_ECHOPGM_P(SP_E_STR, VOLUMETRIC_UNIT(planner.settings.max_acceleration_mm_per_s2[E_AXIS]));
  #endif

  #if NUM_AXES || (HAS_EXTRUDERS && DISABLED(DISTINCT_E_FACTORS))
    SERIAL_EOL();
  #endif

  #if ENABLED(DISTINCT_E_FACTORS)
    for (uint_fast8_t i = 0; i < E_STEPPERS; ++i) {
      report_echo_start(forReplay);
      SERIAL_ECHOLNPGM_P(
          PSTR("  M201 T"), i
        , SP_E_STR, VOLUMETRIC_UNIT(planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(i)])
      );
    }
  #endif
}

/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 *
 *       With multiple extruders use T to specify which one.
 */
void GcodeSuite::M203() {
  if (!parser.seen("T" STR_AXES_LOGICAL))
    return M203_report();

  const int_fast8_t target_extruder = get_target_extruder_from_command();
  if (target_extruder < 0) return;

  LOOP_LOGICAL_AXES(i)
    if (parser.seenval(AXIS_CHAR(i))) {
      const AxisEnum a = TERN(HAS_EXTRUDERS, (i == E_AXIS ? E_AXIS_N(target_extruder) : (AxisEnum)i), (AxisEnum)i);
      planner.set_max_feedrate(a, parser.value_axis_units(a));
    }
}

void GcodeSuite::M203_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F(STR_MAX_FEEDRATES));
  #if NUM_AXES
    SERIAL_ECHOPGM_P(
      LIST_N(DOUBLE(NUM_AXES),
        PSTR("  M203 X"), LINEAR_UNIT(planner.settings.max_feedrate_mm_s[X_AXIS]),
        SP_Y_STR, LINEAR_UNIT(planner.settings.max_feedrate_mm_s[Y_AXIS]),
        SP_Z_STR, LINEAR_UNIT(planner.settings.max_feedrate_mm_s[Z_AXIS]),
        SP_I_STR, LINEAR_UNIT(planner.settings.max_feedrate_mm_s[I_AXIS]),
        SP_J_STR, LINEAR_UNIT(planner.settings.max_feedrate_mm_s[J_AXIS]),
        SP_K_STR, LINEAR_UNIT(planner.settings.max_feedrate_mm_s[K_AXIS]),
        SP_U_STR, LINEAR_UNIT(planner.settings.max_feedrate_mm_s[U_AXIS]),
        SP_V_STR, LINEAR_UNIT(planner.settings.max_feedrate_mm_s[V_AXIS]),
        SP_W_STR, LINEAR_UNIT(planner.settings.max_feedrate_mm_s[W_AXIS])
      )
    );
  #endif

  #if HAS_EXTRUDERS && DISABLED(DISTINCT_E_FACTORS)
    SERIAL_ECHOPGM_P(SP_E_STR, VOLUMETRIC_UNIT(planner.settings.max_feedrate_mm_s[E_AXIS]));
  #endif

  #if NUM_AXES || (HAS_EXTRUDERS && DISABLED(DISTINCT_E_FACTORS))
    SERIAL_EOL();
  #endif

  #if ENABLED(DISTINCT_E_FACTORS)
    for (uint_fast8_t i = 0; i < E_STEPPERS; ++i) {
      if (!forReplay) SERIAL_ECHO_START();
      SERIAL_ECHOLNPGM_P(
          PSTR("  M203 T"), i
        , SP_E_STR, VOLUMETRIC_UNIT(planner.settings.max_feedrate_mm_s[E_AXIS_N(i)])
      );
    }
  #endif
}

/**
 * M204: Set Accelerations in units/sec^2 (M204 P1200 R3000 T3000)
 *
 *    P<accel> Printing moves
 *    R<accel> Retract only (no X, Y, Z) moves
 *    T<accel> Travel (non printing) moves
 */
void GcodeSuite::M204() {
  if (!parser.seen("PRST"))
    return M204_report();
  else {
    //planner.synchronize();
    // 'S' for legacy compatibility. Should NOT BE USED for new development
    if (parser.seenval('S')) planner.settings.travel_acceleration = planner.settings.acceleration = parser.value_linear_units();
    if (parser.seenval('P')) planner.settings.acceleration = parser.value_linear_units();
    if (parser.seenval('R')) planner.settings.retract_acceleration = parser.value_linear_units();
    if (parser.seenval('T')) planner.settings.travel_acceleration = parser.value_linear_units();
  }
}

void GcodeSuite::M204_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F(STR_ACCELERATION_P_R_T));
  SERIAL_ECHOLNPGM_P(
      PSTR("  M204 P"), LINEAR_UNIT(planner.settings.acceleration)
    , PSTR(" R"), LINEAR_UNIT(planner.settings.retract_acceleration)
    , SP_T_STR, LINEAR_UNIT(planner.settings.travel_acceleration)
  );
}

#if AXIS_COLLISION('B')
  #define M205_MIN_SEG_TIME_PARAM 'D'
  #define M205_MIN_SEG_TIME_STR "D"
  #warning "Use 'M205 D' for Minimum Segment Time."
#else
  #define M205_MIN_SEG_TIME_PARAM 'B'
  #define M205_MIN_SEG_TIME_STR "B"
#endif

/**
 * M205: Set Advanced Settings
 *
 *    B<µs>          : Min Segment Time
 *    S<units/s>     : Min Feed Rate
 *    T<units/s>     : Min Travel Feed Rate
 *
 * With CLASSIC_JERK:
 *    X<units/sec^2> : Max X Jerk
 *    Y<units/sec^2> : Max Y Jerk
 *    Z<units/sec^2> : Max Z Jerk
 *               ... : etc
 *    E<units/sec^2> : Max E Jerk
 *
 * Without CLASSIC_JERK:
 *    J(mm)          : Junction Deviation
 */
void GcodeSuite::M205() {
  if (!parser.seen_any()) return M205_report();

  //planner.synchronize();
  if (parser.seenval(M205_MIN_SEG_TIME_PARAM)) planner.settings.min_segment_time_us = parser.value_ulong();
  if (parser.seenval('S')) planner.settings.min_feedrate_mm_s = parser.value_linear_units();
  if (parser.seenval('T')) planner.settings.min_travel_feedrate_mm_s = parser.value_linear_units();
  #if HAS_JUNCTION_DEVIATION
    if (parser.seenval('J')) {
      const float junc_dev = parser.value_linear_units();
      if (WITHIN(junc_dev, 0.01f, 0.3f)) {
        planner.junction_deviation_mm = junc_dev;
        TERN_(HAS_LINEAR_E_JERK, planner.recalculate_max_e_jerk());
      }
      else
        SERIAL_ERROR_MSG("?J out of range (0.01 to 0.3)");
    }
  #endif
  #if ENABLED(CLASSIC_JERK)
    bool seenZ = false;
    LOGICAL_AXIS_CODE(
      if (parser.seenval('E')) planner.set_max_jerk(E_AXIS, parser.value_linear_units()),
      if (parser.seenval('X')) planner.set_max_jerk(X_AXIS, parser.value_linear_units()),
      if (parser.seenval('Y')) planner.set_max_jerk(Y_AXIS, parser.value_linear_units()),
      if ((seenZ = parser.seenval('Z'))) planner.set_max_jerk(Z_AXIS, parser.value_linear_units()),
      if (parser.seenval(AXIS4_NAME)) planner.set_max_jerk(I_AXIS, parser.TERN(AXIS4_ROTATES, value_float, value_linear_units)()),
      if (parser.seenval(AXIS5_NAME)) planner.set_max_jerk(J_AXIS, parser.TERN(AXIS5_ROTATES, value_float, value_linear_units)()),
      if (parser.seenval(AXIS6_NAME)) planner.set_max_jerk(K_AXIS, parser.TERN(AXIS6_ROTATES, value_float, value_linear_units)()),
      if (parser.seenval(AXIS7_NAME)) planner.set_max_jerk(U_AXIS, parser.TERN(AXIS7_ROTATES, value_float, value_linear_units)()),
      if (parser.seenval(AXIS8_NAME)) planner.set_max_jerk(V_AXIS, parser.TERN(AXIS8_ROTATES, value_float, value_linear_units)()),
      if (parser.seenval(AXIS9_NAME)) planner.set_max_jerk(W_AXIS, parser.TERN(AXIS9_ROTATES, value_float, value_linear_units)())
    );
    #if HAS_MESH && DISABLED(LIMITED_JERK_EDITING)
      if (seenZ && planner.max_jerk.z <= 0.1f)
        SERIAL_ECHOLNPGM("WARNING! Low Z Jerk may lead to unwanted pauses.");
    #endif
  #endif // CLASSIC_JERK
}

void GcodeSuite::M205_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F(
    "Advanced (" M205_MIN_SEG_TIME_STR "<min_segment_time_us> S<min_feedrate> T<min_travel_feedrate>"
    TERN_(HAS_JUNCTION_DEVIATION, " J<junc_dev>")
    #if ENABLED(CLASSIC_JERK)
      NUM_AXIS_GANG(
        " X<max_jerk>", " Y<max_jerk>", " Z<max_jerk>",
        " " STR_I "<max_jerk>", " " STR_J "<max_jerk>", " " STR_K "<max_jerk>",
        " " STR_U "<max_jerk>", " " STR_V "<max_jerk>", " " STR_W "<max_jerk>"
      )
    #endif
    TERN_(HAS_CLASSIC_E_JERK, " E<max_jerk>")
    ")"
  ));
  SERIAL_ECHOLNPGM_P(
      PSTR("  M205 " M205_MIN_SEG_TIME_STR), LINEAR_UNIT(planner.settings.min_segment_time_us)
    , PSTR(" S"), LINEAR_UNIT(planner.settings.min_feedrate_mm_s)
    , SP_T_STR, LINEAR_UNIT(planner.settings.min_travel_feedrate_mm_s)
    #if HAS_JUNCTION_DEVIATION
      , PSTR(" J"), LINEAR_UNIT(planner.junction_deviation_mm)
    #endif
    #if ENABLED(CLASSIC_JERK) && NUM_AXES
      , LIST_N(DOUBLE(NUM_AXES),
        SP_X_STR, LINEAR_UNIT(planner.max_jerk.x),
        SP_Y_STR, LINEAR_UNIT(planner.max_jerk.y),
        SP_Z_STR, LINEAR_UNIT(planner.max_jerk.z),
        SP_I_STR, I_AXIS_UNIT(planner.max_jerk.i),
        SP_J_STR, J_AXIS_UNIT(planner.max_jerk.j),
        SP_K_STR, K_AXIS_UNIT(planner.max_jerk.k),
        SP_U_STR, U_AXIS_UNIT(planner.max_jerk.u),
        SP_V_STR, V_AXIS_UNIT(planner.max_jerk.v),
        SP_W_STR, W_AXIS_UNIT(planner.max_jerk.w)
      )
      #if HAS_CLASSIC_E_JERK
        , SP_E_STR, LINEAR_UNIT(planner.max_jerk.e)
      #endif
    #endif
  );
}
