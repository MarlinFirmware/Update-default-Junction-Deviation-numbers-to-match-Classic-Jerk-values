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
 * G29.cpp - Auto Bed Leveling
 */

#include "../../../inc/MarlinConfig.h"

#if HAS_ABL_NOT_UBL

#include "../../gcode.h"
#include "../../../feature/bedlevel/bedlevel.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../module/probe.h"
#include "../../../module/temperature.h"
#include "../../queue.h"

#if ENABLED(AUTO_BED_LEVELING_LINEAR)
  #include "../../../libs/least_squares_fit.h"
#endif

#if ABL_PLANAR
  #include "../../../libs/vector_3.h"
#endif
#if ENABLED(BD_SENSOR_PROBE_NO_STOP)
  #include "../../../feature/bedlevel/bdl/bdl.h"
#endif

#include "../../../lcd/marlinui.h"
#if ENABLED(EXTENSIBLE_UI)
  #include "../../../lcd/extui/ui_api.h"
#elif ENABLED(DWIN_CREALITY_LCD)
  #include "../../../lcd/e3v2/creality/dwin.h"
#elif ENABLED(SOVOL_SV06_RTS)
  #include "../../../lcd/sovol_rts/sovol_rts.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../../core/debug_out.h"

#if DISABLED(PROBE_MANUALLY) && FT_MOTION_DISABLE_FOR_PROBING
  #include "../../../module/ft_motion.h"
#endif

#if ABL_USES_GRID
  #if ENABLED(PROBE_Y_FIRST)
    #define PR_OUTER_VAR  abl.meshCount.x
    #define PR_OUTER_SIZE abl.grid_points.x
    #define PR_INNER_VAR  abl.meshCount.y
    #define PR_INNER_SIZE abl.grid_points.y
  #else
    #define PR_OUTER_VAR  abl.meshCount.y
    #define PR_OUTER_SIZE abl.grid_points.y
    #define PR_INNER_VAR  abl.meshCount.x
    #define PR_INNER_SIZE abl.grid_points.x
  #endif
#endif

/**
 * @brief Do some things before returning from G29.
 * @param retry : true if the G29 can and should be retried. false if the failure is too serious.
 * @param   did : true if the leveling procedure completed successfully.
 */
static void pre_g29_return(const bool retry, const bool did) {
  if (!retry) {
    TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_IDLE, false));
  }
  #if DISABLED(G29_RETRY_AND_RECOVER)
    if (!retry || did) {
      TERN_(DWIN_CREALITY_LCD, dwinLevelingDone());
      TERN_(EXTENSIBLE_UI, ExtUI::onLevelingDone());
    }
  #endif
}

#define G29_RETURN(retry, did) do{ \
  pre_g29_return(TERN0(G29_RETRY_AND_RECOVER, retry), did); \
  return TERN_(G29_RETRY_AND_RECOVER, retry); \
}while(0)

// For manual probing values persist over multiple G29
class G29_State {
public:
  int       verbose_level;
  xy_pos_t  probePos;
  float     measured_z;
  bool      dryrun,
            reenable;

  #if ANY(PROBE_MANUALLY, AUTO_BED_LEVELING_LINEAR)
    int abl_probe_index;
  #endif

  #if ENABLED(AUTO_BED_LEVELING_LINEAR)
    grid_count_t abl_points;
  #elif ENABLED(AUTO_BED_LEVELING_3POINT)
    static constexpr grid_count_t abl_points = 3;
  #elif ABL_USES_GRID
    static constexpr grid_count_t abl_points = GRID_MAX_POINTS;
  #endif

  #if ABL_USES_GRID

    xy_int8_t meshCount;

    xy_pos_t probe_position_lf,
             probe_position_rb;

    xy_float_t gridSpacing; // = { 0.0f, 0.0f }

    #if ENABLED(AUTO_BED_LEVELING_LINEAR)
      bool                topography_map;
      xy_uint8_t          grid_points;
    #else // Bilinear
      static constexpr xy_uint8_t grid_points = { GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y };
    #endif

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      float Z_offset;
      bed_mesh_t z_values;
    #endif

    #if ENABLED(AUTO_BED_LEVELING_LINEAR)
      int indexIntoAB[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
      float eqnAMatrix[GRID_MAX_POINTS * 3],  // "A" matrix of the linear system of equations
            eqnBVector[GRID_MAX_POINTS],      // "B" vector of Z points
            mean;
    #endif
  #endif
};

#if ABL_USES_GRID && ANY(AUTO_BED_LEVELING_3POINT, AUTO_BED_LEVELING_BILINEAR)
  constexpr xy_uint8_t G29_State::grid_points;
  constexpr grid_count_t G29_State::abl_points;
#endif

/**
 * G29: Detailed Z probe, probes the bed at 3 or more points.
 *      Will fail if the printer has not been homed with G28.
 *
 * Enhanced G29 Auto Bed Leveling Probe Routine
 *
 *  O  Auto-level only if needed
 *
 *  D  Dry-Run mode. Just evaluate the bed Topology - Don't apply
 *     or alter the bed level data. Useful to check the topology
 *     after a first run of G29.
 *
 *  J  Jettison current bed leveling data
 *
 *  V  Set the verbose level (0-4). Example: "G29 V3"
 *
 * Parameters With LINEAR leveling only:
 *
 *  P  Set the size of the grid that will be probed (P x P points).
 *     Example: "G29 P4"
 *
 *  X  Set the X size of the grid that will be probed (X x Y points).
 *     Example: "G29 X7 Y5"
 *
 *  Y  Set the Y size of the grid that will be probed (X x Y points).
 *
 *  T  Generate a Bed Topology Report. Example: "G29 P5 T" for a detailed report.
 *     This is useful for manual bed leveling and finding flaws in the bed (to
 *     assist with part placement).
 *     Not supported by non-linear delta printer bed leveling.
 *
 * Parameters With LINEAR and BILINEAR leveling only:
 *
 *  S  Set the XY travel speed between probe points (in units/min)
 *
 *  H  Set bounds to a centered square H x H units in size
 *
 *     -or-
 *
 *  F  Set the Front limit of the probing grid
 *  B  Set the Back limit of the probing grid
 *  L  Set the Left limit of the probing grid
 *  R  Set the Right limit of the probing grid
 *
 * Parameters with DEBUG_LEVELING_FEATURE only:
 *
 *  C  Make a totally fake grid with no actual probing.
 *     For use in testing when no probing is possible.
 *
 * Parameters with BILINEAR leveling only:
 *
 *  Z  Supply an additional Z probe offset
 *
 * Extra parameters with PROBE_MANUALLY:
 *
 *  To do manual probing simply repeat G29 until the procedure is complete.
 *  The first G29 accepts parameters. 'G29 Q' for status, 'G29 A' to abort.
 *
 *  Q  Query leveling and G29 state
 *
 *  A  Abort current leveling procedure
 *
 * Extra parameters with BILINEAR only:
 *
 *  W  Write a mesh point. (If G29 is idle.)
 *  I  X index for mesh point
 *  J  Y index for mesh point
 *  X  X for mesh point, overrides I
 *  Y  Y for mesh point, overrides J
 *  Z  Z for mesh point. Otherwise, raw current Z.
 *
 * Without PROBE_MANUALLY:
 *
 *  E  By default G29 will engage the Z probe, test the bed, then disengage.
 *     Include "E" to engage/disengage the Z probe for each sample.
 *     There's no extra effect if you have a fixed Z probe.
 */
G29_TYPE GcodeSuite::G29() {

  DEBUG_SECTION(log_G29, "G29", DEBUGGING(LEVELING));

  // Leveling state is persistent when done manually with multiple G29 commands
  TERN_(PROBE_MANUALLY, static) G29_State abl;

  // Keep powered steppers from timing out
  reset_stepper_timeout();

  // Q = Query leveling and G29 state
  const bool seenQ = ANY(DEBUG_LEVELING_FEATURE, PROBE_MANUALLY) && parser.seen_test('Q');

  // G29 Q is also available if debugging
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (seenQ || DEBUGGING(LEVELING)) log_machine_info();
    if (DISABLED(PROBE_MANUALLY) && seenQ) G29_RETURN(false, false);
  #endif

  // A = Abort manual probing
  // C<bool> = Generate fake probe points (DEBUG_LEVELING_FEATURE)
  const bool seenA = TERN0(PROBE_MANUALLY, parser.seen_test('A')),
         no_action = seenA || seenQ,
              faux = ENABLED(DEBUG_LEVELING_FEATURE) && DISABLED(PROBE_MANUALLY) ? parser.boolval('C') : no_action;

  // O = Don't level if leveling is already active
  if (!no_action && planner.leveling_active && parser.boolval('O')) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> Auto-level not needed, skip");
    G29_RETURN(false, false);
  }

  // Send 'N' to force homing before G29 (internal only)
  if (parser.seen_test('N'))
    process_subcommands_now(TERN(CAN_SET_LEVELING_AFTER_G28, F("G28L0"), FPSTR(G28_STR)));

  // Don't allow auto-leveling without homing first
  if (homing_needed_error()) G29_RETURN(false, false);

  // 3-point leveling gets points from the probe class
  #if ENABLED(AUTO_BED_LEVELING_3POINT)
    vector_3 points[3];
    probe.get_three_points(points);
  #endif

  // Storage for ABL Linear results
  #if ENABLED(AUTO_BED_LEVELING_LINEAR)
    struct linear_fit_data lsf_results;
  #endif

  // Set and report "probing" state to host
  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_PROBE, false));

  #if DISABLED(PROBE_MANUALLY) && FT_MOTION_DISABLE_FOR_PROBING
    FTMotionDisableInScope FT_Disabler; // Disable Fixed-Time Motion for probing
  #endif

  /**
   * On the initial G29 fetch command parameters.
   */
  if (!g29_in_progress) {

    probe.use_probing_tool();

    #ifdef EVENT_GCODE_BEFORE_G29
      if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Before G29 G-code: ", EVENT_GCODE_BEFORE_G29);
      gcode.process_subcommands_now(F(EVENT_GCODE_BEFORE_G29));
    #endif

    #if ANY(PROBE_MANUALLY, AUTO_BED_LEVELING_LINEAR)
      abl.abl_probe_index = -1;
    #endif

    abl.reenable = planner.leveling_active;

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

      const bool seen_w = parser.seen_test('W');
      if (seen_w) {
        if (!leveling_is_valid()) {
          SERIAL_ERROR_MSG("No bilinear grid");
          G29_RETURN(false, false);
        }

        const float rz = parser.seenval('Z') ? RAW_Z_POSITION(parser.value_linear_units()) : current_position.z;
        if (!WITHIN(rz, -10, 10)) {
          SERIAL_ERROR_MSG("Bad Z value");
          G29_RETURN(false, false);
        }

        const float rx = RAW_X_POSITION(parser.linearval('X', NAN)),
                    ry = RAW_Y_POSITION(parser.linearval('Y', NAN));
        int8_t i = parser.byteval('I', -1), j = parser.byteval('J', -1);

        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

        if (!isnan(rx) && !isnan(ry)) {
          // Get nearest i / j from rx / ry
          i = (rx - bedlevel.grid_start.x) / bedlevel.grid_spacing.x + 0.5f;
          j = (ry - bedlevel.grid_start.y) / bedlevel.grid_spacing.y + 0.5f;
          LIMIT(i, 0, (GRID_MAX_POINTS_X) - 1);
          LIMIT(j, 0, (GRID_MAX_POINTS_Y) - 1);
        }

        #pragma GCC diagnostic pop

        if (WITHIN(i, 0, (GRID_MAX_POINTS_X) - 1) && WITHIN(j, 0, (GRID_MAX_POINTS_Y) - 1)) {
          set_bed_leveling_enabled(false);
          bedlevel.z_values[i][j] = rz;
          bedlevel.refresh_bed_level();
          TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(i, j, rz));
          if (abl.reenable) {
            set_bed_leveling_enabled(true);
            report_current_position();
          }
        }
        G29_RETURN(false, false);
      } // parser.seen_test('W')

    #else

      constexpr bool seen_w = false;

    #endif

    // Jettison bed leveling data
    if (!seen_w && parser.seen_test('J')) {
      reset_bed_level();
      G29_RETURN(false, false);
    }

    abl.verbose_level = parser.intval('V');
    if (!WITHIN(abl.verbose_level, 0, 4)) {
      SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(V)erbose level implausible (0-4)."));
      G29_RETURN(false, false);
    }

    abl.dryrun = parser.boolval('D') || TERN0(PROBE_MANUALLY, no_action);

    #if ENABLED(AUTO_BED_LEVELING_LINEAR)

      incremental_LSF_reset(&lsf_results);

      abl.topography_map = abl.verbose_level > 2 || parser.boolval('T');

      // X and Y specify points in each direction, overriding the default
      // These values may be saved with the completed mesh
      abl.grid_points.set(
        parser.byteval('X', GRID_MAX_POINTS_X),
        parser.byteval('Y', GRID_MAX_POINTS_Y)
      );
      if (parser.seenval('P')) abl.grid_points.x = abl.grid_points.y = parser.value_int();

      if (!WITHIN(abl.grid_points.x, 2, GRID_MAX_POINTS_X)) {
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Probe points (X) implausible (2-" STRINGIFY(GRID_MAX_POINTS_X) ")."));
        G29_RETURN(false, false);
      }
      if (!WITHIN(abl.grid_points.y, 2, GRID_MAX_POINTS_Y)) {
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Probe points (Y) implausible (2-" STRINGIFY(GRID_MAX_POINTS_Y) ")."));
        G29_RETURN(false, false);
      }

      abl.abl_points = abl.grid_points.x * abl.grid_points.y;
      abl.mean = 0;

    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

      abl.Z_offset = parser.linearval('Z');

    #endif

    #if ABL_USES_GRID

      constexpr feedRate_t min_probe_feedrate_mm_s = XY_PROBE_FEEDRATE_MIN;
      xy_probe_feedrate_mm_s = MMM_TO_MMS(parser.linearval('S', XY_PROBE_FEEDRATE));
      if (xy_probe_feedrate_mm_s < min_probe_feedrate_mm_s) {
        xy_probe_feedrate_mm_s = min_probe_feedrate_mm_s;
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Feedrate (S) too low. (Using ", min_probe_feedrate_mm_s, ")"));
      }

      const float x_min = probe.min_x(), x_max = probe.max_x(),
                  y_min = probe.min_y(), y_max = probe.max_y();

      if (parser.seen('H')) {
        const int16_t size = (int16_t)parser.value_linear_units();
        abl.probe_position_lf.set(_MAX((X_CENTER) - size / 2, x_min), _MAX((Y_CENTER) - size / 2, y_min));
        abl.probe_position_rb.set(_MIN(abl.probe_position_lf.x + size, x_max), _MIN(abl.probe_position_lf.y + size, y_max));
      }
      else {
        abl.probe_position_lf.set(parser.linearval('L', x_min), parser.linearval('F', y_min));
        abl.probe_position_rb.set(parser.linearval('R', x_max), parser.linearval('B', y_max));
      }

      if (!probe.good_bounds(abl.probe_position_lf, abl.probe_position_rb)) {
        if (DEBUGGING(LEVELING)) {
          DEBUG_ECHOLNPGM("G29 L", abl.probe_position_lf.x, " R", abl.probe_position_rb.x,
                             " F", abl.probe_position_lf.y, " B", abl.probe_position_rb.y);
        }
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG(" (L,R,F,B) out of bounds."));
        G29_RETURN(false, false);
      }

      // Probe at the points of a lattice grid
      abl.gridSpacing.set((abl.probe_position_rb.x - abl.probe_position_lf.x) / (abl.grid_points.x - 1),
                          (abl.probe_position_rb.y - abl.probe_position_lf.y) / (abl.grid_points.y - 1));

    #endif // ABL_USES_GRID

    if (abl.verbose_level > 0) {
      SERIAL_ECHOPGM("G29 Auto Bed Leveling");
      if (abl.dryrun) SERIAL_ECHOPGM(" (DRYRUN)");
      SERIAL_EOL();
    }

    planner.synchronize();

    #if ENABLED(AUTO_BED_LEVELING_3POINT)
      if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> 3-point Leveling");
      points[0].z = points[1].z = points[2].z = 0;  // Probe at 3 arbitrary points
    #endif

    TERN_(EXTENSIBLE_UI, ExtUI::onLevelingStart());

    if (!faux) {
      remember_feedrate_scaling_off();

      #if ENABLED(PREHEAT_BEFORE_LEVELING)
        #if ENABLED(SOVOL_SV06_RTS)
          rts.updateTempE0();
          rts.updateTempBed();
          rts.sendData(1, Wait_VP);
          rts.gotoPage(ID_ABL_HeatWait_L, ID_ABL_HeatWait_D);
        #endif
        if (!abl.dryrun) probe.preheat_for_probing(LEVELING_NOZZLE_TEMP,
          TERN(EXTENSIBLE_UI, ExtUI::getLevelingBedTemp(), LEVELING_BED_TEMP)
        );
      #endif
    }

    // Position bed horizontally and Z probe vertically.
    #if HAS_SAFE_BED_LEVELING
      xyze_pos_t safe_position = current_position;
      #ifdef SAFE_BED_LEVELING_START_X
        safe_position.x = SAFE_BED_LEVELING_START_X;
      #endif
      #ifdef SAFE_BED_LEVELING_START_Y
        safe_position.y = SAFE_BED_LEVELING_START_Y;
      #endif
      #ifdef SAFE_BED_LEVELING_START_Z
        safe_position.z = SAFE_BED_LEVELING_START_Z;
      #endif
      #ifdef SAFE_BED_LEVELING_START_I
        safe_position.i = SAFE_BED_LEVELING_START_I;
      #endif
      #ifdef SAFE_BED_LEVELING_START_J
        safe_position.j = SAFE_BED_LEVELING_START_J;
      #endif
      #ifdef SAFE_BED_LEVELING_START_K
        safe_position.k = SAFE_BED_LEVELING_START_K;
      #endif
      #ifdef SAFE_BED_LEVELING_START_U
        safe_position.u = SAFE_BED_LEVELING_START_U;
      #endif
      #ifdef SAFE_BED_LEVELING_START_V
        safe_position.v = SAFE_BED_LEVELING_START_V;
      #endif
      #ifdef SAFE_BED_LEVELING_START_W
        safe_position.w = SAFE_BED_LEVELING_START_W;
      #endif

      do_blocking_move_to(safe_position);
    #endif // HAS_SAFE_BED_LEVELING

    // Disable auto bed leveling during G29.
    // Be formal so G29 can be done successively without G28.
    if (!no_action) set_bed_leveling_enabled(false);

    // Deploy certain probes before starting probing
    #if ENABLED(BLTOUCH) || ALL(HAS_Z_SERVO_PROBE, Z_SERVO_INTERMEDIATE_STOW)
      do_z_clearance(Z_CLEARANCE_DEPLOY_PROBE);
    #elif HAS_BED_PROBE
      if (probe.deploy()) { // (returns true on deploy failure)
        set_bed_leveling_enabled(abl.reenable);
        G29_RETURN(false, true);
      }
    #endif

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      if (!abl.dryrun && (abl.gridSpacing != bedlevel.grid_spacing || abl.probe_position_lf != bedlevel.grid_start)) {
        reset_bed_level();      // Reset grid to 0.0 or "not probed". (Also disables ABL)
        abl.reenable = false;   // Can't re-enable (on error) until the new grid is written
      }
      // Pre-populate local Z values from the stored mesh
      TERN_(IS_KINEMATIC, COPY(abl.z_values, bedlevel.z_values));
    #endif

  } // !g29_in_progress

  #if ENABLED(PROBE_MANUALLY)

    // For manual probing, get the next index to probe now.
    // On the first probe this will be incremented to 0.
    if (!no_action) {
      ++abl.abl_probe_index;
      g29_in_progress = true;
    }

    // Abort current G29 procedure, go back to idle state
    if (seenA && g29_in_progress) {
      SERIAL_ECHOLNPGM("Manual G29 aborted");
      SET_SOFT_ENDSTOP_LOOSE(false);
      set_bed_leveling_enabled(abl.reenable);
      g29_in_progress = false;
      TERN_(LCD_BED_LEVELING, ui.wait_for_move = false);
    }

    // Query G29 status
    if (abl.verbose_level || seenQ) {
      SERIAL_ECHOPGM("Manual G29 ");
      if (g29_in_progress)
        SERIAL_ECHOLNPGM("point ", _MIN(abl.abl_probe_index + 1, abl.abl_points), " of ", abl.abl_points);
      else
        SERIAL_ECHOLNPGM("idle");
    }

    // For 'A' or 'Q' exit with success state
    if (no_action) G29_RETURN(false, true);

    if (abl.abl_probe_index == 0) {
      // For the initial G29 S2 save software endstop state
      SET_SOFT_ENDSTOP_LOOSE(true);
      // Move close to the bed before the first point
      do_blocking_move_to_z(0);
    }
    else {

      #if ANY(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_3POINT)
        const uint16_t index = abl.abl_probe_index - 1;
      #endif

      // For G29 after adjusting Z.
      // Save the previous Z before going to the next point
      abl.measured_z = current_position.z;

      #if ENABLED(AUTO_BED_LEVELING_LINEAR)

        abl.mean += abl.measured_z;
        abl.eqnBVector[index] = abl.measured_z;
        abl.eqnAMatrix[index + 0 * abl.abl_points] = abl.probePos.x;
        abl.eqnAMatrix[index + 1 * abl.abl_points] = abl.probePos.y;
        abl.eqnAMatrix[index + 2 * abl.abl_points] = 1;

        incremental_LSF(&lsf_results, abl.probePos, abl.measured_z);

      #elif ENABLED(AUTO_BED_LEVELING_3POINT)

        points[index].z = abl.measured_z;

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        const float newz = abl.measured_z + abl.Z_offset;
        abl.z_values[abl.meshCount.x][abl.meshCount.y] = newz;
        TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(abl.meshCount, newz));

        if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM_P(PSTR("Save X"), abl.meshCount.x, SP_Y_STR, abl.meshCount.y, SP_Z_STR, abl.measured_z + abl.Z_offset);

      #endif
    }

    //
    // If there's another point to sample, move there with optional lift.
    //

    #if ABL_USES_GRID

      // Skip any unreachable points
      while (abl.abl_probe_index < abl.abl_points) {

        // Set abl.meshCount.x, abl.meshCount.y based on abl.abl_probe_index, with zig-zag
        PR_OUTER_VAR = abl.abl_probe_index / PR_INNER_SIZE;
        PR_INNER_VAR = abl.abl_probe_index - (PR_OUTER_VAR * PR_INNER_SIZE);

        // Probe in reverse order for every other row/column
        const bool zig = (PR_OUTER_VAR & 1); // != ((PR_OUTER_SIZE) & 1);
        if (zig) PR_INNER_VAR = (PR_INNER_SIZE - 1) - PR_INNER_VAR;

        abl.probePos = abl.probe_position_lf + abl.gridSpacing * abl.meshCount.asFloat();

        TERN_(AUTO_BED_LEVELING_LINEAR, abl.indexIntoAB[abl.meshCount.x][abl.meshCount.y] = abl.abl_probe_index);

        // Keep looping till a reachable point is found
        if (position_is_reachable(abl.probePos)) break;
        ++abl.abl_probe_index;
      }

      // Is there a next point to move to?
      if (abl.abl_probe_index < abl.abl_points) {
        _manual_goto_xy(abl.probePos); // Can be used here too!
        // Disable software endstops to allow manual adjustment
        // If G29 is not completed, they will not be re-enabled
        SET_SOFT_ENDSTOP_LOOSE(true);
        G29_RETURN(false, true);
      }
      else {
        // Leveling done! Fall through to G29 finishing code below
        SERIAL_ECHOLNPGM("Grid probing done.");
        // Re-enable software endstops, if needed
        SET_SOFT_ENDSTOP_LOOSE(false);
      }

    #elif ENABLED(AUTO_BED_LEVELING_3POINT)

      // Probe at 3 arbitrary points
      if (abl.abl_probe_index < abl.abl_points) {
        abl.probePos = xy_pos_t(points[abl.abl_probe_index]);
        _manual_goto_xy(abl.probePos);
        // Disable software endstops to allow manual adjustment
        // If G29 is not completed, they will not be re-enabled
        SET_SOFT_ENDSTOP_LOOSE(true);
        G29_RETURN(false, true);
      }
      else {

        SERIAL_ECHOLNPGM("3-point probing done.");

        // Re-enable software endstops, if needed
        SET_SOFT_ENDSTOP_LOOSE(false);

        if (!abl.dryrun) {
          vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
          if (planeNormal.z < 0) planeNormal *= -1;
          planner.bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

          // Can't re-enable (on error) until the new grid is written
          abl.reenable = false;
        }

      }

    #endif // AUTO_BED_LEVELING_3POINT

  #else // !PROBE_MANUALLY
  {
    const ProbePtRaise raise_after = parser.boolval('E') ? PROBE_PT_STOW : PROBE_PT_RAISE;

    abl.measured_z = 0;

    #if ABL_USES_GRID

      bool zig = PR_OUTER_SIZE & 1;  // Always end at RIGHT and BACK_PROBE_BED_POSITION

      // Outer loop is X with PROBE_Y_FIRST enabled
      // Outer loop is Y with PROBE_Y_FIRST disabled
      for (PR_OUTER_VAR = 0; PR_OUTER_VAR < PR_OUTER_SIZE && !isnan(abl.measured_z); PR_OUTER_VAR++) {

        int8_t inStart, inStop, inInc;

        if (zig) {                      // Zig away from origin
          inStart = 0;                  // Left or front
          inStop = PR_INNER_SIZE;       // Right or back
          inInc = 1;                    // Zig right
        }
        else {                          // Zag towards origin
          inStart = PR_INNER_SIZE - 1;  // Right or back
          inStop = -1;                  // Left or front
          inInc = -1;                   // Zag left
        }

        FLIP(zig); // zag

        // An index to print current state
        grid_count_t pt_index = (PR_OUTER_VAR) * (PR_INNER_SIZE) + 1;

        // Inner loop is Y with PROBE_Y_FIRST enabled
        // Inner loop is X with PROBE_Y_FIRST disabled
        for (PR_INNER_VAR = inStart; PR_INNER_VAR != inStop; pt_index++, PR_INNER_VAR += inInc) {

          abl.probePos = abl.probe_position_lf + abl.gridSpacing * abl.meshCount.asFloat();

          TERN_(AUTO_BED_LEVELING_LINEAR, abl.indexIntoAB[abl.meshCount.x][abl.meshCount.y] = ++abl.abl_probe_index); // 0...

          // Avoid probing outside the round or hexagonal area
          if (TERN0(IS_KINEMATIC, !probe.can_reach(abl.probePos))) continue;

          if (abl.verbose_level) SERIAL_ECHOLNPGM("Probing mesh point ", pt_index, "/", abl.abl_points, ".");
          TERN_(HAS_STATUS_MESSAGE, ui.status_printf(0, F(S_FMT " %i/%i"), GET_TEXT_F(MSG_PROBING_POINT), int(pt_index), int(abl.abl_points)));

          #if ENABLED(BD_SENSOR_PROBE_NO_STOP)
            if (PR_INNER_VAR == inStart) {
              char tmp_1[32];

              // move to the start point of new line
              abl.measured_z = faux ? 0.001f * random(-100, 101) : probe.probe_at_point(abl.probePos, raise_after, abl.verbose_level);
              // Go to the end of the row/column ... and back up by one
              // TODO: Why not just use... PR_INNER_VAR = inStop - inInc
              for (PR_INNER_VAR = inStart; PR_INNER_VAR != inStop; PR_INNER_VAR += inInc);
              PR_INNER_VAR -= inInc;

              // Get the coordinate of the resulting grid point
              abl.probePos = abl.probe_position_lf + abl.gridSpacing * abl.meshCount.asFloat();

              // Coordinate that puts the probe at the grid point
              abl.probePos -= probe.offset_xy;

              // Put a G1 move into the buffer
              // TODO: Instead of G1, we can just add the move directly to the planner...
              //  {
              //  destination = current_position; destination = abl.probePos;
              //  REMEMBER(fr, feedrate_mm_s, XY_PROBE_FEEDRATE_MM_S);
              //  prepare_line_to_destination();
              //  }
              sprintf_P(tmp_1, PSTR("G1X%d.%d Y%d.%d F%d"),
                int(abl.probePos.x), int(abl.probePos.x * 10) % 10,
                int(abl.probePos.y), int(abl.probePos.y * 10) % 10,
                XY_PROBE_FEEDRATE
              );
              gcode.process_subcommands_now(tmp_1);

              if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("destX: ", abl.probePos.x, " Y:", abl.probePos.y);

              // Reset the inner counter back to the start
              PR_INNER_VAR = inStart;

              // Get the coordinate of the start of the row/column
              abl.probePos = abl.probe_position_lf + abl.gridSpacing * abl.meshCount.asFloat();
            }

            // Wait around until the real axis position reaches the comparison point
            // TODO: Use NEAR() because float is imprecise
            constexpr AxisEnum axis = TERN(PROBE_Y_FIRST, Y_AXIS, X_AXIS);
            const float cmp = abl.probePos[axis] - probe.offset_xy[axis];
            float pos;
            for (;;) {
              pos = planner.get_axis_position_mm(axis);
              if (inInc > 0 ? (pos >= cmp) : (pos <= cmp)) break;
              idle_no_sleep();
            }
            //if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM_P(axis == Y_AXIS ? PSTR("Y=") : PSTR("X=", pos);

            safe_delay(4);
            abl.measured_z = current_position.z - bdl.read();
            if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("x_cur ", planner.get_axis_position_mm(X_AXIS), " z ", abl.measured_z);

          #else // !BD_SENSOR_PROBE_NO_STOP

            abl.measured_z = faux ? 0.001f * random(-100, 101) : probe.probe_at_point(abl.probePos, raise_after, abl.verbose_level);

          #endif

          if (isnan(abl.measured_z)) {
            set_bed_leveling_enabled(abl.reenable);
            break; // Breaks out of both loops
          }

          #if ENABLED(AUTO_BED_LEVELING_LINEAR)

            abl.mean += abl.measured_z;
            abl.eqnBVector[abl.abl_probe_index] = abl.measured_z;
            abl.eqnAMatrix[abl.abl_probe_index + 0 * abl.abl_points] = abl.probePos.x;
            abl.eqnAMatrix[abl.abl_probe_index + 1 * abl.abl_points] = abl.probePos.y;
            abl.eqnAMatrix[abl.abl_probe_index + 2 * abl.abl_points] = 1;

            incremental_LSF(&lsf_results, abl.probePos, abl.measured_z);

          #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

            const float z = abl.measured_z + abl.Z_offset;
            abl.z_values[abl.meshCount.x][abl.meshCount.y] = z;
            TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(abl.meshCount, z));

            #if ENABLED(SOVOL_SV06_RTS)
              if (pt_index <= GRID_MAX_POINTS) rts.sendData(pt_index, AUTO_BED_LEVEL_ICON_VP);
              rts.sendData(z * 100.0f, AUTO_BED_LEVEL_1POINT_VP + (pt_index - 1) * 2);
              rts.gotoPage(ID_ABL_Wait_L, ID_ABL_Wait_D);
            #endif

          #endif

          abl.reenable = false; // Don't re-enable after modifying the mesh
          idle_no_sleep();

        } // inner
      } // outer

    #elif ENABLED(AUTO_BED_LEVELING_3POINT)

      // Probe at 3 arbitrary points

      for (uint8_t i = 0; i < 3; ++i) {
        if (abl.verbose_level) SERIAL_ECHOLNPGM("Probing point ", i + 1, "/3.");
        TERN_(HAS_STATUS_MESSAGE, ui.status_printf(0, F(S_FMT " %i/3"), GET_TEXT_F(MSG_PROBING_POINT), int(i + 1)));

        // Retain the last probe position
        abl.probePos = xy_pos_t(points[i]);
        abl.measured_z = faux ? 0.001 * random(-100, 101) : probe.probe_at_point(abl.probePos, raise_after, abl.verbose_level);
        if (isnan(abl.measured_z)) {
          set_bed_leveling_enabled(abl.reenable);
          break;
        }
        points[i].z = abl.measured_z;
      }

      if (!abl.dryrun && !isnan(abl.measured_z)) {
        vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
        if (planeNormal.z < 0) planeNormal *= -1;
        planner.bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

        // Can't re-enable (on error) until the new grid is written
        abl.reenable = false;
      }

    #endif // AUTO_BED_LEVELING_3POINT

    ui.reset_status();

    // Stow the probe. No raise for FIX_MOUNTED_PROBE.
    if (probe.stow()) {
      set_bed_leveling_enabled(abl.reenable);
      abl.measured_z = NAN;
    }
  }
  #endif // !PROBE_MANUALLY

  //
  // G29 Finishing Code
  //
  // Unless this is a dry run, auto bed leveling will
  // definitely be enabled after this point.
  //
  // If code above wants to continue leveling, it should
  // return or loop before this point.
  //

  if (DEBUGGING(LEVELING)) DEBUG_POS("> probing complete", current_position);

  #if ENABLED(PROBE_MANUALLY)
    g29_in_progress = false;
    TERN_(LCD_BED_LEVELING, ui.wait_for_move = false);
  #endif

  // Calculate leveling, print reports, correct the position
  if (!isnan(abl.measured_z)) {
    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

      if (abl.dryrun)
        bedlevel.print_leveling_grid(&abl.z_values);
      else {
        bedlevel.set_grid(abl.gridSpacing, abl.probe_position_lf);
        COPY(bedlevel.z_values, abl.z_values);
        TERN_(IS_KINEMATIC, bedlevel.extrapolate_unprobed_bed_level());
        bedlevel.refresh_bed_level();

        bedlevel.print_leveling_grid();
      }

    #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

      // For LINEAR leveling calculate matrix, print reports, correct the position

      /**
       * solve the plane equation ax + by + d = z
       * A is the matrix with rows [x y 1] for all the probed points
       * B is the vector of the Z positions
       * the normal vector to the plane is formed by the coefficients of the
       * plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
       * so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z
       */
      struct { float a, b, d; } plane_equation_coefficients;

      finish_incremental_LSF(&lsf_results);
      plane_equation_coefficients.a = -lsf_results.A;  // We should be able to eliminate the '-' on these three lines and down below
      plane_equation_coefficients.b = -lsf_results.B;  // but that is not yet tested.
      plane_equation_coefficients.d = -lsf_results.D;

      abl.mean /= abl.abl_points;

      if (abl.verbose_level) {
        SERIAL_ECHOPGM("Eqn coefficients: a: ", p_float_t(plane_equation_coefficients.a, 8),
                                        " b: ", p_float_t(plane_equation_coefficients.b, 8),
                                        " d: ", p_float_t(plane_equation_coefficients.d, 8));
        if (abl.verbose_level > 2)
          SERIAL_ECHOPGM("\nMean of sampled points: ", p_float_t(abl.mean, 8));
        SERIAL_EOL();
      }

      // Create the matrix but don't correct the position yet
      if (!abl.dryrun)
        planner.bed_level_matrix = matrix_3x3::create_look_at(
          vector_3(-plane_equation_coefficients.a, -plane_equation_coefficients.b, 1)    // We can eliminate the '-' here and up above
        );

      // Show the Topography map if enabled
      if (abl.topography_map) {

        float min_diff = 999;

        auto print_topo_map = [&](FSTR_P const title, const bool get_min) {
          SERIAL_ECHO(title);
          for (int8_t yy = abl.grid_points.y - 1; yy >= 0; yy--) {
            for (uint8_t xx = 0; xx < abl.grid_points.x; ++xx) {
              const int ind = abl.indexIntoAB[xx][yy];
              xyz_float_t tmp = { abl.eqnAMatrix[ind + 0 * abl.abl_points],
                                  abl.eqnAMatrix[ind + 1 * abl.abl_points], 0 };
              planner.bed_level_matrix.apply_rotation_xyz(tmp.x, tmp.y, tmp.z);
              if (get_min) NOMORE(min_diff, abl.eqnBVector[ind] - tmp.z);
              const float subval = get_min ? abl.mean : tmp.z + min_diff,
                            diff = abl.eqnBVector[ind] - subval;
              SERIAL_CHAR(' '); if (diff >= 0.0) SERIAL_CHAR('+');   // Include + for column alignment
              SERIAL_ECHO(p_float_t(diff, 5));
            } // xx
            SERIAL_EOL();
          } // yy
          SERIAL_EOL();
        };

        print_topo_map(F("\nBed Height Topography:\n"
                           "   +--- BACK --+\n"
                           "   |           |\n"
                           " L |    (+)    | R\n"
                           " E |           | I\n"
                           " F | (-) N (+) | G\n"
                           " T |           | H\n"
                           "   |    (-)    | T\n"
                           "   |           |\n"
                           "   O-- FRONT --+\n"
                           " (0,0)\n"), true);
        if (abl.verbose_level > 3)
          print_topo_map(F("\nCorrected Bed Height vs. Bed Topology:\n"), false);

      } // abl.topography_map

    #endif // AUTO_BED_LEVELING_LINEAR

    #if ABL_PLANAR

      // For LINEAR and 3POINT leveling correct the current position

      if (abl.verbose_level > 0)
        planner.bed_level_matrix.debug(F("\n\nBed Level Correction Matrix:"));

      if (!abl.dryrun) {
        //
        // Correct the current XYZ position based on the tilted plane.
        //

        if (DEBUGGING(LEVELING)) DEBUG_POS("G29 uncorrected XYZ", current_position);

        xyze_pos_t converted = current_position;
        planner.force_unapply_leveling(converted); // use conversion machinery

        // Use the last measured distance to the bed, if possible
        if ( NEAR(current_position.x, abl.probePos.x - probe.offset_xy.x)
          && NEAR(current_position.y, abl.probePos.y - probe.offset_xy.y)
        ) {
          const float simple_z = current_position.z - abl.measured_z;
          if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Probed Z", simple_z, "  Matrix Z", converted.z, "  Discrepancy ", simple_z - converted.z);
          converted.z = simple_z;
        }

        // The rotated XY and corrected Z are now current_position
        current_position = converted;

        if (DEBUGGING(LEVELING)) DEBUG_POS("G29 corrected XYZ", current_position);

        abl.reenable = true;
      }

      // Auto Bed Leveling is complete! Enable if possible.
      if (abl.reenable) {
        planner.leveling_active = true;
        sync_plan_position();
      }

    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

      // Auto Bed Leveling is complete! Enable if possible.
      if (!abl.dryrun || abl.reenable) set_bed_leveling_enabled(true);

    #endif

  } // !isnan(abl.measured_z)

  // Restore state after probing
  if (!faux) restore_feedrate_and_scaling();

  TERN_(HAS_BED_PROBE, probe.move_z_after_probing());

  #ifdef EVENT_GCODE_AFTER_G29
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("After G29 G-code: ", EVENT_GCODE_AFTER_G29);
    planner.synchronize();
    process_subcommands_now(F(EVENT_GCODE_AFTER_G29));
  #endif

  TERN_(SOVOL_SV06_RTS, RTS_AutoBedLevelPage());

  probe.use_probing_tool(false);

  report_current_position();

  G29_RETURN(isnan(abl.measured_z), true);
}

#endif // HAS_ABL_NOT_UBL
