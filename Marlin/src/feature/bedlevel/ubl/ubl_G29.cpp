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

#if ENABLED(AUTO_BED_LEVELING_UBL)

#include "../bedlevel.h"

#include "../../../MarlinCore.h"
#include "../../../HAL/shared/eeprom_api.h"
#include "../../../libs/hex_print.h"
#include "../../../module/settings.h"
#include "../../../lcd/marlinui.h"
#include "../../../module/planner.h"
#include "../../../module/motion.h"
#include "../../../module/probe.h"
#include "../../../gcode/gcode.h"
#include "../../../libs/least_squares_fit.h"

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../../core/debug_out.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../../../lcd/extui/ui_api.h"
#endif

#if ENABLED(UBL_HILBERT_CURVE)
  #include "../hilbert_curve.h"
#endif

#if FT_MOTION_DISABLE_FOR_PROBING
  #include "../../../module/ft_motion.h"
#endif

#include <math.h>

#define UBL_G29_P31

#if HAS_MARLINUI_MENU

  bool unified_bed_leveling::lcd_map_control = false;

  void unified_bed_leveling::steppers_were_disabled() {
    if (lcd_map_control) {
      lcd_map_control = false;
      ui.defer_status_screen(false);
    }
  }

  void ubl_map_screen();

#endif

#define SIZE_OF_LITTLE_RAISE 1
#define BIG_RAISE_NOT_NEEDED 0

/**
 *   G29: Unified Bed Leveling by Roxy
 *
 *   Parameters understood by this leveling system:
 *
 *   A     Activate   Activate the Unified Bed Leveling system.
 *
 *   B #   Business   Use the 'Business Card' mode of the Manual Probe subsystem with P2.
 *                    Note: A non-compressible Spark Gap feeler gauge is recommended over a business card.
 *                    In this mode of G29 P2, a business or index card is used as a shim that the nozzle can
 *                    grab onto as it is lowered. In principle, the nozzle-bed distance is the same when the
 *                    same resistance is felt in the shim. You can omit the numerical value on first invocation
 *                    of G29 P2 B to measure shim thickness. Subsequent use of 'B' will apply the previously-
 *                    measured thickness by default.
 *
 *   C     Continue   G29 P1 C continues the generation of a partially-constructed Mesh without invalidating
 *                    previous measurements.
 *
 *   C                G29 P2 C tells the Manual Probe subsystem to not use the current nozzle
 *                    location in its search for the closest unmeasured Mesh Point. Instead, attempt to
 *                    start at one end of the uprobed points and Continue sequentially.
 *
 *                    G29 P3 C specifies the Constant for the fill. Otherwise, uses a "reasonable" value.
 *
 *   C     Current    G29 Z C uses the Current location (instead of bed center or nearest edge).
 *
 *   D     Disable    Disable the Unified Bed Leveling system.
 *
 *   E     Stow_probe Stow the probe after each sampled point.
 *
 *   F #   Fade       Fade the amount of Mesh Based Compensation over a specified height. At the
 *                    specified height, no correction is applied and natural printer kenimatics take over. If no
 *                    number is specified for the command, 10mm is assumed to be reasonable.
 *
 *   H #   Height     With P2, 'H' specifies the Height to raise the nozzle after each manual probe of the bed.
 *                    If omitted, the nozzle will raise by Z_CLEARANCE_BETWEEN_PROBES.
 *
 *   H #   Offset     With P4, 'H' specifies the Offset above the mesh height to place the nozzle.
 *                    If omitted, Z_TWEEN_SAFE_CLEARANCE will be used.
 *
 *   I #   Invalidate Invalidate the specified number of Mesh Points near the given 'X' 'Y'. If X or Y are omitted,
 *                    the nozzle location is used. If no 'I' value is given, only the point nearest to the location
 *                    is invalidated. Use 'T' to produce a map afterward. This command is useful to invalidate a
 *                    portion of the Mesh so it can be adjusted using other UBL tools. When attempting to invalidate
 *                    an isolated bad mesh point, the 'T' option shows the nozzle position in the Mesh with (#). You
 *                    can move the nozzle around and use this feature to select the center of the area (or cell) to
 *                    invalidate.
 *
 *   J #   Grid       Perform a Grid Based Leveling of the current Mesh using a grid with n points on a side.
 *                    Not specifying a grid size will invoke the 3-Point leveling function.
 *
 *   L     Load       Load Mesh from the previously activated location in the EEPROM.
 *
 *   L #   Load       Load Mesh from the specified location in the EEPROM. Set this location as activated
 *                    for subsequent Load and Store operations.
 *
 *   The P or Phase commands are used for the bulk of the work to setup a Mesh. In general, your Mesh will
 *   start off being initialized with a G29 P0 or a G29 P1. Further refinement of the Mesh happens with
 *   each additional Phase that processes it.
 *
 *   P0    Phase 0    Zero Mesh Data and turn off the Mesh Compensation System. This reverts the
 *                    3D Printer to the same state it was in before the Unified Bed Leveling Compensation
 *                    was turned on. Setting the entire Mesh to Zero is a special case that allows
 *                    a subsequent G or T leveling operation for backward compatibility.
 *
 *   P1    Phase 1    Invalidate entire Mesh and continue with automatic generation of the Mesh data using
 *                    the Z-Probe. Usually the probe can't reach all areas that the nozzle can reach. For delta
 *                    printers only the areas where the probe and nozzle can both reach will be automatically probed.
 *
 *                    Unreachable points will be handled in Phase 2 and Phase 3.
 *
 *                    Use 'C' to leave the previous mesh intact and automatically probe needed points. This allows you
 *                    to invalidate parts of the Mesh but still use Automatic Probing.
 *
 *                    The 'X' and 'Y' parameters prioritize where to try and measure points. If omitted, the current
 *                    probe position is used.
 *
 *                    Use 'T' (Topology) to generate a report of mesh generation.
 *
 *                    P1 will suspend Mesh generation if the controller button is held down. Note that you may need
 *                    to press and hold the switch for several seconds if moves are underway.
 *
 *   P2    Phase 2    Probe unreachable points.
 *
 *                    Use 'H' to set the height between Mesh points. If omitted, Z_CLEARANCE_BETWEEN_PROBES is used.
 *                    Smaller values will be quicker. Move the nozzle down till it barely touches the bed. Make sure the
 *                    nozzle is clean and unobstructed. Use caution and move slowly. This can damage your printer!
 *                    (Uses SIZE_OF_LITTLE_RAISE mm if the nozzle is moving less than BIG_RAISE_NOT_NEEDED mm.)
 *
 *                    The 'H' value can be negative if the Mesh dips in a large area. Press and hold the
 *                    controller button to terminate the current Phase 2 command. You can then re-issue "G29 P 2"
 *                    with an 'H' parameter more suitable for the area you're manually probing. Note that the command
 *                    tries to start in a corner of the bed where movement will be predictable. Override the distance
 *                    calculation location with the X and Y parameters. You can print a Mesh Map (G29 T) to see where
 *                    the mesh is invalidated and where the nozzle needs to move to complete the command. Use 'C' to
 *                    indicate that the search should be based on the current position.
 *
 *                    The 'B' parameter for this command is described above. It places the manual probe subsystem into
 *                    Business Card mode where the thickness of a business card is measured and then used to accurately
 *                    set the nozzle height in all manual probing for the duration of the command. A Business card can
 *                    be used, but you'll get better results with a flexible Shim that doesn't compress. This makes it
 *                    easier to produce similar amounts of force and get more accurate measurements. Google if you're
 *                    not sure how to use a shim.
 *
 *                    The 'T' (Map) parameter helps track Mesh building progress.
 *
 *                    NOTE: P2 requires an LCD controller!
 *
 *   P3    Phase 3    Fill the unpopulated regions of the Mesh with a fixed value. There are two different paths to
 *                    go down:
 *
 *                    - If a 'C' constant is specified, the closest invalid mesh points to the nozzle will be filled,
 *                      and a repeat count can then also be specified with 'R'.
 *
 *                    - Leaving out 'C' invokes Smart Fill, which scans the mesh from the edges inward looking for
 *                      invalid mesh points. Adjacent points are used to determine the bed slope. If the bed is sloped
 *                      upward from the invalid point, it takes the value of the nearest point. If sloped downward, it's
 *                      replaced by a value that puts all three points in a line. This version of G29 P3 is a quick, easy
 *                      and (usually) safe way to populate unprobed mesh regions before continuing to G26 Mesh Validation
 *                      Pattern. Note that this populates the mesh with unverified values. Pay attention and use caution.
 *
 *   P4    Phase 4    Fine tune the Mesh. The Delta Mesh Compensation System assumes the existence of
 *                    an LCD Panel. It is possible to fine tune the mesh without an LCD Panel using
 *                    G42 and M421. See the UBL documentation for further details.
 *
 *                    Phase 4 is meant to be used with G26 Mesh Validation to fine tune the mesh by direct editing
 *                    of Mesh Points. Raise and lower points to fine tune the mesh until it gives consistently reliable
 *                    adhesion.
 *
 *                    P4 moves to the closest Mesh Point (and/or the given X Y), raises the nozzle above the mesh height
 *                    by the given 'H' offset (or default 0), and waits while the controller is used to adjust the nozzle
 *                    height. On click the displayed height is saved in the mesh.
 *
 *                    Start Phase 4 at a specific location with X and Y. Adjust a specific number of Mesh Points with
 *                    the 'R' (Repeat) parameter. (If 'R' is left out, the whole matrix is assumed.) This command can be
 *                    terminated early (e.g., after editing the area of interest) by pressing and holding the encoder button.
 *
 *                    The general form is G29 P4 [R points] [X position] [Y position]
 *
 *                    The H [offset] parameter is useful if a shim is used to fine-tune the mesh. For a 0.4mm shim the
 *                    command would be G29 P4 H0.4. The nozzle is moved to the shim height, you adjust height to the shim,
 *                    and on click the height minus the shim thickness will be saved in the mesh.
 *
 *                    !!Use with caution, as a very poor mesh could cause the nozzle to crash into the bed!!
 *
 *                    NOTE:  P4 is not available unless you have LCD support enabled!
 *
 *   P5    Phase 5    Find Mean Mesh Height and Standard Deviation. Typically, it is easier to use and
 *                    work with the Mesh if it is Mean Adjusted. You can specify a C parameter to
 *                    Correct the Mesh to a 0.00 Mean Height. Adding a C parameter will automatically
 *                    execute a G29 P6 C <mean height>.
 *
 *   P6    Phase 6    Shift Mesh height. The entire Mesh's height is adjusted by the height specified
 *                    with the C parameter. Being able to adjust the height of a Mesh is useful tool. It
 *                    can be used to compensate for poorly calibrated Z-Probes and other errors. Ideally,
 *                    you should have the Mesh adjusted for a Mean Height of 0.00 and the Z-Probe measuring
 *                    0.000 at the Z Home location.
 *
 *   Q     Test       Load specified Test Pattern to assist in checking correct operation of system. This
 *                    command is not anticipated to be of much value to the typical user. It is intended
 *                    for developers to help them verify correct operation of the Unified Bed Leveling System.
 *
 *   R #   Repeat     Repeat this command the specified number of times. If no number is specified the
 *                    command will be repeated GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y times.
 *
 *   S     Store      Store the current Mesh in the Activated area of the EEPROM. It will also store the
 *                    current state of the Unified Bed Leveling system in the EEPROM.
 *
 *   S #   Store      Store the current Mesh at the specified location in EEPROM. Activate this location
 *                    for subsequent Load and Store operations. Valid storage slot numbers begin at 0 and
 *                    extend to a limit related to the available EEPROM storage.
 *
 *   S -1  Store      Print the current Mesh as G-code that can be used to restore the mesh anytime.
 *
 *   T     Topology   Display the Mesh Map Topology.
 *                    'T' can be used alone (e.g., G29 T) or in combination with most of the other commands.
 *                    This option works with all Phase commands (e.g., G29 P4 R 5 T X 50 Y100 C -.1 O)
 *                    This parameter can also specify a Map Type. T0 (the default) is user-readable. T1
 *                    is suitable to paste into a spreadsheet for a 3D graph of the mesh.
 *
 *   U     Unlevel    Perform a probe of the outer perimeter to assist in physically leveling unlevel beds.
 *                    Only used for G29 P1 T U. This speeds up the probing of the edge of the bed. Useful
 *                    when the entire bed doesn't need to be probed because it will be adjusted.
 *
 *   V #   Verbosity  Set the verbosity level (0-4) for extra details. (Default 0)
 *
 *   X #              X Location for this command
 *
 *   Y #              Y Location for this command
 *
 * With UBL_DEVEL_DEBUGGING:
 *
 *   K #  Kompare     Kompare current Mesh with stored Mesh #, replacing current Mesh with the result.
 *                    This command literally performs a diff between two Meshes.
 *
 *   Q-1  Dump EEPROM Dump the UBL contents stored in EEPROM as HEX format. Useful for developers to help
 *                    verify correct operation of the UBL.
 *
 *   W    What?       Display valuable UBL data.
 *
 *
 *   Release Notes:
 *   You MUST do M502, M500 to initialize the storage. Failure to do this will cause all
 *   kinds of problems. Enabling EEPROM Storage is required.
 *
 *   When you do a G28 and G29 P1 to automatically build your first mesh, you are going to notice that
 *   UBL probes points increasingly further from the starting location. (The starting location defaults
 *   to the center of the bed.) In contrast, ABL and MBL follow a zigzag pattern. The spiral pattern is
 *   especially better for Delta printers, since it populates the center of the mesh first, allowing for
 *   a quicker test print to verify settings. You don't need to populate the entire mesh to use it.
 *   After all, you don't want to spend a lot of time generating a mesh only to realize the resolution
 *   or probe offsets are incorrect. Mesh-generation gathers points starting closest to the nozzle unless
 *   an (X,Y) coordinate pair is given.
 *
 *   Unified Bed Leveling uses a lot of EEPROM storage to hold its data, and it takes some effort to get
 *   the mesh just right. To prevent this valuable data from being destroyed as the EEPROM structure
 *   evolves, UBL stores all mesh data at the end of EEPROM.
 *
 *   UBL is founded on Edward Patel's Mesh Bed Leveling code. A big 'Thanks!' to him and the creators of
 *   3-Point and Grid Based leveling. Combining their contributions we now have the functionality and
 *   features of all three systems combined.
 */

G29_parameters_t unified_bed_leveling::param;

void unified_bed_leveling::G29() {

  #ifdef EVENT_GCODE_AFTER_G29
    bool probe_deployed = false;
    #define SET_PROBE_DEPLOYED(N) probe_deployed = N
  #else
    #define SET_PROBE_DEPLOYED(N)
  #endif

  if (G29_parse_parameters()) return; // Abort on parameter error

  const uint8_t p_val = parser.byteval('P');
  const bool may_move = p_val == 1 || p_val == 2 || p_val == 4 || parser.seen_test('J');

  #if FT_MOTION_DISABLE_FOR_PROBING
    FTMotionDisableInScope FT_Disabler; // Disable Fixed-Time Motion for probing
  #endif

  // Check for commands that require the printer to be homed
  if (may_move) {
    planner.synchronize();
    #if ALL(DWIN_LCD_PROUI, ZHOME_BEFORE_LEVELING)
      save_ubl_active_state_and_disable();
      gcode.process_subcommands_now(F("G28Z"));
      restore_ubl_active_state(false); // ...without telling ExtUI "done"
    #else
      // Send 'N' to force homing before G29 (internal only)
      if (axes_should_home() || parser.seen_test('N')) gcode.home_all_axes();
    #endif
    probe.use_probing_tool();

    #ifdef EVENT_GCODE_BEFORE_G29
      if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Before G29 G-code: ", EVENT_GCODE_BEFORE_G29);
      gcode.process_subcommands_now(F(EVENT_GCODE_BEFORE_G29));
    #endif

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
  }

  // Invalidate one or more nearby mesh points, possibly all.
  if (parser.seen('I')) {
    grid_count_t count = parser.has_value() ? parser.value_ushort() : 1;
    bool invalidate_all = count >= GRID_MAX_POINTS;
    if (!invalidate_all) {
      while (count--) {
        if ((count & 0x0F) == 0x0F) idle();
        const mesh_index_pair closest = find_closest_mesh_point_of_type(REAL, param.XY_pos);
        // No more REAL mesh points to invalidate? Assume the user meant
        // to invalidate the ENTIRE mesh, which can't be done with
        // find_closest_mesh_point (which only returns REAL points).
        if (closest.pos.x < 0) { invalidate_all = true; break; }
        z_values[closest.pos.x][closest.pos.y] = NAN;
        TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(closest.pos, 0.0f));
      }
    }
    if (invalidate_all) {
      invalidate();
      SERIAL_ECHOPGM("Entire Mesh");
    }
    else
      SERIAL_ECHOPGM("Locations");
    SERIAL_ECHOLNPGM(" invalidated.\n");
  }

  if (parser.seen('Q')) {
    const int16_t test_pattern = parser.has_value() ? parser.value_int() : -99;
    if (!WITHIN(test_pattern, TERN0(UBL_DEVEL_DEBUGGING, -1), 2)) {
      SERIAL_ECHOLNPGM("?Invalid (Q) test pattern. (" TERN(UBL_DEVEL_DEBUGGING, "-1", "0") " to 2)\n");
      return;
    }
    SERIAL_ECHOLNPGM("Applying test pattern.\n");
    switch (test_pattern) {

      default:
      case -1: TERN_(UBL_DEVEL_DEBUGGING, g29_eeprom_dump()); break;

      case 0:
        GRID_LOOP(x, y) {                                     // Create a bowl shape similar to a poorly-calibrated Delta
          const float p1 = 0.5f * (GRID_MAX_POINTS_X) - x,
                      p2 = 0.5f * (GRID_MAX_POINTS_Y) - y;
          z_values[x][y] += 2.0f * HYPOT(p1, p2);
          TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, z_values[x][y]));
        }
        break;

      case 1:
        for (uint8_t x = 0; x < GRID_MAX_POINTS_X; ++x) {                     // Create a diagonal line several Mesh cells thick that is raised
          const uint8_t x2 = x + (x < (GRID_MAX_POINTS_Y) - 1 ? 1 : -1);
          z_values[x][x] += 9.999f;
          z_values[x][x2] += 9.999f; // We want the altered line several mesh points thick
          #if ENABLED(EXTENSIBLE_UI)
            ExtUI::onMeshUpdate(x, x, z_values[x][x]);
            ExtUI::onMeshUpdate(x, x2, z_values[x][x2]);
          #endif
        }
        break;

      case 2:
        // Allow the user to specify the height because 10mm is a little extreme in some cases.
        for (uint8_t x = (GRID_MAX_POINTS_X) / 3; x < 2 * (GRID_MAX_POINTS_X) / 3; x++)     // Create a rectangular raised area in
          for (uint8_t y = (GRID_MAX_POINTS_Y) / 3; y < 2 * (GRID_MAX_POINTS_Y) / 3; y++) { // the center of the bed
            z_values[x][y] += parser.seen_test('C') ? param.C_constant : 9.99f;
            TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, z_values[x][y]));
          }
        break;
    }
  }

  #if HAS_BED_PROBE

    if (parser.seen_test('J')) {
      save_ubl_active_state_and_disable();
      tilt_mesh_based_on_probed_grid(param.J_grid_size == 0); // Zero size does 3-Point
      restore_ubl_active_state();
      #if ENABLED(UBL_G29_J_RECENTER)
        do_blocking_move_to_xy(0.5f * ((MESH_MIN_X) + (MESH_MAX_X)), 0.5f * ((MESH_MIN_Y) + (MESH_MAX_Y)));
      #endif
      report_current_position();
      SET_PROBE_DEPLOYED(true);
    }

  #endif // HAS_BED_PROBE

  if (parser.seen_test('P')) {
    if (WITHIN(param.P_phase, 0, 1) && storage_slot == -1) {
      storage_slot = 0;
      SERIAL_ECHOLNPGM("Default storage slot 0 selected.");
    }

    switch (param.P_phase) {
      case 0:
        //
        // Zero Mesh Data
        //
        reset();
        SERIAL_ECHOLNPGM("Mesh zeroed.");
        break;

      #if HAS_BED_PROBE

        case 1: {
          //
          // Invalidate Entire Mesh and Automatically Probe Mesh in areas that can be reached by the probe
          //
          if (!parser.seen_test('C')) {
            invalidate();
            SERIAL_ECHOLNPGM("Mesh invalidated. Probing mesh.");
          }
          if (param.V_verbosity > 1)
            SERIAL_ECHOLN(F("Probing around ("), param.XY_pos.x, C(','), param.XY_pos.y, F(").\n"));
          probe_entire_mesh(param.XY_pos, parser.seen_test('T'), parser.seen_test('E'), parser.seen_test('U'));

          report_current_position();
          SET_PROBE_DEPLOYED(true);
        } break;

      #endif // HAS_BED_PROBE

      case 2: {
        #if HAS_MARLINUI_MENU
          //
          // Manually Probe Mesh in areas that can't be reached by the probe
          //
          SERIAL_ECHOLNPGM("Manually probing unreachable points.");
          do_z_clearance(Z_CLEARANCE_BETWEEN_PROBES);

          if (parser.seen_test('C') && !param.XY_seen) {

            /**
             * Use a good default location for the path.
             * The flipped > and < operators in these comparisons is intentional.
             * It should cause the probed points to follow a nice path on Cartesian printers.
             * It may make sense to have Delta printers default to the center of the bed.
             * Until that is decided, this can be forced with the X and Y parameters.
             */
            param.XY_pos.set(
              #if IS_KINEMATIC
                X_HOME_POS, Y_HOME_POS
              #else
                probe.offset_xy.x > 0 ? X_BED_SIZE : 0,
                probe.offset_xy.y < 0 ? Y_BED_SIZE : 0
              #endif
            );
          }

          if (parser.seen('B')) {
            param.B_shim_thickness = parser.has_value() ? parser.value_float() : measure_business_card_thickness();
            if (ABS(param.B_shim_thickness) > 1.5f) {
              SERIAL_ECHOLNPGM("?Error in Business Card measurement.");
              return;
            }
            SET_PROBE_DEPLOYED(true);
          }

          if (!position_is_reachable(param.XY_pos)) {
            SERIAL_ECHOLNPGM("XY outside printable radius.");
            return;
          }

          const float height = parser.floatval('H', Z_CLEARANCE_BETWEEN_PROBES);
          manually_probe_remaining_mesh(param.XY_pos, height, param.B_shim_thickness, parser.seen_test('T'));

          SERIAL_ECHOLNPGM("G29 P2 finished.");

          report_current_position();

        #else

          SERIAL_ECHOLNPGM("?P2 is only available when an LCD is present.");
          return;

        #endif
      } break;

      case 3: {
        /**
         * Populate invalid mesh areas. Proceed with caution.
         * Two choices are available:
         *   - Specify a constant with the 'C' parameter.
         *   - Allow 'G29 P3' to choose a 'reasonable' constant.
         */

        if (param.C_seen) {
          if (param.R_repetition >= GRID_MAX_POINTS) {
            set_all_mesh_points_to_value(param.C_constant);
          }
          else {
            while (param.R_repetition--) {  // this only populates reachable mesh points near
              const mesh_index_pair closest = find_closest_mesh_point_of_type(INVALID, param.XY_pos);
              const xy_int8_t &cpos = closest.pos;
              if (cpos.x < 0) {
                // No more REAL INVALID mesh points to populate, so we ASSUME
                // user meant to populate ALL INVALID mesh points to value
                GRID_LOOP(x, y) if (isnan(z_values[x][y])) z_values[x][y] = param.C_constant;
                break; // No more invalid Mesh Points to populate
              }
              else {
                z_values[cpos.x][cpos.y] = param.C_constant;
                TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(cpos, param.C_constant));
              }
            }
          }
        }
        else {
          const float cvf = parser.value_float();
          switch ((int)TRUNC(cvf * 10.0f) - 30) {   // 3.1 -> 1
            #if ENABLED(UBL_G29_P31)
              case 1: {

                // P3.1  use least squares fit to fill missing mesh values
                // P3.10 zero weighting for distance, all grid points equal, best fit tilted plane
                // P3.11 10X weighting for nearest grid points versus farthest grid points
                // P3.12 100X distance weighting
                // P3.13 1000X distance weighting, approaches simple average of nearest points

                const float weight_power  = (cvf - 3.10f) * 100.0f,  // 3.12345 -> 2.345
                            weight_factor = weight_power ? POW(10.0f, weight_power) : 0;
                smart_fill_wlsf(weight_factor);
              }
              break;
            #endif
            case 0:   // P3 or P3.0
            default:  // and anything P3.x that's not P3.1
              smart_fill_mesh();  // Do a 'Smart' fill using nearby known values
              break;
          }
        }
        break;
      }

      case 4: // Fine Tune (i.e., Edit) the Mesh
        #if HAS_MARLINUI_MENU
          fine_tune_mesh(param.XY_pos, parser.seen_test('T'));
        #else
          SERIAL_ECHOLNPGM("?P4 is only available when an LCD is present.");
          return;
        #endif
        break;

      case 5: adjust_mesh_to_mean(param.C_seen, param.C_constant); break;

      case 6: shift_mesh_height(); break;
    }
  }

  #if ENABLED(UBL_DEVEL_DEBUGGING)

    //
    // Much of the 'What?' command can be eliminated. But until we are fully debugged, it is
    // good to have the extra information. Soon... we prune this to just a few items
    //
    if (parser.seen_test('W')) g29_what_command();

    //
    // When we are fully debugged, this may go away. But there are some valid
    // use cases for the users. So we can wait and see what to do with it.
    //

    if (parser.seen('K')) // Kompare Current Mesh Data to Specified Stored Mesh
      g29_compare_current_mesh_to_stored_mesh();

  #endif // UBL_DEVEL_DEBUGGING

  //
  // Load a Mesh from the EEPROM
  //

  if (parser.seen('L')) {     // Load Current Mesh Data
    param.KLS_storage_slot = parser.has_value() ? (int8_t)parser.value_int() : storage_slot;

    int16_t a = settings.calc_num_meshes();

    if (!a) {
      SERIAL_ECHOLNPGM("?EEPROM storage not available.");
      return;
    }

    if (!WITHIN(param.KLS_storage_slot, 0, a - 1)) {
      SERIAL_ECHOLNPGM("?Invalid storage slot.\n?Use 0 to ", a - 1);
      return;
    }

    settings.load_mesh(param.KLS_storage_slot);
    storage_slot = param.KLS_storage_slot;

    SERIAL_ECHOLNPGM(STR_DONE);
  }

  //
  // Store a Mesh in the EEPROM
  //

  if (parser.seen('S')) {     // Store (or Save) Current Mesh Data
    param.KLS_storage_slot = parser.has_value() ? (int8_t)parser.value_int() : storage_slot;

    if (param.KLS_storage_slot == -1)               // Special case: 'Export' the mesh to the
      return report_current_mesh();                 // host so it can be saved in a file.

    int16_t a = settings.calc_num_meshes();

    if (!a) {
      SERIAL_ECHOLNPGM("?EEPROM storage not available.");
      goto LEAVE;
    }

    if (!WITHIN(param.KLS_storage_slot, 0, a - 1)) {
      SERIAL_ECHOLNPGM("?Invalid storage slot.\n?Use 0 to ", a - 1);
      goto LEAVE;
    }

    settings.store_mesh(param.KLS_storage_slot);
    storage_slot = param.KLS_storage_slot;

    SERIAL_ECHOLNPGM(STR_DONE);
  }

  if (parser.seen_test('T'))
    display_map(param.T_map_type);

  LEAVE:

  #if HAS_MARLINUI_MENU
    ui.reset_alert_level();
    ui.quick_feedback();
    ui.reset_status();
    ui.release();
  #endif

  #ifdef EVENT_GCODE_AFTER_G29
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("After G29 G-code: ", EVENT_GCODE_AFTER_G29);
    if (probe_deployed) {
      planner.synchronize();
      gcode.process_subcommands_now(F(EVENT_GCODE_AFTER_G29));
    }
  #endif

  probe.use_probing_tool(false);
  return;
}

/**
 * M420 C<value>
 * G29 P5 C<value> : Adjust Mesh To Mean (and subtract the given offset).
 *                   Find the mean average and shift the mesh to center on that value.
 */
void unified_bed_leveling::adjust_mesh_to_mean(const bool cflag, const_float_t offset) {
  float sum = 0;
  uint8_t n = 0;
  GRID_LOOP(x, y)
    if (!isnan(z_values[x][y])) {
      sum += z_values[x][y];
      n++;
    }

  const float mean = sum / n;

  //
  // Sum the squares of difference from mean
  //
  float sum_of_diff_squared = 0;
  GRID_LOOP(x, y)
    if (!isnan(z_values[x][y]))
      sum_of_diff_squared += sq(z_values[x][y] - mean);

  SERIAL_ECHOLNPGM("# of samples: ", n);
  SERIAL_ECHOLNPGM("Mean Mesh Height: ", p_float_t(mean, 6));

  const float sigma = SQRT(sum_of_diff_squared / (n + 1));
  SERIAL_ECHOLNPGM("Standard Deviation: ", p_float_t(sigma, 6));

  if (cflag)
    GRID_LOOP(x, y)
      if (!isnan(z_values[x][y])) {
        z_values[x][y] -= mean + offset;
        TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, z_values[x][y]));
      }
}

/**
 * G29 P6 C<offset> : Shift Mesh Height by a uniform constant.
 */
void unified_bed_leveling::shift_mesh_height() {
  GRID_LOOP(x, y)
    if (!isnan(z_values[x][y])) {
      z_values[x][y] += param.C_constant;
      TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, z_values[x][y]));
    }
}

#if HAS_BED_PROBE
  /**
   * G29 P1 T<maptype> V<verbosity> : Probe Entire Mesh
   *   Probe all invalidated locations of the mesh that can be reached by the probe.
   *   This attempts to fill in locations closest to the nozzle's start location first.
   */
  void unified_bed_leveling::probe_entire_mesh(const xy_pos_t &nearby, const bool do_ubl_mesh_map, const bool stow_probe, const bool do_furthest) {
    probe.deploy(); // Deploy before ui.capture() to allow for PAUSE_BEFORE_DEPLOY_STOW

    TERN_(HAS_MARLINUI_MENU, ui.capture());
    TERN_(EXTENSIBLE_UI, ExtUI::onLevelingStart());

    save_ubl_active_state_and_disable();  // No bed level correction so only raw data is obtained
    grid_count_t count = GRID_MAX_POINTS;

    mesh_index_pair best;
    TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(best.pos, ExtUI::G29_START));
    do {
      if (do_ubl_mesh_map) display_map(param.T_map_type);

      const grid_count_t point_num = (GRID_MAX_POINTS - count) + 1;
      SERIAL_ECHOLNPGM("Probing mesh point ", point_num, "/", GRID_MAX_POINTS, ".");
      TERN_(HAS_STATUS_MESSAGE, ui.status_printf(0, F(S_FMT " %i/%i"), GET_TEXT_F(MSG_PROBING_POINT), point_num, int(GRID_MAX_POINTS)));
      TERN_(HAS_BACKLIGHT_TIMEOUT, ui.refresh_backlight_timeout());

      #if HAS_MARLINUI_MENU
        if (ui.button_pressed()) {
          ui.quick_feedback(false); // Preserve button state for click-and-hold
          SERIAL_ECHOLNPGM("\nMesh only partially populated.\n");
          ui.wait_for_release();
          ui.quick_feedback();
          ui.release();
          probe.stow(); // Release UI before stow to allow for PAUSE_BEFORE_DEPLOY_STOW
          return restore_ubl_active_state();
        }
      #endif

      #ifndef HUGE_VALF
        #define HUGE_VALF __FLT_MAX__
      #endif

      best = do_furthest // Points with valid data or HUGE_VALF are skipped
        ? find_furthest_invalid_mesh_point()
        : find_closest_mesh_point_of_type(INVALID, nearby, true);

      if (best.pos.x >= 0) {    // mesh point found and is reachable by probe
        TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(best.pos, ExtUI::G29_POINT_START));
        const float measured_z = probe.probe_at_point(best.meshpos(), stow_probe ? PROBE_PT_STOW : PROBE_PT_RAISE, param.V_verbosity);
        z_values[best.pos.x][best.pos.y] = isnan(measured_z) ? HUGE_VALF : measured_z;  // Mark invalid point already probed with HUGE_VALF to omit it in the next loop
        #if ENABLED(EXTENSIBLE_UI)
          ExtUI::onMeshUpdate(best.pos, ExtUI::G29_POINT_FINISH);
          ExtUI::onMeshUpdate(best.pos, measured_z);
        #endif
      }
      SERIAL_FLUSH(); // Prevent host M105 buffer overrun.

    } while (best.pos.x >= 0 && --count);

    GRID_LOOP(x, y) if (z_values[x][y] == HUGE_VALF) z_values[x][y] = NAN; // Restore NAN for HUGE_VALF marks

    TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(best.pos, ExtUI::G29_FINISH));

    // Release UI during stow to allow for PAUSE_BEFORE_DEPLOY_STOW
    TERN_(HAS_MARLINUI_MENU, ui.release());
    probe.stow();
    TERN_(HAS_MARLINUI_MENU, ui.capture());

    probe.move_z_after_probing();

    do_blocking_move_to_xy(
      constrain(nearby.x - probe.offset_xy.x, MESH_MIN_X, MESH_MAX_X),
      constrain(nearby.y - probe.offset_xy.y, MESH_MIN_Y, MESH_MAX_Y)
    );

    restore_ubl_active_state();
  }

#endif // HAS_BED_PROBE

void set_message_with_feedback(FSTR_P const fstr) {
  #if HAS_MARLINUI_MENU
    ui.set_status(fstr);
    ui.quick_feedback();
  #else
    UNUSED(fstr);
  #endif
}

#if HAS_MARLINUI_MENU

  typedef void (*clickFunc_t)();

  bool _click_and_hold(const clickFunc_t func=nullptr) {
    if (ui.button_pressed()) {
      ui.quick_feedback(false);         // Preserve button state for click-and-hold
      const millis_t nxt = millis() + 1500UL;
      while (ui.button_pressed()) {     // Loop while the encoder is pressed. Uses hardware flag!
        idle();                         // idle, of course
        if (ELAPSED(millis(), nxt)) {   // After 1.5 seconds
          ui.quick_feedback();
          if (func) (*func)();
          ui.wait_for_release();
          return true;
        }
      }
    }
    serial_delay(15);
    return false;
  }

  void unified_bed_leveling::move_z_with_encoder(const_float_t multiplier) {
    ui.wait_for_release();
    while (!ui.button_pressed()) {
      idle();
      gcode.reset_stepper_timeout(); // Keep steppers powered
      if (encoder_diff) {
        do_blocking_move_to_z(current_position.z + float(encoder_diff) * multiplier);
        encoder_diff = 0;
      }
    }
  }

  float unified_bed_leveling::measure_point_with_encoder() {
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    const float z_step = 0.01f;
    move_z_with_encoder(z_step);
    return current_position.z;
  }

  static void echo_and_take_a_measurement() { SERIAL_ECHOLNPGM(" and take a measurement."); }

  float unified_bed_leveling::measure_business_card_thickness() {
    ui.capture();
    save_ubl_active_state_and_disable();   // Disable bed level correction for probing

    do_blocking_move_to(
      xyz_pos_t({
        0.5f * ((MESH_MAX_X) - (MESH_MIN_X)),
        0.5f * ((MESH_MAX_Y) - (MESH_MIN_Y)),
        MANUAL_PROBE_START_Z
        #ifdef SAFE_BED_LEVELING_START_I
          , SAFE_BED_LEVELING_START_I
        #endif
        #ifdef SAFE_BED_LEVELING_START_J
          , SAFE_BED_LEVELING_START_J
        #endif
        #ifdef SAFE_BED_LEVELING_START_K
          , SAFE_BED_LEVELING_START_K
        #endif
        #ifdef SAFE_BED_LEVELING_START_U
          , SAFE_BED_LEVELING_START_U
        #endif
        #ifdef SAFE_BED_LEVELING_START_V
          , SAFE_BED_LEVELING_START_V
        #endif
        #ifdef SAFE_BED_LEVELING_START_W
          , SAFE_BED_LEVELING_START_W
        #endif
      })
      //, _MIN(planner.settings.max_feedrate_mm_s[X_AXIS], planner.settings.max_feedrate_mm_s[Y_AXIS]) * 0.5f
    );
    planner.synchronize();

    SERIAL_ECHOPGM("Place shim under nozzle");
    LCD_MESSAGE(MSG_UBL_BC_INSERT);
    ui.return_to_status();
    echo_and_take_a_measurement();

    const float z1 = measure_point_with_encoder();
    do_z_clearance_by(SIZE_OF_LITTLE_RAISE);

    SERIAL_ECHOPGM("Remove shim");
    LCD_MESSAGE(MSG_UBL_BC_REMOVE);
    echo_and_take_a_measurement();

    const float z2 = measure_point_with_encoder();
    do_z_clearance_by(Z_CLEARANCE_BETWEEN_PROBES);

    const float thickness = ABS(z1 - z2);

    if (param.V_verbosity > 1)
      SERIAL_ECHOLNPGM("Business Card is ", p_float_t(thickness, 4), "mm thick.");

    restore_ubl_active_state();

    return thickness;
  }

  /**
   * G29 P2 : Manually Probe Remaining Mesh Points.
   *          Move to INVALID points and
   *          NOTE: Blocks the G-code queue and captures Marlin UI during use.
   */
  void unified_bed_leveling::manually_probe_remaining_mesh(const xy_pos_t &pos, const_float_t z_clearance, const_float_t thick, const bool do_ubl_mesh_map) {
    ui.capture();
    TERN_(EXTENSIBLE_UI, ExtUI::onLevelingStart());

    save_ubl_active_state_and_disable();  // No bed level correction so only raw data is obtained
    do_blocking_move_to_xy_z(current_position, z_clearance);

    ui.return_to_status();

    mesh_index_pair location;
    const xy_int8_t &lpos = location.pos;
    do {
      location = find_closest_mesh_point_of_type(INVALID, pos);
      // It doesn't matter if the probe can't reach the NAN location. This is a manual probe.
      if (!location.valid()) continue;

      const xyz_pos_t ppos = { get_mesh_x(lpos.x), get_mesh_y(lpos.y), z_clearance };

      if (!position_is_reachable(ppos)) break; // SHOULD NOT OCCUR (find_closest_mesh_point only returns reachable points)

      LCD_MESSAGE(MSG_UBL_MOVING_TO_NEXT);

      do_blocking_move_to(ppos);
      do_z_clearance(z_clearance);

      KEEPALIVE_STATE(PAUSED_FOR_USER);
      ui.capture();

      if (do_ubl_mesh_map) display_map(param.T_map_type);   // Show user where we're probing

      if (parser.seen_test('B')) {
        SERIAL_ECHOPGM("Place Shim & Measure");
        LCD_MESSAGE(MSG_UBL_BC_INSERT);
      }
      else {
        SERIAL_ECHOPGM("Measure");
        LCD_MESSAGE(MSG_UBL_BC_INSERT2);
      }

      const float z_step = 0.01f;                         // 0.01mm per encoder tick, occasionally step
      move_z_with_encoder(z_step);

      if (_click_and_hold([]{
        SERIAL_ECHOLNPGM("\nMesh only partially populated.");
        do_z_clearance(Z_CLEARANCE_DEPLOY_PROBE);
      })) return restore_ubl_active_state();

      // Store the Z position minus the shim height
      z_values[lpos.x][lpos.y] = current_position.z - thick;

      // Tell the external UI to update
      TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(location, z_values[lpos.x][lpos.y]));

      if (param.V_verbosity > 2)
        SERIAL_ECHOLNPGM("Mesh Point Measured at: ", p_float_t(z_values[lpos.x][lpos.y], 6));
      SERIAL_FLUSH(); // Prevent host M105 buffer overrun.
    } while (location.valid());

    if (do_ubl_mesh_map) display_map(param.T_map_type);  // show user where we're probing

    restore_ubl_active_state();
    do_blocking_move_to_xy_z(pos, Z_CLEARANCE_DEPLOY_PROBE);
  }

  /**
   * G29 P4 : Mesh Fine-Tuning. Go to point(s) and adjust values with the LCD.
   *          NOTE: Blocks the G-code queue and captures Marlin UI during use.
   */
  void unified_bed_leveling::fine_tune_mesh(const xy_pos_t &pos, const bool do_ubl_mesh_map) {
    if (!parser.seen_test('R')) // fine_tune_mesh() is special. If no repetition count flag is specified
      param.R_repetition = 1;   // do exactly one mesh location. Otherwise use what the parser decided.

    #if ENABLED(UBL_MESH_EDIT_MOVES_Z)
      const float h_offset = parser.seenval('H') ? parser.value_linear_units() : MANUAL_PROBE_START_Z;
      if (!WITHIN(h_offset, 0, 10)) {
        SERIAL_ECHOLNPGM("Offset out of bounds. (0 to 10mm)\n");
        return;
      }
    #endif

    mesh_index_pair location;

    if (!position_is_reachable(pos)) {
      SERIAL_ECHOLNPGM("(X,Y) outside printable radius.");
      return;
    }

    save_ubl_active_state_and_disable();

    LCD_MESSAGE(MSG_UBL_FINE_TUNE_MESH);
    ui.capture();                                         // Take over control of the LCD encoder

    do_blocking_move_to_xy_z(pos, Z_TWEEN_SAFE_CLEARANCE);  // Move to the given XY with probe clearance

    MeshFlags done_flags{0};
    const xy_int8_t &lpos = location.pos;

    #if IS_TFTGLCD_PANEL
      ui.ubl_mesh_edit_start(0);                          // Change current screen before calling ui.ubl_plot
      safe_delay(50);
    #endif

    do {
      location = find_closest_mesh_point_of_type(SET_IN_BITMAP, pos, false, &done_flags);

      if (lpos.x < 0) break;                              // Stop when there are no more reachable points

      done_flags.mark(lpos);                              // Mark this location as 'adjusted' so a new
                                                          // location is used on the next loop
      const xyz_pos_t raw = { get_mesh_x(lpos.x), get_mesh_y(lpos.y), Z_TWEEN_SAFE_CLEARANCE };

      if (!position_is_reachable(raw)) break;             // SHOULD NOT OCCUR (find_closest_mesh_point_of_type only returns reachable)

      do_blocking_move_to(raw);                           // Move the nozzle to the edit point with probe clearance

      TERN_(UBL_MESH_EDIT_MOVES_Z, do_blocking_move_to_z(h_offset)); // Move Z to the given 'H' offset before editing

      KEEPALIVE_STATE(PAUSED_FOR_USER);

      if (do_ubl_mesh_map) display_map(param.T_map_type); // Display the current point

      #if IS_TFTGLCD_PANEL
        ui.ubl_plot(lpos.x, lpos.y);   // update plot screen
      #endif

      ui.refresh();

      float new_z = z_values[lpos.x][lpos.y];
      if (isnan(new_z)) new_z = 0;                        // Invalid points begin at 0
      new_z = FLOOR(new_z * 1000) * 0.001f;               // Chop off digits after the 1000ths place

      ui.ubl_mesh_edit_start(new_z);

      SET_SOFT_ENDSTOP_LOOSE(true);

      do {
        idle_no_sleep();
        new_z = ui.ubl_mesh_value();
        TERN_(UBL_MESH_EDIT_MOVES_Z, do_blocking_move_to_z(h_offset + new_z)); // Move the nozzle as the point is edited
        SERIAL_FLUSH();                                   // Prevent host M105 buffer overrun.
      } while (!ui.button_pressed());

      SET_SOFT_ENDSTOP_LOOSE(false);

      if (!lcd_map_control) ui.return_to_status();        // Just editing a single point? Return to status

      // Button held down? Abort editing
      if (_click_and_hold([]{
        ui.return_to_status();
        do_z_clearance(Z_TWEEN_SAFE_CLEARANCE);
        set_message_with_feedback(GET_TEXT_F(MSG_EDITING_STOPPED));
      })) break;

      // TODO: Disable leveling here so the Z value becomes the 'native' Z value.

      z_values[lpos.x][lpos.y] = new_z;                   // Save the updated Z value

      // TODO: Re-enable leveling here so Z is correctly based on the updated mesh.

      TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(location, new_z));

      serial_delay(20);                                   // No switch noise
      ui.refresh();

    } while (lpos.x >= 0 && --param.R_repetition > 0);

    if (do_ubl_mesh_map) display_map(param.T_map_type);
    restore_ubl_active_state();

    do_blocking_move_to_xy_z(pos, Z_TWEEN_SAFE_CLEARANCE);

    LCD_MESSAGE(MSG_UBL_DONE_EDITING_MESH);
    SERIAL_ECHOLNPGM("Done Editing Mesh");

    if (lcd_map_control)
      ui.goto_screen(ubl_map_screen);
    else
      ui.return_to_status();
  }

#endif // HAS_MARLINUI_MENU

/**
 * Parse and validate most G29 parameters, store for use by G29 functions.
 */
bool unified_bed_leveling::G29_parse_parameters() {
  bool err_flag = false;

  set_message_with_feedback(GET_TEXT_F(MSG_UBL_DOING_G29));

  param.C_constant = 0;
  param.R_repetition = 0;

  if (parser.seen('R')) {
    param.R_repetition = parser.has_value() ? parser.value_ushort() : GRID_MAX_POINTS;
    NOMORE(param.R_repetition, GRID_MAX_POINTS);
    if (param.R_repetition < 1) {
      SERIAL_ECHOLNPGM("?(R)epetition count invalid (1+).\n");
      return UBL_ERR;
    }
  }

  param.V_verbosity = parser.byteval('V');
  if (!WITHIN(param.V_verbosity, 0, 4)) {
    SERIAL_ECHOLNPGM("?(V)erbose level implausible (0-4).\n");
    err_flag = true;
  }

  if (parser.seen('P')) {
    const uint8_t pv = parser.value_byte();
    #if !HAS_BED_PROBE
      if (pv == 1) {
        SERIAL_ECHOLNPGM("G29 P1 requires a probe.\n");
        err_flag = true;
      }
      else
    #endif
      {
        param.P_phase = pv;
        if (!WITHIN(param.P_phase, 0, 6)) {
          SERIAL_ECHOLNPGM("?(P)hase value invalid (0-6).\n");
          err_flag = true;
        }
      }
  }

  if (parser.seen('J')) {
    #if HAS_BED_PROBE
      param.J_grid_size = parser.value_byte();
      if (param.J_grid_size && !WITHIN(param.J_grid_size, 2, 9)) {
        SERIAL_ECHOLNPGM("?Invalid grid size (J) specified (2-9).\n");
        err_flag = true;
      }
    #else
      SERIAL_ECHOLNPGM("G29 J action requires a probe.\n");
      err_flag = true;
    #endif
  }

  param.XY_seen.x = parser.seenval('X');
  float sx = param.XY_seen.x ? parser.value_float() : current_position.x;
  param.XY_seen.y = parser.seenval('Y');
  float sy = param.XY_seen.y ? parser.value_float() : current_position.y;

  if (param.XY_seen.x != param.XY_seen.y) {
    SERIAL_ECHOLNPGM("Both X & Y locations must be specified.\n");
    err_flag = true;
  }

  // If X or Y are not valid, use center of the bed values
  // (for UBL_HILBERT_CURVE default to lower-left corner instead)
  if (!COORDINATE_OKAY(sx, X_MIN_BED, X_MAX_BED)) sx = TERN(UBL_HILBERT_CURVE, 0, X_CENTER);
  if (!COORDINATE_OKAY(sy, Y_MIN_BED, Y_MAX_BED)) sy = TERN(UBL_HILBERT_CURVE, 0, Y_CENTER);

  if (err_flag) return UBL_ERR;

  param.XY_pos.set(sx, sy);

  /**
   * Activate or deactivate UBL
   * Note: UBL's G29 restores the state set here when done.
   *       Leveling is being enabled here with old data, possibly
   *       none. Error handling should disable for safety...
   */
  if (parser.seen_test('A')) {
    if (parser.seen_test('D')) {
      SERIAL_ECHOLNPGM("?Can't activate and deactivate at the same time.\n");
      return UBL_ERR;
    }
    set_bed_leveling_enabled(true);
    report_state();
  }
  else if (parser.seen_test('D')) {
    set_bed_leveling_enabled(false);
    report_state();
  }

  // Set global 'C' flag and its value
  if ((param.C_seen = parser.seen('C')))
    param.C_constant = parser.value_float();

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    if (parser.seenval('F')) {
      const float fh = parser.value_float();
      if (!WITHIN(fh, 0, 100)) {
        SERIAL_ECHOLNPGM("?(F)ade height for Bed Level Correction not plausible.\n");
        return UBL_ERR;
      }
      set_z_fade_height(fh);
    }
  #endif

  param.T_map_type = parser.byteval('T');
  if (!WITHIN(param.T_map_type, 0, 2)) {
    SERIAL_ECHOLNPGM("Invalid map type.\n");
    return UBL_ERR;
  }
  return UBL_OK;
}

static uint8_t ubl_state_at_invocation = 0;

#if ENABLED(UBL_DEVEL_DEBUGGING)
  static uint8_t ubl_state_recursion_chk = 0;
#endif

void unified_bed_leveling::save_ubl_active_state_and_disable() {
  #if ENABLED(UBL_DEVEL_DEBUGGING)
    ubl_state_recursion_chk++;
    if (ubl_state_recursion_chk != 1) {
      SERIAL_ECHOLNPGM("save_ubl_active_state_and_disabled() called multiple times in a row.");
      set_message_with_feedback(GET_TEXT_F(MSG_UBL_SAVE_ERROR));
      return;
    }
  #endif
  ubl_state_at_invocation = planner.leveling_active;
  set_bed_leveling_enabled(false);
}

void unified_bed_leveling::restore_ubl_active_state(const bool is_done/*=true*/) {
  TERN_(HAS_MARLINUI_MENU, ui.release());
  #if ENABLED(UBL_DEVEL_DEBUGGING)
    if (--ubl_state_recursion_chk) {
      SERIAL_ECHOLNPGM("restore_ubl_active_state() called too many times.");
      set_message_with_feedback(GET_TEXT_F(MSG_UBL_RESTORE_ERROR));
      return;
    }
  #endif
  set_bed_leveling_enabled(ubl_state_at_invocation);

  if (is_done) {
    TERN_(EXTENSIBLE_UI, ExtUI::onLevelingDone());
  }
}

mesh_index_pair unified_bed_leveling::find_furthest_invalid_mesh_point() {

  bool found_a_NAN = false, found_a_real = false;

  mesh_index_pair farthest { -1, -1, -99999.99 };

  GRID_LOOP(i, j) {
    if (!isnan(z_values[i][j])) continue;  // Skip valid mesh points

    // Skip unreachable points
    if (!probe.can_reach(get_mesh_x(i), get_mesh_y(j)))
      continue;

    found_a_NAN = true;

    xy_int8_t nearby { -1, -1 };
    float d1, d2 = 99999.9f;
    GRID_LOOP(k, l) {
      if (isnan(z_values[k][l])) continue;

      found_a_real = true;

      // Add in a random weighting factor that scrambles the probing of the
      // last half of the mesh (when every unprobed mesh point is one index
      // from a probed location).

      d1 = HYPOT(i - k, j - l) + (1.0f / ((millis() % 47) + 13));

      if (d1 < d2) {    // Invalid mesh point (i,j) is closer to the defined point (k,l)
        d2 = d1;
        nearby.set(i, j);
      }
    }

    //
    // At this point d2 should have the near defined mesh point to invalid mesh point (i,j)
    //

    if (found_a_real && nearby.x >= 0 && d2 > farthest.distance) {
      farthest.pos = nearby; // Found an invalid location farther from the defined mesh point
      farthest.distance = d2;
    }
  } // GRID_LOOP

  if (!found_a_real && found_a_NAN) {        // if the mesh is totally unpopulated, start the probing
    farthest.pos.set((GRID_MAX_POINTS_X) / 2, (GRID_MAX_POINTS_Y) / 2);
    farthest.distance = 1;
  }
  return farthest;
}

#if ENABLED(UBL_HILBERT_CURVE)

  typedef struct {
    MeshPointType   type;
    MeshFlags       *done_flags;
    bool            probe_relative;
    mesh_index_pair closest;
  } find_closest_t;

  static bool test_func(uint8_t i, uint8_t j, void *data) {
    find_closest_t *d = (find_closest_t*)data;
    if (  d->type == CLOSEST || d->type == (isnan(bedlevel.z_values[i][j]) ? INVALID : REAL)
      || (d->type == SET_IN_BITMAP && !d->done_flags->marked(i, j))
    ) {
      // Found a Mesh Point of the specified type!
      const xy_pos_t mpos = { bedlevel.get_mesh_x(i), bedlevel.get_mesh_y(j) };

      // If using the probe as the reference there are some unreachable locations.
      // Also for round beds, there are grid points outside the bed the nozzle can't reach.
      // Prune them from the list and ignore them till the next Phase (manual nozzle probing).

      if (!(d->probe_relative ? probe.can_reach(mpos) : position_is_reachable(mpos)))
        return false;
      d->closest.pos.set(i, j);
      return true;
    }
    return false;
  }

#endif

mesh_index_pair unified_bed_leveling::find_closest_mesh_point_of_type(const MeshPointType type, const xy_pos_t &pos, const bool probe_relative/*=false*/, MeshFlags *done_flags/*=nullptr*/) {

  #if ENABLED(UBL_HILBERT_CURVE)

    find_closest_t d;
    d.type           = type;
    d.done_flags     = done_flags;
    d.probe_relative = probe_relative;
    d.closest.invalidate();
    hilbert_curve::search_from_closest(pos, test_func, &d);
    return d.closest;

  #else

    mesh_index_pair closest;
    closest.invalidate();
    closest.distance = -99999.9f;

    // Get the reference position, either nozzle or probe
    const xy_pos_t ref = probe_relative ? pos + probe.offset_xy : pos;

    float best_so_far = 99999.99f;

    GRID_LOOP(i, j) {
      if (  type == CLOSEST || type == (isnan(z_values[i][j]) ? INVALID : REAL)
        || (type == SET_IN_BITMAP && !done_flags->marked(i, j))
      ) {
        // Found a Mesh Point of the specified type!
        const xy_pos_t mpos = { get_mesh_x(i), get_mesh_y(j) };

        // If using the probe as the reference there are some unreachable locations.
        // Also for round beds, there are grid points outside the bed the nozzle can't reach.
        // Prune them from the list and ignore them till the next Phase (manual nozzle probing).

        if (!(probe_relative ? probe.can_reach(mpos) : position_is_reachable(mpos)))
          continue;

        // Reachable. Check if it's the best_so_far location to the nozzle.

        const xy_pos_t diff = current_position - mpos;
        const float distance = (ref - mpos).magnitude() + diff.magnitude() * 0.1f;

        // factor in the distance from the current location for the normal case
        // so the nozzle isn't running all over the bed.
        if (distance < best_so_far) {
          best_so_far = distance;   // Found a closer location with the desired value type.
          closest.pos.set(i, j);
          closest.distance = best_so_far;
        }
      }
    } // GRID_LOOP

    return closest;

  #endif
}

/**
 * 'Smart Fill': Scan from the outward edges of the mesh towards the center.
 * If an invalid location is found, use the next two points (if valid) to
 * calculate a 'reasonable' value for the unprobed mesh point.
 */

bool unified_bed_leveling::smart_fill_one(const uint8_t x, const uint8_t y, const int8_t xdir, const int8_t ydir) {
  const float v = z_values[x][y];
  if (isnan(v)) {                           // A NAN...
    const int8_t dx = x + xdir, dy = y + ydir;
    const float v1 = z_values[dx][dy];
    if (!isnan(v1)) {                       // ...next to a pair of real values?
      const float v2 = z_values[dx + xdir][dy + ydir];
      if (!isnan(v2)) {
        z_values[x][y] = v1 < v2 ? v1 : v1 + v1 - v2;
        TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, z_values[x][y]));
        return true;
      }
    }
  }
  return false;
}

typedef struct { uint8_t sx, ex, sy, ey; bool yfirst; } smart_fill_info;

void unified_bed_leveling::smart_fill_mesh() {
  static const smart_fill_info
    info0 PROGMEM = { 0, GRID_MAX_POINTS_X,       0, (GRID_MAX_POINTS_Y) - 2, false },  // Bottom of the mesh looking up
    info1 PROGMEM = { 0, GRID_MAX_POINTS_X,     (GRID_MAX_POINTS_Y) - 1, 0,   false },  // Top of the mesh looking down
    info2 PROGMEM = { 0, (GRID_MAX_POINTS_X) - 2, 0, GRID_MAX_POINTS_Y,       true  },  // Left side of the mesh looking right
    info3 PROGMEM = { (GRID_MAX_POINTS_X) - 1, 0, 0, GRID_MAX_POINTS_Y,       true  };  // Right side of the mesh looking left
  static const smart_fill_info * const info[] PROGMEM = { &info0, &info1, &info2, &info3 };

  for (uint8_t i = 0; i < COUNT(info); ++i) {
    const smart_fill_info *f = (smart_fill_info*)pgm_read_ptr(&info[i]);
    const int8_t sx = pgm_read_byte(&f->sx), sy = pgm_read_byte(&f->sy),
                 ex = pgm_read_byte(&f->ex), ey = pgm_read_byte(&f->ey);
    if (pgm_read_byte(&f->yfirst)) {
      const int8_t dir = ex > sx ? 1 : -1;
      for (uint8_t y = sy; y != ey; ++y)
        for (uint8_t x = sx; x != ex; x += dir)
          if (smart_fill_one(x, y, dir, 0)) break;
    }
    else {
      const int8_t dir = ey > sy ? 1 : -1;
       for (uint8_t x = sx; x != ex; ++x)
        for (uint8_t y = sy; y != ey; y += dir)
          if (smart_fill_one(x, y, 0, dir)) break;
    }
  }
}

#if HAS_BED_PROBE

  //#define VALIDATE_MESH_TILT

  #include "../../../libs/vector_3.h"

  void unified_bed_leveling::tilt_mesh_based_on_probed_grid(const bool do_3_pt_leveling) {

    float measured_z;
    bool abort_flag = false;

    struct linear_fit_data lsf_results;
    incremental_LSF_reset(&lsf_results);

    if (do_3_pt_leveling) {
      xy_float_t points[3];
      probe.get_three_points(points);

      #if ENABLED(UBL_TILT_ON_MESH_POINTS_3POINT)
        mesh_index_pair cpos[3];
        for (uint8_t ix = 0; ix < 3; ++ix) { // Convert points to coordinates of mesh points
          cpos[ix] = find_closest_mesh_point_of_type(REAL, points[ix], true);
          points[ix] = cpos[ix].meshpos();
        }
      #endif

      #if ENABLED(VALIDATE_MESH_TILT)
        float gotz[3];  // Used for algorithm validation below
      #endif

      for (uint8_t i = 0; i < 3; ++i) {
        SERIAL_ECHOLNPGM("Tilting mesh (", i + 1, "/3)");
        TERN_(HAS_STATUS_MESSAGE, ui.status_printf(0, F(S_FMT " %i/3"), GET_TEXT_F(MSG_LCD_TILTING_MESH), i + 1));

        measured_z = probe.probe_at_point(points[i], i < 2 ? PROBE_PT_RAISE : PROBE_PT_LAST_STOW, param.V_verbosity);
        if ((abort_flag = isnan(measured_z))) break;

        measured_z -= TERN(UBL_TILT_ON_MESH_POINTS_3POINT, z_values[cpos[i].pos.x][cpos[i].pos.y], get_z_correction(points[i]));
        TERN_(VALIDATE_MESH_TILT, gotz[i] = measured_z);

        if (param.V_verbosity > 3) { SERIAL_ECHO_SP(16); SERIAL_ECHOLNPGM("Corrected_Z=", measured_z); }

        incremental_LSF(&lsf_results, points[i], measured_z);
      }

      probe.stow();
      probe.move_z_after_probing();

      if (abort_flag) {
        SERIAL_ECHOLNPGM("?Error probing point. Aborting operation.");
        return;
      }
    }
    else { // !do_3_pt_leveling

      #ifndef G29J_MESH_TILT_MARGIN
        #define G29J_MESH_TILT_MARGIN 0
      #endif
      const float x_min = _MAX((X_MIN_POS) + (G29J_MESH_TILT_MARGIN), MESH_MIN_X, probe.min_x()),
                  x_max = _MIN((X_MAX_POS) - (G29J_MESH_TILT_MARGIN), MESH_MAX_X, probe.max_x()),
                  y_min = _MAX((Y_MIN_POS) + (G29J_MESH_TILT_MARGIN), MESH_MIN_Y, probe.min_y()),
                  y_max = _MIN((Y_MAX_POS) - (G29J_MESH_TILT_MARGIN), MESH_MAX_Y, probe.max_y()),
                  dx = (x_max - x_min) / (param.J_grid_size - 1),
                  dy = (y_max - y_min) / (param.J_grid_size - 1);

      bool zig_zag = false;

      const uint16_t total_points = sq(param.J_grid_size);
      uint16_t point_num = 1;

      for (uint8_t ix = 0; ix < param.J_grid_size; ++ix) {
        xy_pos_t rpos;
        rpos.x = x_min + ix * dx;
        for (uint8_t iy = 0; iy < param.J_grid_size; ++iy) {
          rpos.y = y_min + dy * (zig_zag ? param.J_grid_size - 1 - iy : iy);

          #if ENABLED(UBL_TILT_ON_MESH_POINTS)
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              xy_pos_t oldRpos;
              if (DEBUGGING(LEVELING)) oldRpos = rpos;
            #endif
            mesh_index_pair cpos;
            rpos -= probe.offset;
            cpos = find_closest_mesh_point_of_type(REAL, rpos, true);
            rpos = cpos.meshpos();
          #endif

          SERIAL_ECHOLNPGM("Tilting mesh point ", point_num, "/", total_points, "\n");
          TERN_(HAS_STATUS_MESSAGE, ui.status_printf(0, F(S_FMT " %i/%i"), GET_TEXT_F(MSG_LCD_TILTING_MESH), point_num, total_points));

          measured_z = probe.probe_at_point(rpos, parser.seen_test('E') ? PROBE_PT_STOW : PROBE_PT_RAISE, param.V_verbosity); // TODO: Needs error handling

          if ((abort_flag = isnan(measured_z))) break;

          const float zcorr = TERN(UBL_TILT_ON_MESH_POINTS, z_values[cpos.pos.x][cpos.pos.y], get_z_correction(rpos));

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              #if ENABLED(UBL_TILT_ON_MESH_POINTS)
                const xy_pos_t oldLpos = oldRpos.asLogical();
                DEBUG_ECHO(F("Calculated point: ("), p_float_t(oldRpos.x, 7), C(','), p_float_t(oldRpos.y, 7),
                           F(")   logical: ("), p_float_t(oldLpos.x, 7), C(','), p_float_t(oldLpos.y, 7),
                           F(")\nSelected mesh point: ")
                );
              #endif
              const xy_pos_t lpos = rpos.asLogical();
              DEBUG_ECHO(      C('('), p_float_t(rpos.x, 7), C(','), p_float_t(rpos.y, 7),
                         F(")  logical: ("), p_float_t(lpos.x, 7), C(','), p_float_t(lpos.y, 7),
                         F(")  measured: "), p_float_t(measured_z, 7),
                         F("  correction: "), p_float_t(zcorr, 7)
              );
            }
          #endif

          measured_z -= zcorr;

          if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("  final >>>---> ", p_float_t(measured_z, 7));

          if (param.V_verbosity > 3) {
            SERIAL_ECHO_SP(16);
            SERIAL_ECHOLNPGM("Corrected_Z=", measured_z);
          }
          incremental_LSF(&lsf_results, rpos, measured_z);

          point_num++;
        }

        if (abort_flag) break;
        FLIP(zig_zag);
      }
    }
    probe.stow();
    probe.move_z_after_probing();

    if (abort_flag || finish_incremental_LSF(&lsf_results)) {
      SERIAL_ECHOLNPGM("Could not complete LSF!");
      return;
    }

    vector_3 normal = vector_3(lsf_results.A, lsf_results.B, 1).get_normal();

    if (param.V_verbosity > 2)
      SERIAL_ECHOLN(F("bed plane normal = ["), p_float_t(normal.x, 7), C(','), p_float_t(normal.y, 7), C(','), p_float_t(normal.z, 7), C(']'));

    matrix_3x3 rotation = matrix_3x3::create_look_at(vector_3(lsf_results.A, lsf_results.B, 1));

    GRID_LOOP(i, j) {
      float mx = get_mesh_x(i), my = get_mesh_y(j), mz = z_values[i][j];

      if (DEBUGGING(LEVELING)) {
        DEBUG_ECHOLN(F("before rotation = ["), p_float_t(mx, 7), C(','), p_float_t(my, 7), C(','), p_float_t(mz, 7), F("]   ---> "));
        DEBUG_DELAY(20);
      }

      rotation.apply_rotation_xyz(mx, my, mz);

      if (DEBUGGING(LEVELING)) {
        DEBUG_ECHOLN(F("after rotation = ["), p_float_t(mx, 7), C(','), p_float_t(my, 7), C(','), p_float_t(mz, 7), F("]   ---> "));
        DEBUG_DELAY(20);
      }

      z_values[i][j] = mz - lsf_results.D;
      TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(i, j, z_values[i][j]));
    }

    if (DEBUGGING(LEVELING)) {
      rotation.debug(F("rotation matrix:\n"));
      DEBUG_ECHOLN(F("LSF Results A="), p_float_t(lsf_results.A, 7), F("  B="), p_float_t(lsf_results.B, 7), F("  D="), p_float_t(lsf_results.D, 7));
      DEBUG_DELAY(55);
      DEBUG_ECHOLN(F("bed plane normal = ["), p_float_t(normal.x, 7), C(','), p_float_t(normal.y, 7), C(','), p_float_t(normal.z, 7), C(']'));
      DEBUG_EOL();

      /**
       * Use the code below to check the validity of the mesh tilting algorithm.
       * 3-Point Mesh Tilt uses the same algorithm as grid-based tilting, but only
       * three points are used in the calculation. This guarantees that each probed point
       * has an exact match when get_z_correction() for that location is calculated.
       * The Z error between the probed point locations and the get_z_correction()
       * numbers for those locations should be 0.
       */
      #if ENABLED(VALIDATE_MESH_TILT)
        auto d_from = []{ DEBUG_ECHOPGM("D from "); };
        auto normed = [&](const xy_pos_t &pos, const_float_t zadd) {
          return normal.x * pos.x + normal.y * pos.y + zadd;
        };
        auto debug_pt = [](const int num, const xy_pos_t &pos, const_float_t zadd) {
          d_from();
          DEBUG_ECHOLN(F("Point "), num, C(':'), p_float_t(normed(pos, zadd), 6), F("   Z error = "), p_float_t(zadd - get_z_correction(pos), 6));
        };
        debug_pt(1, probe_pt[0], normal.z * gotz[0]);
        debug_pt(2, probe_pt[1], normal.z * gotz[1]);
        debug_pt(3, probe_pt[2], normal.z * gotz[2]);
        #if ENABLED(Z_SAFE_HOMING)
          constexpr xy_float_t safe_xy = { Z_SAFE_HOMING_X_POINT, Z_SAFE_HOMING_Y_POINT };
          d_from(); DEBUG_ECHOLN(F("safe home with Z="), F("0 : "), p_float_t(normed(safe_xy, 0), 6));
          d_from(); DEBUG_ECHOLN(F("safe home with Z="), F("mesh value "), p_float_t(normed(safe_xy, get_z_correction(safe_xy)), 6));
          DEBUG_ECHO(F("   Z error = ("), Z_SAFE_HOMING_X_POINT, C(','), Z_SAFE_HOMING_Y_POINT, F(") = "), p_float_t(get_z_correction(safe_xy), 6));
        #endif
      #endif
    } // DEBUGGING(LEVELING)

  }

#endif // HAS_BED_PROBE

#if ENABLED(UBL_G29_P31)
  void unified_bed_leveling::smart_fill_wlsf(const_float_t weight_factor) {

    // For each undefined mesh point, compute a distance-weighted least squares fit
    // from all the originally populated mesh points, weighted toward the point
    // being extrapolated so that nearby points will have greater influence on
    // the point being extrapolated.  Then extrapolate the mesh point from WLSF.

    static_assert((GRID_MAX_POINTS_Y) <= 16, "GRID_MAX_POINTS_Y too big");
    uint16_t bitmap[GRID_MAX_POINTS_X] = { 0 };
    struct linear_fit_data lsf_results;

    SERIAL_ECHOPGM("Extrapolating mesh...");

    const float weight_scaled = weight_factor * _MAX(MESH_X_DIST, MESH_Y_DIST);

    GRID_LOOP(jx, jy) if (!isnan(z_values[jx][jy])) SBI(bitmap[jx], jy);

    xy_pos_t ppos;
    for (uint8_t ix = 0; ix < GRID_MAX_POINTS_X; ++ix) {
      ppos.x = get_mesh_x(ix);
      for (uint8_t iy = 0; iy < GRID_MAX_POINTS_Y; ++iy) {
        ppos.y = get_mesh_y(iy);
        if (isnan(z_values[ix][iy])) {
          // undefined mesh point at (ppos.x,ppos.y), compute weighted LSF from original valid mesh points.
          incremental_LSF_reset(&lsf_results);
          xy_pos_t rpos;
          for (uint8_t jx = 0; jx < GRID_MAX_POINTS_X; ++jx) {
            rpos.x = get_mesh_x(jx);
            for (uint8_t jy = 0; jy < GRID_MAX_POINTS_Y; ++jy) {
              if (TEST(bitmap[jx], jy)) {
                rpos.y = get_mesh_y(jy);
                const float rz = z_values[jx][jy],
                             w = 1.0f + weight_scaled / (rpos - ppos).magnitude();
                incremental_WLSF(&lsf_results, rpos, rz, w);
              }
            }
          }
          if (finish_incremental_LSF(&lsf_results)) {
            SERIAL_ECHOLNPGM(" Insufficient data");
            return;
          }
          const float ez = -lsf_results.D - lsf_results.A * ppos.x - lsf_results.B * ppos.y;
          z_values[ix][iy] = ez;
          TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(ix, iy, z_values[ix][iy]));
          idle(); // housekeeping
        }
      }
    }

    SERIAL_ECHOLNPGM(" done.");
  }
#endif // UBL_G29_P31

#if ENABLED(UBL_DEVEL_DEBUGGING)
  /**
   * Much of the 'What?' command can be eliminated. But until we are fully debugged, it is
   * good to have the extra information. Soon... we prune this to just a few items
   */
  void unified_bed_leveling::g29_what_command() {
    report_state();

    if (storage_slot == -1)
      SERIAL_ECHOLNPGM("No Mesh Loaded.");
    else
      SERIAL_ECHOLNPGM("Mesh ", storage_slot, " Loaded.");
    serial_delay(50);

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      SERIAL_ECHOLN(F("Fade Height M420 Z"), p_float_t(planner.z_fade_height, 4));
    #endif

    adjust_mesh_to_mean(param.C_seen, param.C_constant);

    #if HAS_BED_PROBE
      SERIAL_ECHOLNPGM("Probe Offset M851 Z", p_float_t(probe.offset.z, 7));
    #endif

    SERIAL_ECHOLNPGM("MESH_MIN_X  " STRINGIFY(MESH_MIN_X) "=", MESH_MIN_X); serial_delay(50);
    SERIAL_ECHOLNPGM("MESH_MIN_Y  " STRINGIFY(MESH_MIN_Y) "=", MESH_MIN_Y); serial_delay(50);
    SERIAL_ECHOLNPGM("MESH_MAX_X  " STRINGIFY(MESH_MAX_X) "=", MESH_MAX_X); serial_delay(50);
    SERIAL_ECHOLNPGM("MESH_MAX_Y  " STRINGIFY(MESH_MAX_Y) "=", MESH_MAX_Y); serial_delay(50);
    SERIAL_ECHOLNPGM("GRID_MAX_POINTS_X  ", GRID_MAX_POINTS_X);             serial_delay(50);
    SERIAL_ECHOLNPGM("GRID_MAX_POINTS_Y  ", GRID_MAX_POINTS_Y);             serial_delay(50);
    SERIAL_ECHOLNPGM("MESH_X_DIST  ", MESH_X_DIST);
    SERIAL_ECHOLNPGM("MESH_Y_DIST  ", MESH_Y_DIST);                         serial_delay(50);

    SERIAL_ECHOPGM("X-Axis Mesh Points at: ");
    for (uint8_t i = 0; i < GRID_MAX_POINTS_X; ++i) {
      SERIAL_ECHO(p_float_t(LOGICAL_X_POSITION(get_mesh_x(i)), 3), F("  "));
      serial_delay(25);
    }
    SERIAL_EOL();

    SERIAL_ECHOPGM("Y-Axis Mesh Points at: ");
    for (uint8_t i = 0; i < GRID_MAX_POINTS_Y; ++i) {
      SERIAL_ECHO(p_float_t(LOGICAL_Y_POSITION(get_mesh_y(i)), 3), F("  "));
      serial_delay(25);
    }
    SERIAL_EOL();

    #if HAS_KILL
      SERIAL_ECHOLNPGM("Kill pin on :", KILL_PIN, "  state:", kill_state());
    #endif

    SERIAL_EOL();
    serial_delay(50);

    SERIAL_ECHOLNPGM("ubl_state_at_invocation :", ubl_state_at_invocation, "\nubl_state_recursion_chk :", ubl_state_recursion_chk);
    serial_delay(50);

    SERIAL_ECHOLNPGM("Meshes go from ", _hex_word(settings.meshes_start_index()), " to ", _hex_word(settings.meshes_end_index()));
    serial_delay(50);

    SERIAL_ECHOLNPGM("sizeof(unified_bed_leveling) :  ", sizeof(unified_bed_leveling));
    SERIAL_ECHOLNPGM("z_value[][] size: ", sizeof(z_values));
    serial_delay(25);

    SERIAL_ECHOLNPGM("EEPROM free for UBL: ", _hex_word(settings.meshes_end_index() - settings.meshes_start_index()));
    serial_delay(50);

    SERIAL_ECHOLNPGM("EEPROM can hold ", settings.calc_num_meshes(), " meshes.\n");
    serial_delay(25);

    if (!sanity_check()) {
      echo_name();
      SERIAL_ECHOLNPGM(" sanity checks passed.");
    }
  }

  /**
   * When we are fully debugged, the EEPROM dump command will get deleted also. But
   * right now, it is good to have the extra information. Soon... we prune this.
   */
  void unified_bed_leveling::g29_eeprom_dump() {
    uint8_t cccc;

    SERIAL_ECHO_MSG("EEPROM Dump:");
    persistentStore.access_start();
    for (uint16_t i = 0; i < persistentStore.capacity(); i += 16) {
      if (!(i & 0x3)) idle();
      print_hex_word(i);
      SERIAL_ECHOPGM(": ");
      for (uint16_t j = 0; j < 16; j++) {
        int pos = i + j;
        persistentStore.read_data(pos, &cccc, sizeof(uint8_t));
        print_hex_byte(cccc);
        SERIAL_CHAR(' ');
      }
      SERIAL_EOL();
    }
    SERIAL_EOL();
    persistentStore.access_finish();
  }

  /**
   * When we are fully debugged, this may go away. But there are some valid
   * use cases for the users. So we can wait and see what to do with it.
   */
  void unified_bed_leveling::g29_compare_current_mesh_to_stored_mesh() {
    const int16_t a = settings.calc_num_meshes();

    if (!a) {
      SERIAL_ECHOLNPGM("?EEPROM storage not available.");
      return;
    }

    if (!parser.has_value() || !WITHIN(parser.value_int(), 0, a - 1)) {
      SERIAL_ECHOLNPGM("?Invalid storage slot.\n?Use 0 to ", a - 1);
      return;
    }

    param.KLS_storage_slot = (int8_t)parser.value_int();

    float tmp_z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];
    settings.load_mesh(param.KLS_storage_slot, &tmp_z_values);

    SERIAL_ECHOLNPGM("Subtracting mesh in slot ", param.KLS_storage_slot, " from current mesh.");

    GRID_LOOP(x, y) {
      z_values[x][y] -= tmp_z_values[x][y];
      TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, z_values[x][y]));
    }
  }

#endif // UBL_DEVEL_DEBUGGING

#endif // AUTO_BED_LEVELING_UBL
