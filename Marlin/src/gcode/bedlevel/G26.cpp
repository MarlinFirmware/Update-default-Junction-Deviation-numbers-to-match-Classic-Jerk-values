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
 *   G26 Mesh Validation Tool
 *
 *   G26 is a Mesh Validation Tool intended to provide support for the Marlin Unified Bed Leveling System.
 *   In order to fully utilize and benefit from the Marlin Unified Bed Leveling System an accurate Mesh must
 *   be defined. G29 is designed to allow the user to quickly validate the correctness of her Mesh. It will
 *   first heat the bed and nozzle. It will then print lines and circles along the Mesh Cell boundaries and
 *   the intersections of those lines (respectively).
 *
 *   This action allows the user to immediately see where the Mesh is properly defined and where it needs to
 *   be edited. The command will generate the Mesh lines closest to the nozzle's starting position. Alternatively
 *   the user can specify the X and Y position of interest with command parameters. This allows the user to
 *   focus on a particular area of the Mesh where attention is needed.
 *
 *   B #  Bed         Set the Bed Temperature. If not specified, a default of 60 C. will be assumed.
 *
 *   C    Current     When searching for Mesh Intersection points to draw, use the current nozzle location
 *                    as the base for any distance comparison.
 *
 *   D    Disable     Disable the Unified Bed Leveling System. In the normal case the user is invoking this
 *                    command to see how well a Mesh as been adjusted to match a print surface. In order to do
 *                    this the Unified Bed Leveling System is turned on by the G26 command. The D parameter
 *                    alters the command's normal behavior and disables the Unified Bed Leveling System even if
 *                    it is on.
 *
 *   H #  Hotend      Set the Nozzle Temperature. If not specified, a default of 205 C. will be assumed.
 *
 *   I #  Preset      Heat the Nozzle and Bed based on a Material Preset (if material presets are defined).
 *
 *   F #  Filament    Used to specify the diameter of the filament being used. If not specified
 *                    1.75mm filament is assumed. If you are not getting acceptable results by using the
 *                    'correct' numbers, you can scale this number up or down a little bit to change the amount
 *                    of filament that is being extruded during the printing of the various lines on the bed.
 *
 *   K    Keep-On     Keep the heaters turned on at the end of the command.
 *
 *   L #  Layer       Layer height. (Height of nozzle above bed)  If not specified .20mm will be used.
 *
 *   O #  Ooze        How much your nozzle will Ooooze filament while getting in position to print. If not
 *                    specified, a filament length of .3mm is assumed. This might be overkill, but this
 *                    parameter ensures the very first 'circle' is perfect (providing an ideal trophy to hang
 *                    up to show off your perfectly calibrated Mesh).
 *
 *   P #  Prime       Prime the nozzle with specified length of filament. If this parameter is not
 *                    given, no prime action will take place. If the parameter specifies an amount, that much
 *                    will be purged before continuing. If no amount is specified the command will start
 *                    purging filament until the user provides an LCD Click and then it will continue with
 *                    printing the Mesh. You can carefully remove the spent filament with a needle nose
 *                    pliers while holding the LCD Click wheel in a depressed state. If you do not have
 *                    an LCD, you must specify a value if you use P.
 *
 *   Q #  Multiplier  Retraction Multiplier. (Normally not needed.) During G26 retraction will use the length
 *                    specified by this parameter (1mm by default). Recover will be 1.2x the retract distance.
 *
 *   R #  Repeat      Prints the number of patterns given as a parameter, starting at the current location.
 *                    If a parameter isn't given, every point will be printed unless G26 is interrupted.
 *                    This works the same way that the UBL G29 P4 R parameter works.
 *
 *                    NOTE:  If you do not have an LCD, you -must- specify R. This is to ensure that you are
 *                    aware that there's some risk associated with printing without the ability to abort in
 *                    cases where mesh point Z value may be inaccurate. As above, if you do not include a
 *                    parameter, every point will be printed.
 *
 *   S #  Nozzle      Used to control the size of nozzle diameter. If not specified, a .4mm nozzle is assumed.
 *
 *   U #  Random      Randomize the order that the circles are drawn on the bed. The search for the closest
 *                    un-drawn circle is still done. But the distance to the location for each circle has a
 *                    random number of the specified size added to it. Specifying S50 will give an interesting
 *                    deviation from the normal behavior on a 10 x 10 Mesh.
 *
 *   X #  X Coord.    Specify the starting location of the drawing activity.
 *
 *   Y #  Y Coord.    Specify the starting location of the drawing activity.
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(G26_MESH_VALIDATION)

#define G26_OK false
#define G26_ERR true

#include "../gcode.h"
#include "../../feature/bedlevel/bedlevel.h"

#include "../../MarlinCore.h"
#include "../../module/planner.h"
#include "../../module/motion.h"
#include "../../module/tool_change.h"
#include "../../module/temperature.h"
#include "../../lcd/marlinui.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#endif

#if ENABLED(UBL_HILBERT_CURVE)
  #include "../../feature/bedlevel/hilbert_curve.h"
#endif

#define EXTRUSION_MULTIPLIER 1.0
#define PRIME_LENGTH 10.0
#define OOZE_AMOUNT 0.3

#define INTERSECTION_CIRCLE_RADIUS 5
#define CROSSHAIRS_SIZE 3

#ifndef G26_RETRACT_MULTIPLIER
  #define G26_RETRACT_MULTIPLIER 1.0 // x 1mm
#endif

#ifndef G26_XY_FEEDRATE
  #define G26_XY_FEEDRATE (PLANNER_XY_FEEDRATE_MM_S / 3.0)
#endif

#ifndef G26_XY_FEEDRATE_TRAVEL
  #define G26_XY_FEEDRATE_TRAVEL (PLANNER_XY_FEEDRATE_MM_S / 1.5)
#endif

#if CROSSHAIRS_SIZE >= INTERSECTION_CIRCLE_RADIUS
  #error "CROSSHAIRS_SIZE must be less than INTERSECTION_CIRCLE_RADIUS."
#endif

#define G26_OK false
#define G26_ERR true

#if ENABLED(ARC_SUPPORT)
  void plan_arc(const xyze_pos_t&, const ab_float_t&, const bool, const uint8_t);
#endif

constexpr float g26_e_axis_feedrate = 0.025;

static MeshFlags circle_flags;
float g26_random_deviation = 0.0;

#if HAS_MARLINUI_MENU

  /**
   * If the LCD is clicked, cancel, wait for release, return true
   */
  bool user_canceled() {
    if (!ui.button_pressed()) return false; // Return if the button isn't pressed
    LCD_MESSAGE_MAX(MSG_G26_CANCELED);
    ui.quick_feedback();
    ui.wait_for_release();
    return true;
  }

#endif

void move_to(const_float_t rx, const_float_t ry, const_float_t z, const_float_t e_delta) {
  static float last_z = -999.99;

  const xy_pos_t dest = { rx, ry };

  const bool has_xy_component = dest != current_position, // Check if X or Y is involved in the movement.
             has_e_component = e_delta != 0.0;

  if (z != last_z) {
    last_z = z;
    destination.set(current_position.x, current_position.y, z, current_position.e);
    const feedRate_t fr_mm_s = planner.settings.max_feedrate_mm_s[Z_AXIS] * 0.5f; // Use half of the Z_AXIS max feed rate
    prepare_internal_move_to_destination(fr_mm_s);
  }

  // If X or Y in combination with E is involved do a 'normal' move.
  // If X or Y with no E is involved do a 'fast' move
  // Otherwise retract/recover/hop.
  destination = dest;
  destination.e += e_delta;
  const feedRate_t fr_mm_s = has_xy_component
    ? (has_e_component ? feedRate_t(G26_XY_FEEDRATE) : feedRate_t(G26_XY_FEEDRATE_TRAVEL))
    : planner.settings.max_feedrate_mm_s[E_AXIS] * 0.666f;
  prepare_internal_move_to_destination(fr_mm_s);
}

void move_to(const xyz_pos_t &where, const_float_t de) { move_to(where.x, where.y, where.z, de); }

typedef struct {
  float extrusion_multiplier  = EXTRUSION_MULTIPLIER,
        retraction_multiplier = G26_RETRACT_MULTIPLIER,
        layer_height          = MESH_TEST_LAYER_HEIGHT,
        prime_length          = PRIME_LENGTH;

  celsius_t bed_temp          = MESH_TEST_BED_TEMP,
            hotend_temp       = MESH_TEST_HOTEND_TEMP;

  float nozzle                = MESH_TEST_NOZZLE_SIZE,
        filament_diameter     = DEFAULT_NOMINAL_FILAMENT_DIA,
        ooze_amount;            // 'O' ... OOZE_AMOUNT

  bool continue_with_closest,   // 'C'
       keep_heaters_on;         // 'K'

  xy_pos_t xy_pos; // = { 0, 0 }

  int8_t prime_flag = 0;

  bool g26_retracted = false;  // Track the retracted state during G26 so mismatched
                               // retracts/recovers don't result in a bad state.

  void retract_filament(const xyz_pos_t &where) {
    if (!g26_retracted) { // Only retract if we are not already retracted!
      g26_retracted = true;
      move_to(where, -1.0f * retraction_multiplier);
    }
  }

  // TODO: Parameterize the Z lift with a define
  void retract_lift_move(const xyz_pos_t &s) {
    retract_filament(destination);
    move_to(current_position.x, current_position.y, current_position.z + 0.5f, 0.0f);  // Z lift to minimize scraping
    move_to(s.x, s.y, s.z + 0.5f, 0.0f);  // Get to the starting point with no extrusion while lifted
  }

  void recover_filament(const xyz_pos_t &where) {
    if (g26_retracted) { // Only un-retract if we are retracted.
      move_to(where, 1.2f * retraction_multiplier);
      g26_retracted = false;
    }
  }

  /**
   * print_line_from_here_to_there() takes two cartesian coordinates and draws a line from one
   * to the other. But there are really three sets of coordinates involved. The first coordinate
   * is the present location of the nozzle. We don't necessarily want to print from this location.
   * We first need to move the nozzle to the start of line segment where we want to print. Once
   * there, we can use the two coordinates supplied to draw the line.
   *
   * Note:  Although we assume the first set of coordinates is the start of the line and the second
   * set of coordinates is the end of the line, it does not always work out that way. This function
   * optimizes the movement to minimize the travel distance before it can start printing. This saves
   * a lot of time and eliminates a lot of nonsensical movement of the nozzle. However, it does
   * cause a lot of very little short retracement of th nozzle when it draws the very first line
   * segment of a 'circle'. The time this requires is very short and is easily saved by the other
   * cases where the optimization comes into play.
   */
  void print_line_from_here_to_there(const xyz_pos_t &s, const xyz_pos_t &e) {

    // Distances to the start / end of the line
    xy_float_t svec = current_position - s, evec = current_position - e;

    const float dist_start = HYPOT2(svec.x, svec.y),
                dist_end = HYPOT2(evec.x, evec.y),
                line_length = HYPOT(e.x - s.x, e.y - s.y);

    // If the end point of the line is closer to the nozzle, flip the direction,
    // moving from the end to the start. On very small lines the optimization isn't worth it.
    if (dist_end < dist_start && (INTERSECTION_CIRCLE_RADIUS) < ABS(line_length))
      return print_line_from_here_to_there(e, s);

    // Decide whether to retract & lift
    if (dist_start > 2.0) retract_lift_move(s);

    move_to(s, 0.0); // Get to the starting point with no extrusion / un-Z lift

    const float e_pos_delta = line_length * g26_e_axis_feedrate * extrusion_multiplier;

    recover_filament(destination);
    move_to(e, e_pos_delta);  // Get to the ending point with an appropriate amount of extrusion
  }

  void connect_neighbor_with_line(const xy_int8_t &p1, int8_t dx, int8_t dy) {
    xy_int8_t p2;
    p2.x = p1.x + dx;
    p2.y = p1.y + dy;

    if (p2.x < 0 || p2.x >= (GRID_MAX_POINTS_X)) return;
    if (p2.y < 0 || p2.y >= (GRID_MAX_POINTS_Y)) return;

    if (circle_flags.marked(p1.x, p1.y) && circle_flags.marked(p2.x, p2.y)) {
      xyz_pos_t s, e;
      s.x = bedlevel.get_mesh_x(p1.x) + (INTERSECTION_CIRCLE_RADIUS - (CROSSHAIRS_SIZE)) * dx;
      e.x = bedlevel.get_mesh_x(p2.x) - (INTERSECTION_CIRCLE_RADIUS - (CROSSHAIRS_SIZE)) * dx;
      s.y = bedlevel.get_mesh_y(p1.y) + (INTERSECTION_CIRCLE_RADIUS - (CROSSHAIRS_SIZE)) * dy;
      e.y = bedlevel.get_mesh_y(p2.y) - (INTERSECTION_CIRCLE_RADIUS - (CROSSHAIRS_SIZE)) * dy;
      s.z = e.z = layer_height;

      #if HAS_ENDSTOPS
        LIMIT(s.y, Y_MIN_POS + 1, Y_MAX_POS - 1);
        LIMIT(e.y, Y_MIN_POS + 1, Y_MAX_POS - 1);
        LIMIT(s.x, X_MIN_POS + 1, X_MAX_POS - 1);
        LIMIT(e.x, X_MIN_POS + 1, X_MAX_POS - 1);
      #endif

      if (position_is_reachable(s) && position_is_reachable(e))
        print_line_from_here_to_there(s, e);
    }
  }

  /**
   * Turn on the bed and nozzle heat and
   * wait for them to get up to temperature.
   */
  bool turn_on_heaters() {

    SERIAL_ECHOLNPGM("Waiting for heatup.");

    #if HAS_HEATED_BED

      if (bed_temp > 25) {
        LCD_MESSAGE_MAX(MSG_G26_HEATING_BED);
        ui.quick_feedback();
        TERN_(HAS_MARLINUI_MENU, ui.capture());
        thermalManager.setTargetBed(bed_temp);

        // Wait for the temperature to stabilize
        if (!thermalManager.wait_for_bed(true OPTARG(G26_CLICK_CAN_CANCEL, true)))
          return G26_ERR;
      }

    #else

      UNUSED(bed_temp);

    #endif // HAS_HEATED_BED

    // Start heating the active nozzle
    LCD_MESSAGE_MAX(MSG_G26_HEATING_NOZZLE);
    ui.quick_feedback();
    thermalManager.setTargetHotend(hotend_temp, active_extruder);

    // Wait for the temperature to stabilize
    if (!thermalManager.wait_for_hotend(active_extruder, true OPTARG(G26_CLICK_CAN_CANCEL, true)))
      return G26_ERR;

    ui.reset_status();
    ui.completion_feedback();

    return G26_OK;
  }

  /**
   * Prime the nozzle if needed. Return true on error.
   */
  bool prime_nozzle() {

    const feedRate_t fr_slow_e = planner.settings.max_feedrate_mm_s[E_AXIS] / 15.0f;
    #if HAS_MARLINUI_MENU && !HAS_TOUCH_BUTTONS // ui.button_pressed issue with touchscreen
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        float Total_Prime = 0.0;
      #endif

      if (prime_flag == -1) {  // The user wants to control how much filament gets purged
        ui.capture();
        LCD_MESSAGE_MAX(MSG_G26_MANUAL_PRIME);
        ui.chirp();

        destination = current_position;

        recover_filament(destination); // Make sure G26 doesn't think the filament is retracted().

        while (!ui.button_pressed()) {
          ui.chirp();
          destination.e += 0.25;
          #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
            Total_Prime += 0.25;
            if (Total_Prime >= EXTRUDE_MAXLENGTH) {
              ui.release();
              return G26_ERR;
            }
          #endif
          prepare_internal_move_to_destination(fr_slow_e);
          destination = current_position;
          planner.synchronize();    // Without this synchronize, the purge is more consistent,
                                    // but because the planner has a buffer, we won't be able
                                    // to stop as quickly. So we put up with the less smooth
                                    // action to give the user a more responsive 'Stop'.
        }

        ui.wait_for_release();

        LCD_MESSAGE_MAX(MSG_G26_PRIME_DONE);
        ui.quick_feedback();
        ui.release();
      }
      else
    #endif
    {
      LCD_MESSAGE_MAX(MSG_G26_FIXED_LENGTH);
      ui.quick_feedback();
      destination = current_position;
      destination.e += prime_length;
      prepare_internal_move_to_destination(fr_slow_e);
      destination.e -= prime_length;
      retract_filament(destination);
    }

    return G26_OK;
  }

  /**
   * Find the nearest point at which to print a circle
   */
  mesh_index_pair find_closest_circle_to_print(const xy_pos_t &pos) {

    mesh_index_pair out_point;
    out_point.pos = -1;

    #if ENABLED(UBL_HILBERT_CURVE)

      auto test_func = [](uint8_t i, uint8_t j, void *data) -> bool {
        if (!circle_flags.marked(i, j)) {
          mesh_index_pair *out_point = (mesh_index_pair*)data;
          out_point->pos.set(i, j);  // Save its data
          return true;
        }
        return false;
      };

      hilbert_curve::search_from_closest(pos, test_func, &out_point);

    #else

      float closest = 99999.99;

      GRID_LOOP(i, j) {
        if (!circle_flags.marked(i, j)) {
          // We found a circle that needs to be printed
          const xy_pos_t m = { bedlevel.get_mesh_x(i), bedlevel.get_mesh_y(j) };

          // Get the distance to this intersection
          float f = (pos - m).magnitude();

          // It is possible that we are being called with the values
          // to let us find the closest circle to the start position.
          // But if this is not the case, add a small weighting to the
          // distance calculation to help it choose a better place to continue.
          f += (xy_pos - m).magnitude() / 15.0f;

          // Add the specified amount of Random Noise to our search
          if (g26_random_deviation > 1.0) f += random(0.0, g26_random_deviation);

          if (f < closest) {
            closest = f;          // Found a closer un-printed location
            out_point.pos.set(i, j);  // Save its data
            out_point.distance = closest;
          }
        }
      }

    #endif

    circle_flags.mark(out_point); // Mark this location as done.
    return out_point;
  }

} g26_helper_t;

/**
 * G26: Mesh Validation Pattern generation.
 *
 * Used to interactively edit the mesh by placing the
 * nozzle in a problem area and doing a G29 P4 R command.
 *
 * Parameters:
 *
 *  B  Bed Temperature
 *  C  Continue from the Closest mesh point
 *  D  Disable leveling before starting
 *  F  Filament diameter
 *  H  Hotend Temperature
 *  K  Keep heaters on when completed
 *  L  Layer Height
 *  O  Ooze extrusion length
 *  P  Prime length
 *  Q  Retraction multiplier
 *  R  Repetitions (number of grid points)
 *  S  Nozzle Size (diameter) in mm
 *  T  Tool index to change to, if included
 *  U  Random deviation (50 if no value given)
 *  X  X position
 *  Y  Y position
 */
void GcodeSuite::G26() {
  SERIAL_ECHOLNPGM("G26 starting...");

  // Don't allow Mesh Validation without homing first,
  // or if the parameter parsing did not go OK, abort
  if (homing_needed_error()) return;

  #if HAS_TOOLCHANGE
    // Change the tool first, if specified
    if (parser.seenval('T')) tool_change(parser.value_int());
  #endif

  g26_helper_t g26;

  g26.ooze_amount           = parser.linearval('O', OOZE_AMOUNT);
  g26.continue_with_closest = parser.boolval('C');
  g26.keep_heaters_on       = parser.boolval('K');

  // Accept 'I' if temperature presets are defined
  #if HAS_PREHEAT
    const uint8_t preset_index = parser.seenval('I') ? _MIN(parser.value_byte(), PREHEAT_COUNT - 1) + 1 : 0;
  #endif

  #if HAS_HEATED_BED

    // Get a temperature from 'I' or 'B'
    celsius_t bedtemp = 0;

    // Use the 'I' index if temperature presets are defined
    #if HAS_PREHEAT
      if (preset_index) bedtemp = ui.material_preset[preset_index - 1].bed_temp;
    #endif

    // Look for 'B' Bed Temperature
    if (parser.seenval('B')) bedtemp = parser.value_celsius();

    if (bedtemp) {
      if (!WITHIN(bedtemp, 40, BED_MAX_TARGET)) {
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Specified bed temperature not plausible (40-", BED_MAX_TARGET, "C)."));
        return;
      }
      g26.bed_temp = bedtemp;
    }

  #endif // HAS_HEATED_BED

  if (parser.seenval('L')) {
    g26.layer_height = parser.value_linear_units();
    if (!WITHIN(g26.layer_height, 0.0, 2.0)) {
      SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Specified layer height not plausible."));
      return;
    }
  }

  if (parser.seen('Q')) {
    if (parser.has_value()) {
      g26.retraction_multiplier = parser.value_float();
      if (!WITHIN(g26.retraction_multiplier, 0.05, 15.0)) {
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Specified Retraction Multiplier not plausible."));
        return;
      }
    }
    else {
      SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Retraction Multiplier must be specified."));
      return;
    }
  }

  if (parser.seenval('S')) {
    g26.nozzle = parser.value_float();
    if (!WITHIN(g26.nozzle, 0.1, 2.0)) {
      SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Specified nozzle size not plausible."));
      return;
    }
  }

  if (parser.seen('P')) {
    if (!parser.has_value()) {
      #if HAS_MARLINUI_MENU
        g26.prime_flag = -1;
      #else
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Prime length must be specified when not using an LCD."));
        return;
      #endif
    }
    else {
      g26.prime_flag++;
      g26.prime_length = parser.value_linear_units();
      if (!WITHIN(g26.prime_length, 0.0, 25.0)) {
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Specified prime length not plausible."));
        return;
      }
    }
  }

  if (parser.seenval('F')) {
    g26.filament_diameter = parser.value_linear_units();
    if (!WITHIN(g26.filament_diameter, 1.0, 4.0)) {
      SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Specified filament size not plausible."));
      return;
    }
  }
  g26.extrusion_multiplier *= sq(1.75) / sq(g26.filament_diameter); // If we aren't using 1.75mm filament, we need to
                                                                    // scale up or down the length needed to get the
                                                                    // same volume of filament

  g26.extrusion_multiplier *= g26.filament_diameter * sq(g26.nozzle) / sq(0.3); // Scale up by nozzle size

  // Get a temperature from 'I' or 'H'
  celsius_t noztemp = 0;

  // Accept 'I' if temperature presets are defined
  #if HAS_PREHEAT
    if (preset_index) noztemp = ui.material_preset[preset_index - 1].hotend_temp;
  #endif

  // Look for 'H' Hotend Temperature
  if (parser.seenval('H')) noztemp = parser.value_celsius();

  // If any preset or temperature was specified
  if (noztemp) {
    if (!WITHIN(noztemp, 165, thermalManager.hotend_max_target(active_extruder))) {
      SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Specified nozzle temperature not plausible."));
      return;
    }
    g26.hotend_temp = noztemp;
  }

  // 'U' to Randomize and optionally set circle deviation
  if (parser.seen('U')) {
    randomSeed(millis());
    // This setting will persist for the next G26
    g26_random_deviation = parser.has_value() ? parser.value_float() : 50.0;
  }

  // Get repeat from 'R', otherwise do one full circuit
  grid_count_t g26_repeats;
  #if HAS_MARLINUI_MENU
    g26_repeats = parser.intval('R', GRID_MAX_POINTS + 1);
  #else
    if (parser.seen('R'))
      g26_repeats = parser.has_value() ? parser.value_int() : GRID_MAX_POINTS + 1;
    else {
      SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(R)epeat must be specified when not using an LCD."));
      return;
    }
  #endif
  if (g26_repeats < 1) {
    SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(R)epeat value not plausible; must be at least 1."));
    return;
  }

  // Set a position with 'X' and/or 'Y'. Default: current_position
  g26.xy_pos.set(parser.seenval('X') ? RAW_X_POSITION(parser.value_linear_units()) : current_position.x,
                 parser.seenval('Y') ? RAW_Y_POSITION(parser.value_linear_units()) : current_position.y);
  if (!position_is_reachable(g26.xy_pos)) {
    SERIAL_ECHOLNPGM(GCODE_ERR_MSG("Specified X,Y coordinate out of bounds."));
    return;
  }

  /**
   * Wait until all parameters are verified before altering the state!
   */
  set_bed_leveling_enabled(!parser.seen_test('D'));

  do_z_clearance(Z_CLEARANCE_BETWEEN_PROBES);

  #if DISABLED(NO_VOLUMETRICS)
    bool volumetric_was_enabled = parser.volumetric_enabled;
    parser.volumetric_enabled = false;
    planner.calculate_volumetric_multipliers();
  #endif

  if (g26.turn_on_heaters() != G26_OK) goto LEAVE;

  current_position.e = 0.0;
  sync_plan_position_e();

  if (g26.prime_flag && g26.prime_nozzle() != G26_OK) goto LEAVE;

  /**
   *  Bed is preheated
   *
   *  Nozzle is at temperature
   *
   *  Filament is primed!
   *
   *  It's  "Show Time" !!!
   */

  circle_flags.reset();

  // Move nozzle to the specified height for the first layer
  destination = current_position;
  destination.z = g26.layer_height;
  move_to(destination, 0.0);
  move_to(destination, g26.ooze_amount);

  TERN_(HAS_MARLINUI_MENU, ui.capture());

  #if DISABLED(ARC_SUPPORT)

    /**
     * Pre-generate radius offset values at 30 degree intervals to reduce CPU load.
     */
    #define A_INT 30
    #define _ANGS (360 / A_INT)
    #define A_CNT (_ANGS / 2)
    #define _IND(A) ((A + _ANGS * 8) % _ANGS)
    #define _COS(A) (trig_table[_IND(A) % A_CNT] * (_IND(A) >= A_CNT ? -1 : 1))
    #define _SIN(A) (-_COS((A + A_CNT / 2) % _ANGS))
    #if A_CNT & 1
      #error "A_CNT must be a positive value. Please change A_INT."
    #endif
    float trig_table[A_CNT];
    for (uint8_t i = 0; i < A_CNT; ++i)
      trig_table[i] = INTERSECTION_CIRCLE_RADIUS * cos(RADIANS(i * A_INT));

  #endif // !ARC_SUPPORT

  mesh_index_pair location;
  TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(location.pos, ExtUI::G26_START));
  do {
    // Find the nearest confluence
    location = g26.find_closest_circle_to_print(g26.continue_with_closest ? xy_pos_t(current_position) : g26.xy_pos);

    if (location.valid()) {
      TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(location.pos, ExtUI::G26_POINT_START));
      const xy_pos_t circle = { bedlevel.get_mesh_x(location.pos.a), bedlevel.get_mesh_y(location.pos.b) };

      // If this mesh location is outside the printable radius, skip it.
      if (!position_is_reachable(circle)) continue;

      // Determine where to start and end the circle,
      // which is always drawn counter-clockwise.
      const xy_int8_t st = location;
      const bool f = st.y == 0,
                 r = st.x >= (GRID_MAX_POINTS_X) - 1,
                 b = st.y >= (GRID_MAX_POINTS_Y) - 1;

      #if ENABLED(ARC_SUPPORT)

        #define ARC_LENGTH(quarters)  (INTERSECTION_CIRCLE_RADIUS * M_PI * (quarters) / 2)
        #define INTERSECTION_CIRCLE_DIAM  ((INTERSECTION_CIRCLE_RADIUS) * 2)

        xy_float_t e = { circle.x + INTERSECTION_CIRCLE_RADIUS, circle.y };
        xyz_float_t s = e;

        // Figure out where to start and end the arc - we always print counterclockwise
        float arc_length = ARC_LENGTH(4);
        if (st.x == 0) {                             // left edge
          if (!f) { s.x = circle.x; s.y -= INTERSECTION_CIRCLE_RADIUS; }
          if (!b) { e.x = circle.x; e.y += INTERSECTION_CIRCLE_RADIUS; }
          arc_length = (f || b) ? ARC_LENGTH(1) : ARC_LENGTH(2);
        }
        else if (r) {                               // right edge
          if (b) s.set(circle.x - (INTERSECTION_CIRCLE_RADIUS), circle.y);
          else   s.set(circle.x, circle.y + INTERSECTION_CIRCLE_RADIUS);
          if (f) e.set(circle.x - (INTERSECTION_CIRCLE_RADIUS), circle.y);
          else   e.set(circle.x, circle.y - (INTERSECTION_CIRCLE_RADIUS));
          arc_length = (f || b) ? ARC_LENGTH(1) : ARC_LENGTH(2);
        }
        else if (f) {
          e.x -= INTERSECTION_CIRCLE_DIAM;
          arc_length = ARC_LENGTH(2);
        }
        else if (b) {
          s.x -= INTERSECTION_CIRCLE_DIAM;
          arc_length = ARC_LENGTH(2);
        }

        const ab_float_t arc_offset = circle - s;
        const xy_float_t dist = current_position - s;   // Distance from the start of the actual circle
        const float dist_start = HYPOT2(dist.x, dist.y);
        const xyze_pos_t endpoint = {
          e.x, e.y, g26.layer_height,
          current_position.e + (arc_length * g26_e_axis_feedrate * g26.extrusion_multiplier)
        };

        if (dist_start > 2.0) {
          s.z = g26.layer_height + 0.5f;
          g26.retract_lift_move(s);
        }

        s.z = g26.layer_height;
        move_to(s, 0.0);  // Get to the starting point with no extrusion / un-Z lift

        g26.recover_filament(destination);

        { REMEMBER(fr, feedrate_mm_s, PLANNER_XY_FEEDRATE_MM_S * 0.1f);
          plan_arc(endpoint, arc_offset, false, 0);  // Draw a counter-clockwise arc
          destination = current_position;
        }

        if (TERN0(HAS_MARLINUI_MENU, user_canceled())) goto LEAVE; // Check if the user wants to stop the Mesh Validation

      #else // !ARC_SUPPORT

        int8_t start_ind = -2, end_ind = 9; // Assume a full circle (from 5:00 to 5:00)
        if (st.x == 0) {                    // Left edge? Just right half.
          start_ind = f ? 0 : -3;           //  03:00 to 12:00 for front-left
          end_ind = b ? 0 : 2;              //  06:00 to 03:00 for back-left
        }
        else if (r) {                       // Right edge? Just left half.
          start_ind = b ? 6 : 3;            //  12:00 to 09:00 for front-right
          end_ind = f ? 5 : 8;              //  09:00 to 06:00 for back-right
        }
        else if (f) {                       // Front edge? Just back half.
          start_ind = 0;                    //  03:00
          end_ind = 5;                      //  09:00
        }
        else if (b) {                       // Back edge? Just front half.
          start_ind = 6;                    //  09:00
          end_ind = 11;                     //  03:00
        }

        for (int8_t ind = start_ind; ind <= end_ind; ind++) {

          if (TERN0(HAS_MARLINUI_MENU, user_canceled())) goto LEAVE; // Check if the user wants to stop the Mesh Validation

          xyz_float_t p = { circle.x + _COS(ind    ), circle.y + _SIN(ind    ), g26.layer_height },
                      q = { circle.x + _COS(ind + 1), circle.y + _SIN(ind + 1), g26.layer_height };

          #if IS_KINEMATIC
            // Check to make sure this segment is entirely on the bed, skip if not.
            if (!position_is_reachable(p) || !position_is_reachable(q)) continue;
          #elif HAS_ENDSTOPS
            LIMIT(p.x, X_MIN_POS + 1, X_MAX_POS - 1); // Prevent hitting the endstops
            LIMIT(p.y, Y_MIN_POS + 1, Y_MAX_POS - 1);
            LIMIT(q.x, X_MIN_POS + 1, X_MAX_POS - 1);
            LIMIT(q.y, Y_MIN_POS + 1, Y_MAX_POS - 1);
          #endif

          g26.print_line_from_here_to_there(p, q);
          SERIAL_FLUSH();   // Prevent host M105 buffer overrun.
        }

      #endif // !ARC_SUPPORT

      g26.connect_neighbor_with_line(location.pos, -1,  0);
      g26.connect_neighbor_with_line(location.pos,  1,  0);
      g26.connect_neighbor_with_line(location.pos,  0, -1);
      g26.connect_neighbor_with_line(location.pos,  0,  1);
      planner.synchronize();
      TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(location.pos, ExtUI::G26_POINT_FINISH));
      if (TERN0(HAS_MARLINUI_MENU, user_canceled())) goto LEAVE;
    }

    SERIAL_FLUSH(); // Prevent host M105 buffer overrun.

  } while (--g26_repeats && location.valid());

  LEAVE:
  LCD_MESSAGE_MIN(MSG_G26_LEAVING);
  TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(location, ExtUI::G26_FINISH));

  g26.retract_filament(destination);
  destination.z = Z_CLEARANCE_BETWEEN_PROBES;
  move_to(destination, 0);                                   // Raise the nozzle

  #if DISABLED(NO_VOLUMETRICS)
    parser.volumetric_enabled = volumetric_was_enabled;
    planner.calculate_volumetric_multipliers();
  #endif

  TERN_(HAS_MARLINUI_MENU, ui.release()); // Give back control of the LCD

  if (!g26.keep_heaters_on) {
    TERN_(HAS_HEATED_BED, thermalManager.setTargetBed(0));
    thermalManager.setTargetHotend(active_extruder, 0);
  }
}

#endif // G26_MESH_VALIDATION
