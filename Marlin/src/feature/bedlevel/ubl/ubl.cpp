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

unified_bed_leveling bedlevel;

#include "../../../MarlinCore.h"
#include "../../../gcode/gcode.h"

#include "../../../module/settings.h"
#include "../../../module/planner.h"
#include "../../../module/motion.h"
#include "../../../module/probe.h"
#include "../../../module/temperature.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../../../lcd/extui/ui_api.h"
#endif

#include "math.h"

void unified_bed_leveling::echo_name() { SERIAL_ECHOPGM("Unified Bed Leveling"); }

void unified_bed_leveling::report_current_mesh() {
  if (!leveling_is_valid()) return;
  SERIAL_ECHO_MSG("  G29 I999");
  GRID_LOOP(x, y)
    if (!isnan(z_values[x][y])) {
      SERIAL_ECHO_START();
      SERIAL_ECHOLN(F("  M421 I"), x, F(" J"), y, FPSTR(SP_Z_STR), p_float_t(z_values[x][y], 4));
      serial_delay(75); // Prevent Printrun from exploding
    }
}

void unified_bed_leveling::report_state() {
  echo_name();
  serial_ternary(F(" System v" UBL_VERSION " "), planner.leveling_active, nullptr, F("in"), F("active\n"));
  serial_delay(50);
}

int8_t unified_bed_leveling::storage_slot;

bed_mesh_t unified_bed_leveling::z_values;

#if DISABLED(PROUI_MESH_EDIT)
  #define _GRIDPOS(A,N) (MESH_MIN_##A + N * (MESH_##A##_DIST))

  const float
  unified_bed_leveling::_mesh_index_to_xpos[GRID_MAX_POINTS_X] PROGMEM = ARRAY_N(GRID_MAX_POINTS_X,
    _GRIDPOS(X,  0), _GRIDPOS(X,  1), _GRIDPOS(X,  2), _GRIDPOS(X,  3),
    _GRIDPOS(X,  4), _GRIDPOS(X,  5), _GRIDPOS(X,  6), _GRIDPOS(X,  7),
    _GRIDPOS(X,  8), _GRIDPOS(X,  9), _GRIDPOS(X, 10), _GRIDPOS(X, 11),
    _GRIDPOS(X, 12), _GRIDPOS(X, 13), _GRIDPOS(X, 14), _GRIDPOS(X, 15)
  ),
  unified_bed_leveling::_mesh_index_to_ypos[GRID_MAX_POINTS_Y] PROGMEM = ARRAY_N(GRID_MAX_POINTS_Y,
    _GRIDPOS(Y,  0), _GRIDPOS(Y,  1), _GRIDPOS(Y,  2), _GRIDPOS(Y,  3),
    _GRIDPOS(Y,  4), _GRIDPOS(Y,  5), _GRIDPOS(Y,  6), _GRIDPOS(Y,  7),
    _GRIDPOS(Y,  8), _GRIDPOS(Y,  9), _GRIDPOS(Y, 10), _GRIDPOS(Y, 11),
    _GRIDPOS(Y, 12), _GRIDPOS(Y, 13), _GRIDPOS(Y, 14), _GRIDPOS(Y, 15)
  );
#endif

volatile int16_t unified_bed_leveling::encoder_diff;

unified_bed_leveling::unified_bed_leveling() { reset(); }

void unified_bed_leveling::reset() {
  const bool was_enabled = planner.leveling_active;
  set_bed_leveling_enabled(false);
  storage_slot = -1;
  ZERO(z_values);
  #if ENABLED(EXTENSIBLE_UI)
    GRID_LOOP(x, y) ExtUI::onMeshUpdate(x, y, 0);
  #endif
  if (was_enabled) report_current_position();
}

void unified_bed_leveling::invalidate() {
  set_bed_leveling_enabled(false);
  set_all_mesh_points_to_value(NAN);
}

void unified_bed_leveling::set_all_mesh_points_to_value(const_float_t value) {
  GRID_LOOP(x, y) {
    z_values[x][y] = value;
    TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, value));
  }
}

#if ENABLED(OPTIMIZED_MESH_STORAGE)

  constexpr float mesh_store_scaling = 1000;
  constexpr int16_t Z_STEPS_NAN = INT16_MAX;

  void unified_bed_leveling::set_store_from_mesh(const bed_mesh_t &in_values, mesh_store_t &stored_values) {
    auto z_to_store = [](const_float_t z) {
      if (isnan(z)) return Z_STEPS_NAN;
      const int32_t z_scaled = TRUNC(z * mesh_store_scaling);
      if (z_scaled == Z_STEPS_NAN || !WITHIN(z_scaled, INT16_MIN, INT16_MAX))
        return Z_STEPS_NAN; // If Z is out of range, return our custom 'NaN'
      return int16_t(z_scaled);
    };
    GRID_LOOP(x, y) stored_values[x][y] = z_to_store(in_values[x][y]);
  }

  void unified_bed_leveling::set_mesh_from_store(const mesh_store_t &stored_values, bed_mesh_t &out_values) {
    auto store_to_z = [](const int16_t z_scaled) {
      return z_scaled == Z_STEPS_NAN ? NAN : z_scaled / mesh_store_scaling;
    };
    GRID_LOOP(x, y) out_values[x][y] = store_to_z(stored_values[x][y]);
  }

#endif // OPTIMIZED_MESH_STORAGE

static void serial_echo_xy(const uint8_t sp, const int16_t x, const int16_t y) {
  SERIAL_ECHO_SP(sp);
  SERIAL_CHAR('(');
  if (x < 100) { SERIAL_CHAR(' '); if (x < 10) SERIAL_CHAR(' '); }
  SERIAL_ECHO(x);
  SERIAL_CHAR(',');
  if (y < 100) { SERIAL_CHAR(' '); if (y < 10) SERIAL_CHAR(' '); }
  SERIAL_ECHO(y);
  SERIAL_CHAR(')');
  serial_delay(5);
}

static void serial_echo_column_labels(const uint8_t sp) {
  SERIAL_ECHO_SP(7);
  for (uint8_t i = 0; i < GRID_MAX_POINTS_X; ++i) {
    if (i < 10) SERIAL_CHAR(' ');
    SERIAL_ECHO(i);
    SERIAL_ECHO_SP(sp);
  }
  serial_delay(10);
}

/**
 * Produce one of these mesh maps:
 *   0: Human-readable
 *   1: CSV format for spreadsheet import
 *   2: TODO: Display on Graphical LCD
 *   4: Compact Human-Readable
 */
void unified_bed_leveling::display_map(const uint8_t map_type) {
  const bool was = gcode.set_autoreport_paused(true);

  constexpr uint8_t eachsp = 1 + 6 + 1,    // [-3.567]
                    twixt = eachsp * (GRID_MAX_POINTS_X) - 9 * 2; // Leading 4sp, Coordinates 9sp each

  const bool human = !(map_type & 0x3), csv = map_type == 1, lcd = map_type == 2, comp = map_type & 0x4;

  SERIAL_ECHOPGM("\nBed Topography Report");
  if (human) {
    SERIAL_ECHOLNPGM(":\n");
    serial_echo_xy(4, MESH_MIN_X, MESH_MAX_Y);
    serial_echo_xy(twixt, MESH_MAX_X, MESH_MAX_Y);
    SERIAL_EOL();
    serial_echo_column_labels(eachsp - 2);
  }
  else
    SERIAL_ECHOPGM(" for ", csv ? F("CSV:\n") : F("LCD:\n"));

  // Add XY probe offset from extruder because probe.probe_at_point() subtracts them when
  // moving to the XY position to be measured. This ensures better agreement between
  // the current Z position after G28 and the mesh values.
  const xy_int8_t curr = closest_indexes(xy_pos_t(current_position) + probe.offset_xy);

  if (!lcd) SERIAL_EOL();
  for (int8_t j = (GRID_MAX_POINTS_Y) - 1; j >= 0; j--) {

    // Row Label (J index)
    if (human) {
      if (j < 10) SERIAL_CHAR(' ');
      SERIAL_ECHO(j);
      SERIAL_ECHOPGM(" |");
    }

    // Row Values (I indexes)
    for (uint8_t i = 0; i < GRID_MAX_POINTS_X; ++i) {

      // Opening Brace or Space
      const bool is_current = i == curr.x && j == curr.y;
      if (human) SERIAL_CHAR(is_current ? '[' : ' ');

      // Z Value at current I, J
      const float f = z_values[i][j];
      if (lcd) {
        // TODO: Display on Graphical LCD
        TERN_(DWIN_LCD_PROUI, dwinMeshViewer());
      }
      else if (isnan(f))
        SERIAL_ECHO(human ? F("  .   ") : F("NAN"));
      else if (human || csv) {
        if (human && f >= 0) SERIAL_CHAR(f > 0 ? '+' : ' ');  // Display sign also for positive numbers (' ' for 0)
        SERIAL_ECHO(p_float_t(f, 3));                         // Positive: 5 digits, Negative: 6 digits
      }
      if (csv && i < (GRID_MAX_POINTS_X) - 1) SERIAL_CHAR('\t');

      // Closing Brace or Space
      if (human) SERIAL_CHAR(is_current ? ']' : ' ');

      SERIAL_FLUSHTX();
      idle_no_sleep();
    }
    if (!lcd) SERIAL_EOL();

    // A blank line between rows (unless compact)
    if (j && human && !comp) SERIAL_ECHOLNPGM("   |");
  }

  if (human) {
    serial_echo_column_labels(eachsp - 2);
    SERIAL_EOL();
    serial_echo_xy(4, MESH_MIN_X, MESH_MIN_Y);
    serial_echo_xy(twixt, MESH_MAX_X, MESH_MIN_Y);
    SERIAL_EOL();
    SERIAL_EOL();
  }

  gcode.set_autoreport_paused(was);
}

bool unified_bed_leveling::sanity_check() {
  uint8_t error_flag = 0;

  if (settings.calc_num_meshes() < 1) {
    SERIAL_ECHOLNPGM("?Mesh too big for EEPROM.");
    error_flag++;
  }

  return !!error_flag;
}

#if ENABLED(UBL_MESH_WIZARD)

  /**
   * M1004: UBL Mesh Wizard - One-click mesh creation with or without a probe
   */
  void GcodeSuite::M1004() {

    #define ALIGN_GCODE TERN(Z_STEPPER_AUTO_ALIGN, "G34\n", "")
    #define PROBE_GCODE TERN(HAS_BED_PROBE, "G29P1\nG29P3", "G29P4R")

    #if HAS_HOTEND
      if (parser.seenval('H')) {                          // Handle H# parameter to set Hotend temp
        const celsius_t hotend_temp = parser.value_int(); // Marlin never sends itself F or K, always C
        thermalManager.setTargetHotend(hotend_temp, 0);
        thermalManager.wait_for_hotend(false);
      }
    #endif

    #if HAS_HEATED_BED
      if (parser.seenval('B')) {                        // Handle B# parameter to set Bed temp
        const celsius_t bed_temp = parser.value_int();  // Marlin never sends itself F or K, always C
        thermalManager.setTargetBed(bed_temp);
        thermalManager.wait_for_bed(false);
      }
    #endif

    process_subcommands_now(FPSTR(G28_STR));      // Home
    process_subcommands_now(F(ALIGN_GCODE         // Align multi z axis if available
                              PROBE_GCODE "\n"    // Build mesh with available hardware
                              "G29P3\nG29P3"));   // Ensure mesh is complete by running smart fill twice

    if (parser.seenval('S')) {
      char umw_gcode[32];
      sprintf_P(umw_gcode, PSTR("G29S%i"), parser.value_int());
      queue.inject(umw_gcode);
    }

    process_subcommands_now(F("G29A\nG29F10\n"    // Set UBL Active & Fade 10
                              "M140S0\nM104S0\n"  // Turn off heaters
                              "M500"));           // Store settings
  }

#endif // UBL_MESH_WIZARD

#endif // AUTO_BED_LEVELING_UBL
