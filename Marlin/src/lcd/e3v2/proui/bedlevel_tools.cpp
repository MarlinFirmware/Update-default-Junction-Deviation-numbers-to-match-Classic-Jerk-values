/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Bed Level Tools for Pro UI
 *
 * Based on the original work of: Henri-J-Norden
 * https://github.com/Jyers/Marlin/pull/126
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

#include "../../../inc/MarlinConfigPre.h"

#if ALL(DWIN_LCD_PROUI, HAS_LEVELING)

#include "../../marlinui.h"
#include "../../../core/types.h"
#include "../../../feature/bedlevel/bedlevel.h"
#include "../../../module/probe.h"
#include "../../../gcode/gcode.h"
#include "../../../module/planner.h"
#include "../../../gcode/queue.h"
#include "../../../libs/least_squares_fit.h"
#include "../../../libs/vector_3.h"

#include "dwin.h"
#include "dwinui.h"
#include "dwin_popup.h"
#include "bedlevel_tools.h"

BedLevelTools bedLevelTools;

#if ENABLED(USE_GRID_MESHVIEWER)
  bool BedLevelTools::viewer_print_value = false;
#endif
bool BedLevelTools::goto_mesh_value = false;
uint8_t BedLevelTools::mesh_x = 0;
uint8_t BedLevelTools::mesh_y = 0;
uint8_t BedLevelTools::tilt_grid = 1;

bool drawing_mesh = false;

#if ENABLED(AUTO_BED_LEVELING_UBL)

  void BedLevelTools::manualValueUpdate(const uint8_t mesh_x, const uint8_t mesh_y, bool undefined/*=false*/) {
    MString<MAX_CMD_SIZE> cmd;
    cmd.set(F("M421 I"), mesh_x, 'J', mesh_y, 'Z', p_float_t(current_position.z, 3));
    if (undefined) cmd += F(" N");
    gcode.process_subcommands_now(cmd);
    planner.synchronize();
  }

  bool BedLevelTools::createPlaneFromMesh() {
    struct linear_fit_data lsf_results;
    incremental_LSF_reset(&lsf_results);
    GRID_LOOP(x, y) {
      const float z = bedlevel.z_values[x][y];
      if (!isnan(z)) {
        xy_pos_t rpos = { bedlevel.get_mesh_x(x), bedlevel.get_mesh_y(y) };
        incremental_LSF(&lsf_results, rpos, z);
      }
    }

    if (finish_incremental_LSF(&lsf_results)) {
      SERIAL_ECHOPGM("Could not complete LSF!");
      return true;
    }

    bedlevel.set_all_mesh_points_to_value(0);

    matrix_3x3 rotation = matrix_3x3::create_look_at(vector_3(lsf_results.A, lsf_results.B, 1));
    GRID_LOOP(i, j) {
      float mx = bedlevel.get_mesh_x(i), my = bedlevel.get_mesh_y(j), mz = bedlevel.z_values[i][j];

      if (DEBUGGING(LEVELING)) {
        DEBUG_ECHOLN(F("before rotation = ["), p_float_t(mx, 7), C(','), p_float_t(my, 7), C(','), p_float_t(mz, 7), F("]   ---> "));
        DEBUG_DELAY(20);
      }

      rotation.apply_rotation_xyz(mx, my, mz);

      if (DEBUGGING(LEVELING)) {
        DEBUG_ECHOLN(F("after rotation = ["), p_float_t(mx, 7), C(','), p_float_t(my, 7), C(','), p_float_t(mz, 7), F("]   ---> "));
        DEBUG_DELAY(20);
      }

      bedlevel.z_values[i][j] = mz - lsf_results.D;
    }
    return false;
  }

#else

  void BedLevelTools::manualValueUpdate(const uint8_t mesh_x, const uint8_t mesh_y) {
    gcode.process_subcommands_now(
      TS(F("G29 I"), mesh_x, 'J', mesh_y, 'Z', p_float_t(current_position.z, 3))
    );
    planner.synchronize();
  }

#endif

void BedLevelTools::manualMove(const uint8_t mesh_x, const uint8_t mesh_y, bool zmove/*=false*/) {
  gcode.process_subcommands_now(F("G28O"));
  if (!zmove) {
    dwinShowPopup(ICON_BLTouch, F("Moving to Point"), F("Please wait until done."));
    hmiSaveProcessID(ID_NothingToDo);
    gcode.process_subcommands_now(TS(F("G0 F300 Z"), p_float_t(Z_CLEARANCE_BETWEEN_PROBES, 3)));
    gcode.process_subcommands_now(TS(F("G42 F4000 I"), mesh_x, F(" J"), mesh_y));
  }
  planner.synchronize();
  current_position.z = goto_mesh_value ? bedlevel.z_values[mesh_x][mesh_y] : Z_CLEARANCE_BETWEEN_PROBES;
  planner.buffer_line(current_position, homing_feedrate(Z_AXIS), active_extruder);
  planner.synchronize();
  if (!zmove) hmiReturnScreen();
}

// Move / Probe methods.
void BedLevelTools::moveToXYZ() {
  goto_mesh_value = true;
  manualMove(mesh_x, mesh_y, false);
}
void BedLevelTools::moveToXY() {
  goto_mesh_value = false;
  manualMove(mesh_x, mesh_y, false);
}
void BedLevelTools::moveToZ() {
  goto_mesh_value = true;
  manualMove(mesh_x, mesh_y, true);
}
void BedLevelTools::probeXY() {
  gcode.process_subcommands_now(
    MString<MAX_CMD_SIZE>(
      F("G28O\nG0Z"), uint16_t(Z_CLEARANCE_DEPLOY_PROBE),
      F("\nG30X"), p_float_t(bedlevel.get_mesh_x(mesh_x), 2),
      F("Y"), p_float_t(bedlevel.get_mesh_y(mesh_y), 2)
    )
  );
}

void BedLevelTools::meshReset() {
  ZERO(bedlevel.z_values);
  TERN_(AUTO_BED_LEVELING_BILINEAR, bedlevel.refresh_bed_level());
}

// Accessors
float BedLevelTools::getMaxValue() {
  float max = -(__FLT_MAX__);
  GRID_LOOP(x, y) { const float z = bedlevel.z_values[x][y]; if (!isnan(z)) NOLESS(max, z); }
  return max;
}

float BedLevelTools::getMinValue() {
  float min = __FLT_MAX__;
  GRID_LOOP(x, y) { const float z = bedlevel.z_values[x][y]; if (!isnan(z)) NOMORE(min, z); }
  return min;
}

// Return 'true' if mesh is good and within LCD limits
bool BedLevelTools::meshValidate() {
  GRID_LOOP(x, y) {
    const float z = bedlevel.z_values[x][y];
    if (isnan(z) || !WITHIN(z, Z_OFFSET_MIN, Z_OFFSET_MAX)) return false;
  }
  return true;
}

#if ENABLED(USE_GRID_MESHVIEWER)

  constexpr uint8_t meshfont = TERN(TJC_DISPLAY, font8x16, font6x12);

  void BedLevelTools::drawBedMesh(int16_t selected/*=-1*/, uint8_t gridline_width/*=1*/, uint16_t padding_x/*=8*/, uint16_t padding_y_top/*=(40 + 53 - 7)*/) {
    drawing_mesh = true;
    const uint16_t total_width_px = DWIN_WIDTH - padding_x - padding_x,
                   cell_width_px  = total_width_px / (GRID_MAX_POINTS_X),
                   cell_height_px = total_width_px / (GRID_MAX_POINTS_Y);
    const float v_max = abs(getMaxValue()), v_min = abs(getMinValue()), rmax = _MAX(v_min, v_max);

    // Clear background from previous selection and select new square
    dwinDrawRectangle(1, COLOR_BG_BLACK, _MAX(0, padding_x - gridline_width), _MAX(0, padding_y_top - gridline_width), padding_x + total_width_px, padding_y_top + total_width_px);
    if (selected >= 0) {
      const auto selected_y = selected / (GRID_MAX_POINTS_X);
      const auto selected_x = selected - (GRID_MAX_POINTS_X) * selected_y;
      const auto start_y_px = padding_y_top + selected_y * cell_height_px;
      const auto start_x_px = padding_x + selected_x * cell_width_px;
      dwinDrawRectangle(1, COLOR_WHITE, _MAX(0, start_x_px - gridline_width), _MAX(0, start_y_px - gridline_width), start_x_px + cell_width_px, start_y_px + cell_height_px);
    }

    // Draw value square grid
    GRID_LOOP(x, y) {
      const auto start_x_px = padding_x + x * cell_width_px;
      const auto end_x_px   = start_x_px + cell_width_px - 1 - gridline_width;
      const auto start_y_px = padding_y_top + ((GRID_MAX_POINTS_Y) - y - 1) * cell_height_px;
      const auto end_y_px   = start_y_px + cell_height_px - 1 - gridline_width;
      const float z = bedlevel.z_values[x][y];
      const uint16_t color = isnan(z) ? COLOR_GREY : (   // Gray if undefined
        (z < 0 ? uint16_t(round(0x1F * -z / rmax)) << 11 // Red for negative mesh point
               : uint16_t(round(0x3F *  z / rmax)) << 5) // Green for positive mesh point
               | _MIN(0x1F, (uint8_t(abs(z) * 0.4)))     // + Blue stepping for every mm
      );

      dwinDrawRectangle(1, color, start_x_px, start_y_px, end_x_px, end_y_px);

      safe_delay(10);
      LCD_SERIAL.flushTX();

      // Draw value text on
      if (!viewer_print_value) continue;

      const uint8_t fs = DWINUI::fontWidth(meshfont);
      const int8_t offset_y = cell_height_px / 2 - fs;
      if (isnan(z)) { // undefined
        dwinDrawString(false, meshfont, COLOR_WHITE, COLOR_BG_BLUE, start_x_px + cell_width_px / 2 - 5, start_y_px + offset_y, F("X"));
      }
      else {          // has value
        MString<12> msg;
        constexpr bool is_wide = (GRID_MAX_POINTS_X) >= TERN(TJC_DISPLAY, 8, 10);
        if (is_wide)
          msg.setf(F("%02i"), uint16_t(z * 100) % 100);
        else
          msg.set(p_float_t(abs(z), 2));
        const int8_t offset_x = cell_width_px / 2 - (fs / 2) * msg.length() - 2;
        if (is_wide)
          dwinDrawString(false, meshfont, COLOR_WHITE, COLOR_BG_BLUE, start_x_px - 2 + offset_x, start_y_px + offset_y, F("."));
        dwinDrawString(false, meshfont, COLOR_WHITE, COLOR_BG_BLUE, start_x_px + 1 + offset_x, start_y_px + offset_y, msg);
      }

      safe_delay(10);
      LCD_SERIAL.flushTX();

    } // GRID_LOOP
  }

  void BedLevelTools::setMeshViewerStatus() { // TODO: draw gradient with values as a legend instead
    float v_max = abs(getMaxValue()), v_min = abs(getMinValue()), rmax = _MAX(v_min, v_max), rmin = _MIN(v_min, v_max);
    if (rmax > 3e+10f) rmax = 0.0000001f;
    if (rmin > 3e+10f) rmin = 0.0000001f;
    ui.set_status(&MString<47>(F("Red "),  p_float_t(-rmax, 3), F("..0.."), p_float_t(rmin, 3), F(" Green")));
    drawing_mesh = false;
  }

#endif // USE_GRID_MESHVIEWER

#endif // DWIN_LCD_PROUI && HAS_LEVELING
