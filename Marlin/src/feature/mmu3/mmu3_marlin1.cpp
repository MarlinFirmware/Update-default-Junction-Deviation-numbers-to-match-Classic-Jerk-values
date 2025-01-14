/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * mmu2_marlin1.cpp
 * MK3 / Marlin1 implementation of support routines for the MMU3
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

#include "../../MarlinCore.h"
#include "../../module/stepper.h"
#include "../../module/planner.h"
#include "../../module/temperature.h"

#include "../pause.h"
#include "../../libs/nozzle.h"
#include "mmu3_marlin.h"

namespace MMU3 {

  static void planner_line_to_current_position(float feedRate_mm_s) {
    line_to_current_position(feedRate_mm_s);
  }

  static void planner_line_to_current_position_sync(float feedRate_mm_s) {
    planner_line_to_current_position(feedRate_mm_s);
    planner_synchronize();
  }

  void extruder_move(const_float_t delta, const_float_t feedRate_mm_s, const bool sync/*=true*/) {
    current_position.e += delta / planner.e_factor[active_extruder];
    planner_line_to_current_position(feedRate_mm_s);
    if (sync) planner.synchronize();
  }

  float move_raise_z(const_float_t delta) {
    //return raise_z(delta);
    xyze_pos_t current_position_before = current_position;
    do_z_clearance_by(delta);
    return (current_position - current_position_before).z;
  }

  void planner_abort_queued_moves() {
    //planner_abort_hard();
    quickstop_stepper();

    // Unblock the planner. This should be safe in the
    // toolchange context. Currently we are mainly aborting
    // excess E-moves after detecting filament during toolchange.
    // If a MMU error is reported, the planner must be unblocked
    // as well so the extruder can be parked safely.
    //planner_aborted = false;
    // eoyilmaz: we don't need this part, the print is not aborted
  }

  void planner_synchronize() {
    planner.synchronize();
  }

  bool planner_any_moves() {
    return planner.has_blocks_queued();
  }

  float planner_get_machine_position_E_mm() {
    return current_position.e;
  }

  float stepper_get_machine_position_E_mm() {
    return planner.get_axis_position_mm(E_AXIS);
  }

  float planner_get_current_position_E() {
    return current_position.e;
  }

  void planner_set_current_position_E(float e) {
    current_position.e = e;
  }

  xyz_pos_t planner_current_position() {
    return xyz_pos_t(current_position);
  }

  void motion_do_blocking_move_to_xy(float rx, float ry, float feedRate_mm_s) {
    current_position[X_AXIS] = rx;
    current_position[Y_AXIS] = ry;
    planner_line_to_current_position_sync(feedRate_mm_s);
  }

  void motion_do_blocking_move_to_z(float z, float feedRate_mm_s) {
    current_position[Z_AXIS] = z;
    planner_line_to_current_position_sync(feedRate_mm_s);
  }

  void nozzle_park() {
    #if ANY(NOZZLE_CLEAN_FEATURE, NOZZLE_PARK_FEATURE)
      #if ALL(ADVANCED_PAUSE_FEATURE)
        xyz_pos_t park_point = NOZZLE_PARK_POINT;
        nozzle.park(0, park_point);
      #endif
    #endif
  }

  bool marlin_printingIsActive() { return printingIsActive(); }

  void marlin_manage_heater() { thermalManager.task(); }

  void marlin_manage_inactivity(const bool b) { idle(b); }

  void marlin_idle(bool b) {
    thermalManager.task();
    idle(b);
  }

  void marlin_refresh_print_state_in_ram() {
    // refresh_print_state_in_ram();
    // TODO: I don't see a comparable implementation in Marlin.
  }

  void marlin_clear_print_state_in_ram() {
    // clear_print_state_in_ram();
    // TODO: I don't see a comparable implementation in Marlin.
  }

  void marlin_stop_and_save_print_to_ram() {
    // stop_and_save_print_to_ram(0,0);
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      constexpr xyz_pos_t park_point = NOZZLE_PARK_POINT;
      pause_print(0, park_point);
    #endif
  }

  int16_t thermal_degTargetHotend() {
    return thermalManager.degTargetHotend(0);
  }

  int16_t thermal_degHotend() {
    return thermalManager.degHotend(0);
  }

  void thermal_setExtrudeMintemp(int16_t t) {
    thermalManager.extrude_min_temp = t;
  }

  void thermal_setTargetHotend(int16_t t) {
    thermalManager.setTargetHotend(t, 0);
  }

  void safe_delay_keep_alive(uint16_t t) {
    idle(true);
    safe_delay(t);
  }

  void Enable_E0() {
    stepper.enable_extruder(TERN_(HAS_EXTRUDERS, 0));
  }

  void Disable_E0() {
    stepper.disable_extruder(TERN_(HAS_EXTRUDERS, 0));
  }

  bool xy_are_trusted() {
    return axis_is_trusted(X_AXIS) && axis_is_trusted(Y_AXIS);
  }

} // MMU3

#endif // HAS_PRUSA_MMU3
