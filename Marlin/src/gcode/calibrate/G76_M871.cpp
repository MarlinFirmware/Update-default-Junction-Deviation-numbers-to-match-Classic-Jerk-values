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
 * G76_M871.cpp - Temperature calibration/compensation for z-probing
 */

#include "../../inc/MarlinConfig.h"

#if HAS_PTC

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/probe.h"
#include "../../feature/bedlevel/bedlevel.h"
#include "../../module/temperature.h"
#include "../../feature/probe_temp_comp.h"
#include "../../lcd/marlinui.h"

/**
 * G76: calibrate probe and/or bed temperature offsets
 *  Notes:
 *  - When calibrating probe, bed temperature is held constant.
 *    Compensation values are deltas to first probe measurement at probe temp. = 30°C.
 *  - When calibrating bed, probe temperature is held constant.
 *    Compensation values are deltas to first probe measurement at bed temp. = 60°C.
 *  - The hotend will not be heated at any time.
 *  - On my Průša MK3S clone I put a piece of paper between the probe and the hotend
 *    so the hotend fan would not cool my probe constantly. Alternatively you could just
 *    make sure the fan is not running while running the calibration process.
 *
 *  Probe calibration:
 *  - Moves probe to cooldown point.
 *  - Heats up bed to 100°C.
 *  - Moves probe to probing point (1mm above heatbed).
 *  - Waits until probe reaches target temperature (30°C).
 *  - Does a z-probing (=base value) and increases target temperature by 5°C.
 *  - Waits until probe reaches increased target temperature.
 *  - Does a z-probing (delta to base value will be a compensation value) and increases target temperature by 5°C.
 *  - Repeats last two steps until max. temperature reached or timeout (i.e. probe does not heat up any further).
 *  - Compensation values of higher temperatures will be extrapolated (using linear regression first).
 *    While this is not exact by any means it is still better than simply using the last compensation value.
 *
 *  Bed calibration:
 *  - Moves probe to cooldown point.
 *  - Heats up bed to 60°C.
 *  - Moves probe to probing point (1mm above heatbed).
 *  - Waits until probe reaches target temperature (30°C).
 *  - Does a z-probing (=base value) and increases bed temperature by 5°C.
 *  - Moves probe to cooldown point.
 *  - Waits until probe is below 30°C and bed has reached target temperature.
 *  - Moves probe to probing point and waits until it reaches target temperature (30°C).
 *  - Does a z-probing (delta to base value will be a compensation value) and increases bed temperature by 5°C.
 *  - Repeats last four points until max. bed temperature reached (110°C) or timeout.
 *  - Compensation values of higher temperatures will be extrapolated (using linear regression first).
 *    While this is not exact by any means it is still better than simply using the last compensation value.
 *
 *  G76 [B | P]
 *  - no flag - Both calibration procedures will be run.
 *  - `B` - Run bed temperature calibration.
 *  - `P` - Run probe temperature calibration.
 */

#if ALL(PTC_PROBE, PTC_BED)

  static void say_waiting_for()               { SERIAL_ECHOPGM("Waiting for "); }
  static void say_waiting_for_probe_heating() { say_waiting_for(); SERIAL_ECHOLNPGM("probe heating."); }
  static void say_successfully_calibrated()   { SERIAL_ECHOPGM("Successfully calibrated"); }
  static void say_failed_to_calibrate()       { SERIAL_ECHOPGM("!Failed to calibrate"); }

  void GcodeSuite::G76() {
    auto report_temps = [](millis_t &ntr, millis_t timeout=0) {
      idle_no_sleep();
      const millis_t ms = millis();
      if (ELAPSED(ms, ntr)) {
        ntr = ms + 1000;
        thermalManager.print_heater_states(active_extruder);
      }
      return (timeout && ELAPSED(ms, timeout));
    };

    auto wait_for_temps = [&](const celsius_t tb, const celsius_t tp, millis_t &ntr, const millis_t timeout=0) {
      say_waiting_for(); SERIAL_ECHOLNPGM("bed and probe temperature.");
      while (thermalManager.wholeDegBed() != tb || thermalManager.wholeDegProbe() > tp)
        if (report_temps(ntr, timeout)) return true;
      return false;
    };

    auto g76_probe = [](const TempSensorID sid, celsius_t &targ, const xy_pos_t &nozpos) {
      ptc.set_enabled(false);
      const float measured_z = probe.probe_at_point(nozpos, PROBE_PT_STOW, 0, false);  // verbose=0, probe_relative=false
      ptc.set_enabled(true);
      if (isnan(measured_z))
        SERIAL_ECHOLNPGM("!Received NAN. Aborting.");
      else {
        SERIAL_ECHOLNPGM("Measured: ", p_float_t(measured_z, 2));
        if (targ == ProbeTempComp::cali_info[sid].start_temp)
          ptc.prepare_new_calibration(measured_z);
        else
          ptc.push_back_new_measurement(sid, measured_z);
        targ += ProbeTempComp::cali_info[sid].temp_resolution;
      }
      return measured_z;
    };

    #if ENABLED(BLTOUCH)
      // Make sure any BLTouch error condition is cleared
      bltouch_command(BLTOUCH_RESET, BLTOUCH_RESET_DELAY);
      set_bltouch_deployed(false);
    #endif

    bool do_bed_cal = parser.boolval('B'), do_probe_cal = parser.boolval('P');
    if (!do_bed_cal && !do_probe_cal) do_bed_cal = do_probe_cal = true;

    // Synchronize with planner
    planner.synchronize();

    #ifndef PTC_PROBE_HEATING_OFFSET
      #define PTC_PROBE_HEATING_OFFSET 0
    #endif
    const xyz_pos_t parkpos = PTC_PARK_POS,
              probe_pos_xyz = xyz_pos_t(PTC_PROBE_POS) + xyz_pos_t({ 0.0f, 0.0f, PTC_PROBE_HEATING_OFFSET }),
                noz_pos_xyz = probe_pos_xyz - probe.offset_xy;  // Nozzle position based on probe position

    if (do_bed_cal || do_probe_cal) {
      // Ensure park position is reachable
      bool reachable = position_is_reachable(parkpos) || WITHIN(parkpos.z, Z_MIN_POS - fslop, Z_MAX_POS + fslop);
      if (!reachable)
        SERIAL_ECHOLNPGM("!Park");
      else {
        // Ensure probe position is reachable
        reachable = probe.can_reach(probe_pos_xyz);
        if (!reachable) SERIAL_ECHOLNPGM("!Probe");
      }

      if (!reachable) {
        SERIAL_ECHOLNPGM(" position unreachable - aborting.");
        return;
      }

      process_subcommands_now(FPSTR(G28_STR));
    }

    remember_feedrate_scaling_off();

    /******************************************
     * Calibrate bed temperature offsets
     ******************************************/

    // Report temperatures every second and handle heating timeouts
    millis_t next_temp_report = millis() + 1000;

    auto report_targets = [&](const celsius_t tb, const celsius_t tp) {
      SERIAL_ECHOLNPGM("Target Bed:", tb, " Probe:", tp);
    };

    if (do_bed_cal) {

      celsius_t target_bed = PTC_BED_START,
                target_probe = PTC_PROBE_TEMP;

      say_waiting_for(); SERIAL_ECHOLNPGM(" cooling.");
      while (thermalManager.wholeDegBed() > target_bed || thermalManager.wholeDegProbe() > target_probe)
        report_temps(next_temp_report);

      // Disable leveling so it won't mess with us
      TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));

      for (uint_fast8_t idx = 0; idx <= PTC_BED_COUNT; idx++) {
        thermalManager.setTargetBed(target_bed);

        report_targets(target_bed, target_probe);

        // Park nozzle
        do_blocking_move_to(parkpos);

        // Wait for heatbed to reach target temp and probe to cool below target temp
        if (wait_for_temps(target_bed, target_probe, next_temp_report, millis() + MIN_TO_MS(15))) {
          SERIAL_ECHOLNPGM("!Bed heating timeout.");
          break;
        }

        // Move the nozzle to the probing point and wait for the probe to reach target temp
        do_blocking_move_to(noz_pos_xyz);
        say_waiting_for_probe_heating();
        SERIAL_EOL();
        while (thermalManager.wholeDegProbe() < target_probe)
          report_temps(next_temp_report);

        const float measured_z = g76_probe(TSI_BED, target_bed, noz_pos_xyz);
        if (isnan(measured_z) || target_bed > (BED_MAX_TARGET)) break;
      }

      SERIAL_ECHOLNPGM("Retrieved measurements: ", ptc.get_index());
      if (ptc.finish_calibration(TSI_BED)) {
        say_successfully_calibrated();
        SERIAL_ECHOLNPGM(" bed.");
      }
      else {
        say_failed_to_calibrate();
        SERIAL_ECHOLNPGM(" bed. Values reset.");
      }

      // Cleanup
      thermalManager.setTargetBed(0);
      TERN_(HAS_LEVELING, set_bed_leveling_enabled(true));
    } // do_bed_cal

    /********************************************
     * Calibrate probe temperature offsets
     ********************************************/

    if (do_probe_cal) {

      // Park nozzle
      do_blocking_move_to(parkpos);

      // Initialize temperatures
      const celsius_t target_bed = BED_MAX_TARGET;
      thermalManager.setTargetBed(target_bed);

      celsius_t target_probe = PTC_PROBE_START;

      report_targets(target_bed, target_probe);

      // Wait for heatbed to reach target temp and probe to cool below target temp
      wait_for_temps(target_bed, target_probe, next_temp_report);

      // Disable leveling so it won't mess with us
      TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));

      bool timeout = false;
      for (uint_fast8_t idx = 0; idx <= PTC_PROBE_COUNT; idx++) {
        // Move probe to probing point and wait for it to reach target temperature
        do_blocking_move_to(noz_pos_xyz);

        say_waiting_for_probe_heating();
        SERIAL_ECHOLNPGM(" Bed:", target_bed, " Probe:", target_probe);
        const millis_t probe_timeout_ms = millis() + MIN_TO_MS(15);
        while (thermalManager.degProbe() < target_probe) {
          if (report_temps(next_temp_report, probe_timeout_ms)) {
            SERIAL_ECHOLNPGM("!Probe heating timed out.");
            timeout = true;
            break;
          }
        }
        if (timeout) break;

        const float measured_z = g76_probe(TSI_PROBE, target_probe, noz_pos_xyz);
        if (isnan(measured_z)) break;
      }

      SERIAL_ECHOLNPGM("Retrieved measurements: ", ptc.get_index());
      if (ptc.finish_calibration(TSI_PROBE))
        say_successfully_calibrated();
      else
        say_failed_to_calibrate();
      SERIAL_ECHOLNPGM(" probe.");

      // Cleanup
      thermalManager.setTargetBed(0);
      TERN_(HAS_LEVELING, set_bed_leveling_enabled(true));

      SERIAL_ECHOLNPGM("Final compensation values:");
      ptc.print_offsets();
    } // do_probe_cal

    restore_feedrate_and_scaling();
  }

#endif // PTC_PROBE && PTC_BED

/**
 * M871: Report / reset temperature compensation offsets.
 *       Note: This does not affect values in EEPROM until M500.
 *
 *   M871 [ R | B | P | E ]
 *
 *    No Parameters - Print current offset values.
 *
 * Select only one of these flags:
 *    R - Reset all offsets to zero (i.e., disable compensation).
 *    B - Manually set offset for bed
 *    P - Manually set offset for probe
 *    E - Manually set offset for extruder
 *
 * With B, P, or E:
 *    I[index] - Index in the array
 *    V[value] - Adjustment in µm
 */
void GcodeSuite::M871() {

  if (parser.seen('R')) {
    // Reset z-probe offsets to factory defaults
    ptc.clear_all_offsets();
    SERIAL_ECHOLNPGM("Offsets reset to default.");
  }
  else if (parser.seen("BPE")) {
    if (!parser.seenval('V')) return;
    const int16_t offset_val = parser.value_int();
    if (!parser.seenval('I')) return;
    const int16_t idx = parser.value_int();
    const TempSensorID mod = TERN_(PTC_BED,    parser.seen_test('B') ? TSI_BED   :)
                             TERN_(PTC_HOTEND, parser.seen_test('E') ? TSI_EXT   :)
                             TERN_(PTC_PROBE,  parser.seen_test('P') ? TSI_PROBE :) TSI_COUNT;
    if (mod == TSI_COUNT)
      SERIAL_ECHOLNPGM("!Invalid sensor.");
    else if (idx > 0 && ptc.set_offset(mod, idx - 1, offset_val))
      SERIAL_ECHOLNPGM("Set value: ", offset_val);
    else
      SERIAL_ECHOLNPGM("!Invalid index. Failed to set value (note: value at index 0 is constant).");
  }
  else // Print current Z-probe adjustments. Note: Values in EEPROM might differ.
    ptc.print_offsets();
}

#endif // HAS_PTC
