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

#if HAS_BED_PROBE

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/probe.h"
#include "../../feature/bedlevel/bedlevel.h"
#include "../../lcd/marlinui.h"

#if HAS_PTC
  #include "../../feature/probe_temp_comp.h"
#endif

#if ANY(DWIN_CREALITY_LCD_JYERSUI, EXTENSIBLE_UI)
  #define VERBOSE_SINGLE_PROBE
#endif

/**
 * G30: Do a single Z probe at the given XY (default: current)
 *
 * Parameters:
 *
 *   X   Probe X position (default current X)
 *   Y   Probe Y position (default current Y)
 *   E   Engage the probe for each probe (default 1)
 *   C   Enable probe temperature compensation (0 or 1, default 1)
 */
void GcodeSuite::G30() {

  xy_pos_t probepos = current_position;

  const bool seenX = parser.seenval('X');
  if (seenX) probepos.x = RAW_X_POSITION(parser.value_linear_units());
  const bool seenY = parser.seenval('Y');
  if (seenY) probepos.y = RAW_Y_POSITION(parser.value_linear_units());

  probe.use_probing_tool();

  if (probe.can_reach(probepos)) {

    // Disable leveling so the planner won't mess with us
    TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));

    // Disable feedrate scaling so movement speeds are correct
    remember_feedrate_scaling_off();

    // With VERBOSE_SINGLE_PROBE home only if needed
    TERN_(VERBOSE_SINGLE_PROBE, process_subcommands_now(F("G28O")));

    // Raise after based on the 'E' parameter
    const ProbePtRaise raise_after = parser.boolval('E', true) ? PROBE_PT_STOW : PROBE_PT_NONE;

    // Use 'C' to set Probe Temperature Compensation ON/OFF (on by default)
    TERN_(HAS_PTC, ptc.set_enabled(parser.boolval('C', true)));

    // Probe the bed, optionally raise, and return the measured height
    const float measured_z = probe.probe_at_point(probepos, raise_after);

    // After probing always re-enable Probe Temperature Compensation
    TERN_(HAS_PTC, ptc.set_enabled(true));

    // Report a good probe result to the host and LCD
    if (!isnan(measured_z)) {
      const xy_pos_t lpos = probepos.asLogical();
      SString<30> msg(
        F("Bed X:"), p_float_t(lpos.x, 2),
        F(  " Y:"), p_float_t(lpos.y, 2),
        F(  " Z:"), p_float_t(measured_z, 3)
      );
      msg.echoln();
      TERN_(VERBOSE_SINGLE_PROBE, ui.set_status(msg));
    }

    // Restore feedrate scaling
    restore_feedrate_and_scaling();

    // Move the nozzle to the position of the probe
    do_blocking_move_to(probepos);

    if (raise_after == PROBE_PT_STOW)
      probe.move_z_after_probing();

    report_current_position();
  }
  else {
    SERIAL_ECHOLN(GET_EN_TEXT_F(MSG_ZPROBE_OUT));
    LCD_MESSAGE(MSG_ZPROBE_OUT);
  }

  probe.use_probing_tool(false);
}

#endif // HAS_BED_PROBE
