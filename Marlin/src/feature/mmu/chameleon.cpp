/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if HAS_CHAMELEON

#include "chameleon.h"
//#include "../../MarlinCore.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"
#include "../../lcd/marlinui.h"

//#define DEBUG_OUT ENABLED(DEBUG_CHAMELEON)
#include "../../core/debug_out.h"

Chameleon chameleon;

struct E_Step {
  float extrude;
  feedRate_t feedRate;
};

// Change tools the Chameleon 3D way
void Chameleon::tool_change(const uint8_t index) {
  const uint8_t switches = active_extruder ^ index;
  if (!switches) return;

  ui.status_printf(0, GET_TEXT_F(MSG_MMU2_LOADING_FILAMENT), int(index + 1));

  #ifdef CHAMELEON_TOOLCHANGE_Y
    current_position.y = CHAMELEON_TOOLCHANGE_Y;
  #endif
  #ifdef CHAMELEON_TOOLCHANGE_Z
    NOLESS(current_position.z, CHAMELEON_TOOLCHANGE_Z);
    line_to_current_position(5000);
  #endif
  if (switches & 0x1) {
    current_position.x = CHAMELEON_BUTTON1_X + (X_CENTER > (CHAMELEON_BUTTON1_X)) ? 10 : -10;
  }
  else {
    #if EXTRUDERS > 2
      if (switches & 0x2)
        current_position.x = CHAMELEON_BUTTON2_X + (X_CENTER > (CHAMELEON_BUTTON2_X)) ? 10 : -10;
    #endif
  }
  line_to_current_position(5000);

  static constexpr E_Step ramming_sequence[] PROGMEM = { CHAMELEON_RAMMING_SEQUENCE };
  execute_extruder_sequence(ramming_sequence, COUNT(ramming_sequence));

  toggle(switches);

  active_extruder = index;

  ui.reset_status();
}

// Execute a list of E moves
void Chameleon::execute_extruder_sequence(const E_Step * const sequence, const uint8_t steps) {
  planner.synchronize();
  stepper.enable_extruder();

  const E_Step *step = sequence;

  for (uint8_t i = 0; i < steps; ++i) {
    const float es = pgm_read_float(&(step->extrude));
    const uint16_t fr_or_ms = pgm_read_float(&(step->feedRate));

    DEBUG_ECHO_MSG("Move E", es, " @ ", fr_or_ms);

    if (es) {
      current_position.e += es;
      line_to_current_position(MMM_TO_MMS(fr_or_ms));
      planner.synchronize();
    }
    else
      safe_delay(fr_or_ms);

    step++;
  }

  // Reset E to 0 for the new extruder tool
  current_position.e = 0;
  sync_plan_position_e();

  stepper.disable_extruder();
}

// Bump the switches needed to change feeders
void Chameleon::toggle(const uint8_t switches) {
  if (switches & 0x1) {
    current_position.x = CHAMELEON_BUTTON1_X;
    line_to_current_position(2000);
    current_position.x += (X_CENTER > (CHAMELEON_BUTTON1_X)) ? 10 : -10;
    line_to_current_position(2000);
  }
  #if EXTRUDERS > 2
    if (switches & 0x2) {
      current_position.x = CHAMELEON_BUTTON2_X;
      line_to_current_position(2000);
      current_position.x += (X_CENTER > (CHAMELEON_BUTTON2_X)) ? 10 : -10;
      line_to_current_position(2000);
    }
  #endif
  planner.synchronize();
}

#endif // HAS_CHAMELEON
