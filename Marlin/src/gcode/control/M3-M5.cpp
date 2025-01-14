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

#if HAS_CUTTER

#include "../gcode.h"
#include "../../feature/spindle_laser.h"
#include "../../module/planner.h"

/**
 * Laser:
 *  M3 - Laser ON/Power (Ramped power)
 *  M4 - Laser ON/Power (Ramped power)
 *
 *  M3I - Enable continuous inline power to be processed by the planner, with power
 *        calculated and set in the planner blocks, processed inline during stepping.
 *        In inline mode M3 S-Values will set the power for the next moves.
 *        (e.g., G1 X10 Y10 powers on with the last S-Value.)
 *        M3I must be set before using planner-synced M3 inline S-Values (LASER_POWER_SYNC).
 *
 *  M4I - Set dynamic mode which calculates laser power OCR based on the current feedrate.
 *
 * Spindle:
 *  M3 - Spindle ON (Clockwise)
 *  M4 - Spindle ON (Counter-clockwise)
 *
 * Parameters:
 *  S<power> - Set power. S0 will turn the spindle/laser off.
 *  O<power> - Set power in PWM units 0-255
 *
 *  If no PWM pin is defined then M3/M4 just turns it on or off.
 *
 *  At least 12.8kHz (50Hz * 256) is needed for Spindle PWM.
 *  Hardware PWM is required on AVR. ISRs are too slow.
 *
 * NOTE: WGM for timers 3, 4, and 5 must be either Mode 1 or Mode 5.
 *       No other settings give a PWM signal that goes from 0 to 5 volts.
 *
 *       The system automatically sets WGM to Mode 1, so no special
 *       initialization is needed.
 *
 *       WGM bits for timer 2 are automatically set by the system to
 *       Mode 1. This produces an acceptable 0 to 5 volt signal.
 *       No special initialization is needed.
 *
 * NOTE: A minimum PWM frequency of 50 Hz is needed. All prescaler
 *       factors for timers 2, 3, 4, and 5 are acceptable.
 *
 *  SPINDLE_LASER_ENA_PIN needs an external pullup or it may power on
 *  the spindle/laser during power-up or when connecting to the host
 *  (usually goes through a reset which sets all I/O pins to tri-state)
 *
 *  PWM duty cycle goes from 0 (off) to 255 (always on).
 */
void GcodeSuite::M3_M4(const bool is_M4) {
  #if LASER_SAFETY_TIMEOUT_MS > 0
    reset_stepper_timeout(); // Reset timeout to allow subsequent G-code to power the laser (imm.)
  #endif

  if (cutter.cutter_mode == CUTTER_MODE_STANDARD)
    planner.synchronize();   // Wait for previous movement commands (G0/G1/G2/G3) to complete before changing power

  #if ENABLED(LASER_FEATURE)
    if (parser.seen_test('I')) {
      cutter.cutter_mode = is_M4 ? CUTTER_MODE_DYNAMIC : CUTTER_MODE_CONTINUOUS;
      cutter.inline_power(0);
      cutter.set_enabled(true);
    }
  #endif

  auto get_s_power = [] {
    if (parser.seenval('S')) {
      const float v = parser.value_float();
      cutter.menuPower = cutter.unitPower = TERN(LASER_POWER_TRAP, constrain( v, 0, CUTTER_POWER_MAX), cutter.power_to_range(v));
    }
    else if (parser.seenval('O')) { // pwr in PWM units
      const float v = parser.value_float();
      cutter.menuPower = cutter.unitPower = CUTTER_PWM_TO_SPWR(constrain(v, 0, 255));
    }
    else if (cutter.cutter_mode == CUTTER_MODE_STANDARD)
      cutter.menuPower = cutter.unitPower = cutter.cpwr_to_upwr(SPEED_POWER_STARTUP);

    // PWM not implied, power converted to OCR from unit definition and on/off if not PWM.
    cutter.power = TERN(SPINDLE_LASER_USE_PWM, cutter.upower_to_ocr(cutter.unitPower), cutter.unitPower > 0 ? 255 : 0);
  };

  if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS || cutter.cutter_mode == CUTTER_MODE_DYNAMIC) {  // Laser power in inline mode
    #if ENABLED(LASER_FEATURE)
      planner.laser_inline.status.isPowered = true;                                                 // M3 or M4 is powered either way
      get_s_power();                                                                                // Update cutter.power if seen
      #if ENABLED(LASER_POWER_SYNC)
        // With power sync we only set power so it does not effect queued inline power sets
        planner.buffer_sync_block(BLOCK_BIT_LASER_PWR);                                            // Send the flag, queueing inline power
      #else
        planner.synchronize();
        cutter.inline_power(cutter.power);
      #endif
    #endif
  }
  else {
    cutter.set_enabled(true);
    get_s_power();
    cutter.apply_power(
      #if ENABLED(SPINDLE_SERVO)
        cutter.unitPower
      #elif ENABLED(SPINDLE_LASER_USE_PWM)
        cutter.upower_to_ocr(cutter.unitPower)
      #else
        cutter.unitPower > 0 ? 255 : 0
      #endif
    );
    TERN_(SPINDLE_CHANGE_DIR, cutter.set_reverse(is_M4));
  }
}

/**
 * M5 - Cutter OFF (when moves are complete)
 *
 * Laser:
 *  M5  - Set power output to 0 (leaving inline mode unchanged).
 *  M5I - Clear inline mode and set power to 0.
 *
 * Spindle:
 *  M5 - Spindle OFF
 */
void GcodeSuite::M5() {
  planner.synchronize();
  cutter.power = 0;
  cutter.apply_power(0);                          // M5 just kills power, leaving inline mode unchanged
  if (cutter.cutter_mode != CUTTER_MODE_STANDARD) {
    if (parser.seen_test('I')) {
      TERN_(LASER_FEATURE, cutter.inline_power(cutter.power));
      cutter.set_enabled(false);                  // Needs to happen while we are in inline mode to clear inline power.
      cutter.cutter_mode = CUTTER_MODE_STANDARD;  // Switch from inline to standard mode.
    }
  }
  cutter.set_enabled(false);                      // Disable enable output setting
}

#endif // HAS_CUTTER
