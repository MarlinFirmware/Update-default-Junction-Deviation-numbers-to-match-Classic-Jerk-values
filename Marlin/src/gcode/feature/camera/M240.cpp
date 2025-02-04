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

#if ENABLED(PHOTO_GCODE)

#include "../../gcode.h"
#include "../../../module/motion.h" // for active_extruder and current_position

#if PIN_EXISTS(CHDK)
  millis_t chdk_timeout; // = 0
#endif

#if defined(PHOTO_POSITION) && PHOTO_DELAY_MS > 0
  #include "../../../MarlinCore.h" // for idle()
#endif

#ifdef PHOTO_RETRACT_MM

  #define _PHOTO_RETRACT_MM (PHOTO_RETRACT_MM + 0)

  #include "../../../module/planner.h"
  #include "../../../module/temperature.h"

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    #include "../../../feature/pause.h"
  #endif

  #ifdef PHOTO_RETRACT_MM
    inline void e_move_m240(const float length, const_feedRate_t fr_mm_s) {
      if (length && thermalManager.hotEnoughToExtrude(active_extruder))
        unscaled_e_move(length, fr_mm_s);
    }
  #endif

#endif

#if PIN_EXISTS(PHOTOGRAPH)

  FORCE_INLINE void set_photo_pin(const uint8_t state) {
    constexpr uint32_t pulse_length = (
      #ifdef PHOTO_PULSES_US
        PHOTO_PULSE_DELAY_US
      #else
        15                    // 15.24 from _delay_ms(0.01524)
      #endif
    );
    WRITE(PHOTOGRAPH_PIN, state);
    delayMicroseconds(pulse_length);
  }

  FORCE_INLINE void tweak_photo_pin() { set_photo_pin(HIGH); set_photo_pin(LOW); }

  #ifdef PHOTO_PULSES_US

    inline void pulse_photo_pin(const uint32_t duration, const uint8_t state) {
      if (state) {
        for (const uint32_t stop = micros() + duration; micros() < stop;)
          tweak_photo_pin();
      }
      else
        delayMicroseconds(duration);
    }

    inline void spin_photo_pin() {
      static constexpr uint32_t sequence[] = PHOTO_PULSES_US;
      for (uint_fast8_t i = 0; i < COUNT(sequence); ++i)
        pulse_photo_pin(sequence[i], !(i & 1));
    }

  #else

    constexpr uint8_t NUM_PULSES = 16;
    inline void spin_photo_pin() { for (uint_fast8_t i = NUM_PULSES; i--;) tweak_photo_pin(); }

  #endif
#endif

/**
 * M240: Trigger a camera by...
 *
 *  - CHDK                  : Emulate a Canon RC-1 with a configurable ON duration.
 *                            https://captain-slow.dk/2014/03/09/3d-printing-timelapses/
 *  - PHOTOGRAPH_PIN        : Pulse a digital pin 16 times.
 *                            See https://www.doc-diy.net/photo/rc-1_hacked/
 *  - PHOTO_SWITCH_POSITION : Bump a physical switch with the X-carriage using a
 *                            configured position, delay, and retract length.
 *
 * PHOTO_POSITION parameters:
 *    A - X offset to the return position
 *    B - Y offset to the return position
 *    F - Override the XY movement feedrate
 *    R - Retract/recover length (current units)
 *    S - Retract/recover feedrate (mm/min)
 *    X - Move to X before triggering the shutter
 *    Y - Move to Y before triggering the shutter
 *    Z - Raise Z by a distance before triggering the shutter
 *
 * PHOTO_SWITCH_POSITION parameters:
 *    D - Duration (ms) to hold down switch (Requires PHOTO_SWITCH_MS)
 *    P - Delay (ms) after triggering the shutter (Requires PHOTO_SWITCH_MS)
 *    I - Switch trigger position override X
 *    J - Switch trigger position override Y
 */
void GcodeSuite::M240() {

  #ifdef PHOTO_POSITION

    if (homing_needed_error()) return;

    const xyz_pos_t old_pos = NUM_AXIS_ARRAY(
      current_position.x + parser.linearval('A'),
      current_position.y + parser.linearval('B'),
      current_position.z,
      current_position.i, current_position.j, current_position.k,
      current_position.u, current_position.v, current_position.w
    );

    #ifdef PHOTO_RETRACT_MM
      const float rval = parser.linearval('R', _PHOTO_RETRACT_MM);
      const feedRate_t sval = parser.feedrateval('S', TERN(ADVANCED_PAUSE_FEATURE, PAUSE_PARK_RETRACT_FEEDRATE, TERN(FWRETRACT, RETRACT_FEEDRATE, 45)));
      e_move_m240(-rval, sval);
    #endif

    feedRate_t fr_mm_s = parser.feedrateval('F');
    if (fr_mm_s) NOLESS(fr_mm_s, 10.0f);

    constexpr xyz_pos_t photo_position = PHOTO_POSITION;
    xyz_pos_t raw = {
       parser.seenval('X') ? RAW_X_POSITION(parser.value_linear_units()) : photo_position.x,
       parser.seenval('Y') ? RAW_Y_POSITION(parser.value_linear_units()) : photo_position.y,
      (parser.seenval('Z') ? parser.value_linear_units() : photo_position.z) + current_position.z
    };
    apply_motion_limits(raw);
    do_blocking_move_to(raw, fr_mm_s);

    #ifdef PHOTO_SWITCH_POSITION
      constexpr xy_pos_t photo_switch_position = PHOTO_SWITCH_POSITION;
      const xy_pos_t sraw = {
         parser.seenval('I') ? RAW_X_POSITION(parser.value_linear_units()) : photo_switch_position.x,
         parser.seenval('J') ? RAW_Y_POSITION(parser.value_linear_units()) : photo_switch_position.y
      };
      do_blocking_move_to_xy(sraw, get_homing_bump_feedrate(X_AXIS));
      #if PHOTO_SWITCH_MS > 0
        safe_delay(parser.intval('D', PHOTO_SWITCH_MS));
      #endif
      do_blocking_move_to(raw);
    #endif

  #endif

  #if PIN_EXISTS(CHDK)

    OUT_WRITE(CHDK_PIN, HIGH);
    chdk_timeout = millis() + parser.intval('D', PHOTO_SWITCH_MS);

  #elif HAS_PHOTOGRAPH

    spin_photo_pin();
    delay(7.33);
    spin_photo_pin();

  #endif

  #ifdef PHOTO_POSITION
    #if PHOTO_DELAY_MS > 0
      const millis_t timeout = millis() + parser.intval('P', PHOTO_DELAY_MS);
      while (PENDING(millis(), timeout)) idle();
    #endif
    do_blocking_move_to(old_pos, fr_mm_s);
    #ifdef PHOTO_RETRACT_MM
      e_move_m240(rval, sval);
    #endif
  #endif
}

#endif // PHOTO_GCODE
