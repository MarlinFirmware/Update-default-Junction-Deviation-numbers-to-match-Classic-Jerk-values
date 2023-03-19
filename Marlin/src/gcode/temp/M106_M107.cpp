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

#if HAS_FAN

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/temperature.h"

#if ENABLED(LASER_SYNCHRONOUS_M106_M107)
  #include "../../module/planner.h"
#endif

#if HAS_PREHEAT
  #include "../../lcd/marlinui.h"
#endif

#if ENABLED(SINGLENOZZLE)
  #define _ALT_P active_extruder
  #define _CNT_P EXTRUDERS
#else
  #define _ALT_P _MIN(active_extruder, FAN_COUNT - 1)
  #define _CNT_P FAN_COUNT
#endif

/**
 * M106: Set Fan Speed
 *
 *  I<index> Material Preset index (if material presets are defined)
 *  S<int>   Speed between 0-255
 *  P<index> Fan index, if more than one fan
 *
 * With EXTRA_FAN_SPEED enabled:
 *
 *  T<int>   Restore/Use/Set Temporary Speed:
 *           1     = Restore previous speed after T2
 *           2     = Use temporary speed set with T3-255
 *           3-255 = Set the speed for use with T2
 */
void GcodeSuite::M106() {
  const uint8_t pfan = parser.byteval('P', _ALT_P);
  if (pfan >= _CNT_P) return;
  if (FAN_IS_REDUNDANT(pfan)) return;

  #if ENABLED(EXTRA_FAN_SPEED)
    const uint16_t t = parser.intval('T');
    if (t > 0) return fans[pfan].set_temp_speed(t);
  #endif

  const uint16_t dspeed = parser.seen_test('A') ? fans[active_extruder].speed : 255;

  uint16_t speed = dspeed;

  // Accept 'I' if temperature presets are defined
  #if HAS_PREHEAT
    const bool got_preset = parser.seenval('I');
    if (got_preset) speed = ui.material_preset[_MIN(parser.value_byte(), PREHEAT_COUNT - 1)].fan_speed;
  #else
    constexpr bool got_preset = false;
  #endif

  if (!got_preset && parser.seenval('S'))
    speed = parser.value_ushort();

  TERN_(FOAMCUTTER_XYUV, speed *= 2.55f); // Get command in % of max heat

  // Set speed, with constraint
  thermalManager.set_fan_speed(pfan, speed);

  TERN_(LASER_SYNCHRONOUS_M106_M107, planner.buffer_sync_block(BLOCK_BIT_SYNC_FANS));

  if (TERN0(DUAL_X_CARRIAGE, idex_is_duplicating()))  // pfan == 0 when duplicating
    thermalManager.set_fan_speed(1 - pfan, speed);
}

/**
 * M107: Fan Off
 */
void GcodeSuite::M107() {
  const uint8_t pfan = parser.byteval('P', _ALT_P);
  if (pfan >= _CNT_P) return;
  if (FAN_IS_REDUNDANT(pfan)) return;

  thermalManager.set_fan_speed(pfan, 0);

  if (TERN0(DUAL_X_CARRIAGE, idex_is_duplicating()))  // pfan == 0 when duplicating
    thermalManager.set_fan_speed(1 - pfan, 0);

  TERN_(LASER_SYNCHRONOUS_M106_M107, planner.buffer_sync_block(BLOCK_BIT_SYNC_FANS));
}

#endif // HAS_FAN
