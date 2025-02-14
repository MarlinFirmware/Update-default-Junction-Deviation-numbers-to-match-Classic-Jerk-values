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

#if HAS_PREHEAT

#include "../gcode.h"
#include "../../lcd/marlinui.h"

#if HAS_HOTEND
  #include "../../module/temperature.h"
#endif

/**
 * M145: Set the heatup state for a material in the LCD menu
 *
 *   S<material>
 *   H<hotend temp>
 *   B<bed temp>
 *   C<chamber temp>
 *   F<fan speed>
 */
void GcodeSuite::M145() {
  const uint8_t material = (uint8_t)parser.intval('S');
  if (material >= PREHEAT_COUNT)
    SERIAL_ERROR_MSG(STR_ERR_MATERIAL_INDEX);
  else {
    preheat_t &mat = ui.material_preset[material];
    #if HAS_HOTEND
      if (parser.seenval('H'))
        mat.hotend_temp = constrain(parser.value_int(), thermalManager.extrude_min_temp, thermalManager.hotend_max_target(0));
    #endif
    #if HAS_HEATED_BED
      if (parser.seenval('B'))
        mat.bed_temp = constrain(parser.value_int(), BED_MINTEMP, BED_MAX_TARGET);
    #endif
    #if HAS_HEATED_CHAMBER
      if (parser.seenval('C'))
        mat.chamber_temp = constrain(parser.value_int(), CHAMBER_MINTEMP, CHAMBER_MAX_TARGET);
    #endif
    #if HAS_FAN
      if (parser.seenval('F'))
        mat.fan_speed = constrain(parser.value_int(), 0, 255);
    #endif
  }
}

void GcodeSuite::M145_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading(forReplay, F(STR_MATERIAL_HEATUP));
  for (uint8_t i = 0; i < PREHEAT_COUNT; ++i) {
    report_echo_start(forReplay);
    SERIAL_ECHOLNPGM_P(
      PSTR("  M145 S"), i
      #if HAS_HOTEND
        , PSTR(" H"), parser.to_temp_units(ui.material_preset[i].hotend_temp)
      #endif
      #if HAS_HEATED_BED
        , SP_B_STR, parser.to_temp_units(ui.material_preset[i].bed_temp)
      #endif
      #if HAS_FAN
        , PSTR(" F"), ui.material_preset[i].fan_speed
      #endif
    );
  }
}

#endif // HAS_PREHEAT
