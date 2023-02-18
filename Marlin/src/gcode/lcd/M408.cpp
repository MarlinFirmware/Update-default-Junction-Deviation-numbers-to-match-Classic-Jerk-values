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

#include "../../inc/MarlinConfig.h"

#if ENABLED(PANELDUE)

#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/printcounter.h"
#include "../../module/temperature.h"
#include "../../sd/cardreader.h"
#include "../../lcd/marlinui.h"

#if ENABLED(LCD_SET_PROGRESS_MANUALLY)
  extern uint8_t progress_bar_percent;
#endif

/**
 * M408: Report machine state in JSON format for PanelDue
 *
 *  S<style> Include static values with S1
 *
 * Since Marlin doesn't use sequence numbers, the R parameter is omitted.
 *
 */

inline void json_key(const char * const name) {
  SERIAL_ECHOPGM(",\"", name, "\":");
}

inline void json_array_print(const char * const name, const float val[], const int8_t size) {
  json_key(name);
  SERIAL_CHAR('[');
  for (uint8_t i = 0; i < size; i++) {
    SERIAL_ECHO(val[i]);
    if (i < size - 1) SERIAL_CHAR(',');
  }
  SERIAL_CHAR(']');
  safe_delay(1);
}

void GcodeSuite::M408() {
  float tmp[10];
  SERIAL_CHAR('{');

  // status: SD printing or idle
  json_key(PSTR("status"));
  SERIAL_ECHO(IS_SD_PRINTING() ? F("\"P\"") : F("\"I\""));

  // heaters: current bed, hotend temperatures
  tmp[0] = TERN0(HAS_HEATED_BED, thermalManager.degBed());
  HOTEND_LOOP() tmp[e + 1] = thermalManager.degHotend(e);
  json_array_print(PSTR("heaters"), tmp, HOTENDS + 1);

  // active: target bed, hotend temperatures
  tmp[0] = TERN0(HAS_HEATED_BED, thermalManager.degTargetBed());
  HOTEND_LOOP() tmp[e + 1] = thermalManager.degTargetHotend(e);
  json_array_print(PSTR("active"), tmp, HOTENDS + 1);

  // standby: in Marlin, same as active
  json_array_print(PSTR("standby"), tmp, HOTENDS + 1);

  // hstat: in Marlin, heating or off
  json_key(PSTR("hstat"));
  SERIAL_CHAR('[');
  SERIAL_CHAR(TERN0(HAS_HEATED_BED, thermalManager.degTargetBed()) ? '2' : '0');
  HOTEND_LOOP() {
    SERIAL_CHAR(',');
    SERIAL_CHAR(thermalManager.degTargetHotend(e) ? '2' : '0');
  }
  SERIAL_CHAR(']');

  // pos: tool position
  tmp[0] = current_position.x; tmp[1] = current_position.y; tmp[2] = current_position.z;
  json_array_print(PSTR("pos"), tmp, 3);

  // extr: extruder position
  for (uint8_t e = 0; e < EXTRUDERS; e++) tmp[e] = current_position.e;
  json_array_print(PSTR("extr"), tmp, EXTRUDERS);

  // sfactor: feedrate %
  json_key(PSTR("sfactor"));
  SERIAL_ECHO(feedrate_percentage);

  // efactor: flow %
  for (uint8_t e = 0; e < EXTRUDERS; e++) tmp[e] = planner.flow_percentage[e];
  json_array_print(PSTR("efactor"), tmp, EXTRUDERS);

  // tool: selected extruder
  json_key(PSTR("tool"));
  SERIAL_ECHO(active_extruder);

  // probe: the last Z probe reading (just 0 for now)
  json_key(PSTR("probe"));
  SERIAL_ECHOPGM("\"0\"");

  #if FAN_COUNT > 0
    // fanPercent: the last Z probe reading
    for (uint8_t i = 0; i < FAN_COUNT; i++) tmp[i] = map(thermalManager.fan_speed[i], 0, 255, 0, 100);
    json_array_print(PSTR("fanPercent"), tmp, FAN_COUNT);

    // fanRPM: print cooling fan faux RPM
    json_key(PSTR("fanRPM"));
    SERIAL_ECHO(int(thermalManager.fan_speed[0] * 10));
  #endif

  // homed: axis homed status
  json_key(PSTR("homed"));
  LOOP_NUM_AXES(i) {
    SERIAL_CHAR(i ? ',' : '[');
    SERIAL_ECHO(int(TEST(axes_homed, i)));
  }
  SERIAL_CHAR(']');

  #if ENABLED(SDSUPPORT)
    // fraction_printed: print progress
    json_key(PSTR("fraction_printed"));
    SERIAL_ECHO(0.01 * ui.get_progress_percent());
  #endif

  #if HAS_DISPLAY
    // message
    if (ui.status_message[0]) {
      json_key(PSTR("message"));
      SERIAL_ECHOPGM("\"", &ui.status_message);
      SERIAL_CHAR('"');
    }
  #endif

  // Extra fields
  if (parser.intval('S') == 1) {
    // myName
    json_key(PSTR("myName"));
    SERIAL_ECHOPGM("\"" MACHINE_NAME "\"");
    // firmwareName
    json_key(PSTR("firmwareName"));
    SERIAL_ECHOPGM("\"Marlin\"");
    // geometry
    json_key(PSTR("geometry"));
    SERIAL_ECHOPGM("\""
      TERN_(IS_FULL_CARTESIAN, "cartesian")
      TERN_(IS_SCARA, "scara")
      TERN_(DELTA, "delta")
      TERN_(COREXY, "corexy")
      TERN_(COREXZ, "corexz")
      TERN_(COREYZ, "coreyz")
      TERN_(COREYX, "coreyx")
      TERN_(COREZX, "corezx")
      TERN_(COREZY, "corezy")
      "\""
    );
    // axes
    json_key(PSTR("axes"));
    SERIAL_CHAR('3');
    // volumes: the number of SD card slots available
    json_key(PSTR("volumes"));
    SERIAL_CHAR(TERN(SDSUPPORT, '1', '0'));
    // numTools: extruder count
    json_key(PSTR("numTools"));
    SERIAL_CHAR('0' + EXTRUDERS);
  }

  SERIAL_EOL();
}

#endif // PANELDUE
