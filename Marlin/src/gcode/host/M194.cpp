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

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(MENU_RESET_WIFI)

#include "../gcode.h"
#include "../../MarlinCore.h"

#if ENABLED(CREALITY_RTS)
  #include "../../../src/lcd/rts/lcd_rts.h"
#endif

/**
 * M194: Reset WiFi Module
 */
void GcodeSuite::M194() {
  TERN_(CREALITY_RTS, if (parser.seen('S')) wifi_enable_flag = parser.boolval('S'));

  // record pressed state and output low level
  rts_wifi_state = PRESSED;
  OUT_WRITE(RESET_WIFI_PIN, LOW);
  SERIAL_ECHOLNPGM(STR_OK " wifi is resetting");
}

#endif // MENU_RESET_WIFI
