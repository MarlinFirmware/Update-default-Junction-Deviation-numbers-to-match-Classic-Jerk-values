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

#include "../../inc/MarlinConfig.h"

#if ENABLED(CREALITY_WIFI)

#include "../gcode.h"
#include "../../lcd/rts/lcd_rts.h"

/**
 * M930: WIFI Box Function
 */
void GcodeSuite::M930() {
  if (parser.boolval('F')) {
    switch (parser.intval('S')) {
      case 1:
        if (flag_counter_wifi_reset) {
          flag_counter_wifi_reset = false;
          rts.sendData(ExchangePageBase + 33, ExchangepageAddr);
          change_page_font = 33;
        }
        rts.sendData(1, WIFI_CONNECTED_DISPLAY_ICON_VP);
        break;

      case 2:
        rts.sendData(0, ADV_SETTING_WIFI_ICON_VP);
        rts.sendData(1, WIFI_CONNECTED_DISPLAY_ICON_VP);
        break;

      case 3:
        rts.sendData(1, ADV_SETTING_WIFI_ICON_VP);
        rts.sendData(0, WIFI_CONNECTED_DISPLAY_ICON_VP);
        break;
    }
  }
}

#endif // CREALITY_WIFI
