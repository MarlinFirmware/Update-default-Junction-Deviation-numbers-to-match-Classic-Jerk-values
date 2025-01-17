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
 * M421.cpp - Auto Bed Leveling
 */

#include "../../../inc/MarlinConfig.h"

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

#include "../../gcode.h"
#include "../../../feature/bedlevel/bedlevel.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../../../lcd/extui/ui_api.h"
#endif

/**
 * M421: Set one or more Mesh Bed Leveling Z coordinates
 *
 * Usage:
 *   M421 I<xindex> J<yindex> Z<linear>
 *   M421 I<xindex> J<yindex> Q<offset>
 *
 *  - If I is omitted, set the entire row
 *  - If J is omitted, set the entire column
 *  - If both I and J are omitted, set all
 */
void GcodeSuite::M421() {
  int8_t ix = parser.intval('I', -1), iy = parser.intval('J', -1);
  const bool hasZ = parser.seenval('Z'),
             hasQ = !hasZ && parser.seenval('Q');

  if (hasZ || hasQ) {
    if (WITHIN(ix, -1, GRID_MAX_POINTS_X - 1) && WITHIN(iy, -1, GRID_MAX_POINTS_Y - 1)) {
      const float zval = parser.value_linear_units();
      uint8_t sx = ix >= 0 ? ix : 0, ex = ix >= 0 ? ix : GRID_MAX_POINTS_X - 1,
              sy = iy >= 0 ? iy : 0, ey = iy >= 0 ? iy : GRID_MAX_POINTS_Y - 1;
      for (uint_fast8_t x = sx; x <= ex; ++x) {
        for (uint_fast8_t y = sy; y <= ey; ++y) {
          bedlevel.z_values[x][y] = zval + (hasQ ? bedlevel.z_values[x][y] : 0);
          TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, bedlevel.z_values[x][y]));
        }
      }
      bedlevel.refresh_bed_level();
    }
    else
      SERIAL_ERROR_MSG(STR_ERR_MESH_XY);
  }
  else
    SERIAL_ERROR_MSG(STR_ERR_M421_PARAMETERS);
}

#endif // AUTO_BED_LEVELING_BILINEAR
