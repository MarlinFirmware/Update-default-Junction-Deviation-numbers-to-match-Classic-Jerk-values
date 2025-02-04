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

/**
 * Bed Level Tools for Pro UI
 *
 * Based on the original work of: Henri-J-Norden
 * https://github.com/Jyers/Marlin/pull/126
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
#pragma once

#include "../../../inc/MarlinConfigPre.h"

class BedLevelTools {
public:
  #if USE_GRID_MESHVIEWER
    static bool view_mesh;
    static bool viewer_print_value;
  #endif
  static bool goto_mesh_value;
  static uint8_t mesh_x, mesh_y;
  static uint8_t tilt_grid;

  #if ENABLED(AUTO_BED_LEVELING_UBL)
    static void manualValueUpdate(const uint8_t mesh_x, const uint8_t mesh_y, bool undefined=false);
    static bool createPlaneFromMesh();
  #else
    static void manualValueUpdate(const uint8_t mesh_x, const uint8_t mesh_y);
  #endif
  static void manualMove(const uint8_t mesh_x, const uint8_t mesh_y, bool zmove=false);
  static void moveToXYZ();
  static void moveToXY();
  static void moveToZ();
  static void probeXY();
  static void meshReset();
  static float getMaxValue();
  static float getMinValue();
  static bool meshValidate();
  #if USE_GRID_MESHVIEWER
    static void drawBedMesh(int16_t selected=-1, uint8_t gridline_width=1, uint16_t padding_x=8, uint16_t padding_y_top=(40 + 53 - 7));
    static void setMeshViewerStatus();
  #endif
};

extern BedLevelTools bedLevelTools;
