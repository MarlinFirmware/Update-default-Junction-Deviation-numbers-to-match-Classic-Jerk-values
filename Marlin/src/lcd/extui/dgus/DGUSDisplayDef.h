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
#pragma once

/**
 * lcd/extui/dgus/DGUSDisplayDef.h
 * Defines the interaction between Marlin and the display firmware
 */

#include "DGUSVPVariable.h"

#include <stdint.h>

// Information on which screen which VP is displayed.
// As this is a sparse table, two arrays are needed:
// one to list the VPs of one screen and one to map screens to the lists.
// (Strictly this would not be necessary, but allows to only send data the display needs and reducing load on Marlin)
struct VPMapping {
  const uint8_t screen;
  const uint16_t *VPList;  // The list is null-terminated.
};

extern const struct VPMapping VPMap[];

// List of VPs handled by Marlin / The Display.
extern const struct DGUS_VP_Variable ListOfVP[];

#include "../../../inc/MarlinConfig.h"

#if DGUS_LCD_UI_ORIGIN
  #include "origin/DGUSDisplayDef.h"
#elif DGUS_LCD_UI_MKS
  #include "mks/DGUSDisplayDef.h"
#elif DGUS_LCD_UI_FYSETC
  #include "fysetc/DGUSDisplayDef.h"
#elif DGUS_LCD_UI_HIPRECY
  #include "hiprecy/DGUSDisplayDef.h"
#elif DGUS_LCD_UI_CREALITY_TOUCH
  #include "creality/DGUSDisplayDef.h"
#endif
