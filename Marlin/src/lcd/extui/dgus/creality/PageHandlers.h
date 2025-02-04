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
#pragma once

// Mapping of handlers per page. This construction is necessary because the CR-6 touch screen re-uses the same button IDs all over the place.
typedef void (*DGUS_CREALITY_SCREEN_BUTTON_HANDLER)(DGUS_VP_Variable &var, unsigned short buttonValue);

struct PageHandler {
  DGUS_ScreenID ScreenID;
  DGUS_CREALITY_SCREEN_BUTTON_HANDLER Handler;
};

void DGUSCrealityDisplay_HandleReturnKeyEvent(DGUS_VP_Variable &var, void *val_ptr);
