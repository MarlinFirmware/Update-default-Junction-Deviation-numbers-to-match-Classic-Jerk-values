/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "marlinui_DOGM.h"
#include "../lcdprint.h"

typedef uint8_t game_dim_t;
typedef const u8g_pgm_uint8_t* pgm_bitmap_t;

constexpr game_dim_t GAME_WIDTH = LCD_PIXEL_WIDTH;
constexpr game_dim_t GAME_HEIGHT = LCD_PIXEL_HEIGHT;
constexpr game_dim_t GAME_FONT_WIDTH = MENU_FONT_WIDTH;
constexpr game_dim_t GAME_FONT_ASCENT = MENU_FONT_ASCENT;
