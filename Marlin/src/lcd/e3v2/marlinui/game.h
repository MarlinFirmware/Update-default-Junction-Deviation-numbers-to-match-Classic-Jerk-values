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

#include "../marlinui/marlinui_dwin.h"

typedef uint8_t game_dim_t;
typedef uint16_t screen_dim_t;
typedef const uint8_t* pgm_bitmap_t;

namespace dwin_game {
  /**
   * @brief Target the renderer at 128x64 pixels to match U8G screens
   */
  constexpr screen_dim_t TARGET_WIDTH = 128;
  constexpr screen_dim_t TARGET_HEIGHT = 64;

  constexpr int calculate_scale() {
    // Use whichever is smaller: the width or height scaling factor
    float scaling_factor = _MIN(
      static_cast<float>(DWIN_WIDTH) / static_cast<float>(TARGET_WIDTH),
      static_cast<float>(DWIN_HEIGHT) / static_cast<float>(TARGET_HEIGHT)
    );

    // Round DOWN to closest integer
    return static_cast<int>(scaling_factor);
  }

  /**
   * @brief Game render scale.
   */
  constexpr int scale = calculate_scale();

  /**
   * @brief Scale a game dimension to screen dimensions
   */
  constexpr game_dim_t screen_to_game(const screen_dim_t x) {
    return x / scale;
  }

  /**
   * @brief Scale a screen dimension to game dimensions
   */
  constexpr screen_dim_t game_to_screen(const game_dim_t x) {
    return x * scale;
  }

  /**
   * @brief Offset of the game window on the screen. Applied after scaling.
   */
  constexpr screen_dim_t x_offset = (DWIN_WIDTH - game_to_screen(TARGET_WIDTH)) / 2;
  constexpr screen_dim_t y_offset = (DWIN_HEIGHT - game_to_screen(TARGET_HEIGHT)) / 2;

  static_assert(game_to_screen(TARGET_WIDTH) + (x_offset * 2) <= DWIN_WIDTH, "DWIN game renderer failed to auto-scale, is too wide");
  static_assert(game_to_screen(TARGET_HEIGHT) + (y_offset * 2) <= DWIN_HEIGHT, "DWIN game renderer failed to auto-scale, is too high");
} // namespace dwin_game

constexpr game_dim_t GAME_WIDTH = dwin_game::screen_to_game(DWIN_WIDTH - (dwin_game::x_offset * 2));
constexpr game_dim_t GAME_HEIGHT = dwin_game::screen_to_game(DWIN_HEIGHT - (dwin_game::y_offset * 2));
constexpr game_dim_t GAME_FONT_WIDTH = dwin_game::screen_to_game(MENU_FONT_WIDTH);
constexpr game_dim_t GAME_FONT_ASCENT = dwin_game::screen_to_game(MENU_FONT_ASCENT);

// DWIN screens don't page, so these macros are always true
#define PAGE_OVER(ya) true
#define PAGE_UNDER(yb) true
#define PAGE_CONTAINS(ya, yb) true
