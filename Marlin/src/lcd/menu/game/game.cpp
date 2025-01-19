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

#include "../../../inc/MarlinConfigPre.h"

#if HAS_GAMES

#include "game.h"

int MarlinGame::score;
uint8_t MarlinGame::game_state;
millis_t MarlinGame::next_frame;

MarlinGameData marlin_game_data;

bool MarlinGame::game_frame() {
  static int8_t slew;
  if (ui.first_page) slew = 2;
  ui.refresh(LCDVIEW_CALL_NO_REDRAW); // Refresh as often as possible
  return (game_state && slew-- > 0);
}

void MarlinGame::draw_game_over() {
  constexpr int8_t gowide = (GAME_FONT_WIDTH) * 9,
                   gohigh = GAME_FONT_ASCENT - 3,
                       lx = (GAME_WIDTH - gowide) / 2,
                       ly = (GAME_HEIGHT + gohigh) / 2;
  if (PAGE_CONTAINS(ly - gohigh - 1, ly + 1)) {
    set_color(color::BLACK);
    draw_box(lx - 1, ly - gohigh - 1, gowide + 2, gohigh + 2);
    set_color(color::WHITE);
    if (ui.get_blink()) draw_string(lx, ly, F("GAME OVER"));
  }
}

void MarlinGame::init_game(const uint8_t init_state, const screenFunc_t screen) {
  score = 0;
  game_state = init_state;
  ui.goto_screen(screen);
  ui.defer_status_screen();
}

void MarlinGame::exit_game() {
  ui.goto_previous_screen_no_defer();
}

#endif // HAS_GAMES
