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
#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_U8GLIB && HAS_GAMES

#include "../menu/game/types.h" // includes dogm/game.h

void MarlinGame::frame_start() {
  set_color(color::WHITE);
}

void MarlinGame::frame_end() {}

void MarlinGame::set_color(const color color) {
  switch (color) {
    default:
    case color::WHITE: u8g.setColorIndex(1); break;
    case color::BLACK: u8g.setColorIndex(0); break;
  }
}

void MarlinGame::draw_hline(const game_dim_t x, const game_dim_t y, const game_dim_t w) {
  u8g.drawHLine(x, y, w);
}

void MarlinGame::draw_vline(const game_dim_t x, const game_dim_t y, const game_dim_t h) {
  u8g.drawVLine(x, y, h);
}

void MarlinGame::draw_frame(const game_dim_t x, const game_dim_t y, const game_dim_t w, const game_dim_t h) {
  u8g.drawFrame(x, y, w, h);
}

void MarlinGame::draw_box(const game_dim_t x, const game_dim_t y, const game_dim_t w, const game_dim_t h) {
  u8g.drawBox(x, y, w, h);
}

void MarlinGame::draw_pixel(const game_dim_t x, const game_dim_t y) {
  u8g.drawPixel(x, y);
}

void MarlinGame::draw_bitmap(const game_dim_t x, const game_dim_t y, const game_dim_t bytes_per_row, const game_dim_t rows, const pgm_bitmap_t bitmap) {
  u8g.drawBitmapP(x, y, bytes_per_row, rows, bitmap);
}

int MarlinGame::draw_string(const game_dim_t x, const game_dim_t y, const char* str) {
  lcd_moveto(x, y);
  return lcd_put_u8str_P(str);
}

int MarlinGame::draw_string(const game_dim_t x, const game_dim_t y, FSTR_P const fstr) {
  lcd_moveto(x, y);
  return lcd_put_u8str(fstr);
}

void MarlinGame::draw_int(const game_dim_t x, const game_dim_t y, const int value) {
  lcd_put_int(x, y, value);
}

#endif // HAS_MARLINUI_U8GLIB && HAS_GAMES
