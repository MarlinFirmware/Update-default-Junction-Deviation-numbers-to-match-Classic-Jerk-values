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
#include "../../../inc/MarlinConfigPre.h"

#if IS_DWIN_MARLINUI && HAS_GAMES

// Enable performance counters (draw call count, frame timing) for debugging
//#define GAME_PERFORMANCE_COUNTERS

#include "../../menu/game/types.h" // includes e3v2/marlinui/game.h
#include "../../lcdprint.h"
#include "lcdprint_dwin.h"
#include "marlinui_dwin.h"

#if ENABLED(GAME_PERFORMANCE_COUNTERS)

  typedef struct {
    /**
     * Number of draw calls sent to the LCD
     */
    uint32_t draw_calls;

    /**
     * millis() value at the start of the current frame
     */
    millis_t frame_draw_millis;

    /**
     * millis() value at the end of the previous frame (in frame_start)
     * or time spend waiting for the next frame (in frame_end)
     */
    millis_t frame_wait_millis;
  } dwin_game_perf_t;

  static dwin_game_perf_t dwin_game_perf;

  #define COUNT_DRAW_CALLS(n) dwin_game_perf.draw_calls += n

#else // !GAME_PERFORMANCE_COUNTERS

  #define COUNT_DRAW_CALLS(...) NOOP

#endif // !GAME_PERFORMANCE_COUNTERS

void MarlinGame::frame_start() {
  // Clear the screen before each frame
  //dwinFrameClear(CLEAR_COLOR);

  // Instead of using dwinFrameClear, fill the play area with the background color
  // This tends to be faster than clearing the whole screen
  const uint16_t fg = dwin_font.fg;
  dwin_font.fg = COLOR_BG_BLACK;
  draw_box(0, 0, GAME_WIDTH, GAME_HEIGHT);
  dwin_font.fg = fg;

  // Ensure the correct font is selected
  dwin_font.index = DWIN_FONT_MENU;

  #if ENABLED(GAME_PERFORMANCE_COUNTERS)
    // Reset draw call counters
    dwin_game_perf.draw_calls = 0;

    // Update timing information
    const millis_t now = millis();
    dwin_game_perf.frame_draw_millis = now;
    dwin_game_perf.frame_wait_millis = now - dwin_game_perf.frame_wait_millis;
  #endif
}

void MarlinGame::frame_end() {
  #if ENABLED(GAME_PERFORMANCE_COUNTERS)
    const millis_t now = millis();
    const millis_t frame_wait_millis = dwin_game_perf.frame_wait_millis;
    const millis_t frame_draw_millis = now - dwin_game_perf.frame_draw_millis;

    dwin_game_perf.frame_wait_millis = now;

    // Save previous font settings and set new ones
    const uint16_t fg = dwin_font.fg;
    const bool solid = dwin_font.solid;
    set_color(color::YELLOW);
    dwin_font.solid = true;

    // Draw performance counters information
    char perf_str[32];
    sprintf_P(
      perf_str,
      PSTR("d%04lu w%04lu c%04lu"),
      frame_draw_millis,
      frame_wait_millis,
      dwin_game_perf.draw_calls
    );
    lcd_moveto_xy(0, 0);
    lcd_put_u8str(perf_str);

    // Restore previous font settings
    dwin_font.fg = fg;
    dwin_font.solid = solid;
  #endif // GAME_PERFORMANCE_COUNTERS
}

void MarlinGame::set_color(const color color) {
  switch (color) {
    default:
    case color::WHITE:  dwin_font.fg = COLOR_WHITE; break;
    case color::BLACK:  dwin_font.fg = COLOR_BG_BLACK; break;

    // https://rgbcolorpicker.com/565/table
    case color::RED:    dwin_font.fg = RGB(0x1F, 0x00, 0x00); break;
    case color::GREEN:  dwin_font.fg = RGB(0x00, 0x3F, 0x00); break;
    case color::BLUE:   dwin_font.fg = RGB(0x00, 0x00, 0x1F); break;
    case color::YELLOW: dwin_font.fg = RGB(0x1F, 0x3F, 0x00); break;
    case color::CYAN:   dwin_font.fg = RGB(0x00, 0x3F, 0x1F); break;
    case color::MAGENTA:dwin_font.fg = RGB(0x1F, 0x00, 0x1F); break;
  }
}

void MarlinGame::draw_hline(const game_dim_t x, const game_dim_t y, const game_dim_t w) {
  // Draw lines as boxes, since DWIN lines are always 1px wide but we want to scale them
  draw_box(x, y, w, 1);

  COUNT_DRAW_CALLS(1);
}

void MarlinGame::draw_vline(const game_dim_t x, const game_dim_t y, const game_dim_t h) {
  // Draw lines as boxes, since DWIN lines are always 1px wide but we want to scale them
  draw_box(x, y, 1, h);

  COUNT_DRAW_CALLS(1);
}

void MarlinGame::draw_frame(const game_dim_t x, const game_dim_t y, const game_dim_t w, const game_dim_t h) {
  dwinDrawBox(
    0, // mode = frame
    dwin_font.fg, // color
    dwin_game::game_to_screen(x) + dwin_game::x_offset,
    dwin_game::game_to_screen(y) + dwin_game::y_offset,
    dwin_game::game_to_screen(w),
    dwin_game::game_to_screen(h)
  );

  COUNT_DRAW_CALLS(1);
}

void MarlinGame::draw_box(const game_dim_t x, const game_dim_t y, const game_dim_t w, const game_dim_t h) {
  dwinDrawBox(
    1, // mode = fill
    dwin_font.fg, // color
    dwin_game::game_to_screen(x) + dwin_game::x_offset,
    dwin_game::game_to_screen(y) + dwin_game::y_offset,
    dwin_game::game_to_screen(w),
    dwin_game::game_to_screen(h)
  );

  COUNT_DRAW_CALLS(1);
}

void MarlinGame::draw_pixel(const game_dim_t x, const game_dim_t y) {
  // Draw pixels using boxes.
  // While DWIN protocol supports drawing points with different sizes, the
  // 0x02 'draw point' command is slower per pixel than 0x05 'fill rectangle'
  // (0.4 us vs 0.14 us per pixel)
  draw_box(x, y, 1, 1);
}

void MarlinGame::draw_bitmap(const game_dim_t x, const game_dim_t y, const game_dim_t bytes_per_row, const game_dim_t rows, const pgm_bitmap_t bitmap) {
  // DWIN theorethically supports bitmaps since kernel 2.1, but most screens don't support it
  // (either because they use an older kernel version, or because they just (badly) emulate the DWIN protocol).
  // So instead, we have to fall back to drawing points manually.

  #if DISABLED(TJC_DISPLAY)

    // DWIN T5UI actually supports drawing multiple points in one go using the 0x02 'draw point' command, ever since kernel 1.2.
    // So we use that to draw the bitmap as a series of points, which is faster than drawing rectangles using draw_pixel.
    dwinDrawPointMap(
      dwin_font.fg,
      dwin_game::game_to_screen(1),
      dwin_game::game_to_screen(1),
      dwin_game::game_to_screen(x) + dwin_game::x_offset,
      dwin_game::game_to_screen(y) + dwin_game::y_offset,
      bytes_per_row,
      rows,
      bitmap
    );

    COUNT_DRAW_CALLS(1);

  #else // TJC_DISPLAY

    // TJC displays don't seem to support the 0x02 'draw point' command, so instead we have to draw the bitmap
    // as a series of rectangles using draw_pixel.
    // This will absolutely suck for performance, but it's the best we can do on these screens.
    for (game_dim_t row = 0; row < rows; row++) {
      for (game_dim_t col = 0; col < bytes_per_row; col++) {
        const uint8_t byte = bitmap[(row * bytes_per_row) + col];
        for (uint8_t bit = 0; bit < 8; bit++) {
          // Assuming that the drawing area was cleared before drawing
          if (byte & (1 << bit)) {
            draw_pixel(x + (col * 8) + (7 - bit + 1), y + row);
            COUNT_DRAW_CALLS(1);
          }
        }
      }
    }

  #endif // TJC_DISPLAY
}

int MarlinGame::draw_string(const game_dim_t x, const game_dim_t y, const char* str) {
  COUNT_DRAW_CALLS(1);

  lcd_moveto_xy(
    dwin_game::game_to_screen(x) + dwin_game::x_offset,
    dwin_game::game_to_screen(y) + dwin_game::y_offset
  );

  return lcd_put_u8str_max_P(str, PIXEL_LEN_NOLIMIT);
}

int MarlinGame::draw_string(const game_dim_t x, const game_dim_t y, FSTR_P const str) {
  return draw_string(x, y, FTOP(str));
}

void MarlinGame::draw_int(const game_dim_t x, const game_dim_t y, const int value) {
  COUNT_DRAW_CALLS(1);

  lcd_moveto_xy(
    dwin_game::game_to_screen(x) + dwin_game::x_offset,
    dwin_game::game_to_screen(y) + dwin_game::y_offset
  );

  lcd_put_int(value);
}

#endif // IS_DWIN_MARLINUI && HAS_GAMES
