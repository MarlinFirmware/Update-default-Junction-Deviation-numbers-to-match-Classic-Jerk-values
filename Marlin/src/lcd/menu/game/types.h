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

#include "../../../inc/MarlinConfigPre.h"
#include "../../marlinui.h"

#if HAS_MARLINUI_U8GLIB
  #include "../../dogm/game.h"
#elif IS_DWIN_MARLINUI
  #include "../../e3v2/marlinui/game.h"
#endif

typedef struct { int8_t x, y; } pos_t;

// Simple 8:8 fixed-point
typedef int16_t fixed_t;
#define FTOF(F) fixed_t((F)*256.0f)
#define PTOF(P) (float(P)*(1.0f/256.0f))
#define BTOF(X) (fixed_t(X)<<8)
#define FTOB(X) int8_t(fixed_t(X)>>8)

class MarlinGame {
protected:
  static int score;
  static uint8_t game_state;
  static millis_t next_frame;

  static bool game_frame();
  static void draw_game_over();
  static void exit_game();

public:
  static void init_game(const uint8_t init_state, const screenFunc_t screen);

  // Game rendering API, based on U8glib's API.
  // See the @see comments for the U8glib API documentation corresponding to each function.

  /**
   * @brief The colors available for drawing games.
   * @note Renderer implementations will map these colors to the closest
   *       available color on the screen, as long as that color is not black.
   *       Thus, black is guranteed to be black on all screens, but other colors may differ.
   *       On black-and-white screens, all colors but black will be white.
   */
  enum class color {
    BLACK, WHITE
    #if IS_DWIN_MARLINUI
      , RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA
    #endif
  };

protected:
  /**
   * @brief Called before any draw calls in the current frame.
   */
  static void frame_start();

  /**
   * @brief Called after all draw calls in the current frame.
   */
  static void frame_end();

  /**
   * @brief Set the color for subsequent draw calls.
   * @param color The color to use for subsequent draw calls.
   */
  static void set_color(const color color);

  /**
   * @brief Draw a horizontal line.
   * @param x The x-coordinate of the start of the line.
   * @param y The y-coordinate of the line.
   * @param l The length of the line.
   * @see https://github.com/olikraus/u8glib/wiki/userreference#drawhline
   */
  static void draw_hline(const game_dim_t x, const game_dim_t y, const game_dim_t l);

  /**
   * @brief Draw a vertical line.
   * @param x The x-coordinate of the line.
   * @param y The y-coordinate of the start of the line.
   * @param l The length of the line.
   * @see https://github.com/olikraus/u8glib/wiki/userreference#drawvline
   */
  static void draw_vline(const game_dim_t x, const game_dim_t y, const game_dim_t l);

  /**
   * @brief Draw a outlined rectangle (frame).
   * @param x The x-coordinate of the top-left corner of the frame.
   * @param y The y-coordinate of the top-left corner of the frame.
   * @param w The width of the frame.
   * @param h The height of the frame.
   * @see https://github.com/olikraus/u8glib/wiki/userreference#drawframe
   */
  static void draw_frame(const game_dim_t x, const game_dim_t y, const game_dim_t w, const game_dim_t h);

  /**
   * @brief Draw a filled rectangle (box).
   * @param x The x-coordinate of the top-left corner of the box.
   * @param y The y-coordinate of the top-left corner of the box.
   * @param w The width of the box.
   * @param h The height of the box.
   * @see https://github.com/olikraus/u8glib/wiki/userreference#drawbox
   */
  static void draw_box(const game_dim_t x, const game_dim_t y, const game_dim_t w, const game_dim_t h);

  /**
   * @brief Draw a pixel.
   * @param x The x-coordinate of the pixel.
   * @param y The y-coordinate of the pixel.
   * @see https://github.com/olikraus/u8glib/wiki/userreference#drawpixel
   */
  static void draw_pixel(const game_dim_t x, const game_dim_t y);

  /**
   * @brief Draw a bitmap.
   * @param x The x-coordinate of the top-left corner of the bitmap.
   * @param y The y-coordinate of the top-left corner of the bitmap.
   * @param bytes_per_row The number of bytes per row in the bitmap (Width = bytes_per_row * 8).
   * @param rows The number of rows in the bitmap (= Height).
   * @param bitmap The bitmap to draw.
   * @see https://github.com/olikraus/u8glib/wiki/userreference#drawbitmap
   */
  static void draw_bitmap(const game_dim_t x, const game_dim_t y, const game_dim_t bytes_per_row, const game_dim_t rows, const pgm_bitmap_t bitmap);

  /**
   * @brief Draw a string.
   * @param x The x-coordinate of the string.
   * @param y The y-coordinate of the string.
   * @param str The string to draw.
   * @see lcd_moveto + lcd_put_u8str
   * @note The font size is available using the GAME_FONT_WIDTH and GAME_FONT_ASCENT constants.
   *
   * @note On the DWIN renderer, strings may flush the screen, which may cause flickering.
   *       Consider drawing strings after all other elements have been drawn.
   */
  static int draw_string(const game_dim_t x, const game_dim_t y, const char *str);
  static int draw_string(const game_dim_t x, const game_dim_t y, FSTR_P const str);

  /**
   * @brief Draw an integer.
   * @param x The x-coordinate of the integer.
   * @param y The y-coordinate of the integer.
   * @param value The integer to draw.
   * @see lcd_put_int
   * @note The font size is available using the GAME_FONT_WIDTH and GAME_FONT_ASCENT constants.
   *
   * @note On the DWIN renderer, strings may flush the screen, which may cause flickering.
   *       Consider drawing strings after all other elements have been drawn.
   */
  static void draw_int(const game_dim_t x, const game_dim_t y, const int value);
};
