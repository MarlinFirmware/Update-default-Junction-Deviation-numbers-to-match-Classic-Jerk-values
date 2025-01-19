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

#if ENABLED(MARLIN_SNAKE)

#include "game.h"

#define SNAKE_BOX 4

#define HEADER_H  (GAME_FONT_ASCENT - 2)
#define SNAKE_WH  (SNAKE_BOX + 1)

#define IDEAL_L   2
#define IDEAL_R   (GAME_WIDTH - 1 - 2)
#define IDEAL_T   (HEADER_H + 2)
#define IDEAL_B   (GAME_HEIGHT - 1 - 2)
#define IDEAL_W   (IDEAL_R - (IDEAL_L) + 1)
#define IDEAL_H   (IDEAL_B - (IDEAL_T) + 1)

#define GAME_W    int((IDEAL_W) / (SNAKE_WH))
#define GAME_H    int((IDEAL_H) / (SNAKE_WH))

#define BOARD_W   ((SNAKE_WH) * (GAME_W) + 1)
#define BOARD_H   ((SNAKE_WH) * (GAME_H) + 1)
#define BOARD_L   ((GAME_WIDTH - (BOARD_W) + 1) / 2)
#define BOARD_R   (BOARD_L + BOARD_W - 1)
#define BOARD_T   (((GAME_HEIGHT + IDEAL_T) - (BOARD_H)) / 2)
#define BOARD_B   (BOARD_T + BOARD_H - 1)

#define GAMEX(X)  (BOARD_L + ((X) * (SNAKE_WH)))
#define GAMEY(Y)  (BOARD_T + ((Y) * (SNAKE_WH)))

#if SNAKE_BOX > 2
  #define FOOD_WH SNAKE_BOX
#else
  #define FOOD_WH 2
#endif

#if SNAKE_BOX < 1
  #define SNAKE_SIZ 1
#else
  #define SNAKE_SIZ SNAKE_BOX
#endif

constexpr fixed_t snakev = FTOF(0.20);

snake_data_t &sdat = marlin_game_data.snake;

// Remove the first pixel from the tail.
// If needed, shift out the first segment.
void shorten_tail() {
  pos_t &p = sdat.snake_tail[0], &q = sdat.snake_tail[1];
  bool shift = false;
  if (p.x == q.x) {
    // Vertical line
    p.y += (q.y > p.y) ? 1 : -1;
    shift = p.y == q.y;
  }
  else {
    // Horizontal line
    p.x += (q.x > p.x) ? 1 : -1;
    shift = p.x == q.x;
  }
  if (shift) {
    sdat.head_ind--;
    for (uint8_t i = 0; i <= sdat.head_ind; ++i)
      sdat.snake_tail[i] = sdat.snake_tail[i + 1];
  }
}

// The food is on a line
inline bool food_on_line() {
  for (uint8_t n = 0; n < sdat.head_ind; ++n) {
    pos_t &p = sdat.snake_tail[n], &q = sdat.snake_tail[n + 1];
    if (p.x == q.x) {
      if ((sdat.foodx == p.x - 1 || sdat.foodx == p.x) && WITHIN(sdat.foody, _MIN(p.y, q.y), _MAX(p.y, q.y)))
        return true;
    }
    else if ((sdat.foody == p.y - 1 || sdat.foody == p.y) && WITHIN(sdat.foodx, _MIN(p.x, q.x), _MAX(p.x, q.x)))
      return true;
  }
  return false;
}

// Add a new food blob
void food_reset() {
  do {
    sdat.foodx = random(0, GAME_W);
    sdat.foody = random(0, GAME_H);
  } while (food_on_line());
}

// Turn the snake cw or ccw
inline void turn_snake(const bool cw) {
  sdat.snake_dir += cw ? 1 : -1;
  sdat.snake_dir &= 0x03;
  sdat.head_ind++;
  sdat.snake_tail[sdat.head_ind].x = FTOB(sdat.snakex);
  sdat.snake_tail[sdat.head_ind].y = FTOB(sdat.snakey);
}

// Reset the snake for a new game
void snake_reset() {
  // Init the head and velocity
  sdat.snakex = BTOF(1);
  sdat.snakey = BTOF(GAME_H / 2);
  //snakev = FTOF(0.25);

  // Init the tail with a cw turn
  sdat.snake_dir = 0;
  sdat.head_ind = 0;
  sdat.snake_tail[0].x = 0;
  sdat.snake_tail[0].y = GAME_H / 2;
  turn_snake(true);

  // Clear food flag
  sdat.food_cnt = 5;

  // Clear the controls
  ui.encoderPosition = 0;
  sdat.old_encoder = 0;
}

// Check if head segment overlaps another
bool snake_overlap() {
  // 4 lines must exist before a collision is possible
  if (sdat.head_ind < 4) return false;
  // Is the last segment crossing any others?
  const pos_t &h1 = sdat.snake_tail[sdat.head_ind - 1], &h2 = sdat.snake_tail[sdat.head_ind];
  // VERTICAL head segment?
  if (h1.x == h2.x) {
    // Loop from oldest to segment two away from head
    for (uint8_t n = 0; n < sdat.head_ind - 2; ++n) {
      // Segment p to q
      const pos_t &p = sdat.snake_tail[n], &q = sdat.snake_tail[n + 1];
      if (p.x != q.x) {
        // Crossing horizontal segment
        if (WITHIN(h1.x, _MIN(p.x, q.x), _MAX(p.x, q.x)) && (h1.y <= p.y) == (h2.y >= p.y)) return true;
      } // Overlapping vertical segment
      else if (h1.x == p.x && _MIN(h1.y, h2.y) <= _MAX(p.y, q.y) && _MAX(h1.y, h2.y) >= _MIN(p.y, q.y)) return true;
    }
  }
  else {
    // Loop from oldest to segment two away from head
    for (uint8_t n = 0; n < sdat.head_ind - 2; ++n) {
      // Segment p to q
      const pos_t &p = sdat.snake_tail[n], &q = sdat.snake_tail[n + 1];
      if (p.y != q.y) {
        // Crossing vertical segment
        if (WITHIN(h1.y, _MIN(p.y, q.y), _MAX(p.y, q.y)) && (h1.x <= p.x) == (h2.x >= p.x)) return true;
      } // Overlapping horizontal segment
      else if (h1.y == p.y && _MIN(h1.x, h2.x) <= _MAX(p.x, q.x) && _MAX(h1.x, h2.x) >= _MIN(p.x, q.x)) return true;
    }
  }
  return false;
}

void SnakeGame::game_screen() {
  // Run the snake logic
  if (game_frame()) do {    // Run logic twice for finer resolution

    // Move the snake's head one unit in the current direction
    const int8_t oldx = FTOB(sdat.snakex), oldy = FTOB(sdat.snakey);
    switch (sdat.snake_dir) {
      case 0: sdat.snakey -= snakev; break;
      case 1: sdat.snakex += snakev; break;
      case 2: sdat.snakey += snakev; break;
      case 3: sdat.snakex -= snakev; break;
    }
    const int8_t x = FTOB(sdat.snakex), y = FTOB(sdat.snakey);

    // If movement took place...
    if (oldx != x || oldy != y) {

      if (!WITHIN(x, 0, GAME_W - 1) || !WITHIN(y, 0, GAME_H - 1)) {
        game_state = 0; // Game Over
        _BUZZ(400, 40); // Bzzzt!
        break;          // ...out of do-while
      }

      sdat.snake_tail[sdat.head_ind].x = x;
      sdat.snake_tail[sdat.head_ind].y = y;

      // Change snake direction if set
      const int8_t enc = int8_t(ui.encoderPosition), diff = enc - sdat.old_encoder;
      if (diff) {
        sdat.old_encoder = enc;
        turn_snake(diff > 0);
      }

      if (sdat.food_cnt) --sdat.food_cnt; else shorten_tail();

      // Did the snake collide with itself or go out of bounds?
      if (snake_overlap()) {
        game_state = 0; // Game Over
        _BUZZ(400, 40); // Bzzzt!
      }
      // Is the snake at the food?
      else if (x == sdat.foodx && y == sdat.foody) {
        _BUZZ(5, 220);
        _BUZZ(5, 280);
        score++;
        sdat.food_cnt = 2;
        food_reset();
      }
    }

  } while(0);

  frame_start();

  // Draw the snake (tail) in green
  TERN_(IS_DWIN_MARLINUI, set_color(color::GREEN));
  #if SNAKE_WH < 2

    // At this scale just draw a line
    for (uint8_t n = 0; n < sdat.head_ind; ++n) {
      const pos_t &p = sdat.snake_tail[n], &q = sdat.snake_tail[n + 1];
      if (p.x == q.x) {
        const int8_t y1 = GAMEY(_MIN(p.y, q.y)), y2 = GAMEY(_MAX(p.y, q.y));
        if (PAGE_CONTAINS(y1, y2))
          draw_vline(GAMEX(p.x), y1, y2 - y1 + 1);
      }
      else if (PAGE_CONTAINS(GAMEY(p.y), GAMEY(p.y))) {
        const int8_t x1 = GAMEX(_MIN(p.x, q.x)), x2 = GAMEX(_MAX(p.x, q.x));
        draw_hline(x1, GAMEY(p.y), x2 - x1 + 1);
      }
    }

  #elif SNAKE_WH == 2

    // At this scale draw two lines
    for (uint8_t n = 0; n < sdat.head_ind; ++n) {
      const pos_t &p = sdat.snake_tail[n], &q = sdat.snake_tail[n + 1];
      if (p.x == q.x) {
        const int8_t y1 = GAMEY(_MIN(p.y, q.y)), y2 = GAMEY(_MAX(p.y, q.y));
        if (PAGE_CONTAINS(y1, y2 + 1))
          draw_frame(GAMEX(p.x), y1, 2, y2 - y1 + 1 + 1);
      }
      else {
        const int8_t py = GAMEY(p.y);
        if (PAGE_CONTAINS(py, py + 1)) {
          const int8_t x1 = GAMEX(_MIN(p.x, q.x)), x2 = GAMEX(_MAX(p.x, q.x));
          draw_frame(x1, py, x2 - x1 + 1 + 1, 2);
        }
      }
    }

  #else

    // Draw a series of boxes
    for (uint8_t n = 0; n < sdat.head_ind; ++n) {
      const pos_t &p = sdat.snake_tail[n], &q = sdat.snake_tail[n + 1];
      if (p.x == q.x) {
        const int8_t y1 = _MIN(p.y, q.y), y2 = _MAX(p.y, q.y);
        if (PAGE_CONTAINS(GAMEY(y1), GAMEY(y2) + SNAKE_SIZ - 1)) {
          for (int8_t i = y1; i <= y2; ++i) {
            const int8_t y = GAMEY(i);
            if (PAGE_CONTAINS(y, y + SNAKE_SIZ - 1))
              draw_box(GAMEX(p.x), y, SNAKE_SIZ, SNAKE_SIZ);
          }
        }
      }
      else {
        const int8_t py = GAMEY(p.y);
        if (PAGE_CONTAINS(py, py + SNAKE_SIZ - 1)) {
          const int8_t x1 = _MIN(p.x, q.x), x2 = _MAX(p.x, q.x);
          for (int8_t i = x1; i <= x2; ++i)
            draw_box(GAMEX(i), py, SNAKE_SIZ, SNAKE_SIZ);
        }
      }
    }

  #endif

  // Draw food in red
  TERN_(IS_DWIN_MARLINUI, set_color(color::RED));
  const int8_t fy = GAMEY(sdat.foody);
  if (PAGE_CONTAINS(fy, fy + FOOD_WH - 1)) {
    const int8_t fx = GAMEX(sdat.foodx);
    draw_frame(fx, fy, FOOD_WH, FOOD_WH);
    if (FOOD_WH == 5) draw_pixel(fx + 2, fy + 2);
  }

  // Draw the playfield border
  TERN_(IS_DWIN_MARLINUI, set_color(color::WHITE));
  draw_frame(BOARD_L - 2, BOARD_T - 2, BOARD_R - BOARD_L + 4, BOARD_B - BOARD_T + 4);

  // Draw Score
  if (PAGE_UNDER(HEADER_H)) draw_int(0, HEADER_H - 1, score);

  // Draw GAME OVER
  if (!game_state) draw_game_over();

  // A click always exits this game
  if (ui.use_click()) exit_game();

  frame_end();
}

void SnakeGame::enter_game() {
  init_game(1, game_screen); // 1 = Game running
  snake_reset();
  food_reset();
}

#endif // MARLIN_SNAKE
