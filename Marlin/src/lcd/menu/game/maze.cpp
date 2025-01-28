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

#if ENABLED(MARLIN_MAZE)

#include "game.h"

int8_t move_dir, last_move_dir, // NESW0
       prizex, prizey, prize_cnt, old_encoder;
fixed_t playerx, playery;

// Up to 50 lines, then you win!
typedef struct { int8_t x, y; } pos_t;
uint8_t head_ind;
pos_t maze_walls[50] = {
  { 0, 0 }
};

// Turn the player cw or ccw
inline void turn_player(const bool cw) {
  if (move_dir == 4) move_dir = last_move_dir;
  move_dir += cw ? 1 : -1;
  move_dir &= 0x03;
  last_move_dir = move_dir;
}

// Reset the player for a new game
void player_reset() {
  // Init position
  playerx = BTOF(1);
  playery = BTOF(GAME_H / 2);

  // Init motion with a ccw turn
  move_dir = 0;
  turn_player(false);

  // Clear prize flag
  prize_cnt = 255;

  // Clear the controls
  ui.encoderPosition = 0;
  old_encoder = 0;
}

void MazeGame::game_screen() {
  // Run the sprite logic
  if (game_frame()) do {     // Run logic twice for finer resolution

    // Move the man one unit in the current direction
    // Direction index 4 is for the stopped man
    const int8_t oldx = FTOB(playerx), oldy = FTOB(playery);
    pos_t dir_add[] = { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, 0 } };
    playerx += dir_add[move_dir].x;
    playery += dir_add[move_dir].y;
    const int8_t x = FTOB(playerx), y = FTOB(playery);

  } while(0);

  u8g.setColorIndex(1);

  // Draw Score
  if (PAGE_UNDER(HEADER_H)) lcd_put_int(0, HEADER_H - 1, score);

  // Draw the maze
  // for (uint_fast8_t n = 0; n < head_ind; ++n) {
  //   const pos_t &p = maze_walls[n], &q = maze_walls[n + 1];
  //   if (p.x == q.x) {
  //     const int8_t y1 = GAMEY(_MIN(p.y, q.y)), y2 = GAMEY(_MAX(p.y, q.y));
  //     if (PAGE_CONTAINS(y1, y2))
  //       u8g.drawVLine(GAMEX(p.x), y1, y2 - y1 + 1);
  //   }
  //   else if (PAGE_CONTAINS(GAMEY(p.y), GAMEY(p.y))) {
  //     const int8_t x1 = GAMEX(_MIN(p.x, q.x)), x2 = GAMEX(_MAX(p.x, q.x));
  //     u8g.drawHLine(x1, GAMEY(p.y), x2 - x1 + 1);
  //   }
  // }

  // Draw Man
  // const int8_t fy = GAMEY(foody);
  // if (PAGE_CONTAINS(fy, fy + FOOD_WH - 1)) {
  //   const int8_t fx = GAMEX(foodx);
  //   u8g.drawFrame(fx, fy, FOOD_WH, FOOD_WH);
  //   if (FOOD_WH == 5) u8g.drawPixel(fx + 2, fy + 2);
  // }

  // Draw Ghosts
  // const int8_t fy = GAMEY(foody);
  // if (PAGE_CONTAINS(fy, fy + FOOD_WH - 1)) {
  //   const int8_t fx = GAMEX(foodx);
  //   u8g.drawFrame(fx, fy, FOOD_WH, FOOD_WH);
  //   if (FOOD_WH == 5) u8g.drawPixel(fx + 2, fy + 2);
  // }

  // Draw Prize
  // if (PAGE_CONTAINS(prizey, prizey + PRIZE_WH - 1)) {
  //   u8g.drawFrame(prizex, prizey, PRIZE_WH, PRIZE_WH);
  //   if (PRIZE_WH == 5) u8g.drawPixel(prizex + 2, prizey + 2);
  // }

  // Draw GAME OVER
  if (!game_state) draw_game_over();

  // A click always exits this game
  if (ui.use_click()) exit_game();
}

void MazeGame::enter_game() {
  init_game(1, game_screen); // Game running
  reset_player();
  reset_enemies();
}

#endif // MARLIN_MAZE
