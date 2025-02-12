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

#if ENABLED(MARLIN_INVADERS)

#include "game.h"

#define CANNON_W      11
#define CANNON_H       8
#define CANNON_VEL     4
#define CANNON_Y      (GAME_HEIGHT - 1 - CANNON_H)

#define INVADER_VEL    3

#define INVADER_TOP   GAME_FONT_ASCENT
#define INVADERS_WIDE ((INVADER_COL_W) * (INVADER_COLS))
#define INVADERS_HIGH ((INVADER_ROW_H) * (INVADER_ROWS))

#define UFO_H          5
#define UFO_W         13

#define LASER_H        4
#define SHOT_H         3
#define EXPL_W        11
#define LIFE_W         8
#define LIFE_H         5

#define INVADER_RIGHT ((INVADER_COLS) * (INVADER_COL_W))

#if IS_DWIN_MARLINUI
  #define INVADER_COLOR { MarlinGame::color::GREEN, MarlinGame::color::CYAN, MarlinGame::color::YELLOW }
  #define CANNON_COLOR  MarlinGame::color::WHITE
  #define LASER_COLOR   MarlinGame::color::WHITE  // Shot by player
  #define BULLET_COLOR  LASER_COLOR               // Shot by invader
  #define LIFE_COLOR    CANNON_COLOR
  #define UFO_COLOR MarlinGame::color::MAGENTA
  #define EXPLOSION_COLOR MarlinGame::color::RED
#endif

// 11x8
const unsigned char invader[3][2][16] PROGMEM = {
  { { B00000110,B00000000,
      B00001111,B00000000,
      B00011111,B10000000,
      B00110110,B11000000,
      B00111111,B11000000,
      B00001001,B00000000,
      B00010110,B10000000,
      B00101001,B01000000
    }, {
      B00000110,B00000000,
      B00001111,B00000000,
      B00011111,B10000000,
      B00110110,B11000000,
      B00111111,B11000000,
      B00010110,B10000000,
      B00100000,B01000000,
      B00010000,B10000000
    }
  }, {
    { B00010000,B01000000,
      B00001000,B10000000,
      B00011111,B11000000,
      B00110111,B01100000,
      B01111111,B11110000,
      B01011111,B11010000,
      B01010000,B01010000,
      B00001101,B10000000
    }, {
      B00010000,B01000000,
      B01001000,B10010000,
      B01011111,B11010000,
      B01110111,B01110000,
      B01111111,B11110000,
      B00011111,B11000000,
      B00010000,B01000000,
      B00100000,B00100000
    }
  }, {
    { B00001111,B00000000,
      B01111111,B11100000,
      B11111111,B11110000,
      B11100110,B01110000,
      B11111111,B11110000,
      B00011001,B10000000,
      B00110110,B11000000,
      B11000000,B00110000
    }, {
      B00001111,B00000000,
      B01111111,B11100000,
      B11111111,B11110000,
      B11100110,B01110000,
      B11111111,B11110000,
      B00011001,B10000000,
      B00110110,B11000000,
      B00011001,B10000000
    }
  }
};
const unsigned char cannon[] PROGMEM = {
  B00000100,B00000000,
  B00001110,B00000000,
  B00001110,B00000000,
  B01111111,B11000000,
  B11111111,B11100000,
  B11111111,B11100000,
  B11111111,B11100000,
  B11111111,B11100000
};
const unsigned char life[] PROGMEM = {
  B00010000,
  B01111100,
  B11111110,
  B11111110,
  B11111110
};
const unsigned char explosion[] PROGMEM = {
  B01000100,B01000000,
  B00100100,B10000000,
  B00000000,B00000000,
  B00110001,B10000000,
  B00000000,B00000000,
  B00100100,B10000000,
  B01000100,B01000000
};
const unsigned char ufo[] PROGMEM = {
  B00011111,B11000000,
  B01111111,B11110000,
  B11011101,B11011000,
  B11111111,B11111000,
  B01111111,B11110000
};

constexpr uint8_t inv_type[] = {
  #if INVADER_ROWS == 5
    0, 1, 1, 2, 2
  #elif INVADER_ROWS == 4
    0, 1, 1, 2
  #elif INVADER_ROWS == 3
    0, 1, 2
  #else
    #error "INVASION_SIZE must be 3, 4, or 5."
  #endif
};

invaders_data_t &idat = marlin_game_data.invaders;

#define INV_X_LEFT(C,T) (idat.pos.x + (C) * (INVADER_COL_W) + inv_off[T])
#define INV_X_CTR(C,T)  (INV_X_LEFT(C,T) + inv_wide[T] / 2)
#define INV_Y_BOT(R)    (idat.pos.y + (R + 1) * (INVADER_ROW_H) - 2)

constexpr uint8_t inv_off[] = { 2, 1, 0 }, inv_wide[] = { 8, 11, 12 };

inline void update_invader_data() {
  uint8_t inv_mask = 0;
  // Get a list of all active invaders
  uint8_t sc = 0;
  for (uint8_t y = 0; y < INVADER_ROWS; ++y) {
    uint8_t m = idat.bugs[y];
    if (m) idat.botmost = y + 1;
    inv_mask |= m;
    for (uint8_t x = 0; x < INVADER_COLS; ++x)
      if (TEST(m, x)) idat.shooters[sc++] = (y << 4) | x;
  }
  idat.leftmost = 0;
  for (uint8_t i = 0; i < INVADER_COLS; ++i) { if (TEST(inv_mask, i)) break; idat.leftmost -= INVADER_COL_W; }
  idat.rightmost = GAME_WIDTH - (INVADERS_WIDE);
  for (uint8_t i = INVADER_COLS; i--;) { if (TEST(inv_mask, i)) break; idat.rightmost += INVADER_COL_W; }
  if (idat.count == 2) idat.dir = idat.dir > 0 ? INVADER_VEL + 1 : -(INVADER_VEL + 1);
}

inline void reset_bullets() {
  for (uint8_t i = 0; i < COUNT(idat.bullet); ++i) idat.bullet[i].v = 0;
}

inline void reset_invaders() {
  idat.pos.x = 0; idat.pos.y = INVADER_TOP;
  idat.dir = INVADER_VEL;
  idat.count = (INVADER_COLS) * (INVADER_ROWS);
  for (uint8_t i = 0; i < INVADER_ROWS; ++i) idat.bugs[i] = _BV(INVADER_COLS) - 1;
  update_invader_data();
  reset_bullets();
}

inline void spawn_ufo() {
  idat.ufov = random(0, 2) ? 1 : -1;
  idat.ufox = idat.ufov > 0 ? -(UFO_W) : GAME_WIDTH - 1;
}

inline void reset_player() {
  idat.cannon_x = 0;
  ui.encoderPosition = 0;
}

inline void fire_cannon() {
  idat.laser.x = idat.cannon_x + CANNON_W / 2;
  idat.laser.y = GAME_HEIGHT - CANNON_H - (LASER_H);
  idat.laser.v = -(LASER_H);
}

inline void explode(const int8_t x, const int8_t y, const int8_t v=4) {
  idat.explod.x = x - (EXPL_W) / 2;
  idat.explod.y = y;
  idat.explod.v = v;
}

inline void kill_cannon(uint8_t &game_state, const uint8_t st) {
  reset_bullets();
  explode(idat.cannon_x + (CANNON_W) / 2, CANNON_Y, 6);
  _BUZZ(1000, 10);
  if (--idat.cannons_left) {
    idat.laser.v = 0;
    game_state = st;
    reset_player();
  }
  else
    game_state = 0;
}

void InvadersGame::game_screen() {
  ui.refresh(LCDVIEW_CALL_NO_REDRAW); // Call as often as possible

  // Run game logic once per full screen
  if (ui.first_page) {

    // Update Cannon Position
    int16_t ep = constrain(int16_t(ui.encoderPosition), 0, (GAME_WIDTH - (CANNON_W)) / (CANNON_VEL));
    ui.encoderPosition = ep;

    ep *= (CANNON_VEL);
    if (ep > idat.cannon_x) { idat.cannon_x += CANNON_VEL - 1; if (ep - idat.cannon_x < 2) idat.cannon_x = ep; }
    if (ep < idat.cannon_x) { idat.cannon_x -= CANNON_VEL - 1; if (idat.cannon_x - ep < 2) idat.cannon_x = ep; }

    // Run the game logic
    if (game_state) do {

      // Move the UFO, if any
      if (idat.ufov) { idat.ufox += idat.ufov; if (!WITHIN(idat.ufox, -(UFO_W), GAME_WIDTH - 1)) idat.ufov = 0; }

      if (game_state > 1) { if (--game_state == 2) { reset_invaders(); } else if (game_state == 100) { game_state = 1; } break; }

      const bool did_blink = (++idat.blink_count > idat.count >> 1);
      if (did_blink) {
        FLIP(idat.game_blink);
        idat.blink_count = 0;
      }

      if (idat.count && did_blink) {
        const int8_t newx = idat.pos.x + idat.dir;
        if (!WITHIN(newx, idat.leftmost, idat.rightmost)) { // Invaders reached the edge?
          idat.dir *= -1;                                   // Invaders change direction
          idat.pos.y += (INVADER_ROW_H) / 2;                        // Invaders move down
          idat.pos.x -= idat.dir;                           // ...and only move down this time.
          if (idat.pos.y + idat.botmost * (INVADER_ROW_H) - 2 >= CANNON_Y) // Invaders reached the bottom?
            kill_cannon(game_state, 20);                    // Kill the cannon. Reset invaders.
        }

        idat.pos.x += idat.dir; // Invaders take one step left/right

        // Randomly shoot if invaders are listed
        if (idat.count && !random(0, 20)) {

          // Find a free bullet
          laser_t *b = nullptr;
          for (uint8_t i = 0; i < COUNT(idat.bullet); ++i) if (!idat.bullet[i].v) { b = &idat.bullet[i]; break; }
          if (b) {
            // Pick a random shooter and update the bullet
            //SERIAL_ECHOLNPGM("free bullet found");
            const uint8_t inv = idat.shooters[random(0, idat.count + 1)], col = inv & 0x0F, row = inv >> 4, type = inv_type[row];
            b->x = INV_X_CTR(col, type);
            b->y = INV_Y_BOT(row);
            b->v = 2 + random(0, 2);
          }
        }
      }

      // Update the laser position
      if (idat.laser.v) {
        idat.laser.y += idat.laser.v;
        if (idat.laser.y < 0) idat.laser.v = 0;
      }

      // Did the laser collide with an invader?
      if (idat.laser.v && WITHIN(idat.laser.y, idat.pos.y, idat.pos.y + INVADERS_HIGH - 1)) {
        const int8_t col = idat.laser_col();
        if (WITHIN(col, 0, INVADER_COLS - 1)) {
          const int8_t row = idat.laser_row();
          if (WITHIN(row, 0, INVADER_ROWS - 1)) {
            const uint8_t mask = _BV(col);
            if (idat.bugs[row] & mask) {
              const uint8_t type = inv_type[row];
              const int8_t invx = INV_X_LEFT(col, type);
              if (WITHIN(idat.laser.x, invx, invx + inv_wide[type] - 1)) {
                // Turn off laser
                idat.laser.v = 0;
                // Remove the invader!
                idat.bugs[row] &= ~mask;
                // Score!
                score += INVADER_ROWS - row;
                // Explode sound!
                _BUZZ(40, 10);
                // Explosion bitmap!
                explode(invx + inv_wide[type] / 2, idat.pos.y + row * (INVADER_ROW_H));
                // If invaders are gone, go to reset invaders state
                if (--idat.count) update_invader_data(); else { game_state = 20; reset_bullets(); }
              } // laser x hit
            } // invader exists
          } // good row
        } // good col
      } // laser in invader zone

      // Handle alien bullets
      for (uint8_t s = 0; s < COUNT(idat.bullet); ++s) {
        laser_t *b = &idat.bullet[s];
        if (b->v) {
          // Update alien bullet position
          b->y += b->v;
          if (b->y >= GAME_HEIGHT)
            b->v = 0; // Offscreen
          else if (b->y >= CANNON_Y && WITHIN(b->x, idat.cannon_x, idat.cannon_x + CANNON_W - 1))
            kill_cannon(game_state, 120); // Hit the cannon
        }
      }

      // Randomly spawn a UFO
      if (!idat.ufov && !random(0,500)) spawn_ufo();

      // Did the laser hit a ufo?
      if (idat.laser.v && idat.ufov && idat.laser.y < UFO_H + 2 && WITHIN(idat.laser.x, idat.ufox, idat.ufox + UFO_W - 1)) {
        // Turn off laser and UFO
        idat.laser.v = idat.ufov = 0;
        // Score!
        score += 10;
        // Explode!
        _BUZZ(40, 10);
        // Explosion bitmap
        explode(idat.ufox + (UFO_W) / 2, 1);
      }

    } while (false);

  }

  // Click-and-hold to abort
  if (ui.button_pressed()) --idat.quit_count; else idat.quit_count = 10;

  // Click to fire or exit
  if (ui.use_click()) {
    if (!game_state)
      idat.quit_count = 0;
    else if (game_state == 1 && !idat.laser.v)
      fire_cannon();
  }

  if (!idat.quit_count) exit_game();

  frame_start();

  // Draw invaders
  if (PAGE_CONTAINS(idat.pos.y, idat.pos.y + idat.botmost * (INVADER_ROW_H) - 2 - 1)) {
    int8_t yy = idat.pos.y;
    for (uint8_t y = 0; y < INVADER_ROWS; ++y) {
      const uint8_t type = inv_type[y];
      if (PAGE_CONTAINS(yy, yy + INVADER_H - 1)) {
        int8_t xx = idat.pos.x;
        for (uint8_t x = 0; x < INVADER_COLS; ++x) {
          if (TEST(idat.bugs[y], x)) {
            #if IS_DWIN_MARLINUI
              constexpr color invader_color[] = INVADER_COLOR;
              set_color(invader_color[type]);
            #endif
            draw_bitmap(xx, yy, 2, INVADER_H, invader[type][idat.game_blink]);
          }
          xx += INVADER_COL_W;
        }
      }
      yy += INVADER_ROW_H;
    }
  }

  // Draw UFO
  if (idat.ufov && PAGE_UNDER(UFO_H + 2)) {
    TERN_(IS_DWIN_MARLINUI, set_color(UFO_COLOR));
    draw_bitmap(idat.ufox, 2, 2, UFO_H, ufo);
  }

  // Draw cannon
  if (game_state && PAGE_CONTAINS(CANNON_Y, CANNON_Y + CANNON_H - 1) && (game_state < 2 || (game_state & 0x02))) {
    TERN_(IS_DWIN_MARLINUI, set_color(CANNON_COLOR));
    draw_bitmap(idat.cannon_x, CANNON_Y, 2, CANNON_H, cannon);
  }

  // Draw laser
  if (idat.laser.v && PAGE_CONTAINS(idat.laser.y, idat.laser.y + LASER_H - 1)) {
    TERN_(IS_DWIN_MARLINUI, set_color(LASER_COLOR));
    draw_vline(idat.laser.x, idat.laser.y, LASER_H);
  }

  // Draw invader bullets
  for (uint8_t i = 0; i < COUNT(idat.bullet); ++i) {
    if (idat.bullet[i].v && PAGE_CONTAINS(idat.bullet[i].y - (SHOT_H - 1), idat.bullet[i].y)) {
      TERN_(IS_DWIN_MARLINUI, set_color(BULLET_COLOR));
      draw_vline(idat.bullet[i].x, idat.bullet[i].y - (SHOT_H - 1), SHOT_H);
    }
  }

  // Draw explosion
  if (idat.explod.v && PAGE_CONTAINS(idat.explod.y, idat.explod.y + 7 - 1)) {
    TERN_(IS_DWIN_MARLINUI, set_color(EXPLOSION_COLOR));
    draw_bitmap(idat.explod.x, idat.explod.y, 2, 7, explosion);
    --idat.explod.v;
  }

  // Everything else is white
  TERN_(IS_DWIN_MARLINUI, set_color(color::WHITE));

  // Blink GAME OVER when game is over
  if (!game_state) draw_game_over();

  if (PAGE_UNDER(GAME_FONT_ASCENT - 1)) {
    // Draw lives
    if (idat.cannons_left)
      for (uint8_t i = 1; i <= idat.cannons_left; ++i) {
        TERN_(IS_DWIN_MARLINUI, set_color(LIFE_COLOR));
        draw_bitmap(GAME_WIDTH - i * (LIFE_W), 6 - (LIFE_H), 1, LIFE_H, life);
      }

    // Draw Score
    //const uint8_t sx = (GAME_WIDTH - (score >= 10 ? score >= 100 ? score >= 1000 ? 4 : 3 : 2 : 1) * GAME_FONT_WIDTH) / 2;
    constexpr uint8_t sx = 0;
    draw_int(sx, GAME_FONT_ASCENT - 1, score);
  }

  frame_end();
}

void InvadersGame::enter_game() {
  init_game(20, game_screen); // countdown to reset invaders
  idat.cannons_left = 3;
  idat.quit_count = 10;
  idat.laser.v = 0;
  reset_invaders();
  reset_player();
}

#endif // MARLIN_INVADERS
