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
#include "types.h"

//#define MUTE_GAMES

#if ENABLED(MUTE_GAMES) || !HAS_SOUND
  #define _BUZZ(D,F) NOOP
#else
  #define _BUZZ(D,F) BUZZ(D,F)
#endif

#if HAS_GAME_MENU
  void menu_game();
#endif

#if ENABLED(MARLIN_BRICKOUT)
  #include "brickout.h"
#endif
#if ENABLED(MARLIN_INVADERS)
  #include "invaders.h"
#endif
#if ENABLED(MARLIN_MAZE)
  #include "maze.h"
#endif
#if ENABLED(MARLIN_SNAKE)
  #include "snake.h"
#endif

// Pool game data to save SRAM
union MarlinGameData {
  #if ENABLED(MARLIN_BRICKOUT)
    brickout_data_t brickout;
  #endif
  #if ENABLED(MARLIN_INVADERS)
    invaders_data_t invaders;
  #endif
  #if ENABLED(MARLIN_SNAKE)
    snake_data_t snake;
  #endif
  #if ENABLED(MARLIN_MAZE)
    maze_data_t maze;
  #endif
};

extern MarlinGameData marlin_game_data;
