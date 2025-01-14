/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../core/types.h"

enum ftMotionShaper_t : uint8_t {
  ftMotionShaper_NONE  = 0, // No compensator
  ftMotionShaper_ZV    = 1, // Zero Vibration
  ftMotionShaper_ZVD   = 2, // Zero Vibration and Derivative
  ftMotionShaper_ZVDD  = 3, // Zero Vibration, Derivative, and Double Derivative
  ftMotionShaper_ZVDDD = 4, // Zero Vibration, Derivative, Double Derivative, and Triple Derivative
  ftMotionShaper_EI    = 5, // Extra-Intensive
  ftMotionShaper_2HEI  = 6, // 2-Hump Extra-Intensive
  ftMotionShaper_3HEI  = 7, // 3-Hump Extra-Intensive
  ftMotionShaper_MZV   = 8  // Modified Zero Vibration
};

enum dynFreqMode_t : uint8_t {
  dynFreqMode_DISABLED   = 0,
  dynFreqMode_Z_BASED    = 1,
  dynFreqMode_MASS_BASED = 2
};

#define AXIS_HAS_SHAPER(A)   (ftMotion.cfg.shaper[_AXIS(A)] != ftMotionShaper_NONE)
#define AXIS_HAS_EISHAPER(A) WITHIN(ftMotion.cfg.shaper[_AXIS(A)], ftMotionShaper_EI, ftMotionShaper_3HEI)

typedef struct XYZEarray<float, FTM_WINDOW_SIZE> xyze_trajectory_t;
typedef struct XYZEarray<float, FTM_BATCH_SIZE> xyze_trajectoryMod_t;

// TODO: Convert ft_command_t to a struct with bitfields instead of using a primitive type
enum {
  LIST_N(DOUBLE(LOGICAL_AXES),
    FT_BIT_DIR_E, FT_BIT_STEP_E,
    FT_BIT_DIR_X, FT_BIT_STEP_X, FT_BIT_DIR_Y, FT_BIT_STEP_Y, FT_BIT_DIR_Z, FT_BIT_STEP_Z,
    FT_BIT_DIR_I, FT_BIT_STEP_I, FT_BIT_DIR_J, FT_BIT_STEP_J, FT_BIT_DIR_K, FT_BIT_STEP_K,
    FT_BIT_DIR_U, FT_BIT_STEP_U, FT_BIT_DIR_V, FT_BIT_STEP_V, FT_BIT_DIR_W, FT_BIT_STEP_W
  ),
  FT_BIT_COUNT
};

#if HAS_FTM_SHAPING
  #define NUM_AXES_SHAPED TERN(HAS_Y_AXIS, 2, 1)
  #define SHAPED_ELEM(A, B) A OPTARG(HAS_Y_AXIS, B)
#else
  #define NUM_AXES_SHAPED 0
  #define SHAPED_ELEM(A, B)
#endif

template<typename T>
struct FTShapedAxes {
  union {
    struct { T SHAPED_ELEM(X, Y); };
    struct { T SHAPED_ELEM(x, y); };
    T val[NUM_AXES_SHAPED];
  };
  T& operator[](int i) { return val[i]; }
};

typedef FTShapedAxes<float>            ft_shaped_float_t;
typedef FTShapedAxes<ftMotionShaper_t> ft_shaped_shaper_t;
typedef FTShapedAxes<dynFreqMode_t>    ft_shaped_dfm_t;

typedef bits_t(FT_BIT_COUNT) ft_command_t;
