/*********************
 * bed_mesh_base.cpp *
 *********************/

/****************************************************************************
 *   Written By Marcio Teixeira 2020                                        *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#include "../config.h"
#include "../screens.h"

#ifdef FTDI_BED_MESH_BASE

using namespace FTDI;

void BedMeshBase::_drawMesh(CommandProcessor &cmd, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t opts, float autoscale_max, uint8_t highlightedTag, mesh_getter_ptr func, void *data) {
  constexpr uint8_t rows = GRID_MAX_POINTS_Y;
  constexpr uint8_t cols = GRID_MAX_POINTS_X;

  #define VALUE(X,Y)  (func ? func(X,Y,data) : 0)
  #define ISVAL(X,Y)  (func ? !isnan(VALUE(X,Y)) : true)
  #define HEIGHT(X,Y) (ISVAL(X,Y) ? (VALUE(X,Y) - val_min) * scale_z : 0)

  // Compute the mean, min and max for the points

  float   val_mean = 0;
  float   val_max  = -INFINITY;
  float   val_min  =  INFINITY;
  uint8_t val_cnt  = 0;

  if (opts & USE_AUTOSCALE) {
    for (uint_fast8_t y = 0; y < rows; y++) {
      for (uint_fast8_t x = 0; x < cols; x++) {
        if (ISVAL(x,y)) {
          const float val = VALUE(x,y);
          val_mean += val;
          val_max   = max(val_max, val);
          val_min   = min(val_min, val);
          val_cnt++;
        }
      }
    }
  }
  if (val_cnt)
    val_mean /= val_cnt;
  else {
    val_mean = 0;
    val_min  = 0;
    val_max  = 0;
  }

  const float scale_z = ((val_max == val_min) ? 1 : 1/(val_max - val_min)) * autoscale_max;

  /**
   * The 3D points go through a 3D graphics pipeline to determine the final 2D point on the screen.
   * This is written out as a stack of macros that each apply an affine transformation to the point.
   * At compile time, the compiler should be able to reduce these expressions.
   *
   * The last transformation in the chain (TRANSFORM_5) is initially set to a no-op so we can measure
   * the dimensions of the grid, but is later replaced with a scaling transform that scales the grid
   * to fit.
   */

  #define TRANSFORM_5(X,Y,Z)  (X), (Y)                                                                   // No transform
  #define TRANSFORM_4(X,Y,Z)  TRANSFORM_5((X)/(Z),(Y)/-(Z), 0)                                           // Perspective
  #define TRANSFORM_3(X,Y,Z)  TRANSFORM_4((X), (Z), (Y))                                                 // Swap Z and Y
  #define TRANSFORM_2(X,Y,Z)  TRANSFORM_3((X), (Y) + 2.5, (Z) - 1)                                       // Translate
  #define TRANSFORM(X,Y,Z)    TRANSFORM_2(float(X)/(cols-1) - 0.5, float(Y)/(rows-1)  - 0.5, (Z))        // Normalize

  // Compute the bounding box for the grid prior to scaling. Do this at compile-time by
  // transforming the four corner points via the transformation equations and finding
  // the min and max for each axis.

  constexpr float bounds[][3]  = {{TRANSFORM(0     , 0     , 0)},
                                  {TRANSFORM(cols-1, 0     , 0)},
                                  {TRANSFORM(0     , rows-1, 0)},
                                  {TRANSFORM(cols-1, rows-1, 0)}};
  #define APPLY(FUNC, AXIS) FUNC(FUNC(bounds[0][AXIS], bounds[1][AXIS]), FUNC(bounds[2][AXIS], bounds[3][AXIS]))
  constexpr float grid_x       = APPLY(min,0);
  constexpr float grid_y       = APPLY(min,1);
  constexpr float grid_w       = APPLY(max,0) - grid_x;
  constexpr float grid_h       = APPLY(max,1) - grid_y;
  constexpr float grid_cx      = grid_x + grid_w/2;
  constexpr float grid_cy      = grid_y + grid_h/2;

  // Figure out scale and offset such that the grid fits within the rectangle given by (x,y,w,h)

  const float scale_x          = float(w)/grid_w;
  const float scale_y          = float(h)/grid_h;
  const float center_x         = x + w/2;
  const float center_y         = y + h/2;

  // Now replace the last transformation in the chain with a scaling operation.

  #undef  TRANSFORM_5
  #define TRANSFORM_6(X,Y,Z)  (X)*16, (Y)*16                                                  // Scale to 1/16 pixel units
  #define TRANSFORM_5(X,Y,Z)  TRANSFORM_6( center_x + ((X) - grid_cx) * scale_x, \
                                           center_y + ((Y) - grid_cy) * scale_y, 0)           // Scale to bounds

  // Draw the grid

  const uint16_t basePointSize = min(w,h) / max(cols,rows);

  cmd.cmd(SAVE_CONTEXT())
     .cmd(TAG_MASK(false))
     .cmd(SAVE_CONTEXT());

  for (uint_fast8_t y = 0; y < rows; y++) {
    for (uint_fast8_t x = 0; x < cols; x++) {
      if (ISVAL(x,y)) {
       const bool hasLeftSegment  = x < cols - 1 && ISVAL(x+1,y);
       const bool hasRightSegment = y < rows - 1 && ISVAL(x,y+1);
       if (hasLeftSegment || hasRightSegment) {
         cmd.cmd(BEGIN(LINE_STRIP));
         if (hasLeftSegment)  cmd.cmd(VERTEX2F(TRANSFORM(x + 1, y    , HEIGHT(x + 1, y    ))));
         cmd.cmd(                     VERTEX2F(TRANSFORM(x    , y    , HEIGHT(x    , y    ))));
         if (hasRightSegment) cmd.cmd(VERTEX2F(TRANSFORM(x    , y + 1, HEIGHT(x    , y + 1))));
       }
      }
    }

    if (opts & USE_POINTS) {
      const float sq_min = sq(val_min - val_mean);
      const float sq_max = sq(val_max - val_mean);
      cmd.cmd(POINT_SIZE(basePointSize * 2));
      cmd.cmd(BEGIN(POINTS));
      for (uint_fast8_t x = 0; x < cols; x++) {
        if (ISVAL(x,y)) {
          if (opts & USE_COLORS) {
            const float val_dev = sq(VALUE(x, y) - val_mean);
            float r = 0, b = 0;
            if (sq_min != sq_max) {
              if (VALUE(x, y) < 0)
                r = val_dev / sq_min;
              else
                b = val_dev / sq_max;
            }
            #ifdef BED_MESH_POINTS_GRAY
              cmd.cmd(COLOR_RGB((1.0f - b + r) * 0x7F, (1.0f - b - r) * 0x7F, (1.0f - r + b) * 0x7F));
            #else
              cmd.cmd(COLOR_RGB((1.0f - b) * 0xFF, (1.0f - b - r) * 0xFF, (1.0f - r) * 0xFF));
            #endif
          }
          cmd.cmd(VERTEX2F(TRANSFORM(x, y, HEIGHT(x, y))));
        }
      }
      if (opts & USE_COLORS) {
        cmd.cmd(RESTORE_CONTEXT())
           .cmd(SAVE_CONTEXT());
      }
    }
  }
  cmd.cmd(RESTORE_CONTEXT())
     .cmd(TAG_MASK(true));

  if (opts & USE_TAGS) {
    cmd.cmd(COLOR_MASK(false, false, false, false))
       .cmd(POINT_SIZE(basePointSize * 10))
       .cmd(BEGIN(POINTS));
    for (uint_fast8_t y = 0; y < rows; y++) {
      for (uint_fast8_t x = 0; x < cols; x++) {
        const uint8_t tag = pointToTag(x, y);
        cmd.tag(tag).cmd(VERTEX2F(TRANSFORM(x, y, HEIGHT(x, y))));
      }
    }
    cmd.cmd(COLOR_MASK(true, true, true, true));
  }

  if (opts & USE_HIGHLIGHT) {
    const uint8_t tag = highlightedTag;
    xy_uint8_t pt;
    if (tagToPoint(tag, pt)) {
      cmd.cmd(COLOR_A(128))
         .cmd(POINT_SIZE(basePointSize * 6))
         .cmd(BEGIN(POINTS))
         .tag(tag).cmd(VERTEX2F(TRANSFORM(pt.x, pt.y, HEIGHT(pt.x, pt.y))));
    }
  }
  cmd.cmd(END());
  cmd.cmd(RESTORE_CONTEXT());
}

uint8_t BedMeshBase::pointToTag(uint8_t x, uint8_t y) {
  return x >= 0 && x < GRID_MAX_POINTS_X && y >= 0 && y < GRID_MAX_POINTS_Y ? y * (GRID_MAX_POINTS_X) + x + 10 : 0;
}

bool BedMeshBase::tagToPoint(uint8_t tag, xy_uint8_t &pt) {
  if (tag < 10) return false;
  pt.x = (tag - 10) % (GRID_MAX_POINTS_X);
  pt.y = (tag - 10) / (GRID_MAX_POINTS_X);
  return true;
}

void BedMeshBase::drawMeshBackground(CommandProcessor &cmd, int16_t x, int16_t y, int16_t w, int16_t h) {
  cmd.cmd(COLOR_RGB(Theme::bed_mesh_shadow_rgb));
  _drawMesh(cmd, x, y, w, h, USE_POINTS | USE_TAGS, 0.1, 0, nullptr, nullptr);
}

void BedMeshBase::drawMeshForeground(CommandProcessor &cmd, int16_t x, int16_t y, int16_t w, int16_t h, mesh_getter_ptr func, void *data, uint8_t highlightedTag, float progress) {
  constexpr float autoscale_max_amplitude = 0.03;

  cmd.cmd(COLOR_RGB(Theme::bed_mesh_lines_rgb));
  _drawMesh(cmd, x, y, w, h,
    USE_POINTS | USE_HIGHLIGHT | USE_AUTOSCALE | (progress > 0.95 ? USE_COLORS : 0),
    autoscale_max_amplitude * progress,
    highlightedTag,
    func, data
  );
}

#endif // FTDI_BED_MESH_BASE
