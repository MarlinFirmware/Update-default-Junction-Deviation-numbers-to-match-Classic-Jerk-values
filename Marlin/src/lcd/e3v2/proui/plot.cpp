/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * DWIN Single var plot
 * Author: Miguel A. Risco-Castillo
 * Version: 2.2.3
 * Date: 2023/01/29
 */

#include "../../../inc/MarlinConfig.h"

#if ENABLED(DWIN_LCD_PROUI)

#include "dwin_defines.h"

#if PROUI_TUNING_GRAPH

#include "dwin.h"
#include "../../marlinui.h"
#include "plot.h"

#define plotBgColor RGB(1, 12, 8)

Plot plot;

Plot::PlotData Plot::data;

void Plot::draw(const frame_rect_t &frame, const_celsius_float_t max, const_float_t ref/*=0*/) {
  data.graphframe = frame;
  data.graphpoints = 0;
  data.scale = frame.h / max;
  data.x2 = frame.x + frame.w - 1;
  data.y2 = frame.y + frame.h - 1;
  data.r = round((data.y2) - ref * data.scale);
  DWINUI::drawBox(1, plotBgColor, frame);
  for (uint_fast8_t i = 1; i < 4; i++) if (i * 60 < frame.w) dwinDrawVLine(COLOR_LINE, i * 60 + frame.x, frame.y, frame.h);
  DWINUI::drawBox(0, COLOR_WHITE, DWINUI::extendFrame(frame, 1));
  dwinDrawHLine(COLOR_RED, frame.x, data.r, frame.w);
}

void Plot::update(const_celsius_float_t value) {
  if (!data.scale) return;
  const uint16_t y = LROUND((data.y2) - value * data.scale);
  if (data.graphpoints < data.graphframe.w) {
    if (data.graphpoints < 1)
      dwinDrawPoint(COLOR_YELLOW, 1, 1, data.graphframe.x, y);
    else
      dwinDrawLine(COLOR_YELLOW, data.graphpoints + data.graphframe.x - 1, data.yP, data.graphpoints + data.graphframe.x, y);
  }
  else {
    dwinFrameAreaMove(1, 0, 1, plotBgColor, data.graphframe.x, data.graphframe.y, data.x2, data.y2);
    if ((data.graphpoints % 60) == 0) dwinDrawVLine(COLOR_LINE, data.x2 - 1, data.graphframe.y + 1, data.graphframe.h - 2);
    dwinDrawPoint(COLOR_RED, 1, 1, data.x2 - 1, data.r);
    dwinDrawLine(COLOR_YELLOW, data.x2 - 2, data.yP, data.x2 - 1, y);
  }
  data.yP = y;
  data.graphpoints++;
  TERN_(HAS_BACKLIGHT_TIMEOUT, ui.refresh_backlight_timeout());
}

#endif // PROUI_TUNING_GRAPH
#endif // DWIN_LCD_PROUI
