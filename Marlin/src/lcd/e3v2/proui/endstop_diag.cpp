/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * DWIN Endstops diagnostic page for PRO UI
 * Based on the original work of: Miguel Risco-Castillo (MRISCOC)
 * https://github.com/mriscoc/Ender3V2S1
 * Version: 1.4.3
 * Date: 2023/05/10
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(DWIN_LCD_PROUI)

#include "dwin_defines.h"

#if HAS_ESDIAG

#include "endstop_diag.h"
#include "../../marlinui.h"
#include "dwin.h"
#include "dwin_popup.h"

#if HAS_FILAMENT_SENSOR
  #include "../../../feature/runout.h"
#endif

#if HAS_BED_PROBE
  #include "../../../module/probe.h"
#endif

ESDiag esDiag;

void draw_es_label(FSTR_P const flabel=nullptr) {
  DWINUI::cursor.x = 40;
  if (flabel) DWINUI::drawString(F(flabel));
  DWINUI::drawString(F(": "));
  DWINUI::moveBy(0, 25);
}

void draw_es_state(const bool is_hit, const bool is_fil) {
  const uint8_t LM = 130;
  DWINUI::cursor.x = LM;
  dwinDrawRectangle(1, hmiData.colorPopupBg, LM, DWINUI::cursor.y, LM + 120, DWINUI::cursor.y + 20);
  is_fil ? (is_hit ? DWINUI::drawString(RGB(16, 63, 16), F(STR_FILAMENT_PRESENT)) : DWINUI::drawString(RGB(31, 31, 16), F(STR_RUNOUT_DETECTED))) :
  (is_hit ? DWINUI::drawString(RGB(31, 31, 16), F(STR_ENDSTOP_HIT)) : DWINUI::drawString(RGB(16, 63, 16), F(STR_ENDSTOP_OPEN)));
  DWINUI::moveBy(0, 25);
}

void ESDiag::draw() {
  title.showCaption(GET_TEXT_F(MSG_ENDSTOP_TEST));
  DWINUI::clearMainArea();
  drawPopupBkgd();
  DWINUI::drawButton(BTN_Continue, 86, 250);
  DWINUI::cursor.y = 80;
  #define ESLABEL(S) draw_es_label(F(STR_##S))
  TERN_(USE_X_MIN, ESLABEL(X_MIN)); TERN_(USE_X_MAX, ESLABEL(X_MAX));
  TERN_(USE_Y_MIN, ESLABEL(Y_MIN)); TERN_(USE_Y_MAX, ESLABEL(Y_MAX));
  IF_DISABLED(USE_Z_MIN_PROBE, TERN_(USE_Z_MIN, ESLABEL(Z_MIN)); TERN_(USE_Z_MAX, ESLABEL(Z_MAX)));
  TERN_(HAS_FILAMENT_SENSOR, ESLABEL(FILAMENT));
  TERN_(USE_Z_MIN_PROBE, ESLABEL(Z_PROBE));
  update();
}

void ESDiag::update() {
  DWINUI::cursor.y = 80;
  #define ESREPORT(S) draw_es_state(READ(S##_PIN) == S##_ENDSTOP_HIT_STATE, false)
  TERN_(USE_X_MIN, ESREPORT(X_MIN)); TERN_(USE_X_MAX, ESREPORT(X_MAX));
  TERN_(USE_Y_MIN, ESREPORT(Y_MIN)); TERN_(USE_Y_MAX, ESREPORT(Y_MAX));
  IF_DISABLED(USE_Z_MIN_PROBE, TERN_(USE_Z_MIN, ESREPORT(Z_MIN)); TERN_(USE_Z_MAX, ESREPORT(Z_MAX)));
  TERN_(HAS_FILAMENT_SENSOR, draw_es_state(READ(FIL_RUNOUT1_PIN) != FIL_RUNOUT1_STATE, true));
  TERN_(USE_Z_MIN_PROBE, ESREPORT(Z_MIN_PROBE));
  dwinUpdateLCD();
}

#endif // HAS_ESDIAG
#endif // DWIN_LCD_PROUI
