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
 * DWIN Enhanced implementation for PRO UI
 * Based on the original work of: Miguel Risco-Castillo (MRISCOC)
 * https://github.com/mriscoc/Ender3V2S1
 * Version: 3.11.1
 * Date: 2022/02/28
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(DWIN_LCD_PROUI)

#include "dwin.h"
#include "dwinui.h"
#include "dwin_popup.h"

#include "../../../MarlinCore.h" // for wait_for_user

popupDrawFunc_t popupDraw = nullptr;
popupClickFunc_t popupClick = nullptr;
popupChangeFunc_t popupChange = nullptr;

uint16_t highlightY = 280;

void drawSelectHighlight(const bool sel, const uint16_t ypos) {
  highlightY = ypos;
  hmiFlag.select_flag = sel;
  const uint16_t c1 = sel ? hmiData.colorHighlight : hmiData.colorPopupBg,
                 c2 = sel ? hmiData.colorPopupBg : hmiData.colorHighlight;
  dwinDrawRectangle(0, c1, 25, ypos - 1, 126, ypos + 38);
  dwinDrawRectangle(0, c1, 24, ypos - 2, 127, ypos + 39);
  dwinDrawRectangle(0, c2, 145, ypos - 1, 246, ypos + 38);
  dwinDrawRectangle(0, c2, 144, ypos - 2, 247, ypos + 39);
}

void dwinPopupConfirmCancel(const uint8_t icon, FSTR_P const fmsg2) {
  dwinDrawPopup(ICON_BLTouch, F("Please confirm"), fmsg2);
  DWINUI::drawButton(BTN_Confirm, 26, 280);
  DWINUI::drawButton(BTN_Cancel, 146, 280);
  drawSelectHighlight(hmiFlag.select_flag);
  dwinUpdateLCD();
}

void gotoPopup(const popupDrawFunc_t fnDraw, const popupClickFunc_t fnClick/*=nullptr*/, const popupChangeFunc_t fnChange/*=nullptr*/) {
  popupDraw = fnDraw;
  popupClick = fnClick;
  popupChange = fnChange;
  hmiSaveProcessID(ID_Popup);
  hmiFlag.select_flag = false;
  popupDraw();
}

void hmiPopup() {
  if (!wait_for_user) {
    if (popupClick) popupClick();
    return;
  }
  else {
    EncoderState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_CW || encoder_diffState == ENCODER_DIFF_CCW) {
      const bool change = encoder_diffState != ENCODER_DIFF_CW;
      if (popupChange) popupChange(change); else drawSelectHighlight(change, highlightY);
      dwinUpdateLCD();
    }
  }
}

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  void dwinPopupPause(FSTR_P const fmsg, uint8_t button/*=0*/) {
    hmiSaveProcessID(button ? ID_WaitResponse : ID_NothingToDo);
    dwinShowPopup(ICON_Pause_1, GET_TEXT_F(MSG_ADVANCED_PAUSE), fmsg, button);
  }

  void drawPopupFilamentPurge() {
    dwinDrawPopup(ICON_AutoLeveling, GET_TEXT_F(MSG_ADVANCED_PAUSE), GET_TEXT_F(MSG_FILAMENT_CHANGE_PURGE_CONTINUE));
    DWINUI::drawButton(BTN_Purge, 26, 280);
    DWINUI::drawButton(BTN_Continue, 146, 280);
    drawSelectHighlight(true);
  }
#endif // ADVANCED_PAUSE_FEATURE

#endif // DWIN_LCD_PROUI
