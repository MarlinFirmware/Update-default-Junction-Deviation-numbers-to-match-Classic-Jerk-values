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
 * DWIN G-code thumbnail preview
 * Author: Miguel A. Risco-Castillo
 * version: 3.3.2
 * Date: 2023/06/18
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(DWIN_LCD_PROUI)

#include "dwin_defines.h"

#if HAS_GCODE_PREVIEW

#include "gcode_preview.h"

#include "../../marlinui.h"
#include "../../../sd/cardreader.h"
#include "dwin_popup.h"
#include "base64.h"

#if ENABLED(TJC_DISPLAY)
  #define THUMBWIDTH  180
  #define THUMBHEIGHT 180
#else
  #define THUMBWIDTH  200
  #define THUMBHEIGHT 200
#endif

Preview preview;
fileprop_t fileprop;

void fileprop_t::setname(const char * const fn) {
  const uint8_t len = _MIN(sizeof(name) - 1, strlen(fn));
  memcpy(name, fn, len);
  name[len] = '\0';
}

void fileprop_t::clear() {
  name[0] = '\0';
  thumbstart = 0;
  thumbsize = 0;
  thumbheight = thumbwidth = 0;
  time = 0;
  filament = 0;
  layer = 0;
  height = width = length = 0;
}

void getValue(const char * const buf, PGM_P const key, float &value) {
  if (value != 0.0f) return;

  const char *posptr = strstr_P(buf, key);
  if (posptr == nullptr) return;

  char num[10] = "";
  for (uint8_t i = 0; i < sizeof(num);) {
    const char c = *posptr;
    if (ISEOL(c) || c == '\0') {
      num[i] = '\0';
      value = atof(num);
      break;
    }
    if (WITHIN(c, '0', '9') || c == '.') num[i++] = c;
    posptr++;
  }
}

bool Preview::hasPreview() {
  const char * const tbstart = PSTR("; thumbnail begin " STRINGIFY(THUMBWIDTH) "x" STRINGIFY(THUMBHEIGHT));
  const char *posptr = nullptr;
  uint32_t indx = 0;
  float tmp = 0;

  fileprop.clear();
  fileprop.setname(card.filename);

  card.openFileRead(fileprop.name);

  char buf[256];
  uint8_t nbyte = 1;
  while (!fileprop.thumbstart && nbyte > 0 && indx < 4 * sizeof(buf)) {
    nbyte = card.read(buf, sizeof(buf) - 1);
    if (nbyte > 0) {
      buf[nbyte] = '\0';
      getValue(buf, PSTR(";TIME:"), fileprop.time);
      getValue(buf, PSTR(";Filament used:"), fileprop.filament);
      getValue(buf, PSTR(";Layer height:"), fileprop.layer);
      getValue(buf, PSTR(";MINX:"), tmp);
      getValue(buf, PSTR(";MAXX:"), fileprop.width);
      fileprop.width -= tmp;
      tmp = 0;
      getValue(buf, PSTR(";MINY:"), tmp);
      getValue(buf, PSTR(";MAXY:"), fileprop.length);
      fileprop.length -= tmp;
      tmp = 0;
      getValue(buf, PSTR(";MINZ:"), tmp);
      getValue(buf, PSTR(";MAXZ:"), fileprop.height);
      fileprop.height -= tmp;
      posptr = strstr_P(buf, tbstart);
      if (posptr != nullptr) {
        fileprop.thumbstart = indx + (posptr - &buf[0]);
      }
      else {
        indx += _MAX(10, nbyte - (signed)strlen_P(tbstart));
        card.setIndex(indx);
      }
    }
  }

  if (!fileprop.thumbstart) {
    card.closefile();
    LCD_MESSAGE_F("Thumbnail not found");
    return false;
  }

  // Get the size of the thumbnail
  card.setIndex(fileprop.thumbstart + strlen_P(tbstart));
  for (uint8_t i = 0; i < 16; i++) {
    const char c = card.get();
    if (ISEOL(c)) { buf[i] = '\0'; break; }
    buf[i] = c;
  }
  fileprop.thumbsize = atoi(buf);

  // Exit if there isn't a thumbnail
  if (!fileprop.thumbsize) {
    card.closefile();
    LCD_MESSAGE_F("Invalid Thumbnail Size");
    return false;
  }

  uint8_t buf64[fileprop.thumbsize + 1];
  uint16_t nread = 0;
  while (nread < fileprop.thumbsize) {
    const uint8_t c = card.get();
    if (!ISEOL(c) && c != ';' && c != ' ')
      buf64[nread++] = c;
  }
  card.closefile();
  buf64[nread] = '\0';

  uint8_t thumbdata[3 + 3 * (fileprop.thumbsize / 4)];  // Reserve space for the JPEG thumbnail
  fileprop.thumbsize = decode_base64(buf64, thumbdata);
  DWINUI::writeToSRAM(0x00, fileprop.thumbsize, thumbdata);

  fileprop.thumbwidth = THUMBWIDTH;
  fileprop.thumbheight = THUMBHEIGHT;

  return true;
}

void Preview::drawFromSD() {
  hasPreview();

  MString<45> buf;
  dwinDrawRectangle(1, hmiData.colorBackground, 0, 0, DWIN_WIDTH, STATUS_Y - 1);
  if (fileprop.time) {
    buf.setf(F("Estimated time: %i:%02i"), (uint16_t)fileprop.time / 3600, ((uint16_t)fileprop.time % 3600) / 60);
    DWINUI::drawString(20, 10, &buf);
  }
  if (fileprop.filament) {
    buf.set(F("Filament used: "), p_float_t(fileprop.filament, 2), F(" m"));
    DWINUI::drawString(20, 30, &buf);
  }
  if (fileprop.layer) {
    buf.set(F("Layer height: "), p_float_t(fileprop.layer, 2), F(" mm"));
    DWINUI::drawString(20, 50, &buf);
  }
  if (fileprop.width) {
    buf.set(F("Volume: "), p_float_t(fileprop.width, 1), 'x', p_float_t(fileprop.length, 1), 'x', p_float_t(fileprop.height, 1), F(" mm"));
    DWINUI::drawString(20, 70, &buf);
  }

  if (!fileprop.thumbsize) {
    const uint8_t xpos = ((DWIN_WIDTH)  / 2) - 55,  // 55 = iconW/2
                  ypos = ((DWIN_HEIGHT)  / 2) - 125;
    DWINUI::drawIcon(ICON_Info_0, xpos, ypos);
    buf.set(PSTR("No " STRINGIFY(THUMBWIDTH) "x" STRINGIFY(THUMBHEIGHT) " Thumbnail"));
    DWINUI::drawCenteredString(false, (DWINUI::fontID * 3), DWINUI::textColor, DWINUI::backColor, 0, DWIN_WIDTH, (DWIN_HEIGHT / 2), &buf);
  }
  DWINUI::drawButton(BTN_Print, 26, 290);
  DWINUI::drawButton(BTN_Cancel, 146, 290);
  if (fileprop.thumbsize) show();
  drawSelectHighlight(false, 290);
  dwinUpdateLCD();
}

void Preview::invalidate() {
  fileprop.thumbsize = 0;
}

bool Preview::valid() {
  return !!fileprop.thumbsize;
}

void Preview::show() {
  const uint8_t xpos = ((DWIN_WIDTH) - fileprop.thumbwidth) / 2,
                ypos = (205 - fileprop.thumbheight) / 2 + 87;
  dwinIconShow(xpos, ypos, 0x00);
}

#endif // HAS_GCODE_PREVIEW
#endif // DWIN_LCD_PROUI
