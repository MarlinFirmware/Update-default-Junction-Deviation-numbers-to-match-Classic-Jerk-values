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

#if HAS_TFT_LVGL_UI

#include "draw_ui.h"
#include <lv_conf.h>
#include "tft_lvgl_configuration.h"
#include "SPI_TFT.h"
#include "../../../inc/MarlinConfig.h"
#include "mks_hardware.h"

static lv_obj_t *scr;

void lv_draw_error_message(FSTR_P const fmsg) {
  FSTR_P const fhalted = F("PRINTER HALTED"), fplease = F("Please Reset");
  SPI_TFT.lcdClear(0x0000);
  if (fmsg) disp_string((TFT_WIDTH - strlen_P(FTOP(fmsg)) * 16) / 2, 100, fmsg, 0xFFFF, 0x0000);
  disp_string((TFT_WIDTH - strlen_P(FTOP(fhalted)) * 16) / 2, 140, fhalted, 0xFFFF, 0x0000);
  disp_string((TFT_WIDTH - strlen_P(FTOP(fplease)) * 16) / 2, 180, fplease, 0xFFFF, 0x0000);
}

void lv_clear_error_message() { lv_obj_del(scr); }

#endif // HAS_TFT_LVGL_UI
