/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * ultralcd.h
 */

#include "../../MarlinCore.h"
#include "../../lcd/marlinui.h"

#define LCD_LEFT_BUTTON_CHOICE 0
#define LCD_MIDDLE_BUTTON_CHOICE 1
#define LCD_RIGHT_BUTTON_CHOICE 2

#define LCD_STR_ARROW_2_DOWN "\x88"
#define LCD_STR_CONFIRM      "\x89"
#define LCD_STR_SOLID_BLOCK  "\xFF" // from the default character set

/**
 * @brief Helper class to temporarily disable LCD updates
 *
 * When constructed (on stack), original state state of lcd_update_enabled is stored
 * and LCD updates are disabled.
 * When destroyed (gone out of scope), original state of LCD update is restored.
 * It has zero overhead compared to storing bool saved = lcd_update_enabled
 * and calling lcd_update_enable(false) and lcd_update_enable(saved).
 */
class LcdUpdateDisabler {
  public:
    LcdUpdateDisabler() : m_updateEnabled(ui.lcdDrawUpdate) {
      TERN_(HAS_WIRED_LCD, ui.lcdDrawUpdate = LCDViewAction::LCDVIEW_NONE);
    }
    ~LcdUpdateDisabler() {
      #if HAS_WIRED_LCD
        ui.lcdDrawUpdate = m_updateEnabled;
        ui.clear_lcd();
        ui.update();
      #endif
    }

  private:
    LCDViewAction m_updateEnabled;
};

bool pgm_is_whitespace(const char *c_addr);
bool pgm_is_interpunction(const char *c_addr);
FSTR_P const lcd_display_message_fullscreen(FSTR_P const pmsg);
void lcd_show_choices_prompt_P(uint8_t selected, const char *first_choice, const char *second_choice, uint8_t second_col, const char *third_choice=nullptr);
void lcd_show_fullscreen_message_and_wait(FSTR_P const fmsg);
uint8_t lcdui_print_extruder(void);
void lcd_space(uint8_t n);
void lcd_insert_char_into_status(uint8_t position, const char message);
