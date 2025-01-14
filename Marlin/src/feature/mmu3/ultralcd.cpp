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

/**
 * ultralcd.cpp
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

#include "mmu3.h"
#include "mmu3_marlin_macros.h"
#include "mmu_hw/errors_list.h"
#include "ultralcd.h"

#include "../../lcd/menu/menu_item.h"
#include "../../gcode/gcode.h"
#include "../../lcd/marlinui.h"

  //! @brief Show a two-choice prompt on the last line of the LCD
  //! @param selected Show first choice as selected if true, the second otherwise
  //! @param first_choice text caption of first possible choice
  //! @param second_choice text caption of second possible choice
  //! @param second_col column on LCD where second choice is rendered.
  //! @param third_choice text caption of third, optional, choice.
  void lcd_show_choices_prompt_P(uint8_t selected, const char *first_choice, const char *second_choice, uint8_t second_col, const char *third_choice) {
    lcd_put_lchar(0, 3, selected == LCD_LEFT_BUTTON_CHOICE ? '>' : ' ');
    lcd_put_u8str(first_choice);
    lcd_put_lchar(second_col, 3, selected == LCD_MIDDLE_BUTTON_CHOICE ? '>' : ' ');
    lcd_put_u8str(second_choice);
    if (third_choice) {
      lcd_put_lchar(18, 3, selected == LCD_RIGHT_BUTTON_CHOICE ? '>' : ' ');
      lcd_put_u8str(third_choice);
    }
  }

  void lcd_space(uint8_t n) {
    while (n--) lcd_put_lchar(' ');
  }

  // Print extruder status (5 chars total)
  // Scenario 1: "F?"
  //              There is no filament loaded and no tool change is in progress
  // Scenario 2: "F[nr.]"
  //              [nr.] ranges from 1 to 5.
  //              Shows which filament is loaded. No tool change is in progress
  // Scenario 3: "?>[nr.]"
  //              [nr.] ranges from 1 to 5.
  //              There is no filament currently loaded, but [nr.] is currently being loaded via tool change
  // Scenario 4: "[nr.]>?"
  //              [nr.] ranges from 1 to 5.
  //              This scenario indicates a bug in the firmware if ? is on the right side
  // Scenario 5: "[nr1.]>[nr2.]"
  //              [nr1.] ranges from 1 to 5.
  //              [nr2.] ranges from 1 to 5.
  //              Filament [nr1.] was loaded, but [nr2.] is currently being loaded via tool change
  // Scenario 6: "?>?"
  //              This scenario should not be possible and indicates a bug in the firmware
  uint8_t lcdui_print_extruder(void) {
    uint8_t chars = 1;
    lcd_space(1);
    if (mmu3.get_current_tool() == mmu3.get_tool_change_tool()) {
      lcd_put_lchar('F');
      lcd_put_lchar(mmu3.get_current_tool() == (uint8_t)MMU3::FILAMENT_UNKNOWN ? '?' : mmu3.get_current_tool() + '1');
      chars += 2;
    }
    else {
      lcd_put_lchar(mmu3.get_current_tool() == (uint8_t)MMU3::FILAMENT_UNKNOWN ? '?' : mmu3.get_current_tool() + '1');
      lcd_put_lchar('>');
      lcd_put_lchar(mmu3.get_tool_change_tool() == (uint8_t)MMU3::FILAMENT_UNKNOWN ? '?' : mmu3.get_tool_change_tool() + '1');
      chars += 3;
    }
    return chars;
  }

  bool pgm_is_whitespace(const char *c_addr) {
    const char c = pgm_read_byte(c_addr);
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
  }

  bool pgm_is_interpunction(const char *c_addr) {
    const char c = pgm_read_byte(c_addr);
    return c == '.' || c == ',' || c == ':' || c == ';' || c == '?' || c == '!' || c == '/';
  }

  /**
   * @brief show full screen message
   *
   * This function is non-blocking
   * @param msg message to be displayed from PROGMEM
   * @return rest of the text (to be displayed on next page)
   */
  static FSTR_P const lcd_display_message_fullscreen_nonBlocking(FSTR_P const fmsg) {
    PGM_P msg = FTOP(fmsg);
    PGM_P msgend = msg;
    //bool multi_screen = false;
    for (uint8_t row = 0; row < LCD_HEIGHT; ++row) {
      if (pgm_read_byte(msgend) == 0) break;
      SETCURSOR(0, row);

      // Previous row ended with a complete word, so the first character in the
      // next row is a whitespace. We can skip the whitespace on a new line.
      if (pgm_is_whitespace(msg) && ++msg == nullptr) break; // End of the message.

      uint8_t linelen = (strlen_P(msg) > LCD_WIDTH) ? LCD_WIDTH : strlen_P(msg);
      PGM_P const msgend2 = msg + linelen;
      msgend = msgend2;
      if (row == 3 && linelen == LCD_WIDTH) {
        // Last line of the display, full line should be displayed.
        // Find out, whether this message will be split into multiple screens.
        //multi_screen = pgm_read_byte(msgend) != 0;
        // We do not need this...
        //if (multi_screen) msgend = (msgend2 -= 2);
      }
      if (pgm_read_byte(msgend) != 0 && !pgm_is_whitespace(msgend) && !pgm_is_interpunction(msgend)) {
        // Splitting a word. Find the start of the current word.
        while (msgend > msg && !pgm_is_whitespace(msgend - 1)) --msgend;
        if (msgend == msg) msgend = msgend2; // Found a single long word, which cannot be split. Just cut it.
      }
      for (; msg < msgend; ++msg) {
        const char c = char(pgm_read_byte(msg));
        if (c == '\n') {
          // Abort early if '\n' is encountered.
          // This character is used to force the following words to be printed on the next line.
          break;
        }
        lcd_put_lchar(c);
      }
    }
    // We do not need this part...
    //if (multi_screen) {
    //  // Display the double down arrow.
    //  lcd_put_lchar(LCD_WIDTH - 2, LCD_HEIGHT - 2, LCD_STR_ARROW_2_DOWN[0]);
    //}
    //return multi_screen ? msgend : nullptr;
    return FPSTR(msgend);
  }

  FSTR_P const lcd_display_message_fullscreen(FSTR_P const fmsg) {
    // Disable update of the screen by the usual lcd_update(0) routine.
    #if HAS_WIRED_LCD
      //ui.lcdDrawUpdate = LCDViewAction::LCDVIEW_NONE;
      ui.clear_lcd();
      return lcd_display_message_fullscreen_nonBlocking(fmsg);
    #else
      return fmsg
    #endif
  }

  /**
   * @brief show full screen message and wait
   *
   * This function is blocking.
   * @param msg message to be displayed from PROGMEM
   */
  void lcd_show_fullscreen_message_and_wait(FSTR_P const fmsg) {
    LcdUpdateDisabler lcdUpdateDisabler;
    FSTR_P fmsg_next = lcd_display_message_fullscreen(fmsg);
    const bool multi_screen = fmsg_next != nullptr;
    ui.use_click();
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    // Until confirmed by a button click.
    for (;;) {
      if (fmsg_next == nullptr) {
        // Display the confirm char.
        //lcd_put_lchar(LCD_WIDTH - 2, LCD_HEIGHT - 2, LCD_STR_CONFIRM[0]);
      }
      // Wait for 5 seconds before displaying the next text.
      for (uint8_t i = 0; i < 100; ++i) {
        idle(true);
        safe_delay(50);
        if (ui.use_click()) {
          if (fmsg_next == nullptr) {
            KEEPALIVE_STATE(IN_HANDLER);
            return ui.go_back();
          }
          if (!multi_screen) break;
          if (fmsg_next == nullptr) fmsg_next = fmsg;
          fmsg_next = lcd_display_message_fullscreen(fmsg_next);
        }
      }
      //if (multi_screen) {
      //  if (fmsg_next == nullptr) fmsg_next = fmsg;
      //  fmsg_next = lcd_display_message_fullscreen(fmsg_next);
      //}
    }
  }

  void lcd_insert_char_into_status(uint8_t position, const char message) {
    if (position >= LCD_WIDTH) return;
    //int size = ui.status_message.length();
    char *str = ui.status_message.buffer();
    str[position] = message;
    ui.refresh(LCDVIEW_REDRAW_NOW); // force redraw
  }

#endif // HAS_PRUSA_MMU3
