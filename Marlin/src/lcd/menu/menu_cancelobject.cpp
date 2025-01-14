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

//
// Cancel Object Menu
//

#include "../../inc/MarlinConfigPre.h"

#if ALL(HAS_MARLINUI_MENU, CANCEL_OBJECTS)

#include "menu_item.h"
#include "menu_addon.h"

#include "../../feature/cancel_object.h"

static void lcd_cancel_object_confirm() {
  const int8_t v = MenuItemBase::itemIndex;
  const char item_num[] = {
    ' ',
    char((v > 9) ? '0' + (v / 10) : ' '),
    char('0' + (v % 10)),
    '\0'
  };
  MenuItem_confirm::confirm_screen(
    []{
      cancelable.cancel_object(MenuItemBase::itemIndex);
      ui.completion_feedback();
      ui.goto_previous_screen();
    },
    nullptr,
    GET_TEXT_F(MSG_CANCEL_OBJECT), item_num, F("?")
  );
}

void menu_cancelobject() {
  const int8_t ao = cancelable.state.active_object;

  START_MENU();
  BACK_ITEM(MSG_MAIN_MENU);

  // Draw cancelable items in a loop
  for (int8_t i = -1; i < cancelable.state.object_count; i++) {
    if (i == ao) continue;                                          // Active is drawn on -1 index
    const int8_t j = i < 0 ? ao : i;                                // Active or index item
    if (!cancelable.is_canceled(j)) {                               // Not canceled already?
      SUBMENU_N(j, MSG_CANCEL_OBJECT_N, lcd_cancel_object_confirm); // Offer the option.
      if (i < 0) SKIP_ITEM();                                       // Extra line after active
    }
  }

  END_MENU();
}

#endif // HAS_MARLINUI_MENU && CANCEL_OBJECTS
