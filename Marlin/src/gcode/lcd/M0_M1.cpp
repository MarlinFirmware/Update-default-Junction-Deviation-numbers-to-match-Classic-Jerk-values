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

#include "../../inc/MarlinConfigPre.h"

#if HAS_RESUME_CONTINUE

#include "../../inc/MarlinConfig.h"

#include "../gcode.h"

#include "../../module/planner.h" // for synchronize()
#include "../../MarlinCore.h"     // for wait_for_user_response()

#if HAS_MARLINUI_MENU
  #include "../../lcd/marlinui.h"
#elif ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#endif

#if ENABLED(CREALITY_RTS)
  #include "../../lcd/rts/lcd_rts.h"
#endif
#if ENABLED(HOST_PROMPT_SUPPORT)
  #include "../../feature/host_actions.h"
#endif

/**
 * M0: Unconditional stop - Wait for user button press on LCD
 * M1: Conditional stop   - Wait for user button press on LCD
 */
void GcodeSuite::M0_M1() {

  TERN_(CREALITY_RTS, RTS_CommandPause());

  millis_t ms = 0;
  if (parser.seenval('P')) ms = parser.value_millis();              // Milliseconds to wait
  if (parser.seenval('S')) ms = parser.value_millis_from_seconds(); // Seconds to wait

  planner.synchronize();

  #if HAS_MARLINUI_MENU

    if (parser.string_arg)
      ui.set_status_no_expire(parser.string_arg);
    else {
      LCD_MESSAGE(MSG_USERWAIT);
      #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
        ui.reset_progress_bar_timeout();
      #endif
    }

  #elif ENABLED(DWIN_LCD_PROUI) // ExtUI with icon, string, button title

    if (parser.string_arg)
      ExtUI::onUserConfirmRequired(ICON_Continue_1, parser.string_arg, GET_TEXT_F(MSG_USERWAIT));
    else
      ExtUI::onUserConfirmRequired(ICON_Stop_1, GET_TEXT_F(MSG_STOPPED), GET_TEXT_F(MSG_USERWAIT));

  #elif ENABLED(EXTENSIBLE_UI)

    if (parser.string_arg)
      ExtUI::onUserConfirmRequired(parser.string_arg); // String in an SRAM buffer
    else
      ExtUI::onUserConfirmRequired(GET_TEXT_F(MSG_USERWAIT));

  #else

    if (parser.string_arg) {
      SERIAL_ECHO_START();
      SERIAL_ECHOLN(parser.string_arg);
    }

  #endif

  #if ENABLED(HOST_PROMPT_SUPPORT)
    if (parser.string_arg)
      hostui.continue_prompt(parser.string_arg);
    else
      hostui.continue_prompt(parser.codenum ? F("M1 Stop") : F("M0 Stop"));
  #endif

  TERN_(HAS_RESUME_CONTINUE, wait_for_user_response(ms));

  TERN_(HAS_MARLINUI_MENU, ui.reset_status());
}

#endif // HAS_RESUME_CONTINUE
