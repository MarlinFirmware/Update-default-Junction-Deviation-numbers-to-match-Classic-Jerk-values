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

#include "../../../inc/MarlinConfig.h"

#if ENABLED(POWER_LOSS_RECOVERY)

#include "../../gcode.h"
#include "../../../feature/powerloss.h"
#include "../../../module/motion.h"

#if HAS_PLR_BED_THRESHOLD
  #include "../../../module/temperature.h"  // for degBed
#endif

#include "../../../lcd/marlinui.h"
#if ENABLED(EXTENSIBLE_UI)
  #include "../../../lcd/extui/ui_api.h"
#elif ENABLED(DWIN_CREALITY_LCD)
  #include "../../../lcd/e3v2/creality/dwin.h"
#elif ENABLED(DWIN_CREALITY_LCD_JYERSUI)
  #include "../../../lcd/e3v2/jyersui/dwin.h" // Temporary fix until it can be better implemented
#endif

#define DEBUG_OUT ENABLED(DEBUG_POWER_LOSS_RECOVERY)
#include "../../../core/debug_out.h"

void menu_job_recovery();

inline void plr_error(FSTR_P const prefix) {
  #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
    DEBUG_ECHO_START();
    DEBUG_ECHOLN(prefix, F(" Job Recovery Data"));
  #else
    UNUSED(prefix);
  #endif
}

#if HAS_MARLINUI_MENU
  void lcd_power_loss_recovery_cancel();
#endif

/**
 * M1000: Resume from power-loss (undocumented)
 *   - With 'S' go to the Resume/Cancel menu
 *     ...unless the bed temperature is already above a configured minimum temperature.
 *   - With 'C' execute a cancel selection
 *   - With no parameters, run recovery commands
 */
void GcodeSuite::M1000() {

  if (recovery.valid()) {
    const bool force_resume = TERN0(HAS_PLR_BED_THRESHOLD, recovery.bed_temp_threshold && (thermalManager.degBed() >= recovery.bed_temp_threshold));

    if (!force_resume && parser.seen_test('S')) {

      TERN_(PLR_HEAT_BED_ON_REBOOT, recovery.set_bed_temp(true));

      #if HAS_MARLINUI_MENU
        ui.goto_screen(menu_job_recovery);
      #elif ENABLED(EXTENSIBLE_UI)
        ExtUI::onPowerLossResume();
      #elif HAS_PLR_UI_FLAG
        recovery.ui_flag_resume = true;
      #elif ENABLED(DWIN_CREALITY_LCD_JYERSUI) // Temporary fix until it can be better implemented
        jyersDWIN.popupHandler(Popup_Resume);
      #else
        SERIAL_ECHO_MSG("Resume requires LCD.");
      #endif
    }
    else if (parser.seen_test('C')) {
      #if HAS_MARLINUI_MENU
        lcd_power_loss_recovery_cancel();
      #else
        recovery.cancel();
      #endif
      TERN_(EXTENSIBLE_UI, ExtUI::onPrintTimerStopped());
    }
    else
      recovery.resume();
  }
  else
    plr_error(recovery.info.valid_head ? F("No") : F("Invalid"));

}

#endif // POWER_LOSS_RECOVERY
