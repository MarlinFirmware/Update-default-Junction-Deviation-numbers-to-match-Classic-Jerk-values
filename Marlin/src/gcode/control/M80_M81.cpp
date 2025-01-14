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

#include "../gcode.h"

#include "../../module/temperature.h"
#include "../../module/planner.h"       // for planner.finish_and_disable
#include "../../module/printcounter.h"  // for print_job_timer.stop
#include "../../lcd/marlinui.h"         // for LCD_MESSAGE_F

#include "../../inc/MarlinConfig.h"

#if ENABLED(PSU_CONTROL)
  #include "../queue.h"
  #include "../../feature/power.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../feature/powerloss.h"
#endif

#if HAS_SUICIDE
  #include "../../MarlinCore.h"
#endif

#if ENABLED(PSU_CONTROL)

  /**
   * M80   : Turn on the Power Supply
   * M80 S : Report the current state and exit
   */
  void GcodeSuite::M80() {

    // S: Report the current power supply state and exit
    if (parser.seen('S')) {
      SERIAL_ECHO(powerManager.psu_on ? F("PS:1\n") : F("PS:0\n"));
      return;
    }

    powerManager.power_on();

    /**
     * If you have a switch on suicide pin, this is useful
     * if you want to start another print with suicide feature after
     * a print without suicide...
     */
    #if HAS_SUICIDE
      OUT_WRITE(SUICIDE_PIN, !SUICIDE_PIN_STATE);
    #endif

    TERN_(HAS_MARLINUI_MENU, ui.reset_status());
  }

#endif // PSU_CONTROL

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for FULL SHUTDOWN!
 */
void GcodeSuite::M81() {
  planner.finish_and_disable();
  thermalManager.cooldown();

  print_job_timer.stop();

  #if ALL(HAS_FAN, PROBING_FANS_OFF)
    thermalManager.fans_paused = false;
    ZERO(thermalManager.saved_fan_speed);
  #endif

  TERN_(POWER_LOSS_RECOVERY, recovery.purge()); // Clear PLR on intentional shutdown

  safe_delay(1000); // Wait 1 second before switching off

  LCD_MESSAGE_F(MACHINE_NAME " " STR_OFF ".");

  bool delayed_power_off = false;

  #if ENABLED(POWER_OFF_TIMER)
    if (parser.seenval('D')) {
      uint16_t delay = parser.value_ushort();
      if (delay > 1) { // skip already observed 1s delay
        delayed_power_off = true;
        powerManager.setPowerOffTimer(SEC_TO_MS(delay - 1));
      }
    }
  #endif

  #if ENABLED(POWER_OFF_WAIT_FOR_COOLDOWN)
    if (parser.boolval('S')) {
      delayed_power_off = true;
      powerManager.setPowerOffOnCooldown(true);
    }
  #endif

  if (delayed_power_off) {
    SERIAL_ECHOLNPGM(STR_DELAYED_POWEROFF);
    return;
  }

  #if ENABLED(PSU_CONTROL)
    powerManager.power_off_soon();
  #elif HAS_SUICIDE
    suicide();
  #endif
}
