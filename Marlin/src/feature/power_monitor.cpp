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

/**
 * feature/power_monitor.cpp
 */

#include "../inc/MarlinConfigPre.h"

#if HAS_POWER_MONITOR

#include "power_monitor.h"

#if HAS_MARLINUI_MENU
  #include "../lcd/marlinui.h"
  #include "../lcd/lcdprint.h"
#endif

#include "../libs/numtostr.h"

uint8_t PowerMonitor::flags; // = 0

#if ENABLED(POWER_MONITOR_CURRENT)
  pm_lpf_t<PowerMonitor::amps_adc_scale, PM_K_VALUE, PM_K_SCALE> PowerMonitor::amps;
#endif
#if ENABLED(POWER_MONITOR_VOLTAGE)
  pm_lpf_t<PowerMonitor::volts_adc_scale, PM_K_VALUE, PM_K_SCALE> PowerMonitor::volts;
#endif

millis_t PowerMonitor::display_item_ms;
uint8_t PowerMonitor::display_item;

PowerMonitor power_monitor; // Single instance - this calls the constructor

#if HAS_MARLINUI_U8GLIB

  #if ENABLED(POWER_MONITOR_CURRENT)
    void PowerMonitor::draw_current() {
      const float amps = getAmps();
      lcd_put_u8str(amps < 100 ? ftostr31ns(amps) : ui16tostr4rj((uint16_t)amps));
      lcd_put_u8str(F("A"));
    }
  #endif

  #if ENABLED(POWER_MONITOR_VOLTAGE)
    void PowerMonitor::draw_voltage() {
      const float volts = getVolts();
      lcd_put_u8str(volts < 100 ? ftostr31ns(volts) : ui16tostr4rj((uint16_t)volts));
      lcd_put_u8str(F("V"));
    }
  #endif

  #if HAS_POWER_MONITOR_WATTS
    void PowerMonitor::draw_power() {
      const float power = getPower();
      lcd_put_u8str(power < 100 ? ftostr31ns(power) : ui16tostr4rj((uint16_t)power));
      lcd_put_u8str(F("W"));
    }
  #endif

#endif // HAS_MARLINUI_U8GLIB

#endif // HAS_POWER_MONITOR
