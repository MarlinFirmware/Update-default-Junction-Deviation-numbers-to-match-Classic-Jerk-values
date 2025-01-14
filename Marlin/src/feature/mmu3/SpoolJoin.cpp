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
 * SpoolJoin.cpp
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

#include "SpoolJoin.h"
#include "../../module/settings.h"
#include "../../core/language.h"

SpoolJoin spooljoin;

bool SpoolJoin::enabled;            // Initialized by settings.load
int SpoolJoin::epprom_addr;         // Initialized by settings.load
uint8_t SpoolJoin::currentMMUSlot;

SpoolJoin::SpoolJoin() { setSlot(0); }

void SpoolJoin::initStatus() {
  // Useful information to see during bootup
  SERIAL_ECHOLN(F("SpoolJoin is "), enabled ? F("On") : F("Off"));
}

void SpoolJoin::toggle() {
  // Toggle enabled value.
  enabled = !enabled;

  // Following Prusa's implementation let's save the value to the EEPROM
  // TODO: Move to settings.cpp
  #if ENABLED(EEPROM_SETTINGS)
    persistentStore.access_start();
    persistentStore.write_data(epprom_addr, enabled);
    persistentStore.access_finish();
    settings.save();
  #endif
}

bool SpoolJoin::isEnabled() { return enabled; }

void SpoolJoin::setSlot(const uint8_t slot) { currentMMUSlot = slot; }

uint8_t SpoolJoin::nextSlot() {
  SERIAL_ECHOPGM("SpoolJoin: ", currentMMUSlot);
  if (++currentMMUSlot >= 4) currentMMUSlot = 0;
  SERIAL_ECHOLNPGM(" -> ", currentMMUSlot);
  return currentMMUSlot;
}

#endif // HAS_PRUSA_MMU3
