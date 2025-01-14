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
 * mmu2_fsensor.cpp
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

#include "../runout.h"
#include "mmu3_fsensor.h"

namespace MMU3 {

  #if HAS_FILAMENT_SENSOR

    FSensorBlockRunout::FSensorBlockRunout() {
      runout.enabled = false; // Suppress filament runouts while loading filament.
      //fsensor.setAutoLoadEnabled(false); //suppress filament autoloads while loading filament.
    }

    FSensorBlockRunout::~FSensorBlockRunout() {
      //fsensor.settings_init(); // restore filament runout state.
      runout.reset();
      runout.enabled = true;
      //SERIAL_ECHOLNPGM("FSUnBlockRunout");
    }

  #else

    FSensorBlockRunout::FSensorBlockRunout() { }
    FSensorBlockRunout::~FSensorBlockRunout() { }

  #endif


  FilamentState WhereIsFilament() {
    //return fsensor.getFilamentPresent() ? FilamentState::AT_FSENSOR : FilamentState::NOT_PRESENT;
    return FILAMENT_PRESENT() ? FilamentState::AT_FSENSOR : FilamentState::NOT_PRESENT;
  }

} // MMU3

#endif // HAS_PRUSA_MMU3
