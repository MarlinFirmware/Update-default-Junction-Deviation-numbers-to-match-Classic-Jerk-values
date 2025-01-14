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
 * mmu2_fsensor.h
 */

#include "../../core/macros.h"
#include <stdint.h>

#define FILAMENT_PRESENT() (READ(FIL_RUNOUT1_PIN) != FIL_RUNOUT1_STATE)

namespace MMU3 {

  // Can be used to block printer's filament sensor handling - to avoid errorneous injecting of M600
  // while doing a toolchange with the MMU
  // In case of "no filament sensor" these methods default to an empty implementation
  class FSensorBlockRunout {
    public:
    FSensorBlockRunout();
    ~FSensorBlockRunout();
  };

  // Possible states of filament from the perspective of presence in various parts of the printer
  // Beware, the numeric codes are important and sent into the MMU
  enum class FilamentState : uint_fast8_t {
    NOT_PRESENT = 0,  //!< Filament sensor doesn't see the filament
    AT_FSENSOR  = 1,  //!< Filament detected by the filament sensor, but the nozzle has not detected the filament yet
    IN_NOZZLE   = 2,  //!< Filament detected by the filament sensor and also loaded in the nozzle
    UNAVAILABLE = 3   //!< Sensor not available (likely not connected due broken cable)
  };

  FilamentState WhereIsFilament();

} // MMU3
