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
#pragma once

/**
 * feature/fwretract.h - Define firmware-based retraction interface
 */

#include "../inc/MarlinConfigPre.h"

typedef struct {
       float retract_length;                      // M207 S - G10 Retract length
  feedRate_t retract_feedrate_mm_s;               // M207 F - G10 Retract feedrate
       float retract_zraise,                      // M207 Z - G10 Retract hop size
             retract_recover_extra;               // M208 S - G11 Recover length
  feedRate_t retract_recover_feedrate_mm_s;       // M208 F - G11 Recover feedrate
       float swap_retract_length,                 // M207 W - G10 Swap Retract length
             swap_retract_recover_extra;          // M208 W - G11 Swap Recover length
  feedRate_t swap_retract_recover_feedrate_mm_s;  // M208 R - G11 Swap Recover feedrate
} fwretract_settings_t;

#if ENABLED(FWRETRACT)

class FWRetract {
private:
  #if HAS_MULTI_EXTRUDER
    static Flags<EXTRUDERS> retracted_swap;        // Which extruders are swap-retracted
  #endif

public:
  static fwretract_settings_t settings;

  #if ENABLED(FWRETRACT_AUTORETRACT)
    static bool autoretract_enabled;               // M209 S - Autoretract switch
  #else
    static constexpr bool autoretract_enabled = false;
  #endif

  static Flags<EXTRUDERS> retracted;               // Which extruders are currently retracted
  static float current_retract[EXTRUDERS],         // Retract value used by planner
               current_hop;                        // Hop value used by planner

  FWRetract() { reset(); }

  static void reset();

  static void refresh_autoretract() { retracted.reset(); }

  static void enable_autoretract(const bool enable) {
    #if ENABLED(FWRETRACT_AUTORETRACT)
      autoretract_enabled = enable;
      refresh_autoretract();
    #endif
  }

  static void retract(const bool retracting E_OPTARG(bool swapping=false));

  static void M207_report();
  static void M207();
  static void M208_report();
  static void M208();
  #if ENABLED(FWRETRACT_AUTORETRACT)
    static void M209_report();
    static void M209();
  #endif
};

extern FWRetract fwretract;

#endif // FWRETRACT
