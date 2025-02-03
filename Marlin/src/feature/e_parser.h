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
 * e_parser.h - Intercept special commands directly in the serial stream
 */

#include "../inc/MarlinConfigPre.h"

class EmergencyParser {

public:

  // Currently looking for: M108, M112, M410, M524, M876 S[0-9], S000, P000, R000
  enum State : uint8_t {
    EP_RESET,
    EP_N,
    EP_M,
    EP_M1,
    EP_M10, EP_M108,
    EP_M11, EP_M112,
    EP_M4, EP_M41, EP_M410,
    #if HAS_MEDIA
      EP_M5, EP_M52, EP_M524,
    #endif
    #if ENABLED(EP_BABYSTEPPING)
      EP_M2, EP_M29, EP_M293, EP_M294,
    #endif
    #if ENABLED(HOST_PROMPT_SUPPORT)
      EP_M8, EP_M87, EP_M876, EP_M876S, EP_M876SN,
    #endif
    #if ENABLED(REALTIME_REPORTING_COMMANDS)
      EP_S, EP_S0, EP_S00, EP_GRBL_STATUS,
      EP_R, EP_R0, EP_R00, EP_GRBL_RESUME,
      EP_P, EP_P0, EP_P00, EP_GRBL_PAUSE,
    #endif
    #if ENABLED(SOFT_RESET_VIA_SERIAL)
      EP_ctrl,
      EP_K, EP_KI, EP_KIL, EP_KILL,
    #endif
    EP_IGNORE // To '\n'
  };

  static bool killed_by_M112;
  static bool quickstop_by_M410;

  #if HAS_MEDIA
    static bool sd_abort_by_M524;
  #endif

  #if ENABLED(HOST_PROMPT_SUPPORT)
    static uint8_t M876_reason;
  #endif

  EmergencyParser() { enable(); }

  FORCE_INLINE static void enable()  { enabled = true; }
  FORCE_INLINE static void disable() { enabled = false; }

  static void update(State &state, const uint8_t c);

private:
  static bool enabled;
};

extern EmergencyParser emergency_parser;
