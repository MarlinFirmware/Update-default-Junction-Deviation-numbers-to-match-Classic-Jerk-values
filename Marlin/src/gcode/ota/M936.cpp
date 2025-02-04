/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include "../../inc/MarlinConfig.h"

#if ENABLED(OTA_FIRMWARE_UPDATE)

#include "../gcode.h"
#include "../../libs/BL24CXX.h"

#if ENABLED(CREALITY_RTS)
  #include "../../lcd/rts/lcd_rts.h"
#endif

#define OTA_FLAG_EEPROM 90

//#define DEBUG_OUT 1
#include "../../core/debug_out.h"

/**
 * M936: Set one of the OTA update flags.
 *   V2 = Upgrade the motherboard firmware
 *   V3 = Upgrade the RTS controller firmware
 */
void GcodeSuite::M936() {
  static uint8_t ota_update_flag = 0x00;
  const int16_t ota = parser.intval('V', -1);
  switch (ota) {
    case 2:
      // Set the OTA board firmware upgrade flag ahead of reboot.
      ota_update_flag = 0x01;
      DEBUG_ECHOLNPGM("Motherboard upgrade flag set");
      TERN_(CREALITY_RTS, RTS_Error(Error_205));
      break;

    #if ENABLED(CREALITY_RTS)
      case 3:
        // Set the OTA screen firmware upgrade flag ahead of reboot.
        ota_update_flag = 0x02;
        DEBUG_ECHOLNPGM("DWIN upgrade flag set");
        TERN_(CREALITY_RTS, RTS_Error(Error_206));
        break;
    #endif
  }

  switch (ota) {
    case 2: TERN_(CREALITY_RTS, case 3:)
      BL24CXX::write(OTA_FLAG_EEPROM, &ota_update_flag, sizeof(ota_update_flag));
      safe_delay(100);
      hal.reboot();
  }
}

#endif // OTA_FIRMWARE_UPDATE
