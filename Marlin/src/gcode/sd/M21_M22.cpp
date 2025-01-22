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

#include "../../inc/MarlinConfig.h"

#if HAS_MEDIA

#include "../gcode.h"
#include "../../sd/cardreader.h"

/**
 * M21: Select / Mount the SD Card or USB Flash Drive
 *
 * With multiple volumes:
 *  P# - Select and mount the indexed Volume (VOLUME0, VOLUME1, etc.)
 *  S0 - Select and mount the first SD Card. (Also S with no index)
 *  S1 - Select and mount the second SD Card.
 *  U0 - Select and mount the first USB Flash Drive. (Also U with no index)
 */
void GcodeSuite::M21() {
  #if HAS_MULTI_VOLUME
    static int8_t vol = 0; // VOLUME0 is the default
    const int8_t newvol = (
                                     parser.seen_test('S')  ? 0 : // "S" for SD Card
      TERN0(HAS_USB_FLASH_DRIVE,     parser.seen_test('U')) ? 1 : // "U" for USB
      TERN0(ONBOARD_SDIO,            parser.seen_test('O')) ? 2 : // "O" for SDIO (usually onboard)
                                     parser.intval('P', vol)      // "P" for integer volume number
    );

    if (newvol != vol) {
      vol = newvol;
      switch (newvol + 1) {
        default: card.changeMedia(&card.media_driver_sdcard); break;
        #if HAS_USB_FLASH_DRIVE
          case 1: card.changeMedia(&card.media_driver_usbFlash); break;
        #endif
        #if ENABLED(ONBOARD_SDIO)
          case 2: card.changeMedia(&card.media_driver_sdiocard); break;
        #endif
      }
    }

  #endif // HAS_MULTI_VOLUME

  card.mount();
}

/**
 * M22: Release the SD Card or USB Flash Drive
 */
void GcodeSuite::M22() {
  if (!IS_SD_PRINTING()) card.release();
}

#endif // HAS_MEDIA
