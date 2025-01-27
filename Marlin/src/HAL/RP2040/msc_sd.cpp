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
#include "../platforms.h"

#ifdef __PLAT_RP2040__

#include "../../inc/MarlinConfigPre.h"

#if HAS_SD_HOST_DRIVE

#include "../shared/Marduino.h"
#include "msc_sd.h"
#include "usbd_core.h"

#include "../../sd/cardreader.h"

#include <USB.h>
#include <USBMscHandler.h>

#define BLOCK_SIZE 512
#define PRODUCT_ID 0x29

class Sd2CardUSBMscHandler : public USBMscHandler {
public:
  DiskIODriver* diskIODriver() {
    #if HAS_MULTI_VOLUME
      #if SHARED_VOLUME_IS(SD_ONBOARD)
        return &card.media_driver_sdcard;
      #elif SHARED_VOLUME_IS(USB_FLASH_DRIVE)
        return &card.media_driver_usbFlash;
      #endif
    #else
      return card.diskIODriver();
    #endif
  }

  bool GetCapacity(uint32_t *pBlockNum, uint16_t *pBlockSize) {
    *pBlockNum = diskIODriver()->cardSize();
    *pBlockSize = BLOCK_SIZE;
    return true;
  }

  bool Write(uint8_t *pBuf, uint32_t blkAddr, uint16_t blkLen) {
    auto sd2card = diskIODriver();
    // single block
    if (blkLen == 1) {
      watchdog_refresh();
      sd2card->writeBlock(blkAddr, pBuf);
      return true;
    }

    // multi block optimization
    sd2card->writeStart(blkAddr, blkLen);
    while (blkLen--) {
      watchdog_refresh();
      sd2card->writeData(pBuf);
      pBuf += BLOCK_SIZE;
    }
    sd2card->writeStop();
    return true;
  }

  bool Read(uint8_t *pBuf, uint32_t blkAddr, uint16_t blkLen) {
    auto sd2card = diskIODriver();
    // single block
    if (blkLen == 1) {
      watchdog_refresh();
      sd2card->readBlock(blkAddr, pBuf);
      return true;
    }

    // multi block optimization
    sd2card->readStart(blkAddr);
    while (blkLen--) {
      watchdog_refresh();
      sd2card->readData(pBuf);
      pBuf += BLOCK_SIZE;
    }
    sd2card->readStop();
    return true;
  }

  bool IsReady() {
    return diskIODriver()->isReady();
  }
};

Sd2CardUSBMscHandler usbMscHandler;

/* USB Mass storage Standard Inquiry Data */
uint8_t  Marlin_STORAGE_Inquirydata[] = { /* 36 */
  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'M', 'A', 'R', 'L', 'I', 'N', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0', '1',                     /* Version      : 4 Bytes */
};

USBMscHandler *pSingleMscHandler = &usbMscHandler;

void MSC_SD_init() {
  USBDevice.end();
  delay(200);
  USBDevice.registerMscHandlers(1, &pSingleMscHandler, Marlin_STORAGE_Inquirydata);
  USBDevice.begin();
}

#endif // HAS_SD_HOST_DRIVE
#endif // __PLAT_RP2040__
