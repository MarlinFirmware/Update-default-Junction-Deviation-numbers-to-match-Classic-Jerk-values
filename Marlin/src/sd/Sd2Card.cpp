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
 * Arduino Sd2Card Library
 * Copyright (c) 2009 by William Greiman
 * Updated with backports of the latest SdFat library from the same author
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "../inc/MarlinConfig.h"

#if NEED_SD2CARD_SPI

/* Enable FAST CRC computations - You can trade speed for FLASH space if
 * needed by disabling the following define */
#define FAST_CRC 1

#include "Sd2Card.h"

#include "../MarlinCore.h"

#if DISABLED(SD_NO_DEFAULT_TIMEOUT)
  #ifndef SD_INIT_TIMEOUT
    #define SD_INIT_TIMEOUT 2000u   // (ms) Init timeout
  #elif SD_INIT_TIMEOUT < 0
    #error "SD_INIT_TIMEOUT must be greater than or equal to 0."
  #endif
  #ifndef SD_ERASE_TIMEOUT
    #define SD_ERASE_TIMEOUT 10000u // (ms) Erase timeout
  #elif SD_ERASE_TIMEOUT < 0
    #error "SD_ERASE_TIMEOUT must be greater than or equal to 0."
  #endif
  #ifndef SD_READ_TIMEOUT
    #define SD_READ_TIMEOUT 300u    // (ms) Read timeout
  #elif SD_READ_TIMEOUT < 0
    #error "SD_READ_TIMEOUT must be greater than or equal to 0."
  #endif
  #ifndef SD_WRITE_TIMEOUT
    #define SD_WRITE_TIMEOUT 600u   // (ms) Write timeout
  #elif SD_WRITE_TIMEOUT < 0
    #error "SD_WRITE_TIMEOUT must be greater than or equal to 0."
  #endif
#endif // SD_NO_DEFAULT_TIMEOUT

#if ENABLED(SD_CHECK_AND_RETRY)
  #ifndef SD_RETRY_COUNT
    #define SD_RETRY_COUNT 3
  #elif SD_RETRY_COUNT < 1
    #error "SD_RETRY_COUNT must be greater than or equal to 1."
  #endif
#endif

#if ENABLED(SD_CHECK_AND_RETRY)
  static bool crcSupported = true;

  #ifdef FAST_CRC
    static const uint8_t crctab7[] PROGMEM = {
      0x00,0x09,0x12,0x1B,0x24,0x2D,0x36,0x3F,0x48,0x41,0x5A,0x53,0x6C,0x65,0x7E,0x77,
      0x19,0x10,0x0B,0x02,0x3D,0x34,0x2F,0x26,0x51,0x58,0x43,0x4A,0x75,0x7C,0x67,0x6E,
      0x32,0x3B,0x20,0x29,0x16,0x1F,0x04,0x0D,0x7A,0x73,0x68,0x61,0x5E,0x57,0x4C,0x45,
      0x2B,0x22,0x39,0x30,0x0F,0x06,0x1D,0x14,0x63,0x6A,0x71,0x78,0x47,0x4E,0x55,0x5C,
      0x64,0x6D,0x76,0x7F,0x40,0x49,0x52,0x5B,0x2C,0x25,0x3E,0x37,0x08,0x01,0x1A,0x13,
      0x7D,0x74,0x6F,0x66,0x59,0x50,0x4B,0x42,0x35,0x3C,0x27,0x2E,0x11,0x18,0x03,0x0A,
      0x56,0x5F,0x44,0x4D,0x72,0x7B,0x60,0x69,0x1E,0x17,0x0C,0x05,0x3A,0x33,0x28,0x21,
      0x4F,0x46,0x5D,0x54,0x6B,0x62,0x79,0x70,0x07,0x0E,0x15,0x1C,0x23,0x2A,0x31,0x38,
      0x41,0x48,0x53,0x5A,0x65,0x6C,0x77,0x7E,0x09,0x00,0x1B,0x12,0x2D,0x24,0x3F,0x36,
      0x58,0x51,0x4A,0x43,0x7C,0x75,0x6E,0x67,0x10,0x19,0x02,0x0B,0x34,0x3D,0x26,0x2F,
      0x73,0x7A,0x61,0x68,0x57,0x5E,0x45,0x4C,0x3B,0x32,0x29,0x20,0x1F,0x16,0x0D,0x04,
      0x6A,0x63,0x78,0x71,0x4E,0x47,0x5C,0x55,0x22,0x2B,0x30,0x39,0x06,0x0F,0x14,0x1D,
      0x25,0x2C,0x37,0x3E,0x01,0x08,0x13,0x1A,0x6D,0x64,0x7F,0x76,0x49,0x40,0x5B,0x52,
      0x3C,0x35,0x2E,0x27,0x18,0x11,0x0A,0x03,0x74,0x7D,0x66,0x6F,0x50,0x59,0x42,0x4B,
      0x17,0x1E,0x05,0x0C,0x33,0x3A,0x21,0x28,0x5F,0x56,0x4D,0x44,0x7B,0x72,0x69,0x60,
      0x0E,0x07,0x1C,0x15,0x2A,0x23,0x38,0x31,0x46,0x4F,0x54,0x5D,0x62,0x6B,0x70,0x79
    };

    static uint8_t CRC7(const uint8_t *data, uint8_t n) {
      uint8_t crc = 0;
      while (n > 0) {
        crc = pgm_read_byte(&crctab7[ (crc << 1) ^ *data++ ]);
        n--;
      }
      return (crc << 1) | 1;
    }
  #else
    static uint8_t CRC7(const uint8_t *data, uint8_t n) {
      uint8_t crc = 0;
      for (uint_fast8_t i = 0; i < n; ++i) {
        uint8_t d = data[i];
        d ^= crc << 1;
        if (d & 0x80) d ^= 9;
        crc = d ^ (crc & 0x78) ^ (crc << 4) ^ ((crc >> 3) & 15);
        crc &= 0x7F;
      }
      crc = (crc << 1) ^ (crc << 4) ^ (crc & 0x70) ^ ((crc >> 3) & 0x0F);
      return crc | 1;
    }
  #endif
#endif

// Send command and return error code. Return zero for OK
uint8_t DiskIODriver_SPI_SD::cardCommand(const uint8_t cmd, const uint32_t arg) {

  #if ENABLED(SDCARD_COMMANDS_SPLIT)
    if (cmd != CMD12) chipDeselect();
  #endif

  // Select card
  chipSelect();

  #if SD_WRITE_TIMEOUT
    waitNotBusy(SD_WRITE_TIMEOUT);  // Wait up to 600 ms (by default) if busy
  #endif

  uint8_t *pa = (uint8_t *)(&arg);

  #if ENABLED(SD_CHECK_AND_RETRY)

    // Form message
    uint8_t d[6] = { uint8_t(cmd | 0x40), pa[3], pa[2], pa[1], pa[0] };

    // Add crc
    d[5] = CRC7(d, 5);

    // Send message
    for (uint_fast8_t k = 0; k < 6; ++k) spiSend(d[k]);

  #else
    // Send command
    spiSend(cmd | 0x40);

    // Send argument
    for (int8_t i = 3; i >= 0; i--) spiSend(pa[i]);

    // Send CRC - correct for CMD0 with arg zero or CMD8 with arg 0X1AA
    spiSend(cmd == CMD0 ? 0X95 : 0X87);
  #endif

  // Skip stuff byte for stop read
  if (cmd == CMD12) spiRec();

  // Wait for response
  for (uint_fast8_t i = 0; ((status_ = spiRec()) & 0x80) && i != 0xFF; i++) { /* Intentionally left empty */ }
  return status_;
}

/**
 * Determine the size of an SD flash memory card.
 *
 * \return The number of 512 byte data blocks in the card
 *         or zero if an error occurs.
 */
uint32_t DiskIODriver_SPI_SD::cardSize() {
  csd_t csd;
  if (!readCSD(&csd)) return 0;
  if (csd.v1.csd_ver == 0) {
    uint8_t read_bl_len = csd.v1.read_bl_len;
    uint16_t c_size = (csd.v1.c_size_high << 10)
                      | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
    uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
                          | csd.v1.c_size_mult_low;
    return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
  }
  else if (csd.v2.csd_ver == 1) {
    uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16)
                      | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
    return (c_size + 1) << 10;
  }
  else {
    error(SD_CARD_ERROR_BAD_CSD);
    return 0;
  }
}

void DiskIODriver_SPI_SD::chipDeselect() {
  extDigitalWrite(chipSelectPin_, HIGH);
  spiSend(0xFF); // Ensure MISO goes high impedance
}

void DiskIODriver_SPI_SD::chipSelect() {
  spiInit(spiRate_);
  extDigitalWrite(chipSelectPin_, LOW);
}

/**
 * Erase a range of blocks.
 *
 * \param[in] firstBlock The address of the first block in the range.
 * \param[in] lastBlock The address of the last block in the range.
 *
 * \note This function requests the SD card to do a flash erase for a
 * range of blocks.  The data on the card after an erase operation is
 * either 0 or 1, depends on the card vendor.  The card must support
 * single block erase.
 *
 * \return true for success, false for failure.
 */
bool DiskIODriver_SPI_SD::erase(uint32_t firstBlock, uint32_t lastBlock) {
  if (ENABLED(SDCARD_READONLY)) return false;

  bool success = false;
  do {

    csd_t csd;
    if (!readCSD(&csd)) break;

    // check for single block erase
    if (!csd.v1.erase_blk_en) {
      // erase size mask
      uint8_t m = (csd.v1.sector_size_high << 1) | csd.v1.sector_size_low;
      if ((firstBlock & m) || ((lastBlock + 1) & m)) {
        // error card can't erase specified area
        error(SD_CARD_ERROR_ERASE_SINGLE_BLOCK);
        break;
      }
    }
    if (type_ != SD_CARD_TYPE_SDHC) { firstBlock <<= 9; lastBlock <<= 9; }
    if (cardCommand(CMD32, firstBlock) || cardCommand(CMD33, lastBlock) || cardCommand(CMD38, 0)) {
      error(SD_CARD_ERROR_ERASE);
      break;
    }
    #if SD_ERASE_TIMEOUT
      if (!waitNotBusy(SD_ERASE_TIMEOUT)) {
        error(SD_CARD_ERROR_ERASE_TIMEOUT);
        break;
      }
    #else
      while (spiRec() != 0xFF) {}
    #endif

    success = true;

  } while (0);

  chipDeselect();
  return success;
}

/**
 * Determine if card supports single block erase.
 *
 * \return true if single block erase is supported.
 *         false if single block erase is not supported.
 */
bool DiskIODriver_SPI_SD::eraseSingleBlockEnable() {
  csd_t csd;
  return readCSD(&csd) ? csd.v1.erase_blk_en : false;
}

/**
 * Initialize an SD flash memory card.
 *
 * \param[in] sckRateID SPI clock rate selector. See setSckRate().
 * \param[in] chipSelectPin SD chip select pin number.
 *
 * \return true for success, false for failure.
 * The reason for failure can be determined by calling errorCode() and errorData().
 */
bool DiskIODriver_SPI_SD::init(const uint8_t sckRateID, const pin_t chipSelectPin) {
  #if IS_TEENSY_35_36 || IS_TEENSY_40_41
    chipSelectPin_ = BUILTIN_SDCARD;
    const uint8_t ret = SDHC_CardInit();
    type_ = SDHC_CardGetType();
    return (ret == 0);
  #endif

  errorCode_ = type_ = 0;
  chipSelectPin_ = chipSelectPin;

  // 16-bit init start time allows over a minute
  #if SD_INIT_TIMEOUT
    const millis_t init_timeout = millis() + SD_INIT_TIMEOUT;
    #define INIT_TIMEOUT() ELAPSED(millis(), init_timeout)
  #else
    #define INIT_TIMEOUT() false
  #endif

  uint32_t arg;

  hal.watchdog_refresh(); // In case init takes too long

  // Set pin modes
  #if ENABLED(ZONESTAR_12864OLED)
    if (chipSelectPin_ != DOGLCD_CS) {
      SET_OUTPUT(DOGLCD_CS);
      WRITE(DOGLCD_CS, HIGH);
    }
  #else
    extDigitalWrite(chipSelectPin_, HIGH);  // For some CPUs pinMode can write the wrong data so init desired data value first
    pinMode(chipSelectPin_, OUTPUT);        // Solution for #8746 by @benlye
  #endif
  spiBegin();

  // Set SCK rate for initialization commands
  spiRate_ = SPI_SD_INIT_RATE;
  spiInit(spiRate_);

  // Must supply min of 74 clock cycles with CS high.
  for (uint_fast8_t i = 0; i < 10; ++i) spiSend(0xFF);

  hal.watchdog_refresh(); // In case init takes too long

  // Command to go idle in SPI mode
  while ((status_ = cardCommand(CMD0, 0)) != R1_IDLE_STATE) {
    if (INIT_TIMEOUT()) {
      error(SD_CARD_ERROR_CMD0);
      goto FAIL;
    }
  }

  #if ENABLED(SD_CHECK_AND_RETRY)
    crcSupported = (cardCommand(CMD59, 1) == R1_IDLE_STATE);
  #endif

  hal.watchdog_refresh(); // In case init takes too long

  // check SD version
  for (;;) {
    if (cardCommand(CMD8, 0x1AA) == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE)) {
      type(SD_CARD_TYPE_SD1);
      break;
    }

    // Get the last byte of r7 response
    for (uint_fast8_t i = 0; i < 4; ++i) status_ = spiRec();
    if (status_ == 0xAA) {
      type(SD_CARD_TYPE_SD2);
      break;
    }

    if (INIT_TIMEOUT()) {
      error(SD_CARD_ERROR_CMD8);
      goto FAIL;
    }
  }

  hal.watchdog_refresh(); // In case init takes too long

  // Initialize card and send host supports SDHC if SD2
  arg = type() == SD_CARD_TYPE_SD2 ? 0x40000000 : 0;
  while ((status_ = cardAcmd(ACMD41, arg)) != R1_READY_STATE) {
    // Check for timeout
    if (INIT_TIMEOUT()) {
      error(SD_CARD_ERROR_ACMD41);
      goto FAIL;
    }
  }

  // If SD2 read OCR register to check for SDHC card
  if (type() == SD_CARD_TYPE_SD2) {
    if (cardCommand(CMD58, 0)) {
      error(SD_CARD_ERROR_CMD58);
      goto FAIL;
    }
    if ((spiRec() & 0xC0) == 0xC0) type(SD_CARD_TYPE_SDHC);
    // Discard rest of ocr - contains allowed voltage range
    for (uint_fast8_t i = 0; i < 3; ++i) spiRec();
  }

  chipDeselect();

  ready = true;
  return setSckRate(sckRateID);

  FAIL:
  chipDeselect();
  ready = false;
  return false;
}

/**
 * Read a 512 byte block from an SD card.
 *
 * \param[in] blockNumber Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data.
 * \return true for success, false for failure.
 */
bool DiskIODriver_SPI_SD::readBlock(uint32_t blockNumber, uint8_t * const dst) {
  #if IS_TEENSY_35_36 || IS_TEENSY_40_41
    return 0 == SDHC_CardReadBlock(dst, blockNumber);
  #endif

  if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;   // Use address if not SDHC card

  #if ENABLED(SD_CHECK_AND_RETRY)
    uint8_t retryCnt = SD_RETRY_COUNT;
    for (;;) {
      if (cardCommand(CMD17, blockNumber))
        error(SD_CARD_ERROR_CMD17);
      else if (readData(dst, 512))
        return true;

      chipDeselect();
      if (!--retryCnt) break;

      cardCommand(CMD12, 0); // Try sending a stop command, ignore the result.
      errorCode_ = 0;
    }
    return false;
  #else
    if (cardCommand(CMD17, blockNumber)) {
      error(SD_CARD_ERROR_CMD17);
      chipDeselect();
      return false;
    }
    else
      return readData(dst, 512);
  #endif
}

/**
 * Read one data block in a multiple block read sequence
 *
 * \param[in] dst Pointer to the location for the data to be read.
 *
 * \return true for success, false for failure.
 */
bool DiskIODriver_SPI_SD::readData(uint8_t * const dst) {
  chipSelect();
  return readData(dst, 512);
}

#if ENABLED(SD_CHECK_AND_RETRY)
  #ifdef FAST_CRC
    static const uint16_t crctab16[] PROGMEM = {
      0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
      0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
      0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
      0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
      0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
      0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
      0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
      0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
      0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
      0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
      0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
      0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
      0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
      0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
      0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
      0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
      0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
      0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
      0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
      0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
      0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
      0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
      0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
      0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
      0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
      0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
      0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
      0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
      0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
      0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
      0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
      0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
    };
    // faster CRC-CCITT
    // uses the x^16,x^12,x^5,x^1 polynomial.
    static uint16_t CRC_CCITT(const uint8_t *data, size_t n) {
      uint16_t crc = 0;
      for (size_t i = 0; i < n; i++)
        crc = pgm_read_word(&crctab16[(crc >> 8 ^ data[i]) & 0xFF]) ^ (crc << 8);
      return crc;
    }

  #else

    // slower CRC-CCITT
    // uses the x^16,x^12,x^5,x^1 polynomial.
    static uint16_t CRC_CCITT(const uint8_t *data, size_t n) {
      uint16_t crc = 0;
      for (size_t i = 0; i < n; i++) {
        crc = (uint8_t)(crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (uint8_t)(crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
      }
      return crc;
    }

  #endif

#endif // SD_CHECK_AND_RETRY

bool DiskIODriver_SPI_SD::readData(uint8_t * const dst, const uint16_t count) {
  bool success = false;

  #if SD_READ_TIMEOUT
    const millis_t read_timeout = millis() + SD_READ_TIMEOUT;
    #define READ_TIMEOUT() ELAPSED(millis(), read_timeout)
  #else
    #define READ_TIMEOUT() false
  #endif

  while ((status_ = spiRec()) == 0xFF) {      // Wait for start block token
    if (READ_TIMEOUT()) {
      error(SD_CARD_ERROR_READ_TIMEOUT);
      goto FAIL;
    }
  }

  if (status_ == DATA_START_BLOCK) {
    spiRead(dst, count);                      // Transfer data

    const uint16_t recvCrc = ((uint16_t)spiRec() << 8) | (uint16_t)spiRec();
    #if ENABLED(SD_CHECK_AND_RETRY)
      success = !crcSupported || recvCrc == CRC_CCITT(dst, count);
      if (!success) error(SD_CARD_ERROR_READ_CRC);
    #else
      success = true;
      UNUSED(recvCrc);
    #endif
  }
  else
    error(SD_CARD_ERROR_READ);

  FAIL:
  chipDeselect();
  return success;
}

/** read CID or CSR register */
bool DiskIODriver_SPI_SD::readRegister(const uint8_t cmd, void * const buf) {
  uint8_t * const dst = reinterpret_cast<uint8_t*>(buf);
  if (cardCommand(cmd, 0)) {
    error(SD_CARD_ERROR_READ_REG);
    chipDeselect();
    return false;
  }
  return readData(dst, 16);
}

/**
 * Start a read multiple blocks sequence.
 *
 * \param[in] blockNumber Address of first block in sequence.
 *
 * \note This function is used with readData() and readStop() for optimized
 * multiple block reads.  SPI chipSelect must be low for the entire sequence.
 *
 * \return true for success, false for failure.
 */
bool DiskIODriver_SPI_SD::readStart(uint32_t blockNumber) {
  if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;

  const bool success = !cardCommand(CMD18, blockNumber);
  if (!success) error(SD_CARD_ERROR_CMD18);
  chipDeselect();
  return success;
}

/**
 * End a read multiple blocks sequence.
 *
 * \return true for success, false for failure.
 */
bool DiskIODriver_SPI_SD::readStop() {
  chipSelect();
  const bool success = !cardCommand(CMD12, 0);
  if (!success) error(SD_CARD_ERROR_CMD12);
  chipDeselect();
  return success;
}

/**
 * Set the SPI clock rate.
 *
 * \param[in] sckRateID A value in the range [0, 6].
 *
 * The SPI clock will be set to F_CPU/pow(2, 1 + sckRateID). The maximum
 * SPI rate is F_CPU/2 for \a sckRateID = 0 and the minimum rate is F_CPU/128
 * for \a scsRateID = 6.
 *
 * \return The value one, true, is returned for success and the value zero,
 * false, is returned for an invalid value of \a sckRateID.
 */
bool DiskIODriver_SPI_SD::setSckRate(const uint8_t sckRateID) {
  const bool success = (sckRateID <= 6);
  if (success) spiRate_ = sckRateID; else error(SD_CARD_ERROR_SCK_RATE);
  return success;
}

/**
 * Wait for card to become not-busy
 * \param[in] timeout_ms Timeout to abort.
 * \return true for success, false for timeout.
 */
bool DiskIODriver_SPI_SD::waitNotBusy(const millis_t timeout_ms) {
  const millis_t wait_timeout = millis() + timeout_ms;
  while (spiRec() != 0xFF) if (ELAPSED(millis(), wait_timeout)) return false;
  return true;
}

void DiskIODriver_SPI_SD::error(const uint8_t code) { errorCode_ = code; }

/**
 * Write a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return true for success, false for failure.
 */
bool DiskIODriver_SPI_SD::writeBlock(uint32_t blockNumber, const uint8_t * const src) {
  if (ENABLED(SDCARD_READONLY)) return false;

  #if IS_TEENSY_35_36 || IS_TEENSY_40_41
    return 0 == SDHC_CardWriteBlock(src, blockNumber);
  #endif

  if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9; // Use address if not SDHC card
  bool success = !cardCommand(CMD24, blockNumber);
  if (!success) {
    error(SD_CARD_ERROR_CMD24);
  }
  else if (writeData(DATA_START_BLOCK, src)) {
    #if SD_WRITE_TIMEOUT
      success = waitNotBusy(SD_WRITE_TIMEOUT);        // Wait for flashing to complete
      if (!success) error(SD_CARD_ERROR_WRITE_TIMEOUT);
    #else
      while (spiRec() != 0xFF) {}
    #endif
    if (success) {
      success = !(cardCommand(CMD13, 0) || spiRec()); // Response is r2 so get and check two bytes for nonzero
      if (!success) error(SD_CARD_ERROR_WRITE_PROGRAMMING);
    }
  }

  chipDeselect();
  return success;
}

/**
 * Write one data block in a multiple block write sequence
 * \param[in] src Pointer to the location of the data to be written.
 * \return true for success, false for failure.
 */
bool DiskIODriver_SPI_SD::writeData(const uint8_t * const src) {
  if (ENABLED(SDCARD_READONLY)) return false;

  chipSelect();

  bool success = false;
  do {

    // Wait for previous write to finish
    #if SD_WRITE_TIMEOUT
      if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
        error(SD_CARD_ERROR_WRITE_MULTIPLE);
        break;
      }
    #else
      while (spiRec() != 0xFF) {}
    #endif

    success = writeData(WRITE_MULTIPLE_TOKEN, src);

  } while (0);

  chipDeselect();
  return success;
}

// Send one block of data for write block or write multiple blocks
bool DiskIODriver_SPI_SD::writeData(const uint8_t token, const uint8_t * const src) {
  if (ENABLED(SDCARD_READONLY)) return false;

  const uint16_t crc = TERN(SD_CHECK_AND_RETRY, CRC_CCITT(src, 512), 0xFFFF);
  spiSendBlock(token, src);
  spiSend(crc >> 8);
  spiSend(crc & 0xFF);

  status_ = spiRec();
  if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
    error(SD_CARD_ERROR_WRITE);
    chipDeselect();
    return false;
  }
  return true;
}

/**
 * Start a write multiple blocks sequence.
 *
 * \param[in] blockNumber Address of first block in sequence.
 * \param[in] eraseCount The number of blocks to be pre-erased.
 *
 * \note This function is used with writeData() and writeStop()
 * for optimized multiple block writes.
 *
 * \return true for success, false for failure.
 */
bool DiskIODriver_SPI_SD::writeStart(uint32_t blockNumber, const uint32_t eraseCount) {
  if (ENABLED(SDCARD_READONLY)) return false;

  bool success = false;
  if (!cardAcmd(ACMD23, eraseCount)) {                    // Send pre-erase count
    if (type() != SD_CARD_TYPE_SDHC) blockNumber <<= 9;   // Use address if not SDHC card
    success = !cardCommand(CMD25, blockNumber);
    if (!success) error(SD_CARD_ERROR_CMD25);
  }
  else
    error(SD_CARD_ERROR_ACMD23);

  chipDeselect();
  return success;
}

/**
 * End a write multiple blocks sequence.
 *
 * \return true for success, false for failure.
 */
bool DiskIODriver_SPI_SD::writeStop() {
  if (ENABLED(SDCARD_READONLY)) return false;

  chipSelect();

  bool success = false;
  do {

    #if SD_WRITE_TIMEOUT
      if (!waitNotBusy(SD_WRITE_TIMEOUT)) {
        error(SD_CARD_ERROR_STOP_TRAN);
        break;
      }
    #else
      while (spiRec() != 0xFF) {}
    #endif

    spiSend(STOP_TRAN_TOKEN);

    #if SD_WRITE_TIMEOUT
      if (!waitNotBusy(SD_WRITE_TIMEOUT)) break;
    #else
      while (spiRec() != 0xFF) {}
    #endif

    success = true;

  } while (0);

  chipDeselect();
  return success;
}

#endif // NEED_SD2CARD_SPI
