/**
 * Interface from Atmel USB MSD to Marlin SD card
 */

#ifdef ARDUINO_ARCH_SAM

#include "../../../inc/MarlinConfig.h"

#if HAS_MEDIA

#include "../../../sd/cardreader.h"
extern "C" {
  #include "sd_mmc_spi_mem.h"
}

#define SD_MMC_BLOCK_SIZE 512

void sd_mmc_spi_mem_init() {
}

inline bool media_ready() {
  return IS_SD_MOUNTED() && IS_SD_INSERTED() && !IS_SD_FILE_OPEN() && !IS_SD_PRINTING();
}

bool sd_mmc_spi_unload(bool) { return true; }

bool sd_mmc_spi_wr_protect() { return false; }

bool sd_mmc_spi_removal() { return !media_ready(); }

Ctrl_status sd_mmc_spi_test_unit_ready() {
  #ifdef DISABLE_DUE_SD_MMC
    return CTRL_NO_PRESENT;
  #endif
  if (sd_mmc_spi_removal()) return CTRL_NO_PRESENT;
  return CTRL_GOOD;
}

// NOTE: This function is defined as returning the address of the last block
// in the card, which is cardSize() - 1
Ctrl_status sd_mmc_spi_read_capacity(uint32_t *nb_sector) {
  if (sd_mmc_spi_removal()) return CTRL_NO_PRESENT;
  *nb_sector = card.diskIODriver()->cardSize() - 1;
  return CTRL_GOOD;
}

#if ACCESS_USB == true
/**
 * \name MEM <-> USB Interface
 * @{
 */

#include "udi_msc.h"

COMPILER_WORD_ALIGNED
uint8_t sector_buf[SD_MMC_BLOCK_SIZE];

// #define DEBUG_MMC

Ctrl_status sd_mmc_spi_usb_read_10(uint32_t addr, uint16_t nb_sector) {
  #ifdef DISABLE_DUE_SD_MMC
    return CTRL_NO_PRESENT;
  #endif
  if (sd_mmc_spi_removal()) return CTRL_NO_PRESENT;

  #ifdef DEBUG_MMC
  {
    char buffer[80];
    sprintf_P(buffer, PSTR("SDRD: %d @ 0x%08x\n"), nb_sector, addr);
    PORT_REDIRECT(SERIAL_PORTMASK(0));
    SERIAL_ECHO(buffer);
  }
  #endif

  // Start reading
  if (!card.diskIODriver()->readStart(addr))
    return CTRL_FAIL;

  // For each specified sector
  while (nb_sector--) {

    // Read a sector
    card.diskIODriver()->readData(sector_buf);

    // RAM -> USB
    if (!udi_msc_trans_block(true, sector_buf, SD_MMC_BLOCK_SIZE, nullptr)) {
      card.diskIODriver()->readStop();
      return CTRL_FAIL;
    }
  }

  // Stop reading
  card.diskIODriver()->readStop();

  // Done
  return CTRL_GOOD;
}

Ctrl_status sd_mmc_spi_usb_write_10(uint32_t addr, uint16_t nb_sector) {
  #ifdef DISABLE_DUE_SD_MMC
    return CTRL_NO_PRESENT;
  #endif
  if (sd_mmc_spi_removal()) return CTRL_NO_PRESENT;

  #ifdef DEBUG_MMC
  {
    char buffer[80];
    sprintf_P(buffer, PSTR("SDWR: %d @ 0x%08x\n"), nb_sector, addr);
    PORT_REDIRECT(SERIAL_PORTMASK(0));
    SERIAL_ECHO(buffer);
  }
  #endif

  if (!card.diskIODriver()->writeStart(addr, nb_sector))
    return CTRL_FAIL;

  // For each specified sector
  while (nb_sector--) {

    // USB -> RAM
    if (!udi_msc_trans_block(false, sector_buf, SD_MMC_BLOCK_SIZE, nullptr)) {
      card.diskIODriver()->writeStop();
      return CTRL_FAIL;
    }

    // Write a sector
    card.diskIODriver()->writeData(sector_buf);
  }

  // Stop writing
  card.diskIODriver()->writeStop();

  // Done
  return CTRL_GOOD;
}

#endif // ACCESS_USB == true

#endif // HAS_MEDIA
#endif // ARDUINO_ARCH_SAM
