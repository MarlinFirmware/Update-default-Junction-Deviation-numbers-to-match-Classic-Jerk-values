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
#ifdef ARDUINO_ARCH_SAM

#include "../../inc/MarlinConfig.h"

#if ENABLED(FLASH_EEPROM_EMULATION)

/* EEPROM emulation over flash with reduced wear
 *
 * We will use 2 contiguous groups of pages as main and alternate.
 * We want an structure that allows to read as fast as possible,
 * without the need of scanning the whole FLASH memory.
 *
 * FLASH bits default erased state is 1, and can be set to 0
 * on a per bit basis. To reset them to 1, a full page erase
 * is needed.
 *
 * Values are stored as differences that should be applied to a
 * completely erased EEPROM (filled with 0xFFs). We just encode
 * the starting address of the values to change, the length of
 * the block of new values, and the values themselves. All diffs
 * are accumulated into a RAM buffer, compacted into the least
 * amount of non overlapping diffs possible and sorted by starting
 * address before being saved into the next available page of FLASH
 * of the current group.
 * Once the current group is completely full, we compact it and save
 * it into the other group, then erase the current group and switch
 * to that new group and set it as current.
 *
 * The FLASH endurance is about 1/10 ... 1/100 of an EEPROM
 * endurance, but EEPROM endurance is specified per byte, not
 * per page. We can't emulate EE endurance with FLASH for all
 * bytes, but we can emulate endurance for a given percent of
 * bytes.
 */

//#define EE_EMU_DEBUG

#define EEPROMSize     4096
#define PagesPerGroup   128
#define GroupCount        2
#define PageSize        256U

 /* Flash storage */
typedef struct FLASH_SECTOR {
  uint8_t page[PageSize];
} FLASH_SECTOR_T;

#define PAGE_FILL \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF

#define FLASH_INIT_FILL \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL

/* This is the FLASH area used to emulate a 2Kbyte EEPROM  -- We need this buffer aligned
   to a 256 byte boundary. */
static const uint8_t flashStorage[PagesPerGroup * GroupCount * PageSize] __attribute__ ((aligned (PageSize))) = { FLASH_INIT_FILL };

/* Get the address of an specific page */
static const FLASH_SECTOR_T* getFlashStorage(int page) {
  return (const FLASH_SECTOR_T*)&flashStorage[page*PageSize];
}

static uint8_t buffer[256] = {0},   // The RAM buffer to accumulate writes
               curPage = 0,         // Current FLASH page inside the group
               curGroup = 0xFF;     // Current FLASH group

#define DEBUG_OUT ENABLED(EE_EMU_DEBUG)
#include "../../core/debug_out.h"

static void ee_Dump(const int page, const void *data) {

  #ifdef EE_EMU_DEBUG

    const uint8_t *c = (const uint8_t*) data;
    char buffer[80];

    sprintf_P(buffer, PSTR("Page: %d (0x%04x)\n"), page, page);
    DEBUG_ECHO(buffer);

    char* p = &buffer[0];
    for (int i = 0; i< PageSize; ++i) {
      if ((i & 0xF) == 0) p += sprintf_P(p, PSTR("%04x] "), i);

      p += sprintf_P(p, PSTR(" %02x"), c[i]);
      if ((i & 0xF) == 0xF) {
        *p++ = '\n';
        *p = 0;
        DEBUG_ECHO(buffer);
        p = &buffer[0];
      }
    }

  #else
    UNUSED(page);
    UNUSED(data);
  #endif
}

/* Flash Writing Protection Key */
#define FWP_KEY    0x5Au

#if SAM4S_SERIES
  #define EEFC_FCR_FCMD(value) \
  ((EEFC_FCR_FCMD_Msk & ((value) << EEFC_FCR_FCMD_Pos)))
  #define EEFC_ERROR_FLAGS  (EEFC_FSR_FLOCKE | EEFC_FSR_FCMDE | EEFC_FSR_FLERR)
#else
  #define EEFC_ERROR_FLAGS  (EEFC_FSR_FLOCKE | EEFC_FSR_FCMDE)
#endif

/**
 * Writes the contents of the specified page (no previous erase)
 * @param page    (page #)
 * @param data    (pointer to the data buffer)
 */
__attribute__ ((long_call, section (".ramfunc")))
static bool ee_PageWrite(uint16_t page, const void *data) {

  uint16_t i;
  uint32_t addrflash = uint32_t(getFlashStorage(page));

  // Read the flash contents
  uint32_t pageContents[PageSize>>2];
  memcpy(pageContents, (void*)addrflash, PageSize);

  // We ONLY want to toggle bits that have changed, and that have changed to 0.
  // SAM3X8E tends to destroy contiguous bits if reprogrammed without erasing, so
  // we try by all means to avoid this. That is why it says: "The Partial
  // Programming mode works only with 128-bit (or higher) boundaries. It cannot
  // be used with boundaries lower than 128 bits (8, 16 or 32-bit for example)."
  // All bits that did not change, set them to 1.
  for (i = 0; i <PageSize >> 2; i++)
    pageContents[i] = (((uint32_t*)data)[i]) | (~(pageContents[i] ^ ((uint32_t*)data)[i]));

  DEBUG_ECHO_MSG("EEPROM PageWrite   ", page);
  DEBUG_ECHOLNPGM(" in FLASH address ", (uint32_t)addrflash);
  DEBUG_ECHOLNPGM(" base address     ", (uint32_t)getFlashStorage(0));
  DEBUG_FLUSH();

  // Get the page relative to the start of the EFC controller, and the EFC controller to use
  Efc *efc;
  uint16_t fpage;
  if (addrflash >= IFLASH1_ADDR) {
    efc = EFC1;
    fpage = (addrflash - IFLASH1_ADDR) / IFLASH1_PAGE_SIZE;
  }
  else {
    efc = EFC0;
    fpage = (addrflash - IFLASH0_ADDR) / IFLASH0_PAGE_SIZE;
  }

  // Get the page that must be unlocked, then locked
  uint16_t lpage = fpage & (~((IFLASH0_LOCK_REGION_SIZE / IFLASH0_PAGE_SIZE) - 1));

  // Disable all interrupts
  __disable_irq();

  // Get the FLASH wait states
  uint32_t orgWS = (efc->EEFC_FMR & EEFC_FMR_FWS_Msk) >> EEFC_FMR_FWS_Pos;

  // Set wait states to 6 (SAM errata)
  efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(6);

  // Unlock the flash page
  uint32_t status;
  efc->EEFC_FCR = EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(lpage) | EEFC_FCR_FCMD(EFC_FCMD_CLB);
  while (((status = efc->EEFC_FSR) & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) {
    // force compiler to not optimize this -- NOPs don't work!
    __asm__ __volatile__("");
  };

  if ((status & EEFC_ERROR_FLAGS) != 0) {

    // Restore original wait states
    efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

    // Reenable interrupts
    __enable_irq();

    DEBUG_ECHO_MSG("EEPROM Unlock failure for page ", page);
    return false;
  }

  // Write page and lock:  Writing 8-bit and 16-bit data is not allowed and may lead to unpredictable data corruption.
  const uint32_t * aligned_src = (const uint32_t *) &pageContents[0]; /*data;*/
  uint32_t * p_aligned_dest = (uint32_t *) addrflash;
  for (i = 0; i < (IFLASH0_PAGE_SIZE / sizeof(uint32_t)); ++i) {
    *p_aligned_dest++ = *aligned_src++;
  }
  efc->EEFC_FCR = EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(fpage) | EEFC_FCR_FCMD(EFC_FCMD_WPL);
  while (((status = efc->EEFC_FSR) & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) {
    // force compiler to not optimize this -- NOPs don't work!
    __asm__ __volatile__("");
  };

  if ((status & EEFC_ERROR_FLAGS) != 0) {

    // Restore original wait states
    efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

    // Reenable interrupts
    __enable_irq();

    DEBUG_ECHO_MSG("EEPROM Write failure for page ", page);

    return false;
  }

  // Restore original wait states
  efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

  // Reenable interrupts
  __enable_irq();

  // Compare contents
  if (memcmp(getFlashStorage(page),data,PageSize)) {

    #ifdef EE_EMU_DEBUG
      DEBUG_ECHO_MSG("EEPROM Verify Write failure for page ", page);

      ee_Dump( page, (uint32_t *)addrflash);
      ee_Dump(-page, data);

      // Calculate count of changed bits
      uint32_t *p1 = (uint32_t*)addrflash;
      uint32_t *p2 = (uint32_t*)data;
      int count = 0;
      for (i = 0; i < PageSize >> 2; i++) {
        if (p1[i] != p2[i]) {
          uint32_t delta = p1[i] ^ p2[i];
          while (delta) {
            if ((delta&1) != 0)
              count++;
            delta >>= 1;
          }
        }
      }
      DEBUG_ECHOLNPGM("--> Differing bits: ", count);
    #endif

    return false;
  }

  return true;
}

/**
 * Erases the contents of the specified page
 * @param page    (page #)
  */
__attribute__ ((long_call, section (".ramfunc")))
static bool ee_PageErase(uint16_t page) {

  uint16_t i;
  uint32_t addrflash = uint32_t(getFlashStorage(page));

  DEBUG_ECHO_MSG("EEPROM PageErase  ", page);
  DEBUG_ECHOLNPGM(" in FLASH address ", (uint32_t)addrflash);
  DEBUG_ECHOLNPGM(" base address     ", (uint32_t)getFlashStorage(0));
  DEBUG_FLUSH();

  // Get the page relative to the start of the EFC controller, and the EFC controller to use
  Efc *efc;
  uint16_t fpage;
  if (addrflash >= IFLASH1_ADDR) {
    efc = EFC1;
    fpage = (addrflash - IFLASH1_ADDR) / IFLASH1_PAGE_SIZE;
  }
  else {
    efc = EFC0;
    fpage = (addrflash - IFLASH0_ADDR) / IFLASH0_PAGE_SIZE;
  }

  // Get the page that must be unlocked, then locked
  uint16_t lpage = fpage & (~((IFLASH0_LOCK_REGION_SIZE / IFLASH0_PAGE_SIZE) - 1));

  // Disable all interrupts
  __disable_irq();

  // Get the FLASH wait states
  uint32_t orgWS = (efc->EEFC_FMR & EEFC_FMR_FWS_Msk) >> EEFC_FMR_FWS_Pos;

  // Set wait states to 6 (SAM errata)
  efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(6);

  // Unlock the flash page
  uint32_t status;
  efc->EEFC_FCR = EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(lpage) | EEFC_FCR_FCMD(EFC_FCMD_CLB);
  while (((status = efc->EEFC_FSR) & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) {
    // force compiler to not optimize this -- NOPs don't work!
    __asm__ __volatile__("");
  };
  if ((status & EEFC_ERROR_FLAGS) != 0) {

    // Restore original wait states
    efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

    // Reenable interrupts
    __enable_irq();

    DEBUG_ECHO_MSG("EEPROM Unlock failure for page ",page);

    return false;
  }

  // Erase Write page and lock: Writing 8-bit and 16-bit data is not allowed and may lead to unpredictable data corruption.
  uint32_t * p_aligned_dest = (uint32_t *) addrflash;
  for (i = 0; i < (IFLASH0_PAGE_SIZE / sizeof(uint32_t)); ++i) {
    *p_aligned_dest++ = 0xFFFFFFFF;
  }
  efc->EEFC_FCR = EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(fpage) | EEFC_FCR_FCMD(EFC_FCMD_EWPL);
  while (((status = efc->EEFC_FSR) & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) {
    // force compiler to not optimize this -- NOPs don't work!
    __asm__ __volatile__("");
  };
  if ((status & EEFC_ERROR_FLAGS) != 0) {

    // Restore original wait states
    efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

    // Reenable interrupts
    __enable_irq();

    DEBUG_ECHO_MSG("EEPROM Erase failure for page ",page);

    return false;
  }

  // Restore original wait states
  efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

  // Reenable interrupts
  __enable_irq();

  // Check erase
  uint32_t * aligned_src = (uint32_t *) addrflash;
  for (i = 0; i < PageSize >> 2; i++) {
    if (*aligned_src++ != 0xFFFFFFFF) {
      DEBUG_ECHO_MSG("EEPROM Verify Erase failure for page ",page);
      ee_Dump(page, (uint32_t *)addrflash);
      return false;
    }
  }

  return true;
}

static uint8_t ee_Read(uint32_t address, bool excludeRAMBuffer=false) {

  uint32_t baddr;
  uint32_t blen;

  // If we were requested an address outside of the emulated range, fail now
  if (address >= EEPROMSize)
    return false;

  // Check that the value is not contained in the RAM buffer
  if (!excludeRAMBuffer) {
    uint16_t i = 0;
    while (i <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

      // Get the address of the block
      baddr = buffer[i] | (buffer[i + 1] << 8);

      // Get the length of the block
      blen = buffer[i + 2];

      // If we reach the end of the list, break loop
      if (blen == 0xFF)
        break;

      // Check if data is contained in this block
      if (address >= baddr &&
        address < (baddr + blen)) {

        // Yes, it is contained. Return it!
        return buffer[i + 3 + address - baddr];
      }

      // As blocks are always sorted, if the starting address of this block is higher
      // than the address we are looking for, break loop now - We wont find the value
      // associated to the address
      if (baddr > address)
        break;

      // Jump to the next block
      i += 3 + blen;
    }
  }

  // It is NOT on the RAM buffer. It could be stored in FLASH. We are
  //  ensured on a given FLASH page, address contents are never repeated
  //  but on different pages, there is no such warranty, so we must go
  //  backwards from the last written FLASH page to the first one.
  for (int page = curPage - 1; page >= 0; --page) {

    // Get a pointer to the flash page
    uint8_t *pflash = (uint8_t*)getFlashStorage(page + curGroup * PagesPerGroup);

    uint16_t i = 0;
    while (i <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

      // Get the address of the block
      baddr = pflash[i] | (pflash[i + 1] << 8);

      // Get the length of the block
      blen = pflash[i + 2];

      // If we reach the end of the list, break loop
      if (blen == 0xFF)
        break;

      // Check if data is contained in this block
      if (address >= baddr && address < (baddr + blen))
        return pflash[i + 3 + address - baddr]; // Yes, it is contained. Return it!

      // As blocks are always sorted, if the starting address of this block is higher
      // than the address we are looking for, break loop now - We wont find the value
      // associated to the address
      if (baddr > address) break;

      // Jump to the next block
      i += 3 + blen;
    }
  }

  // If reached here, value is not stored, so return its default value
  return 0xFF;
}

static uint32_t ee_GetAddrRange(uint32_t address, bool excludeRAMBuffer=false) {
  uint32_t baddr,
           blen,
           nextAddr = 0xFFFF,
           nextRange = 0;

  // Check that the value is not contained in the RAM buffer
  if (!excludeRAMBuffer) {
    uint16_t i = 0;
    while (i <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

      // Get the address of the block
      baddr = buffer[i] | (buffer[i + 1] << 8);

      // Get the length of the block
      blen = buffer[i + 2];

      // If we reach the end of the list, break loop
      if (blen == 0xFF) break;

      // Check if address and address + 1 is contained in this block
      if (address >= baddr && address < (baddr + blen))
        return address | ((blen - address + baddr) << 16); // Yes, it is contained. Return it!

      // Otherwise, check if we can use it as a limit
      if (baddr > address && baddr < nextAddr) {
        nextAddr = baddr;
        nextRange = blen;
      }

      // As blocks are always sorted, if the starting address of this block is higher
      // than the address we are looking for, break loop now - We wont find the value
      // associated to the address
      if (baddr > address) break;

      // Jump to the next block
      i += 3 + blen;
    }
  }

  // It is NOT on the RAM buffer. It could be stored in FLASH. We are
  //  ensured on a given FLASH page, address contents are never repeated
  //  but on different pages, there is no such warranty, so we must go
  //  backwards from the last written FLASH page to the first one.
  for (int page = curPage - 1; page >= 0; --page) {

    // Get a pointer to the flash page
    uint8_t *pflash = (uint8_t*)getFlashStorage(page + curGroup * PagesPerGroup);

    uint16_t i = 0;
    while (i <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

      // Get the address of the block
      baddr = pflash[i] | (pflash[i + 1] << 8);

      // Get the length of the block
      blen = pflash[i + 2];

      // If we reach the end of the list, break loop
      if (blen == 0xFF) break;

      // Check if data is contained in this block
      if (address >= baddr && address < (baddr + blen))
        return address | ((blen - address + baddr) << 16); // Yes, it is contained. Return it!

      // Otherwise, check if we can use it as a limit
      if (baddr > address && baddr < nextAddr) {
        nextAddr = baddr;
        nextRange = blen;
      }

      // As blocks are always sorted, if the starting address of this block is higher
      // than the address we are looking for, break loop now - We wont find the value
      // associated to the address
      if (baddr > address) break;

      // Jump to the next block
      i += 3 + blen;
    }
  }

  // If reached here, we will return the next valid address
  return nextAddr | (nextRange << 16);
}

static bool ee_IsPageClean(int page) {
  uint32_t *pflash = (uint32_t*) getFlashStorage(page);
  for (uint16_t i = 0; i < (PageSize >> 2); ++i)
    if (*pflash++ != 0xFFFFFFFF) return false;
  return true;
}

static bool ee_Flush(uint32_t overrideAddress = 0xFFFFFFFF, uint8_t overrideData=0xFF) {

  // Check if RAM buffer has something to be written
  bool isEmpty = true;
  uint32_t *p = (uint32_t*) &buffer[0];
  for (uint16_t j = 0; j < (PageSize >> 2); j++) {
    if (*p++ != 0xFFFFFFFF) {
      isEmpty = false;
      break;
    }
  }

  // If something has to be written, do so!
  if (!isEmpty) {

    // Write the current ram buffer into FLASH
    ee_PageWrite(curPage + curGroup * PagesPerGroup, buffer);

    // Clear the RAM buffer
    memset(buffer, 0xFF, sizeof(buffer));

    // Increment the page to use the next time
    ++curPage;
  }

  // Did we reach the maximum count of available pages per group for storage ?
  if (curPage < PagesPerGroup) {

    // Do we have an override address ?
    if (overrideAddress < EEPROMSize) {

      // Yes, just store the value into the RAM buffer
      buffer[0] = overrideAddress & 0xFF;
      buffer[0 + 1] = (overrideAddress >> 8) & 0xFF;
      buffer[0 + 2] = 1;
      buffer[0 + 3] = overrideData;
    }

    // Done!
    return true;
  }

  // We have no space left on the current group - We must compact the values
  uint16_t i = 0;

  // Compute the next group to use
  int curwPage = 0, curwGroup = curGroup + 1;
  if (curwGroup >= GroupCount) curwGroup = 0;

  uint32_t rdAddr = 0;
  do {

    // Get the next valid range
    uint32_t addrRange = ee_GetAddrRange(rdAddr, true);

    // Make sure not to skip the override address, if specified
    int rdRange;
    if (overrideAddress < EEPROMSize &&
      rdAddr <= overrideAddress &&
      (addrRange & 0xFFFF) > overrideAddress) {

      rdAddr = overrideAddress;
      rdRange = 1;
    }
    else {
      rdAddr = addrRange & 0xFFFF;
      rdRange = addrRange >> 16;
    }

    // If no range, break loop
    if (rdRange == 0)
      break;

    do {

      // Get the value
      uint8_t rdValue = overrideAddress == rdAddr ? overrideData : ee_Read(rdAddr, true);

      // Do not bother storing default values
      if (rdValue != 0xFF) {

        // If we have room, add it to the buffer
        if (buffer[i + 2] == 0xFF) {

          // Uninitialized buffer, just add it!
          buffer[i] = rdAddr & 0xFF;
          buffer[i + 1] = (rdAddr >> 8) & 0xFF;
          buffer[i + 2] = 1;
          buffer[i + 3] = rdValue;

        }
        else {
          // Buffer already has contents. Check if we can extend it

          // Get the address of the block
          uint32_t baddr = buffer[i] | (buffer[i + 1] << 8);

          // Get the length of the block
          uint32_t blen = buffer[i + 2];

          // Can we expand it ?
          if (rdAddr == (baddr + blen) &&
            i < (PageSize - 4) && /* This block has a chance to contain data AND */
            buffer[i + 2] < (PageSize - i - 3)) {/* There is room for this block to be expanded */

            // Yes, do it
            ++buffer[i + 2];

            // And store the value
            buffer[i + 3 + rdAddr - baddr] = rdValue;

          }
          else {

            // No, we can't expand it - Skip the existing block
            i += 3 + blen;

            // Can we create a new slot ?
            if (i > (PageSize - 4)) {

              // Not enough space - Write the current buffer to FLASH
              ee_PageWrite(curwPage + curwGroup * PagesPerGroup, buffer);

              // Advance write page (as we are compacting, should never overflow!)
              ++curwPage;

              // Clear RAM buffer
              memset(buffer, 0xFF, sizeof(buffer));

              // Start fresh */
              i = 0;
            }

            // Enough space, add the new block
            buffer[i] = rdAddr & 0xFF;
            buffer[i + 1] = (rdAddr >> 8) & 0xFF;
            buffer[i + 2] = 1;
            buffer[i + 3] = rdValue;
          }
        }
      }

      // Go to the next address
      ++rdAddr;

      // Repeat for bytes of this range
    } while (--rdRange);

    // Repeat until we run out of ranges
  } while (rdAddr < EEPROMSize);

  // We must erase the previous group, in preparation for the next swap
  for (int page = 0; page < curPage; page++) {
    ee_PageErase(page + curGroup * PagesPerGroup);
  }

  // Finally, Now the active group is the created new group
  curGroup = curwGroup;
  curPage = curwPage;

  // Done!
  return true;
}

static bool ee_Write(uint32_t address, uint8_t data) {

  // If we were requested an address outside of the emulated range, fail now
  if (address >= EEPROMSize) return false;

  // Lets check if we have a block with that data previously defined. Block
  //  start addresses are always sorted in ascending order
  uint16_t i = 0;
  while (i <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

    // Get the address of the block
    uint32_t baddr = buffer[i] | (buffer[i + 1] << 8);

    // Get the length of the block
    uint32_t blen = buffer[i + 2];

    // If we reach the end of the list, break loop
    if (blen == 0xFF)
      break;

    // Check if data is contained in this block
    if (address >= baddr &&
      address < (baddr + blen)) {

      // Yes, it is contained. Just modify it
      buffer[i + 3 + address - baddr] = data;

      // Done!
      return true;
    }

    // Maybe we could add it to the front or to the back
    // of this block ?
    if ((address + 1) == baddr || address == (baddr + blen)) {

      // Potentially, it could be done. But we must ensure there is room
      // so we can expand the block. Lets find how much free space remains
      uint32_t iend = i;
      do {
        uint32_t ln = buffer[iend + 2];
        if (ln == 0xFF) break;
        iend += 3 + ln;
      } while (iend <= (PageSize - 4)); /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

      // Here, inxt points to the first free address in the buffer. Do we have room ?
      if (iend < PageSize) {
        // Yes, at least a byte is free - We can expand the block

        // Do we have to insert at the beginning ?
        if ((address + 1) == baddr) {

          // Insert at the beginning

          // Make room at the beginning for our byte
          memmove(&buffer[i + 3 + 1], &buffer[i + 3], iend - i - 3);

          // Adjust the header and store the data
          buffer[i] = address & 0xFF;
          buffer[i + 1] = (address >> 8) & 0xFF;
          buffer[i + 2]++;
          buffer[i + 3] = data;

        }
        else {

          // Insert at the end - There is a very interesting thing that could happen here:
          //  Maybe we could coalesce the next block with this block. Let's try to do it!
          uint16_t inext = i + 3 + blen;
          if (inext <= (PageSize - 4) &&
            (buffer[inext] | uint16_t(buffer[inext + 1] << 8)) == (baddr + blen + 1)) {
            // YES! ... we can coalesce blocks! . Do it!

            // Adjust this block header to include the next one
            buffer[i + 2] += buffer[inext + 2] + 1;

            // Store data at the right place
            buffer[i + 3 + blen] = data;

            // Remove the next block header and append its data
            memmove(&buffer[inext + 1], &buffer[inext + 3], iend - inext - 3);

            // Finally, as we have saved 2 bytes at the end, make sure to clean them
            buffer[iend - 2] = 0xFF;
            buffer[iend - 1] = 0xFF;

          }
          else {
            // NO ... No coalescing possible yet

            // Make room at the end for our byte
            memmove(&buffer[i + 3 + blen + 1], &buffer[i + 3 + blen], iend - i - 3 - blen);

            // And add the data to the block
            buffer[i + 2]++;
            buffer[i + 3 + blen] = data;
          }
        }

        // Done!
        return true;
      }
    }

    // As blocks are always sorted, if the starting address of this block is higher
    // than the address we are looking for, break loop now - We wont find the value
    // associated to the address
    if (baddr > address) break;

    // Jump to the next block
    i += 3 + blen;
  }

  // Value is not stored AND we can't expand previous block to contain it. We must create a new block

  // First, lets find how much free space remains
  uint32_t iend = i;
  while (iend <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */
    uint32_t ln = buffer[iend + 2];
    if (ln == 0xFF) break;
    iend += 3 + ln;
  }

  // If there is room for a new block, insert it at the proper place
  if (iend <= (PageSize - 4)) {

    // We have room to create a new block. Do so --- But add
    // the block at the proper position, sorted by starting
    // address, so it will be possible to compact it with other blocks.

    // Make space
    memmove(&buffer[i + 4], &buffer[i], iend - i);

    // And add the block
    buffer[i] = address & 0xFF;
    buffer[i + 1] = (address >> 8) & 0xFF;
    buffer[i + 2] = 1;
    buffer[i + 3] = data;

    // Done!
    return true;
  }

  // Not enough room to store this information on this FLASH page -  Perform a
  // flush and override the address with the specified contents
  return ee_Flush(address, data);
}

static void ee_Init() {

  // Just init once!
  if (curGroup != 0xFF) return;

  // Clean up the SRAM buffer
  memset(buffer, 0xFF, sizeof(buffer));

  // Now, we must find out the group where settings are stored
  for (curGroup = 0; curGroup < GroupCount; curGroup++)
    if (!ee_IsPageClean(curGroup * PagesPerGroup)) break;

  // If all groups seem to be used, default to first group
  if (curGroup >= GroupCount) curGroup = 0;

  DEBUG_ECHO_MSG("EEPROM Current Group: ",curGroup);
  DEBUG_FLUSH();

  // Now, validate that all the other group pages are empty
  for (int grp = 0; grp < GroupCount; grp++) {
    if (grp == curGroup) continue;

    for (int page = 0; page < PagesPerGroup; page++) {
      if (!ee_IsPageClean(grp * PagesPerGroup + page)) {
        DEBUG_ECHO_MSG("EEPROM Page ", page, " not clean on group ", grp);
        DEBUG_FLUSH();
        ee_PageErase(grp * PagesPerGroup + page);
      }
    }
  }

  // Finally, for the active group, determine the first unused page
  // and also validate that all the other ones are clean
  for (curPage = 0; curPage < PagesPerGroup; curPage++) {
    if (ee_IsPageClean(curGroup * PagesPerGroup + curPage)) {
      ee_Dump(curGroup * PagesPerGroup + curPage, getFlashStorage(curGroup * PagesPerGroup + curPage));
      break;
    }
  }

  DEBUG_ECHO_MSG("EEPROM Active page: ", curPage);
  DEBUG_FLUSH();

  // Make sure the pages following the first clean one are also clean
  for (int page = curPage + 1; page < PagesPerGroup; page++) {
    if (!ee_IsPageClean(curGroup * PagesPerGroup + page)) {
      DEBUG_ECHO_MSG("EEPROM Page ", page, " not clean on active group ", curGroup);
      DEBUG_FLUSH();
      ee_Dump(curGroup * PagesPerGroup + page, getFlashStorage(curGroup * PagesPerGroup + page));
      ee_PageErase(curGroup * PagesPerGroup + page);
    }
  }
}

/* PersistentStore -----------------------------------------------------------*/

#include "../shared/eeprom_api.h"

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE 0x1000 // 4KB
#endif
size_t PersistentStore::capacity()    { return MARLIN_EEPROM_SIZE - eeprom_exclude_size; }
bool PersistentStore::access_start()  { ee_Init();  return true; }
bool PersistentStore::access_finish() { ee_Flush(); return true; }

bool PersistentStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {
  uint16_t written = 0;
  while (size--) {
    uint8_t * const p = (uint8_t * const)REAL_EEPROM_ADDR(pos);
    uint8_t v = *value;
    if (v != ee_Read(uint32_t(p))) { // EEPROM has only ~100,000 write cycles, so only write bytes that have changed!
      ee_Write(uint32_t(p), v);
      if (++written & 0x7F) delay(2); else safe_delay(2); // Avoid triggering watchdog during long EEPROM writes
      if (ee_Read(uint32_t(p)) != v) {
        SERIAL_ECHO_MSG(STR_ERR_EEPROM_WRITE);
        return true;
      }
    }
    crc16(crc, &v, 1);
    pos++;
    value++;
  }
  return false;
}

bool PersistentStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/) {
  do {
    uint8_t c = ee_Read(uint32_t(REAL_EEPROM_ADDR(pos)));
    if (writing) *value = c;
    crc16(crc, &c, 1);
    pos++;
    value++;
  } while (--size);
  return false;
}

#endif // FLASH_EEPROM_EMULATION
#endif // ARDUINO_ARCH_SAM
