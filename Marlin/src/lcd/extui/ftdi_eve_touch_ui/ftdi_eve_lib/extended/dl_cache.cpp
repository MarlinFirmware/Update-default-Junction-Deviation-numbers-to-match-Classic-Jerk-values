/****************
 * dl_cache.cpp *
 ****************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#include "ftdi_extended.h"

#if ENABLED(FTDI_EXTENDED)

/* The Display List Cache mechanism stores the display list corresponding
 * to a menu into RAM_G so that on subsequent calls drawing the menu does
 * not require as much SPI traffic.
 *
 * Layout of Cache memory:
 *
 * The cache memory begins with a table at
 * DL_CACHE_START: each table entry contains
 * an address, size and used bytes for a cached
 * DL slot.
 *
 * Immediately following the table is the
 * DL_FREE_ADDR, which points to free cache
 * space; following this is occupied DL space,
 * and after that free space that is yet to
 * be used.
 *
 *  location        data        sizeof
 *
 *  DL_CACHE_START  slot0_addr     4
 *                  slot0_size     4
 *                  slot0_used     4
 *                  slot1_addr     4
 *                  slot1_size     4
 *                  slot1_used     4
 *                      ...
 *                  slotN_addr     4
 *                  slotN_size     4
 *                  slotN_used     4
 *  DL_FREE_ADDR    dl_free_ptr    4
 *                  cached data
 *                      ...
 *  dl_free_ptr     empty space
 *                      ...
 */

#define DL_CACHE_START   MAP::RAM_G_SIZE - 0xFFFF
#define DL_FREE_ADDR     DL_CACHE_START + DL_CACHE_SLOTS * 12

using namespace FTDI;

// The init function ensures all cache locations are marked as empty

void DLCache::init() {
  CLCD::mem_write_32(DL_FREE_ADDR, DL_FREE_ADDR + 4);
  for (uint_fast8_t slot = 0; slot < DL_CACHE_SLOTS; slot++)
    save_slot(slot, 0, 0, 0);
}

bool DLCache::has_data() {
  return dl_slot_size != 0;
}

bool DLCache::wait_until_idle() {
  const unsigned long startTime = millis();
  do {
    if ((millis() - startTime) > 250) {
      SERIAL_ECHO_MSG("Timeout on DL_Cache::Wait_Until_Idle()");
      CLCD::CommandFifo::reset();
      return false;
    }
    #ifdef __MARLIN_FIRMWARE__
      ExtUI::yield();
    #endif
  } while (CLCD::CommandFifo::is_processing());
  return true;
}

/* This caches the current display list in RAMG so
 * that it can be appended later. The memory is
 * dynamically allocated following DL_FREE_ADDR.
 *
 * If min_bytes is provided, then that many bytes
 * will be reserved so that the cache may be re-written
 * later with potentially a bigger DL.
 */

bool DLCache::store(uint32_t min_bytes /* = 0*/) {
  CLCD::CommandFifo cmd;

  // Execute any commands already in the FIFO
  cmd.execute();
  if (!wait_until_idle())
    return false;

  // Figure out how long the display list is
  const uint32_t dl_size = CLCD::dl_size();

  if (dl_slot_addr == 0) {
    // If we are allocating new space...
    dl_slot_addr = CLCD::mem_read_32(DL_FREE_ADDR);
    dl_slot_size = max(dl_size, min_bytes);

    const uint32_t free_space = MAP::RAM_G_SIZE - dl_slot_addr;
    if (dl_slot_size <= free_space) {
      CLCD::mem_write_32(DL_FREE_ADDR, dl_slot_addr + dl_slot_size);
    }
    else {
      dl_slot_addr = 0;
      dl_slot_size = 0;
      dl_slot_used = 0;
    }
  }

  if (dl_size > dl_slot_size) {
    // Not enough memory to cache the display list.
    #if ENABLED(TOUCH_UI_DEBUG)
      SERIAL_ECHO_MSG("Not enough space in GRAM to cache display list, free space: ", dl_slot_size, " Required: ", dl_size);
    #endif
    dl_slot_used = 0;
    save_slot();
    return false;
  }
  else {
    #if ENABLED(TOUCH_UI_DEBUG)
      SERIAL_ECHO_MSG("Saving DL to RAMG cache, bytes: ", dl_slot_used, " Free space: ", dl_slot_size);
    #endif
    dl_slot_used = dl_size;
    save_slot();
    cmd.memcpy(dl_slot_addr, MAP::RAM_DL, dl_slot_used);
    cmd.execute();
    return true;
  }
}

void DLCache::save_slot(uint8_t indx, uint32_t addr, uint16_t size, uint16_t used) {
  CLCD::mem_write_32(DL_CACHE_START + indx * 12 + 0, addr);
  CLCD::mem_write_32(DL_CACHE_START + indx * 12 + 4, size);
  CLCD::mem_write_32(DL_CACHE_START + indx * 12 + 8, used);
}

void DLCache::load_slot(uint8_t indx, uint32_t &addr, uint16_t &size, uint16_t &used) {
  addr = CLCD::mem_read_32(DL_CACHE_START + indx * 12 + 0);
  size = CLCD::mem_read_32(DL_CACHE_START + indx * 12 + 4);
  used = CLCD::mem_read_32(DL_CACHE_START + indx * 12 + 8);
}

void DLCache::append() {
  CLCD::CommandFifo cmd;
  cmd.append(dl_slot_addr, dl_slot_used);
  #if ENABLED(TOUCH_UI_DEBUG)
    cmd.execute();
    wait_until_idle();
    SERIAL_ECHO_MSG("Appending to DL from RAMG cache, bytes: ", dl_slot_used, " REG_CMD_DL: ", CLCD::mem_read_32(REG::CMD_DL));
  #endif
}

#endif // FTDI_EXTENDED
