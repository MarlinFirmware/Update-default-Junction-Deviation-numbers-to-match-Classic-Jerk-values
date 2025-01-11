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
#include "../platforms.h"

#ifdef HAL_STM32

#include "../../inc/MarlinConfig.h"

#if ENABLED(FLASH_EEPROM_EMULATION)

#include "../shared/eeprom_api.h"

// Better: "utility/stm32_eeprom.h", but only after updating stm32duino to 2.0.0
// Use EEPROM.h for compatibility, for now.
#include <EEPROM.h>

/**
 * The STM32 HAL supports chips that deal with "pages" and some with "sectors" and some that
 * even have multiple "banks" of flash.
 *
 * This code is a bit of a mashup of
 *   framework-arduinoststm32/cores/arduino/stm32/stm32_eeprom.c
 *   hal/hal_lpc1768/persistent_store_flash.cpp
 *
 * This has only be written against those that use a single "sector" design.
 *
 * Those that deal with "pages" could be made to work. Looking at the STM32F07 for example, there are
 * 128 "pages", each 2kB in size. If we continued with our EEPROM being 4Kb, we'd always need to operate
 * on 2 of these pages. Each write, we'd use 2 different pages from a pool of pages until we are done.
 */

#if ENABLED(FLASH_EEPROM_LEVELING)

  #include "stm32_def.h"

  #define DEBUG_OUT ENABLED(EEPROM_CHITCHAT)
  #include "../../core/debug_out.h"

  #ifndef MARLIN_EEPROM_SIZE
    #define MARLIN_EEPROM_SIZE    0x1000 // 4KB
  #endif

  #ifndef FLASH_SECTOR
    #define FLASH_SECTOR          (FLASH_SECTOR_TOTAL - 1)
  #endif
  #ifndef FLASH_UNIT_SIZE
    #define FLASH_UNIT_SIZE       0x20000 // 128kB
  #endif

  #ifndef FLASH_ADDRESS_START
    #define FLASH_ADDRESS_START   (FLASH_END - ((FLASH_SECTOR_TOTAL - (FLASH_SECTOR)) * (FLASH_UNIT_SIZE)) + 1)
  #endif
  #define FLASH_ADDRESS_END       (FLASH_ADDRESS_START + FLASH_UNIT_SIZE  - 1)

  #define EEPROM_SLOTS            ((FLASH_UNIT_SIZE) / (MARLIN_EEPROM_SIZE))
  #define SLOT_ADDRESS(slot)      (FLASH_ADDRESS_START + (slot * (MARLIN_EEPROM_SIZE)))

  #define UNLOCK_FLASH()          if (!flash_unlocked) { \
                                    HAL_FLASH_Unlock(); \
                                    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | \
                                                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR); \
                                    flash_unlocked = true; \
                                  }
  #define LOCK_FLASH()            if (flash_unlocked) { HAL_FLASH_Lock(); flash_unlocked = false; }

  #define EMPTY_UINT32            ((uint32_t)-1)
  #define EMPTY_UINT8             ((uint8_t)-1)

  static uint8_t ram_eeprom[MARLIN_EEPROM_SIZE] __attribute__((aligned(4))) = {0};
  static int current_slot = -1;

  static_assert(0 == MARLIN_EEPROM_SIZE % 4, "MARLIN_EEPROM_SIZE must be a multiple of 4"); // Ensure copying as uint32_t is safe
  static_assert(0 == FLASH_UNIT_SIZE % MARLIN_EEPROM_SIZE, "MARLIN_EEPROM_SIZE must divide evenly into your FLASH_UNIT_SIZE");
  static_assert(FLASH_UNIT_SIZE >= MARLIN_EEPROM_SIZE, "FLASH_UNIT_SIZE must be greater than or equal to your MARLIN_EEPROM_SIZE");
  static_assert(IS_FLASH_SECTOR(FLASH_SECTOR), "FLASH_SECTOR is invalid");
  static_assert(IS_POWER_OF_2(FLASH_UNIT_SIZE), "FLASH_UNIT_SIZE should be a power of 2, please check your chip's spec sheet");

#endif // FLASH_EEPROM_LEVELING

static bool eeprom_data_written = false;

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE size_t(E2END + 1)
#endif
size_t PersistentStore::capacity() { return MARLIN_EEPROM_SIZE - eeprom_exclude_size; }

bool PersistentStore::access_start() {

  EEPROM.begin(); // Avoid STM32 EEPROM.h warning (do nothing)

  #if ENABLED(FLASH_EEPROM_LEVELING)

    if (current_slot == -1 || eeprom_data_written) {
      // This must be the first time since power on that we have accessed the storage, or someone
      // loaded and called write_data and never called access_finish.
      // Lets go looking for the slot that holds our configuration.
      if (eeprom_data_written) DEBUG_ECHOLNPGM("Dangling EEPROM write_data");
      uint32_t address = FLASH_ADDRESS_START;
      while (address <= FLASH_ADDRESS_END) {
        uint32_t address_value = (*(__IO uint32_t*)address);
        if (address_value != EMPTY_UINT32) {
          current_slot = (address - (FLASH_ADDRESS_START)) / (MARLIN_EEPROM_SIZE);
          break;
        }
        address += sizeof(uint32_t);
      }
      if (current_slot == -1) {
        // We didn't find anything, so we'll just initialize to empty
        for (int i = 0; i < MARLIN_EEPROM_SIZE; i++) ram_eeprom[i] = EMPTY_UINT8;
        current_slot = EEPROM_SLOTS;
      }
      else {
        // load current settings
        uint8_t *eeprom_data = (uint8_t *)SLOT_ADDRESS(current_slot);
        for (int i = 0; i < MARLIN_EEPROM_SIZE; i++) ram_eeprom[i] = eeprom_data[i];
        DEBUG_ECHOLNPGM("EEPROM loaded from slot ", current_slot, ".");
      }
      eeprom_data_written = false;
    }

  #else
    eeprom_buffer_fill();
  #endif

  return true;
}

bool PersistentStore::access_finish() {

  if (eeprom_data_written) {
    #ifdef STM32F4xx
      // MCU may come up with flash error bits which prevent some flash operations.
      // Clear flags prior to flash operations to prevent errors.
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    #endif

    #if ENABLED(FLASH_EEPROM_LEVELING)

      HAL_StatusTypeDef status = HAL_ERROR;
      bool flash_unlocked = false;

      if (--current_slot < 0) {
        // all slots have been used, erase everything and start again

        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t SectorError = 0;

        EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        EraseInitStruct.Sector = FLASH_SECTOR;
        EraseInitStruct.NbSectors = 1;

        current_slot = EEPROM_SLOTS - 1;
        UNLOCK_FLASH();

        TERN_(HAS_PAUSE_SERVO_OUTPUT, PAUSE_SERVO_OUTPUT());
        hal.isr_off();
        status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
        hal.isr_on();
        TERN_(HAS_PAUSE_SERVO_OUTPUT, RESUME_SERVO_OUTPUT());
        if (status != HAL_OK) {
          DEBUG_ECHOLNPGM("HAL_FLASHEx_Erase=", status);
          DEBUG_ECHOLNPGM("GetError=", HAL_FLASH_GetError());
          DEBUG_ECHOLNPGM("SectorError=", SectorError);
          LOCK_FLASH();
          return false;
        }
      }

      UNLOCK_FLASH();

      uint32_t offset = 0,
               address = SLOT_ADDRESS(current_slot),
               address_end = address + MARLIN_EEPROM_SIZE,
               data = 0;

      bool success = true;

      while (address < address_end) {
        memcpy(&data, ram_eeprom + offset, sizeof(data));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);
        if (status == HAL_OK) {
          address += sizeof(uint32_t);
          offset += sizeof(uint32_t);
        }
        else {
          DEBUG_ECHOLNPGM("HAL_FLASH_Program=", status);
          DEBUG_ECHOLNPGM("GetError=", HAL_FLASH_GetError());
          DEBUG_ECHOLNPGM("address=", address);
          success = false;
          break;
        }
      }

      LOCK_FLASH();

      if (success) {
        eeprom_data_written = false;
        DEBUG_ECHOLNPGM("EEPROM saved to slot ", current_slot, ".");
      }

      return success;

    #else // !FLASH_EEPROM_LEVELING

      // The following was written for the STM32F4 but may work with other MCUs as well.
      // Most STM32F4 flash does not allow reading from flash during erase operations.
      // This takes about a second on a STM32F407 with a 128kB sector used as EEPROM.
      // Interrupts during this time can have unpredictable results, such as killing Servo
      // output. Servo output still glitches with interrupts disabled, but recovers after the
      // erase.
      TERN_(HAS_PAUSE_SERVO_OUTPUT, PAUSE_SERVO_OUTPUT());
      hal.isr_off();
      eeprom_buffer_flush();
      hal.isr_on();
      TERN_(HAS_PAUSE_SERVO_OUTPUT, RESUME_SERVO_OUTPUT());

      eeprom_data_written = false;

    #endif // !FLASH_EEPROM_LEVELING
  }

  return true;
}

bool PersistentStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {
  while (size--) {
    uint8_t v = *value;
    const int p = REAL_EEPROM_ADDR(pos);
    #if ENABLED(FLASH_EEPROM_LEVELING)
      if (v != ram_eeprom[p]) {
        ram_eeprom[p] = v;
        eeprom_data_written = true;
      }
    #else
      if (v != eeprom_buffered_read_byte(p)) {
        eeprom_buffered_write_byte(p, v);
        eeprom_data_written = true;
      }
    #endif
    crc16(crc, &v, 1);
    pos++;
    value++;
  }
  return false;
}

bool PersistentStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/) {
  do {
    const int p = REAL_EEPROM_ADDR(pos);
    const uint8_t c = TERN(FLASH_EEPROM_LEVELING, ram_eeprom[p], eeprom_buffered_read_byte(p));
    if (writing) *value = c;
    crc16(crc, &c, 1);
    pos++;
    value++;
  } while (--size);
  return false;
}

#endif // FLASH_EEPROM_EMULATION
#endif // HAL_STM32
