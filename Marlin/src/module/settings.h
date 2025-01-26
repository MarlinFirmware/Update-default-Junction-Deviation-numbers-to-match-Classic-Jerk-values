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

//
// settings.cpp - Settings and EEPROM storage
//

#include "../inc/MarlinConfig.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../HAL/shared/eeprom_api.h"
  enum EEPROM_Error : uint8_t {
    ERR_EEPROM_NOERR,
    ERR_EEPROM_VERSION,
    ERR_EEPROM_SIZE,
    ERR_EEPROM_CRC,
    ERR_EEPROM_CORRUPT,
    ERR_EEPROM_NOPROM
  };
#endif

class MarlinSettings {
  public:
    static uint16_t datasize();

    static void reset();
    static bool save();    // Return 'true' if data was saved

    FORCE_INLINE static bool init_eeprom() {
      reset();
      #if ENABLED(EEPROM_SETTINGS)
        const bool success = save();
        if (TERN0(EEPROM_CHITCHAT, success)) report();
        return success;
      #else
        return true;
      #endif
    }

    #if ENABLED(SD_FIRMWARE_UPDATE)
      static bool sd_update_status();                       // True if the SD-Firmware-Update EEPROM flag is set
      static bool set_sd_update_status(const bool enable);  // Return 'true' after EEPROM is set (-> always true)
    #endif

    #if ENABLED(EEPROM_SETTINGS)

      static bool load();      // Return 'true' if data was loaded ok
      static bool validate();  // Return 'true' if EEPROM data is ok

      static EEPROM_Error check_version();

      static void first_load() {
        static bool loaded = false;
        if (!loaded && load()) loaded = true;
      }

      #if HAS_EARLY_LCD_SETTINGS
        // Special cases for LCD contrast and brightness, so
        // some LCDs can display bootscreens earlier in setup().
        #if HAS_LCD_CONTRAST
          static void load_contrast();
        #endif
        #if HAS_LCD_BRIGHTNESS
          static void load_brightness();
        #endif
      #endif

      #if ENABLED(AUTO_BED_LEVELING_UBL) // Eventually make these available if any leveling system
                                         // That can store is enabled
        static uint16_t meshes_start_index();
        FORCE_INLINE static uint16_t meshes_end_index() { return meshes_end; }
        static uint16_t calc_num_meshes();
        static int mesh_slot_offset(const int8_t slot);
        static void store_mesh(const int8_t slot);
        static void load_mesh(const int8_t slot, void * const into=nullptr);

        //static void delete_mesh();    // necessary if we have a MAT
        //static void defrag_meshes();  // "
      #endif

    #else // !EEPROM_SETTINGS

      FORCE_INLINE
      static bool load() { reset(); report(); return true; }
      FORCE_INLINE
      static void first_load() { (void)load(); }

    #endif // !EEPROM_SETTINGS

    #if HAS_EARLY_LCD_SETTINGS
      static void load_lcd_state();
    #endif

    #if DISABLED(DISABLE_M503)
      static void report(const bool forReplay=false);
    #else
      FORCE_INLINE
      static void report(const bool=false) {}
    #endif

  private:
    static void postprocess();

    #if ENABLED(EEPROM_SETTINGS)

      static bool validating;

      #if ENABLED(AUTO_BED_LEVELING_UBL)  // Eventually make these available if any leveling system
                                          // That can store is enabled
        static const uint16_t meshes_end; // 128 is a placeholder for the size of the MAT; the MAT will always
                                          // live at the very end of the eeprom
      #endif

      static EEPROM_Error _load();
      static EEPROM_Error size_error(const uint16_t size);

      static int eeprom_index;
      static uint16_t working_crc;

      static bool EEPROM_START(int eeprom_offset) {
        if (!persistentStore.access_start()) { SERIAL_ECHO_MSG("No EEPROM."); return false; }
        eeprom_index = eeprom_offset;
        working_crc = 0;
        return true;
      }

      static void EEPROM_FINISH(void) { persistentStore.access_finish(); }

      template<typename T>
      static void EEPROM_SKIP(const T &VAR) { eeprom_index += sizeof(VAR); }

      template<typename T>
      static void EEPROM_WRITE(const T &VAR) {
        persistentStore.write_data(eeprom_index, (const uint8_t *) &VAR, sizeof(VAR), &working_crc);
      }

      template<typename T>
      static void EEPROM_READ_(T &VAR) {
        persistentStore.read_data(eeprom_index, (uint8_t *) &VAR, sizeof(VAR), &working_crc, !validating);
      }

      static void EEPROM_READ_(uint8_t *VAR, size_t sizeof_VAR) {
        persistentStore.read_data(eeprom_index, VAR, sizeof_VAR, &working_crc, !validating);
      }

      template<typename T>
      static void EEPROM_READ_ALWAYS_(T &VAR) {
        persistentStore.read_data(eeprom_index, (uint8_t *) &VAR, sizeof(VAR), &working_crc);
      }

    #endif // EEPROM_SETTINGS
};

extern MarlinSettings settings;
