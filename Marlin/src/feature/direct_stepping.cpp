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
 * feature/direct_stepping.cpp
 */

#include "../inc/MarlinConfigPre.h"

#if ENABLED(DIRECT_STEPPING)

#include "direct_stepping.h"

#include "../MarlinCore.h"

#define CHECK_PAGE(I, R) do{                                \
  if (I >= sizeof(page_states) / sizeof(page_states[0])) {  \
    fatal_error = true;                                     \
    return R;                                               \
  }                                                         \
}while(0)

#define CHECK_PAGE_STATE(I, R, S) do { \
  CHECK_PAGE(I, R);                    \
  if (page_states[I] != S) {           \
    fatal_error = true;                \
    return R;                          \
  }                                    \
}while(0)

namespace DirectStepping {

  template<typename Cfg>
  State SerialPageManager<Cfg>::state;

  template<typename Cfg>
  volatile bool SerialPageManager<Cfg>::fatal_error;

  template<typename Cfg>
  volatile PageState SerialPageManager<Cfg>::page_states[Cfg::PAGE_COUNT];

  template<typename Cfg>
  volatile bool SerialPageManager<Cfg>::page_states_dirty;

  template<typename Cfg>
  uint8_t SerialPageManager<Cfg>::pages[Cfg::PAGE_COUNT][Cfg::PAGE_SIZE];

  template<typename Cfg>
  uint8_t SerialPageManager<Cfg>::checksum;

  template<typename Cfg>
  typename Cfg::write_byte_idx_t SerialPageManager<Cfg>::write_byte_idx;

  template<typename Cfg>
  typename Cfg::page_idx_t SerialPageManager<Cfg>::write_page_idx;

  template<typename Cfg>
  typename Cfg::write_byte_idx_t SerialPageManager<Cfg>::write_page_size;

  template <typename Cfg>
  void SerialPageManager<Cfg>::init() {
    for (int i = 0 ; i < Cfg::PAGE_COUNT ; i++)
      page_states[i] = PageState::FREE;

    fatal_error = false;
    state = State::NEWLINE;

    page_states_dirty = false;

    SERIAL_ECHOLNPGM("pages_ready");
  }

  template<typename Cfg>
  FORCE_INLINE bool SerialPageManager<Cfg>::maybe_store_rxd_char(uint8_t c) {
    switch (state) {
      default:
      case State::MONITOR:
        switch (c) {
          case '\n':
          case '\r':
            state = State::NEWLINE;
          default:
            return false;
        }
      case State::NEWLINE:
        switch (c) {
          case Cfg::CONTROL_CHAR:
            state = State::ADDRESS;
            return true;
          case '\n':
          case '\r':
            state = State::NEWLINE;
            return false;
          default:
            state = State::MONITOR;
            return false;
        }
      case State::ADDRESS:
        //TODO: 16 bit address, State::ADDRESS2
        write_page_idx = c;
        write_byte_idx = 0;
        checksum = 0;

        CHECK_PAGE(write_page_idx, true);

        if (page_states[write_page_idx] == PageState::FAIL) {
          // Special case for fail
          state = State::UNFAIL;
          return true;
        }

        set_page_state(write_page_idx, PageState::WRITING);

        state = Cfg::DIRECTIONAL ? State::COLLECT : State::SIZE;

        return true;
      case State::SIZE:
        // Zero means full page size
        write_page_size = c;
        state = State::COLLECT;
        return true;
      case State::COLLECT:
        pages[write_page_idx][write_byte_idx++] = c;
        checksum ^= c;

        // check if still collecting
        if (Cfg::PAGE_SIZE == 256) {
          // special case for 8-bit, check if rolled back to 0
          if (Cfg::DIRECTIONAL || !write_page_size) { // full 256 bytes
            if (write_byte_idx) return true;
          }
          else if (write_byte_idx < write_page_size)
            return true;
        }
        else if (Cfg::DIRECTIONAL) {
          if (write_byte_idx != Cfg::PAGE_SIZE)
            return true;
        }
        else if (write_byte_idx < write_page_size)
          return true;

        state = State::CHECKSUM;
        return true;
      case State::CHECKSUM: {
        const PageState page_state = (checksum == c) ? PageState::OK : PageState::FAIL;
        set_page_state(write_page_idx, page_state);
        state = State::MONITOR;
        return true;
      }
      case State::UNFAIL:
        if (c == 0)
          set_page_state(write_page_idx, PageState::FREE);
        else
          fatal_error = true;
        state = State::MONITOR;
        return true;
    }
  }

  template <typename Cfg>
  void SerialPageManager<Cfg>::write_responses() {
    if (fatal_error) {
      kill(GET_TEXT_F(MSG_BAD_PAGE));
      return;
    }

    if (!page_states_dirty) return;
    page_states_dirty = false;

    SERIAL_CHAR(Cfg::CONTROL_CHAR);
    constexpr int state_bits = 2;
    constexpr int n_bytes = Cfg::PAGE_COUNT >> state_bits;
    volatile uint8_t bits_b[n_bytes] = { 0 };

    for (page_idx_t i = 0 ; i < Cfg::PAGE_COUNT ; i++) {
      bits_b[i >> state_bits] |= page_states[i] << ((i * state_bits) & 0x7);
    }

    uint8_t crc = 0;
    for (uint8_t i = 0 ; i < n_bytes ; i++) {
      crc ^= bits_b[i];
      SERIAL_CHAR(bits_b[i]);
    }

    SERIAL_CHAR(crc);
    SERIAL_EOL();
  }

  template <typename Cfg>
  FORCE_INLINE void SerialPageManager<Cfg>::set_page_state(const page_idx_t page_idx, const PageState page_state) {
    CHECK_PAGE(page_idx,);

    page_states[page_idx] = page_state;
    page_states_dirty = true;
  }

  template <>
  FORCE_INLINE uint8_t *PageManager::get_page(const page_idx_t page_idx) {
    CHECK_PAGE(page_idx, nullptr);

    return pages[page_idx];
  }

  template <>
  FORCE_INLINE void PageManager::free_page(const page_idx_t page_idx) {
    set_page_state(page_idx, PageState::FREE);
  }

};

DirectStepping::PageManager page_manager;

const uint8_t segment_table[DirectStepping::Config::NUM_SEGMENTS][DirectStepping::Config::SEGMENT_STEPS] PROGMEM = {

  #if STEPPER_PAGE_FORMAT == SP_4x4D_128

    { 1, 1, 1, 1, 1, 1, 1 }, //  0 = -7
    { 1, 1, 1, 0, 1, 1, 1 }, //  1 = -6
    { 1, 1, 1, 0, 1, 0, 1 }, //  2 = -5
    { 1, 1, 0, 1, 0, 1, 0 }, //  3 = -4
    { 1, 1, 0, 0, 1, 0, 0 }, //  4 = -3
    { 0, 0, 1, 0, 0, 0, 1 }, //  5 = -2
    { 0, 0, 0, 1, 0, 0, 0 }, //  6 = -1
    { 0, 0, 0, 0, 0, 0, 0 }, //  7 =  0
    { 0, 0, 0, 1, 0, 0, 0 }, //  8 =  1
    { 0, 0, 1, 0, 0, 0, 1 }, //  9 =  2
    { 1, 1, 0, 0, 1, 0, 0 }, // 10 =  3
    { 1, 1, 0, 1, 0, 1, 0 }, // 11 =  4
    { 1, 1, 1, 0, 1, 0, 1 }, // 12 =  5
    { 1, 1, 1, 0, 1, 1, 1 }, // 13 =  6
    { 1, 1, 1, 1, 1, 1, 1 }, // 14 =  7
    { 0 }

  #elif STEPPER_PAGE_FORMAT == SP_4x2_256

    { 0, 0, 0 }, // 0
    { 0, 1, 0 }, // 1
    { 1, 0, 1 }, // 2
    { 1, 1, 1 }, // 3

  #elif STEPPER_PAGE_FORMAT == SP_4x1_512

    {0} // Uncompressed format, table not used

  #endif

};

#endif // DIRECT_STEPPING
