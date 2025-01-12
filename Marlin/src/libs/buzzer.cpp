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

#include "../inc/MarlinConfig.h"

#if HAS_BEEPER

#include "buzzer.h"
#include "../module/temperature.h"
#include "../lcd/marlinui.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

Buzzer::state_t Buzzer::state;
CircularQueue<tone_t, TONE_QUEUE_LENGTH> Buzzer::buffer;
Buzzer buzzer;

/**
 * @brief Add a tone to the queue
 * @details Adds a tone_t structure to the ring buffer, will block IO if the
 *          queue is full waiting for one slot to get available.
 *
 * @param duration Duration of the tone in milliseconds
 * @param frequency Frequency of the tone in hertz
 */
void Buzzer::tone(const uint16_t duration, const uint16_t frequency/*=0*/) {
  if (!ui.sound_on) return;
  while (buffer.isFull()) {
    tick();
    thermalManager.task();
  }
  tone_t tone = { duration, frequency };
  buffer.enqueue(tone);
}

void Buzzer::tick() {
  if (state.endtime) {
    if (ELAPSED(millis(), state.endtime)) reset();
    return;
  }

  if (buffer.isEmpty()) return;

  state.tone = buffer.dequeue();
  state.endtime = millis() + state.tone.duration;

  if (state.tone.frequency > 0) {
    #if ENABLED(EXTENSIBLE_UI) && DISABLED(EXTUI_LOCAL_BEEPER)
      CRITICAL_SECTION_START();
      ExtUI::onPlayTone(state.tone.frequency, state.tone.duration);
      CRITICAL_SECTION_END();
    #elif ENABLED(SPEAKER)
      CRITICAL_SECTION_START();
      ::tone(BEEPER_PIN, state.tone.frequency, state.tone.duration);
      CRITICAL_SECTION_END();
    #else
      on();
    #endif
  }
}

#endif // HAS_BEEPER
