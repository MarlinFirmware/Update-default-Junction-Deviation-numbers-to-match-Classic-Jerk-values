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

/**
 * sound.cpp
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

  //#include "backlight.h"
  #include "../../libs/buzzer.h"
  #include "sound.h"

  // eSOUND_MODE eSoundMode=e_SOUND_MODE_LOUD;
  // doesn't matter if Sound_Init is called (i.e. the value is in the EEPROM)
  // !?! eSOUND_MODE eSoundMode; in ultraldc.cpp :: cd_settings_menu() it appears as a local variable like this
  eSOUND_MODE eSoundMode; // =e_SOUND_MODE_DEFAULT;


  static void Sound_SaveMode(void);
  static void Sound_DoSound_Echo(void);
  static void Sound_DoSound_Prompt(void);
  static void Sound_DoSound_Alert(bool bOnce);
  static void Sound_DoSound_Encoder_Move(void);
  static void Sound_DoSound_Blind_Alert(void);

  void Sound_Init(void) {
    // SET_OUTPUT(BEEPER);
    // eSoundMode = static_cast<eSOUND_MODE>(eeprom_init_default_byte((uint8_t*)EEPROM_SOUND_MODE, e_SOUND_MODE_DEFAULT));
  }

  void Sound_SaveMode(void) {
    // eeprom_update_byte((uint8_t*)EEPROM_SOUND_MODE,(uint8_t)eSoundMode);
  }

  void Sound_CycleState(void) {
    switch (eSoundMode) {
      case e_SOUND_MODE_LOUD:   eSoundMode = e_SOUND_MODE_ONCE; break;
      case e_SOUND_MODE_ONCE:   eSoundMode = e_SOUND_MODE_SILENT; break;
      case e_SOUND_MODE_SILENT: eSoundMode = e_SOUND_MODE_BLIND; break;
      case e_SOUND_MODE_BLIND:  eSoundMode = e_SOUND_MODE_LOUD; break;
      default:                  eSoundMode = e_SOUND_MODE_LOUD;
    }
    Sound_SaveMode();
  }

  // if critical is true then silent and once mode is ignored
  void __attribute__((noinline)) Sound_MakeCustom(uint16_t ms, uint16_t tone_, bool critical) {
    if (critical || eSoundMode != e_SOUND_MODE_SILENT)
      //if (!tone_) {
      //  WRITE(BEEPER, HIGH);
      //  _delay(ms);
      //  WRITE(BEEPER, LOW);
      //}
      //else {
      //  _tone(BEEPER, tone_);
      //  _delay(ms);
      //  _noTone(BEEPER);
      //}
      BUZZ(ms, tone_);
  }

  void Sound_MakeSound(eSOUND_TYPE eSoundType) {
    switch (eSoundMode) {
      case e_SOUND_MODE_LOUD:
        if (eSoundType == e_SOUND_TYPE_ButtonEcho)
          Sound_DoSound_Echo();
        if (eSoundType == e_SOUND_TYPE_StandardPrompt)
          Sound_DoSound_Prompt();
        if (eSoundType == e_SOUND_TYPE_StandardAlert)
          Sound_DoSound_Alert(false);
        break;
      case e_SOUND_MODE_ONCE:
        if (eSoundType == e_SOUND_TYPE_ButtonEcho)
          Sound_DoSound_Echo();
        if (eSoundType == e_SOUND_TYPE_StandardPrompt)
          Sound_DoSound_Prompt();
        if (eSoundType == e_SOUND_TYPE_StandardAlert)
          Sound_DoSound_Alert(true);
        break;
      case e_SOUND_MODE_SILENT:
        if (eSoundType == e_SOUND_TYPE_StandardAlert)
          Sound_DoSound_Alert(true);
        break;
      case e_SOUND_MODE_BLIND:
        if (eSoundType == e_SOUND_TYPE_ButtonEcho)
          Sound_DoSound_Echo();
        if (eSoundType == e_SOUND_TYPE_StandardPrompt)
          Sound_DoSound_Prompt();
        if (eSoundType == e_SOUND_TYPE_StandardAlert)
          Sound_DoSound_Alert(false);
        if (eSoundType == e_SOUND_TYPE_EncoderMove)
          Sound_DoSound_Encoder_Move();
        if (eSoundType == e_SOUND_TYPE_BlindAlert)
          Sound_DoSound_Blind_Alert();
        break;
      default:
        break;
    }
  }

  static void Sound_DoSound_Blind_Alert(void) {
    // backlight_wake(1);
    uint8_t nI;
    for (nI = 0; nI < 20; nI++) {
      BUZZ(94, 404);
      BUZZ(94, 0);
    }
  }

  static void Sound_DoSound_Encoder_Move(void) {
    uint8_t nI;
    for (nI = 0; nI < 5; nI++) {
      BUZZ(75, 404);
      BUZZ(75, 0);
    }
  }

  static void Sound_DoSound_Echo(void) {
    uint8_t nI;
    for (nI = 0; nI < 10; nI++) {
      BUZZ(100, 404);
      BUZZ(100, 0);
    }
  }

  static void Sound_DoSound_Prompt(void) {
    // backlight_wake(2);
    BUZZ(500, 404);
  }

  static void Sound_DoSound_Alert(bool bOnce) {
    uint8_t nI, nMax;
    nMax = bOnce ? 1 : 3;
    for (nI = 0; nI < nMax; nI++) {
      BUZZ(200, 404);
      BUZZ(500, 0);
    }
  }

  static int16_t constexpr CONTINOUS_BEEP_PERIOD = 2000; // in ms
  // static ShortTimer beep_timer; // Timer to keep track of continous beeping
  static bool bFirst; // true if the first beep has occurred, e_SOUND_MODE_ONCE

  // @brief Handles sound when waiting for user input
  // the function must be non-blocking. It is up to the caller
  // to call this function repeatedly.
  // Make sure to call sound_wait_for_user_reset() when the user has clicked the knob
  //     Loud - should continuously beep
  //     Silent - should be silent
  //     Once - should beep once
  //     Assist/Blind - as loud with beep and click on knob rotation and press
  void sound_wait_for_user() {
    #if BEEPER > 0
      if (eSoundMode == e_SOUND_MODE_SILENT) return;

      // Handle case where only one beep is needed
      if (eSoundMode == e_SOUND_MODE_ONCE) {
        if (bFirst) return;
        Sound_MakeCustom(80, 0, false);
        bFirst = true;
      }

      // Handle case where there should be continous beeps
      if (beep_timer.expired_cont(CONTINOUS_BEEP_PERIOD)) {
        beep_timer.start();
        if (eSoundMode == e_SOUND_MODE_LOUD)
          Sound_MakeCustom(80, 0, false);
        else
          // Assist (lower volume sound)
          Sound_MakeSound(e_SOUND_TYPE_ButtonEcho);
      }
    #endif // BEEPER > 0
  }

  // @brief Resets the global state of sound_wait_for_user()
  void sound_wait_for_user_reset() {
    // beep_timer.stop();
    bFirst = false;
  }

#endif // HAS_PRUSA_MMU3
