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
#pragma once

/**
 * sound.h
 */

#include <stdint.h>

#define e_SOUND_MODE_NULL 0xFF
typedef enum : uint8_t {
  e_SOUND_MODE_LOUD,
  e_SOUND_MODE_ONCE,
  e_SOUND_MODE_SILENT,
  e_SOUND_MODE_BLIND
} eSOUND_MODE;

#define e_SOUND_MODE_DEFAULT e_SOUND_MODE_LOUD

typedef enum : uint8_t {
  e_SOUND_TYPE_ButtonEcho,
  e_SOUND_TYPE_EncoderEcho,
  e_SOUND_TYPE_StandardPrompt,
  e_SOUND_TYPE_StandardConfirm,
  e_SOUND_TYPE_StandardWarning,
  e_SOUND_TYPE_StandardAlert,
  e_SOUND_TYPE_EncoderMove,
  e_SOUND_TYPE_BlindAlert
} eSOUND_TYPE;

typedef enum : uint8_t {
  e_SOUND_CLASS_Echo,
  e_SOUND_CLASS_Prompt,
  e_SOUND_CLASS_Confirm,
  e_SOUND_CLASS_Warning,
  e_SOUND_CLASS_Alert
} eSOUND_CLASS;

extern eSOUND_MODE eSoundMode;

extern void Sound_Init(void);
extern void Sound_CycleState(void);
extern void Sound_MakeSound(eSOUND_TYPE eSoundType);
extern void Sound_MakeCustom(uint16_t ms, uint16_t tone_, bool critical);
void sound_wait_for_user();
void sound_wait_for_user_reset();

//static void Sound_DoSound_Echo(void);
//static void Sound_DoSound_Prompt(void);
