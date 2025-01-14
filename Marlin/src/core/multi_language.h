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

/*******************************************************
 * multi_language.h                                    *
 * By Marcio Teixeira 2019 for Aleph Objects           *
 *******************************************************/

#include "../inc/MarlinConfigPre.h"

typedef const char Language_Str[];
#define LSTR PROGMEM Language_Str

#ifdef LCD_LANGUAGE_5
  #define NUM_LANGUAGES 5
#elif defined(LCD_LANGUAGE_4)
  #define NUM_LANGUAGES 4
#elif defined(LCD_LANGUAGE_3)
  #define NUM_LANGUAGES 3
#elif defined(LCD_LANGUAGE_2)
  #define NUM_LANGUAGES 2
#else
  #define NUM_LANGUAGES 1
#endif

// Set unused languages equal to each other so the
// compiler can optimize away the conditionals.
#define LCD_LANGUAGE_1 LCD_LANGUAGE
#ifndef LCD_LANGUAGE_2
  #define LCD_LANGUAGE_2 LCD_LANGUAGE
#endif
#ifndef LCD_LANGUAGE_3
  #define LCD_LANGUAGE_3 LCD_LANGUAGE_2
#endif
#ifndef LCD_LANGUAGE_4
  #define LCD_LANGUAGE_4 LCD_LANGUAGE_3
#endif
#ifndef LCD_LANGUAGE_5
  #define LCD_LANGUAGE_5 LCD_LANGUAGE_4
#endif

#define _GET_LANG(LANG) Language_##LANG
#define GET_LANG(LANG) _GET_LANG(LANG)

#if NUM_LANGUAGES > 1
  #define HAS_MULTI_LANGUAGE 1
  #if HAS_MARLINUI_MENU
    #define HAS_MENU_MULTI_LANGUAGE 1
  #endif
  #define GET_TEXT(MSG) ( \
    ui.language == 4 ? GET_LANG(LCD_LANGUAGE_5)::MSG : \
    ui.language == 3 ? GET_LANG(LCD_LANGUAGE_4)::MSG : \
    ui.language == 2 ? GET_LANG(LCD_LANGUAGE_3)::MSG : \
    ui.language == 1 ? GET_LANG(LCD_LANGUAGE_2)::MSG : \
                       GET_LANG(LCD_LANGUAGE  )::MSG   )
  #define MAX_LANG_CHARSIZE _MAX(GET_LANG(LCD_LANGUAGE  )::CHARSIZE, \
                                 GET_LANG(LCD_LANGUAGE_2)::CHARSIZE, \
                                 GET_LANG(LCD_LANGUAGE_3)::CHARSIZE, \
                                 GET_LANG(LCD_LANGUAGE_4)::CHARSIZE, \
                                 GET_LANG(LCD_LANGUAGE_5)::CHARSIZE  )
#else
  #define GET_TEXT(MSG) GET_LANG(LCD_LANGUAGE)::MSG
  #define MAX_LANG_CHARSIZE LANG_CHARSIZE
#endif
#define GET_TEXT_F(MSG) FPSTR(GET_TEXT(MSG))

#define GET_EN_TEXT(MSG) GET_LANG(en)::MSG
#define GET_EN_TEXT_F(MSG) FPSTR(GET_EN_TEXT(MSG))

#define GET_LANGUAGE_NAME(INDEX) GET_LANG(LCD_LANGUAGE_##INDEX)::LANGUAGE
#define LANG_CHARSIZE GET_TEXT(CHARSIZE)
#define USE_WIDE_GLYPH (LANG_CHARSIZE > 2)

// The final "\0" is added invisibly by the compiler
#define MSG_1_LINE(A)     A "\0"   "\0"
#define MSG_2_LINE(A,B)   A "\0" B "\0"
#define MSG_3_LINE(A,B,C) A "\0" B "\0" C
