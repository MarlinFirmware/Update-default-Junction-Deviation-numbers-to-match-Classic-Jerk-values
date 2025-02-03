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

/**
 * feature/host_actions.h
 */

#include "../inc/MarlinConfigPre.h"
#include "../HAL/shared/Marduino.h"

#if ENABLED(HOST_PROMPT_SUPPORT)

  enum PromptReason : uint8_t {
    PROMPT_NOT_DEFINED,
    PROMPT_FILAMENT_RUNOUT,
    PROMPT_USER_CONTINUE,
    PROMPT_FILAMENT_RUNOUT_REHEAT,
    PROMPT_PAUSE_RESUME,
    PROMPT_INFO
  };

  extern const char CONTINUE_STR[], DISMISS_STR[];

#endif

class HostUI {
  public:

  static void action(FSTR_P const fstr, const bool eol=true);

  #ifdef ACTION_ON_KILL
    static void kill();
  #endif
  #ifdef ACTION_ON_PAUSE
    static void pause(const bool eol=true);
  #endif
  #ifdef ACTION_ON_PAUSED
    static void paused(const bool eol=true);
  #endif
  #ifdef ACTION_ON_RESUME
    static void resume();
  #endif
  #ifdef ACTION_ON_RESUMED
    static void resumed();
  #endif
  #ifdef ACTION_ON_CANCEL
    static void cancel();
  #endif
  #ifdef ACTION_ON_START
    static void start();
  #endif
  #ifdef SHUTDOWN_ACTION
    static void shutdown();
  #endif

  #if ENABLED(G29_RETRY_AND_RECOVER)
    #ifdef ACTION_ON_G29_RECOVER
      static void g29_recover();
    #endif
    #ifdef ACTION_ON_G29_FAILURE
      static void g29_failure();
    #endif
  #endif

  #if ENABLED(HOST_PROMPT_SUPPORT)
    private:
    static void prompt(FSTR_P const ptype, const bool eol=true);
    static void prompt_plus(const bool pgm, FSTR_P const ptype, const char * const str, const char extra_char='\0');
    static void prompt_plus(FSTR_P const ptype, FSTR_P const fstr, const char extra_char='\0') {
      prompt_plus(true, ptype, FTOP(fstr), extra_char);
    }
    static void prompt_plus(FSTR_P const ptype, const char * const cstr, const char extra_char='\0') {
      prompt_plus(false, ptype, cstr, extra_char);
    }

    static void prompt_show();
    static void _prompt_show(FSTR_P const btn1, FSTR_P const btn2);

    public:
    static PromptReason host_prompt_reason;

    static void handle_response(const uint8_t response);

    static void notify_P(PGM_P const message);
    static void notify(FSTR_P const fmsg) { notify_P(FTOP(fmsg)); }
    static void notify(const char * const message);

    static void prompt_begin(const PromptReason reason, FSTR_P const fstr, const char extra_char='\0');
    static void prompt_begin(const PromptReason reason, const char * const cstr, const char extra_char='\0');
    static void prompt_end();

    static void prompt_button(FSTR_P const fstr);
    static void prompt_button(const char * const cstr);

    static void prompt_do(const PromptReason reason, FSTR_P const pstr, FSTR_P const btn1=nullptr, FSTR_P const btn2=nullptr);
    static void prompt_do(const PromptReason reason, const char * const cstr, FSTR_P const btn1=nullptr, FSTR_P const btn2=nullptr);
    static void prompt_do(const PromptReason reason, FSTR_P const pstr, const char extra_char, FSTR_P const btn1=nullptr, FSTR_P const btn2=nullptr);
    static void prompt_do(const PromptReason reason, const char * const cstr, const char extra_char, FSTR_P const btn1=nullptr, FSTR_P const btn2=nullptr);

    static void continue_prompt(FSTR_P const fstr) { prompt_do(PROMPT_USER_CONTINUE, fstr, FPSTR(CONTINUE_STR)); }
    static void continue_prompt(const char * const cstr) { prompt_do(PROMPT_USER_CONTINUE, cstr, FPSTR(CONTINUE_STR)); }

    static void prompt_open(const PromptReason reason, FSTR_P const pstr, FSTR_P const btn1=nullptr, FSTR_P const btn2=nullptr) {
      if (host_prompt_reason == PROMPT_NOT_DEFINED) prompt_do(reason, pstr, btn1, btn2);
    }

    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      static void filament_load_prompt();
    #endif

  #endif

};

extern HostUI hostui;
