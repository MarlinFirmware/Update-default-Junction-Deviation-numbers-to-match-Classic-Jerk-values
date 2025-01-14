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
 * Korean
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */
namespace LanguageNarrow_ko_KR {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 1;
  LSTR LANGUAGE                           = _UxGT("Korean");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" 준비.");
  LSTR MSG_BACK                           = _UxGT("뒤로");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("카드 삽입됨");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("카드 제거됨");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("엔드스탑");
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("소프트 엔드스탑");
  LSTR MSG_MAIN_MENU                      = _UxGT("뒤로");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("고급 설정");
  LSTR MSG_CONFIGURATION                  = _UxGT("설정");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("자동 시작");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("모터 정지");
  LSTR MSG_DEBUG_MENU                     = _UxGT("디버깅 메뉴");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("프로그레스바 테스트");
  LSTR MSG_AUTO_HOME                      = _UxGT("오토홈");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("X 홈으로");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Y 홈으로");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Z 홈으로");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("XYZ 홈으로");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("누르면 시작합니다");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("다음 Point");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("레벨링 완료!");

  LSTR MSG_PREHEAT_1                      = _UxGT("예열하기 - ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("예열하기 - ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("예열하기 - ") PREHEAT_1_LABEL _UxGT(" 노즐");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("예열하기 - ") PREHEAT_1_LABEL _UxGT(" 노즐 ~");

  LSTR MSG_PREHEAT_M                      = _UxGT("예열하기 - $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("예열하기 - $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("예열하기 - $ 노즐");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("예열하기 - $ 노즐 ~");

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Custom 예열");
  LSTR MSG_COOLDOWN                       = _UxGT("식히기");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("스위치 전원 켜기");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("스위치 전원 끄기");
  LSTR MSG_EXTRUDE                        = _UxGT("밀어내기");
  LSTR MSG_RETRACT                        = _UxGT("당기기");
  LSTR MSG_MOVE_AXIS                      = _UxGT("축 이동");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("프로브 & 레벨링");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Z 값 조정");
  LSTR MSG_BED_LEVELING                   = _UxGT("베드 레벨링");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("미러 사본");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("오토레벨링 하기");
  LSTR MSG_NOZZLE                         = _UxGT("노즐");
  LSTR MSG_NOZZLE_N                       = _UxGT("노즐 ~");
  LSTR MSG_BED                            = _UxGT("베드");
  LSTR MSG_FAN_SPEED                      = _UxGT("펜 속도");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("펜 속도 ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("엑스트라 펜 속도");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("엑스트라 펜 속도 ~");
  LSTR MSG_TEMPERATURE                    = _UxGT("온도");
  LSTR MSG_MOTION                         = _UxGT("동작");
  LSTR MSG_STORE_EEPROM                   = _UxGT("설정 저장하기");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("설정 읽어오기");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("설정 되돌리기");
  LSTR MSG_INIT_EEPROM                    = _UxGT("EEPROM 초기화");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("새로고침");
  LSTR MSG_INFO_SCREEN                    = _UxGT("처음으로");
  LSTR MSG_PREPARE                        = _UxGT("준비하기");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("일시정지");
  LSTR MSG_RESUME_PRINT                   = _UxGT("재시작");
  LSTR MSG_STOP_PRINT                     = _UxGT("출력중지");
  LSTR MSG_MEDIA_MENU                     = _UxGT("SD 카드출력");
  LSTR MSG_NO_MEDIA                       = _UxGT("SD 카드없음");
  LSTR MSG_DWELL                          = _UxGT("슬립모드...");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("일시 정지됨");
  LSTR MSG_PRINTING                       = _UxGT("출력중...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("취소됨");
  LSTR MSG_KILLED                         = _UxGT("죽음. ");
  LSTR MSG_STOPPED                        = _UxGT("멈춤. ");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("잘못된 프린터");
}

namespace LanguageWide_ko_KR {
  using namespace LanguageNarrow_ko_KR;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
  #endif
}

namespace LanguageTall_ko_KR {
  using namespace LanguageWide_ko_KR;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
  #endif
}

namespace Language_ko_KR {
  using namespace LanguageTall_ko_KR;
}
