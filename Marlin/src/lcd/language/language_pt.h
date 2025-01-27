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
 * Portuguese
 * UTF-8 for Graphical Display
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define DISPLAY_CHARSET_ISO10646_1

namespace LanguageNarrow_pt {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Portuguese");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" pronta.");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Cartão inserido");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Cartão removido");
  LSTR MSG_MAIN_MENU                      = _UxGT("Menu principal");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Desactivar motores");
  LSTR MSG_AUTO_HOME                      = _UxGT("Ir para origem");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Ir para origem X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Ir para origem Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Ir para origem Z");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Indo para origem");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Click para iniciar");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Próximo ponto");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Pronto !");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Definir desvio");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Offsets aplicados");

  LSTR MSG_PREHEAT_1                      = _UxGT("Pre-aquecer ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Pre-aquecer ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Pre-aquecer ") PREHEAT_1_LABEL _UxGT(" Bico");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Pre-aquecer ") PREHEAT_1_LABEL _UxGT(" Bico ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Pre-aq. ") PREHEAT_1_LABEL _UxGT(" Tudo");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Pre-aq. ") PREHEAT_1_LABEL _UxGT(" ") LCD_STR_THERMOMETER _UxGT("Base");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Definições ") PREHEAT_1_LABEL;

  LSTR MSG_PREHEAT_M                      = _UxGT("Pre-aquecer $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Pre-aquecer $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Pre-aquecer $ Bico");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Pre-aquecer $ Bico ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Pre-aq. $ Tudo");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Pre-aq. $ ") LCD_STR_THERMOMETER _UxGT("Base");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Definições $");

  LSTR MSG_COOLDOWN                       = _UxGT("Arrefecer");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Ligar");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Desligar");
  LSTR MSG_EXTRUDE                        = _UxGT("Extrudir");
  LSTR MSG_RETRACT                        = _UxGT("Retrair");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Mover eixo");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Calibrar e nivelar");
  LSTR MSG_MOVE_X                         = _UxGT("Mover X");
  LSTR MSG_MOVE_Y                         = _UxGT("Mover Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Mover Z");
  LSTR MSG_MOVE_N                         = _UxGT("Mover @");
  LSTR MSG_MOVE_E                         = _UxGT("Mover Extrusor");
  LSTR MSG_MOVE_EN                        = _UxGT("Mover Extrusor *");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Mover $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Mover $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Mover $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Velocidade");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Base Z");
  LSTR MSG_NOZZLE                         = " " LCD_STR_THERMOMETER _UxGT(" Bico");
  LSTR MSG_NOZZLE_N                       = " " LCD_STR_THERMOMETER _UxGT(" Bico ~");
  LSTR MSG_BED                            = " " LCD_STR_THERMOMETER _UxGT(" Base");
  LSTR MSG_FAN_SPEED                      = _UxGT("Vel. ventoinha");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Vel. ventoinha ~");
  LSTR MSG_FLOW                           = _UxGT("Fluxo");
  LSTR MSG_FLOW_N                         = _UxGT("Fluxo ~");
  LSTR MSG_CONTROL                        = _UxGT("Controlo");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Fact");
  LSTR MSG_A_RETRACT                      = _UxGT("A-retracção");
  LSTR MSG_A_TRAVEL                       = _UxGT("A-movimento");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Passo/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" passo/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" passo/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" passo/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ passo/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E passo/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* passo/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperatura");
  LSTR MSG_MOTION                         = _UxGT("Movimento");
  LSTR MSG_FILAMENT                       = _UxGT("Filamento");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E em mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Fil. Diam.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Fil. Diam. *");
  LSTR MSG_CONTRAST                       = _UxGT("Contraste");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Guardar na memoria");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Carregar da memoria");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Rest. de emergen.");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT(" Recarregar");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Monitorizar");
  LSTR MSG_PREPARE                        = _UxGT("Preparar");
  LSTR MSG_TUNE                           = _UxGT("Afinar");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pausar impressão");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Retomar impressão");
  LSTR MSG_STOP_PRINT                     = _UxGT("Parar impressão");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Imprimir do SD");
  LSTR MSG_NO_MEDIA                       = _UxGT("Sem cartão SD");
  LSTR MSG_DWELL                          = _UxGT("Em espera...");
  LSTR MSG_USERWAIT                       = _UxGT("Á espera de ordem");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Impressão cancelada");
  LSTR MSG_NO_MOVE                        = _UxGT("Sem movimento");
  LSTR MSG_KILLED                         = _UxGT("EMERGÊNCIA. ");
  LSTR MSG_STOPPED                        = _UxGT("PARADO. ");
  LSTR MSG_CONTROL_RETRACT                = _UxGT(" Retrair mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Troca Retrair mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT(" Retrair  V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT(" Levantar mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT(" DesRet mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Troca DesRet mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT(" DesRet  V");
  LSTR MSG_AUTORETRACT                    = _UxGT(" Auto-Retract");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Trocar filamento");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Trocar filamento *");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Inici. cartão SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Inici. flash USB");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Inici. cartão SD");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Trocar cartão SD");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Sensor fora/base");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Desvio X");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Desvio Y");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Desvio Z");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Desvio @");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Passo Z ajuste");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Fim de curso");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Aquecimento falhou");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Err: T Máxima");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Err: T Mínima");
  LSTR MSG_HEATING                        = _UxGT("Aquecendo...");
  LSTR MSG_BED_HEATING                    = _UxGT("Aquecendo base...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Calibração Delta");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Calibrar X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Calibrar Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Calibrar Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Calibrar Centro");

  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Fim de curso");

  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Impressora Incorreta");

  LSTR MSG_TOP_LEFT                       = _UxGT("Superior Esquerdo");
  LSTR MSG_BOTTOM_LEFT                    = _UxGT("Inferior Esquerdo");
  LSTR MSG_TOP_RIGHT                      = _UxGT("Superior Direto");
  LSTR MSG_BOTTOM_RIGHT                   = _UxGT("Inferior Direto");
  LSTR MSG_CALIBRATION_COMPLETED          = _UxGT("Calibração Completa");
  LSTR MSG_CALIBRATION_FAILED             = _UxGT("Calibração Falhou");
}

namespace LanguageWide_pt {
  using namespace LanguageNarrow_pt;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
  #endif
}

namespace LanguageTall_pt {
  using namespace LanguageWide_pt;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
  #endif
}

namespace Language_pt {
  using namespace LanguageTall_pt;
}
