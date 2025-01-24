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
 * Portuguese (Brazil)
 * UTF-8 for Graphical Display
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */
namespace LanguageNarrow_pt_br {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Portuguese (BR)");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" pronto.");
  LSTR MSG_YES                            = _UxGT("SIM");
  LSTR MSG_NO                             = _UxGT("NÃO");
  LSTR MSG_BACK                           = _UxGT("Voltar");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Abortando...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Cartão inserido");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Cartão removido");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Aguardando cartão");
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Erro de leitura");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("USB removido");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("USB falhou");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Fins de curso");
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Soft Fins curso");
  LSTR MSG_MAIN_MENU                      = _UxGT("Menu principal");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Config. Avançada");
  LSTR MSG_CONFIGURATION                  = _UxGT("Configuração");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Início automático");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Desabilit. motores");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Menu Debug");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Testar Barra Progres");
  LSTR MSG_AUTO_HOME                      = _UxGT("Ir a origem XYZ");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Ir na origem X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Ir na origem Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Ir na origem Z");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Auto alinhar Z");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Indo para origem");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Clique para Iniciar");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Próximo Ponto");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Fim nivelação!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Suavizar altura");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Compensar origem");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Alteração aplicada");

  LSTR MSG_PREHEAT_1                      = _UxGT("Pre-aquecer ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Pre-aquecer ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Extrusora ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Extrusora ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Pre-aq.Todo ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Pre-aq.Mesa ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Ajustar ") PREHEAT_1_LABEL;

  LSTR MSG_PREHEAT_M                      = _UxGT("Pre-aquecer $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Pre-aquecer $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Extrusora $");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Extrusora $ ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Pre-aq.Todo $");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Pre-aq.Mesa $");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Ajustar $");

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Customizar Pre-aq.");
  LSTR MSG_COOLDOWN                       = _UxGT("Esfriar");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Ligar");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Desligar");
  LSTR MSG_EXTRUDE                        = _UxGT("Extrusar");
  LSTR MSG_RETRACT                        = _UxGT("Retrair");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Mover eixo");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Sondar e Nivelar");
  LSTR MSG_BED_LEVELING                   = _UxGT("Nivelação Mesa");
  LSTR MSG_LEVEL_BED                      = _UxGT("Nivelar Mesa");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Nivelar Cantos");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Próximo Canto");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Editor de Malha");
  LSTR MSG_EDIT_MESH                      = _UxGT("Editar Malha");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Fim da Edição");
  LSTR MSG_PROBING_POINT                  = _UxGT("Sondando ponto");
  LSTR MSG_MESH_X                         = _UxGT("Índice X");
  LSTR MSG_MESH_Y                         = _UxGT("Índice Y");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Valor Z");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Comando customizado");
  LSTR MSG_M48_TEST                       = _UxGT("M48 Teste de sonda");
  LSTR MSG_M48_POINT                      = _UxGT("M48 Ponto");
  LSTR MSG_IDEX_MENU                      = _UxGT("Modo IDEX");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Auto-Estacionar");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Duplicação");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Cópia espelhada");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Controle Total");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2o bico Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2o bico @");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("Executando G29");
  LSTR MSG_UBL_TOOLS                      = _UxGT("Ferramentas UBL");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Fazer malha manual");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Calçar e calibrar");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Medir");
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Remover e calibrar");
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Movendo para Próximo");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Temp. Mesa");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Temp. Mesa");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Temp. Extrusora");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Temp. Extrusora");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Editar Malha Custom");
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Ajuste Fino da Malha");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Fim da Edição");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Montar Malha Custom");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Montar ");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("Montar $");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Checar $");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Montar Malha fria");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Ajustar Altura");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Quant. de Altura");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Validar Malha");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Validar Malha Custom");
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26 Aquecendo Mesa");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26 Aquecendo Ext.");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26 Cancelado");
  LSTR MSG_G26_LEAVING                    = _UxGT("G26 Saindo");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Continuar Malha");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Nivelação da Malha");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("Nivelação 3 pontos");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Nivelação Grid");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Nivelar Malha");
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Cantos");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Tipo de Mapa");
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Salvar Mapa da Malha");
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Enviar Para Host");
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("Salvar Malha CSV");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Salvar Backup");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("Informação do UBL");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Qtd de Enchimento");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Enchimento Manual");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Enchimento Smart");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Preencher malha");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Invalidar tudo");
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Invalidar próximo");
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Ajuste Fino de Todos");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Ajustar Mais Próximo");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Armazenamento Malha");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Slot de Memória");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Ler Malha");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Salvar Malha");
  LSTR MSG_MESH_LOADED                    = _UxGT("Malha %i carregada");
  LSTR MSG_MESH_SAVED                     = _UxGT("Malha %i salva");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("Sem armazenamento");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("Erro ao salvar UBL");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("Erro no restauro UBL");
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Compensação Z: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Compensação Z parou");
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("UBL passo a passo");
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1.Montar Malha fria");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2.Enchimento Smart");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3.Validar Malha");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4.Ajuste Fino de Todos");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5.Validar Malha");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6.Ajuste Fino de Todos");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7.Salvar Malha");

  LSTR MSG_LED_CONTROL                    = _UxGT("Controle do LED");
  LSTR MSG_LIGHTS                         = _UxGT("Luz");
  LSTR MSG_LIGHT_N                        = _UxGT("Luz #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("Configuração da Luz");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Luz Vermelha");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Luz Laranja");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Luz Amarela");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Luz Verde");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Luz Azul");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Luz Indigo");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Luz Violeta");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Luz Branca");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Luz Padrão");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Luz Customizada");
  LSTR MSG_INTENSITY_R                    = _UxGT("Intensidade Vermelho");
  LSTR MSG_INTENSITY_G                    = _UxGT("Intensidade Verde");
  LSTR MSG_INTENSITY_B                    = _UxGT("Intensidade Azul");
  LSTR MSG_INTENSITY_W                    = _UxGT("Intensidade Branco");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Brilho");

  LSTR MSG_MOVING                         = _UxGT("Movendo...");
  LSTR MSG_FREE_XY                        = _UxGT("Liberar XY");
  LSTR MSG_MOVE_X                         = _UxGT("Mover X");
  LSTR MSG_MOVE_Y                         = _UxGT("Mover Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Mover Z");
  LSTR MSG_MOVE_N                         = _UxGT("Mover @");
  LSTR MSG_MOVE_E                         = _UxGT("Mover Extrusor");
  LSTR MSG_MOVE_EN                        = _UxGT("Mover Extrusor *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Extrus. mto fria");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Mover $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Mover $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Mover $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Velocidade");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Base Z");
  LSTR MSG_NOZZLE                         = _UxGT("Bocal");
  LSTR MSG_NOZZLE_N                       = _UxGT("Bocal ~");
  LSTR MSG_BED                            = _UxGT("Mesa");
  LSTR MSG_FAN_SPEED                      = _UxGT("Vel. Ventoinha");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Vel. Ventoinha ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("+Vel. Ventoinha");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("+Vel. Ventoinha ~");
  LSTR MSG_FLOW                           = _UxGT("Vazão");
  LSTR MSG_FLOW_N                         = _UxGT("Vazão ~");
  LSTR MSG_CONTROL                        = _UxGT("Controle");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Máx");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Fator");
  LSTR MSG_AUTOTEMP                       = _UxGT("Temp. Automática");
  LSTR MSG_LCD_ON                         = _UxGT("Ligado");
  LSTR MSG_LCD_OFF                        = _UxGT("Desligado");
  LSTR MSG_SELECT_E                       = _UxGT("Selecionar *");
  LSTR MSG_ACC                            = _UxGT("Acel.");
  LSTR MSG_JERK                           = _UxGT("Arrancada");
  LSTR MSG_VA_JERK                        = _UxGT("arrancada V") STR_A;
  LSTR MSG_VB_JERK                        = _UxGT("arrancada V") STR_B;
  LSTR MSG_VC_JERK                        = _UxGT("arrancada V") STR_C;
  LSTR MSG_VN_JERK                        = _UxGT("arrancada V@");
  LSTR MSG_VE_JERK                        = _UxGT("arrancada VE");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Desv. Junção");
  LSTR MSG_MAX_SPEED                      = _UxGT("Velocidade");
  LSTR MSG_VTRAV_MIN                      = _UxGT("VDeslocamento min");
  LSTR MSG_ACCELERATION                   = _UxGT("Aceleração");
  LSTR MSG_A_RETRACT                      = _UxGT("Retrair A");
  LSTR MSG_A_TRAVEL                       = _UxGT("Movimento A");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Passo/mm");
  LSTR MSG_A_STEPS                        = _UxGT("Passo ") STR_A _UxGT("/mm");
  LSTR MSG_B_STEPS                        = _UxGT("Passo ") STR_B _UxGT("/mm");
  LSTR MSG_C_STEPS                        = _UxGT("Passo ") STR_C _UxGT("/mm");
  LSTR MSG_N_STEPS                        = _UxGT("Passo @/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("*/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperatura");
  LSTR MSG_MOTION                         = _UxGT("Movimento");
  LSTR MSG_FILAMENT                       = _UxGT("Filamento");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("Extrusão em mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Diâmetro Fil.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Diâmetro Fil. *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Descarr. mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Carregar mm");
  LSTR MSG_ADVANCE_K                      = _UxGT("Avanço K");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("Avanço K *");
  LSTR MSG_CONTRAST                       = _UxGT("Contraste");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Salvar Configuração");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Ler Configuração");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Restauro seguro");
  LSTR MSG_INIT_EEPROM                    = _UxGT("Iniciar EEPROM");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("Atualiz. SD");
  LSTR MSG_RESET_PRINTER                  = _UxGT("Resetar Impressora");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Atualização");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Informações");
  LSTR MSG_PREPARE                        = _UxGT("Preparar");
  LSTR MSG_TUNE                           = _UxGT("Ajustar");
  LSTR MSG_START_PRINT                    = _UxGT("Iniciar Impressão");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("Prox.");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Iniciar");
  LSTR MSG_BUTTON_STOP                    = _UxGT("Parar");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Imprimir");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Resetar");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("Cancelar");
  LSTR MSG_BUTTON_DONE                    = _UxGT("Pronto");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Voltar");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Continuar");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pausar impressão");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Continuar impressão");
  LSTR MSG_STOP_PRINT                     = _UxGT("Parar impressão");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Imprimindo objeto");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Cancelar Objeto");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Cancelar Objeto {");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Recuperar Impressão");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Imprimir do SD");
  LSTR MSG_NO_MEDIA                       = _UxGT("Sem cartão SD");
  LSTR MSG_DWELL                          = _UxGT("Dormindo...");
  LSTR MSG_USERWAIT                       = _UxGT("Clique para retomar");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Impressão Pausada");
  LSTR MSG_PRINTING                       = _UxGT("Imprimindo...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Impressão Abortada");
  LSTR MSG_NO_MOVE                        = _UxGT("Sem movimento");
  LSTR MSG_KILLED                         = _UxGT("PARADA DE EMERGÊNCIA");
  LSTR MSG_STOPPED                        = _UxGT("PAROU. ");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Retrair mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Retrair Troca mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Retrair V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Saltar mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Des-Retrair mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Des-RetTroca mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Des-Retrair  V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("Des-RetTroca V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Retração Automática");
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("Distancia Retração");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("Distancia Purga");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Mudar Ferramenta");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Levantar Z");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Preparar Veloc.");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Veloc. Retração");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Standby bico");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Trocar Filamento");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Trocar Filamento *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Carregar Filamento");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Carregar Filamento *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Descarreg. Filamento");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Descarreg. Filamento *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Descarregar Todos");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Iniciar cartão SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Iniciar flash USB");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Iniciar cartão SD");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Trocar SD");
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Liberar SD");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Sonda fora da mesa");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Fator de Cisalho");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("Testar BLTouch");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Reiniciar BLTouch");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("Recolher BLTouch");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("Estender BLTouch");
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("Estender Sonda-Z");
  LSTR MSG_MANUAL_STOW                    = _UxGT("Recolher Sonda-Z");
  LSTR MSG_HOME_FIRST                     = _UxGT("Home %s Primeiro");
  LSTR MSG_ZPROBE_OFFSETS                 = _UxGT("Compensar Sonda");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Compensar Sonda em X");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Compensar Sonda em Y");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Compensar Sonda em Z");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Compensar Sonda em @");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Passo Z ajuste");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Passinho X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Passinho Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Passinho Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Passinho @");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Total");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Abortar Fim de Curso");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Aquecimento falhou");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Erro:Temp Redundante");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("ESCAPE TÉRMICO");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Erro:Temp Máxima");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Erro:Temp Mínima");
  LSTR MSG_HALTED                         = _UxGT("IMPRESSORA PAROU");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Favor resetar");
  LSTR MSG_HEATING                        = _UxGT("Aquecendo...");
  LSTR MSG_COOLING                        = _UxGT("Resfriando...");
  LSTR MSG_BED_HEATING                    = _UxGT("Aquecendo mesa...");
  LSTR MSG_BED_COOLING                    = _UxGT("Esfriando mesa...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Aquecendo Câmara...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Esfriando Câmara...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Calibrar Delta");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Calibrar X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Calibrar Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Calibrar Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Calibrar Centro");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Configuração Delta");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Auto-Calibração");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Haste Diagonal");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Altura");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Raio");
  LSTR MSG_INFO_MENU                      = _UxGT("Sobre");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Impressora");
  LSTR MSG_3POINT_LEVELING                = _UxGT("Nivelamento 3 pontos");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Nivelamento Linear");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Nivelamento Bilinear");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Nivelamento UBL");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Nivelamento da Malha");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Estatísticas");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Info. da Placa");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistores");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Extrusoras");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Taxa de Transmissão");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protocolo");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Luz da Impressora");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Intensidade Brilho");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Impressora Incorreta");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Qtd de Impressões");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Realizadas");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Tempo de Impressão");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Maior trabalho");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("T. Extrusão");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Temp Mín");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Temp Máx");
  LSTR MSG_INFO_PSU                       = _UxGT("PSU");
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Força do Motor");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("Escrever EEPROM DAC");
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("TROCA DE FILAMENTO");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("IMPRESSÃO PAUSADA");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("CARREGAR FILAMENTO");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("DESCARREG. FILAMENTO");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("Config. de Retomada");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Purgar mais");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Continuar Impressão");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Bocal: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Sensor filamento");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Falha ao ir à origem");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Falha ao sondar");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("ESCOLHER FILAMENTO");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");

  LSTR MSG_MMU2_RESUME                    = _UxGT("Continuar Impressão");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("Continuando...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("Carregar Filamento");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("Carregar Todos");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("Carregar para bocal");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("Ejetar Filamento");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("Ejetar Filamento ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("Liberar Filamento");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("Carregando Fil. %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Ejetando Fil. ...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Carregando Fil....");
  LSTR MSG_MMU2_ALL                       = _UxGT("Todos");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("Filamento ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("Resetar MMU");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("Resetando MMU...");

  LSTR MSG_GAMES                          = _UxGT("Jogos");
  LSTR MSG_BRICKOUT                       = _UxGT("Brickout");
  LSTR MSG_INVADERS                       = _UxGT("Invaders");
  LSTR MSG_SNAKE                          = _UxGT("Sn4k3");
  LSTR MSG_MAZE                           = _UxGT("Labirinto");

  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_1_LINE("Clique p. continuar"));
  LSTR MSG_PAUSE_PRINT_INIT               = _UxGT(MSG_1_LINE("Estacionando..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Aguarde..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Insira e Clique"));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_1_LINE("Clique para Aquecer"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Aquecendo..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Ejetando..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Carregando..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("Purgando..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_1_LINE("Clique p. finalizar"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Continuando..."));

  LSTR MSG_TOP_LEFT                       = _UxGT("Superior Esquerdo");
  LSTR MSG_BOTTOM_LEFT                    = _UxGT("Inferior Esquerdo");
  LSTR MSG_TOP_RIGHT                      = _UxGT("Superior Direto");
  LSTR MSG_BOTTOM_RIGHT                   = _UxGT("Inferior Direto");
  LSTR MSG_CALIBRATION_COMPLETED          = _UxGT("Calibração Completa");
  LSTR MSG_CALIBRATION_FAILED             = _UxGT("Calibração Falhou");
}

namespace LanguageWide_pt_br {
  using namespace LanguageNarrow_pt_br;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Total de Impressões");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Realizadas");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Tempo de Impressão");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Trabalho Mais longo");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Total de Extrusão");
  #endif
}

namespace LanguageTall_pt_br {
  using namespace LanguageWide_pt_br;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Aperte o botão para", "continuar impressão"));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Esperando o", "inicio da", "troca de filamento"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Coloque filamento", "pressione o botão", "para continuar..."));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Pressione o botão", "p/ aquecer o bocal"));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Aquecendo o bocal", "Aguarde..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_2_LINE("Esperando", "remoção de filamento"));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_2_LINE("Esperando", "filamento"));
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("Espere pela", "purga de filamento"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_2_LINE("Clique para finaliz.", "purga de filamento"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_2_LINE("Esperando impressão", "continuar"));
  #endif
}

namespace Language_pt_br {
  using namespace LanguageTall_pt_br;
}
