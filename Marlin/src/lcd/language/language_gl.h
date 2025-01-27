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
 * Galician language
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define DISPLAY_CHARSET_ISO10646_1

#define MEDIA_TYPE_GL "SD/FD"

namespace LanguageNarrow_gl {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 1;
  LSTR LANGUAGE                           = _UxGT("Galician");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" lista.");
  LSTR MSG_YES                            = _UxGT("SI");
  LSTR MSG_NO                             = _UxGT("NON");
  LSTR MSG_BACK                           = _UxGT("Atrás");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Cancelando...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Tarxeta inserida");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Tarxeta retirada");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Agardando ao ") MEDIA_TYPE_GL;
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Erro lectura ") MEDIA_TYPE_GL;
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("Disp. USB retirado");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("Inicio USB fallido");
  LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("Desbord. Subch.");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("FinCarro");
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("FinCarro SW");
  LSTR MSG_MAIN_MENU                      = _UxGT("Menú principal");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Axustes avanzados");
  LSTR MSG_CONFIGURATION                  = _UxGT("Configuración");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Autoarranque");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Apagar motores");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Menú depuración");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Test barra progreso");
  LSTR MSG_AUTO_HOME                      = _UxGT("Ir a orixe");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Ir orixe X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Ir orixe Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Ir orixe Z");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Autoaliñar Z");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Ir orixes XYZ");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Prema pulsador");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Seguinte punto");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Fin Nivelación!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Compensación Altura");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Axustar Desfases");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Desfases aplicados");

  LSTR MSG_PREHEAT_1                      = _UxGT("Prequentar ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Prequentar ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Preque. ") PREHEAT_1_LABEL _UxGT(" Bico");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Preque. ") PREHEAT_1_LABEL _UxGT(" Bico ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Preque. ") PREHEAT_1_LABEL _UxGT(" Todo");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Preque. ") PREHEAT_1_LABEL _UxGT(" Cama");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Preque. ") PREHEAT_1_LABEL _UxGT(" conf");

  LSTR MSG_PREHEAT_M                      = _UxGT("Prequentar $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Prequentar $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Preque. $ Bico");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Preque. $ Bico ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Preque. $ Todo");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Preque. $ Cama");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Preque. $ conf");

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Preque. Personali.");
  LSTR MSG_COOLDOWN                       = _UxGT("Arrefriar");
  LSTR MSG_CUTTER_FREQUENCY               = _UxGT("Frecuencia");
  LSTR MSG_LASER_MENU                     = _UxGT("Control Láser");
  LSTR MSG_LASER_POWER                    = _UxGT("Potencia Láser");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("Control Fuso");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("Potencia Fuso");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("Inverter xiro");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Acender");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Apagar");
  LSTR MSG_EXTRUDE                        = _UxGT("Extruír");
  LSTR MSG_RETRACT                        = _UxGT("Retraer");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Mover eixe");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Probar e Nivelar");
  LSTR MSG_BED_LEVELING                   = _UxGT("Nivelando Cama");
  LSTR MSG_LEVEL_BED                      = _UxGT("Nivelar Cama");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Nivelar Cantos");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Seguinte Canto");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Editor Mallado");
  LSTR MSG_EDIT_MESH                      = _UxGT("Editar Mallado");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Ed. Mallado Detida");
  LSTR MSG_PROBING_POINT                  = _UxGT("Punto de Proba");
  LSTR MSG_MESH_X                         = _UxGT("Índice X");
  LSTR MSG_MESH_Y                         = _UxGT("Índice Y");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Valor Z");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Comandos Personaliz.");
  LSTR MSG_M48_TEST                       = _UxGT("M48 Probar Sonda");
  LSTR MSG_M48_POINT                      = _UxGT("M48 Punto");
  LSTR MSG_M48_DEVIATION                  = _UxGT("Desviación");
  LSTR MSG_IDEX_MENU                      = _UxGT("Modo IDEX");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("Ferramentas Compens");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Auto-Estacionar");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Duplicación");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Copia Espello");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Control Total");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2º Bico Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2º Bico @");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("Executando G29");
  LSTR MSG_UBL_TOOLS                      = _UxGT("Ferramentas UBL");
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("Punto de inclinación");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Facer Malla Manual");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Colocar Calzo e Medir");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Medir");
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Quitar e Medir Cama");
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Mover ao Seguinte");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Temp Cama");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Temp Cama");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Temp Bico");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Temp Bico");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Edit. Malla Person.");
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Axuste Fino da Malla");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Fin Edición da Malla");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Crear Malla Person.");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Crear Malla");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("Crear Malla ($)");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Validar Malla ($)");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Crear Malla Fría");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Axustar Altura Malla");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Altura");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Validar Malla");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Validar Malla perso.");
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26 Quentando Cama");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26 Quentando Bico");
  LSTR MSG_G26_MANUAL_PRIME               = _UxGT("Traballo manual...");
  LSTR MSG_G26_FIXED_LENGTH               = _UxGT("Traballo Lonxit Fixa");
  LSTR MSG_G26_PRIME_DONE                 = _UxGT("Fin Traballo");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26 Cancelado");
  LSTR MSG_G26_LEAVING                    = _UxGT("Saíndo de G26");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Continuar Malla");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Nivelación Malla");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("Nivelación 3Puntos");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Nivelación Grid");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Nivelar Malla");
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Puntos Laterais");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Tipo de Mapa ");
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Gardar Mapa");
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Enviar ao Host");
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("Gardar en CSV");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Backup Externo");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("Info do UBL");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Cantidade de Recheo");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Recheo Manual");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Recheo Intelixente");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Recheo da Malla");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Invalidar todo");
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Invalidar cercanos");
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Axustar Fino Todo");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Axustar Fino Cerc");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Almacenamento Malla");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Rañura Memoria");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Cargar Malla Cama");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Gardar Malla Cama");
  LSTR MSG_MESH_LOADED                    = _UxGT("Malla %i Cargada");
  LSTR MSG_MESH_SAVED                     = _UxGT("Malla %i Gardada");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("Sen Gardar");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("Erro: Gardadado UBL");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("Erro: Recuperación UBL");
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Desfase de Z: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Desfase de Z Detido");
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("UBL Paso a Paso");
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1. Crear Malla Fría");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2. Recheo Intelixente");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3. Validar Malla");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4. Axustar Fino Todo");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5. Validar Malla");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6. Axustar Fino Todo");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7. Gardar Malla Cama");

  LSTR MSG_LED_CONTROL                    = _UxGT("Control LED");
  LSTR MSG_LIGHTS                         = _UxGT("Luces");
  LSTR MSG_LIGHT_N                        = _UxGT("Luce #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("Axustes Luz");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Vermello");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Laranxa");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Amarelo");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Verde");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Azul");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Índigo");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Violeta");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Branco");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Por defecto");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Luces personalizadas");
  LSTR MSG_INTENSITY_R                    = _UxGT("Intensidade Vermello");
  LSTR MSG_INTENSITY_G                    = _UxGT("Intensidade Verde");
  LSTR MSG_INTENSITY_B                    = _UxGT("Intensidade Azul");
  LSTR MSG_INTENSITY_W                    = _UxGT("Intensidade Branco");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Brillo");

  LSTR MSG_MOVING                         = _UxGT("Movendo...");
  LSTR MSG_FREE_XY                        = _UxGT("Libre XY");
  LSTR MSG_MOVE_X                         = _UxGT("Mover X");
  LSTR MSG_MOVE_Y                         = _UxGT("Mover Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Mover Z");
  LSTR MSG_MOVE_N                         = _UxGT("Mover @");
  LSTR MSG_MOVE_E                         = _UxGT("Extrusor");
  LSTR MSG_MOVE_EN                        = _UxGT("Extrusor *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Bico moi frío");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Mover $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Mover $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Mover $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Velocidade");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Cama Z");
  LSTR MSG_NOZZLE                         = _UxGT("Bico");
  LSTR MSG_NOZZLE_N                       = _UxGT("Bico ~");
  LSTR MSG_NOZZLE_PARKED                  = _UxGT("Bico Estacionado");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Bico Standby");
  LSTR MSG_BED                            = _UxGT("Cama");
  LSTR MSG_CHAMBER                        = _UxGT("Cámara");
  LSTR MSG_FAN_SPEED                      = _UxGT("Vel. Ventilador");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Vel. Ventilador ~");
  LSTR MSG_STORED_FAN_N                   = _UxGT("Ventilador Mem. ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Vel. Vent. Extra");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Vel. Vent. Extra ~");
  LSTR MSG_CONTROLLER_FAN                 = _UxGT("Controlador Vent.");
  LSTR MSG_CONTROLLER_FAN_IDLE_SPEED      = _UxGT("Velocidade Repouso");
  LSTR MSG_CONTROLLER_FAN_AUTO_ON         = _UxGT("Modo Auto");
  LSTR MSG_CONTROLLER_FAN_SPEED           = _UxGT("Velocidade Activa");
  LSTR MSG_CONTROLLER_FAN_DURATION        = _UxGT("Tempo Repouso");
  LSTR MSG_FLOW                           = _UxGT("Fluxo");
  LSTR MSG_FLOW_N                         = _UxGT("Fluxo ~");
  LSTR MSG_CONTROL                        = _UxGT("Control");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Fact");
  LSTR MSG_AUTOTEMP                       = _UxGT("Temperatura Auto.");
  LSTR MSG_LCD_ON                         = _UxGT("Acender");
  LSTR MSG_LCD_OFF                        = _UxGT("Apagar");
  LSTR MSG_PID_AUTOTUNE                   = _UxGT("Auto-Sint. PID");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("Auto-Sint. PID *");
  LSTR MSG_PID_AUTOTUNE_DONE              = _UxGT("Fin Auto-Sint PID");
  LSTR MSG_PID_BAD_HEATER_ID              = _UxGT("Auto-Sint. fallida! Extrusor danado.");
  LSTR MSG_PID_TEMP_TOO_HIGH              = _UxGT("Auto-Sint. fallida! Temperatura moi alta.");
  LSTR MSG_PID_TIMEOUT                    = _UxGT("Auto-Sint. fallida! Tempo excedido.");
  LSTR MSG_SELECT_E                       = _UxGT("Escolla *");
  LSTR MSG_ACC                            = _UxGT("Acel");
  LSTR MSG_JERK                           = _UxGT("Jerk");
  LSTR MSG_VA_JERK                        = _UxGT("Max ") STR_A _UxGT(" Jerk");
  LSTR MSG_VB_JERK                        = _UxGT("Max ") STR_B _UxGT(" Jerk");
  LSTR MSG_VC_JERK                        = _UxGT("Max ") STR_C _UxGT(" Jerk");
  LSTR MSG_VN_JERK                        = _UxGT("Max @ Jerk");
  LSTR MSG_VE_JERK                        = _UxGT("Max E Jerk");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Desvío Unión");
  LSTR MSG_MAX_SPEED                      = _UxGT("Max Velocidade");
  LSTR MSG_VMAX_A                         = _UxGT("Max ") STR_A _UxGT(" Speed");
  LSTR MSG_VMAX_B                         = _UxGT("Max ") STR_B _UxGT(" Speed");
  LSTR MSG_VMAX_C                         = _UxGT("Max ") STR_C _UxGT(" Speed");
  LSTR MSG_VMAX_N                         = _UxGT("Max @ Speed");
  LSTR MSG_VMAX_E                         = _UxGT("Max E Speed");
  LSTR MSG_VMAX_EN                        = _UxGT("Max * Speed");
  LSTR MSG_VMIN                           = _UxGT("Vmin");
  LSTR MSG_VTRAV_MIN                      = _UxGT("V-viaxe min");
  LSTR MSG_ACCELERATION                   = _UxGT("Aceleración");
  LSTR MSG_AMAX_A                         = _UxGT("Max ") STR_A _UxGT(" Accel");
  LSTR MSG_AMAX_B                         = _UxGT("Max ") STR_B _UxGT(" Accel");
  LSTR MSG_AMAX_C                         = _UxGT("Max ") STR_C _UxGT(" Accel");
  LSTR MSG_AMAX_E                         = _UxGT("Max E Accel");
  LSTR MSG_AMAX_EN                        = _UxGT("Max * Accel");
  LSTR MSG_A_RETRACT                      = _UxGT("A-retrac.");
  LSTR MSG_A_TRAVEL                       = _UxGT("A-viaxe");
  LSTR MSG_XY_FREQUENCY_LIMIT             = _UxGT("Frecuencia max");
  LSTR MSG_XY_FREQUENCY_FEEDRATE          = _UxGT("Avance min");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Pasos/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" pasos/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" pasos/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" pasos/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ pasos/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E pasos/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* pasos/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperatura");
  LSTR MSG_MOTION                         = _UxGT("Movemento");
  LSTR MSG_FILAMENT                       = _UxGT("Filamento");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E en mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Diam. fil.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Diam. fil. *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Descarga mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Carga mm");
  LSTR MSG_ADVANCE_K                      = _UxGT("Avance K");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("Avance K *");
  LSTR MSG_CONTRAST                       = _UxGT("Constraste LCD");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Gardar Configuración");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Cargar Configuración");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Rest. Defecto");
  LSTR MSG_INIT_EEPROM                    = _UxGT("Inicializar EEPROM");
  LSTR MSG_ERR_EEPROM_CRC                 = _UxGT("Erro: CRC EEPROM");
  LSTR MSG_ERR_EEPROM_SIZE                = _UxGT("Erro: Tamaño EEPROM");
  LSTR MSG_ERR_EEPROM_VERSION             = _UxGT("Erro: Versión EEPROM");
  LSTR MSG_SETTINGS_STORED                = _UxGT("Config Gardada");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("Actualizar ") MEDIA_TYPE_GL;
  LSTR MSG_RESET_PRINTER                  = _UxGT("Reiniciar Impresora");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Recargar");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Información");
  LSTR MSG_PREPARE                        = _UxGT("Preparar");
  LSTR MSG_TUNE                           = _UxGT("Axustar");
  LSTR MSG_START_PRINT                    = _UxGT("Comezar impresión");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("Seguinte");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Comezar");
  LSTR MSG_BUTTON_STOP                    = _UxGT("Deter");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Imprimir");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Reiniciar");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("Cancelar");
  LSTR MSG_BUTTON_DONE                    = _UxGT("Listo");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Atrás");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Proceder");
  LSTR MSG_PAUSING                        = _UxGT("Pausando...");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pausar impresión");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Retomar impresión");
  LSTR MSG_STOP_PRINT                     = _UxGT("Deter impresión");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Imprimindo Obxecto");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Cancelar Obxecto");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Cancelar Obxecto {");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Recuperar Impresión");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Tarxeta SD");
  LSTR MSG_NO_MEDIA                       = _UxGT("Sen tarxeta SD");
  LSTR MSG_DWELL                          = _UxGT("En repouso...");
  LSTR MSG_USERWAIT                       = _UxGT("Prema para Retomar..");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Impresión Pausada");
  LSTR MSG_PRINTING                       = _UxGT("Imprimindo...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Impresión Cancelada");
  LSTR MSG_PRINT_DONE                     = _UxGT("Fin Impresión");
  LSTR MSG_NO_MOVE                        = _UxGT("Sen movemento.");
  LSTR MSG_KILLED                         = _UxGT("MORTO.");
  LSTR MSG_STOPPED                        = _UxGT("DETIDO.");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Retraer mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Cambio retra. mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Retraer V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Alzar Z mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Recup. retra. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Cambio recup. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Recuperacion V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("S UnRet V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Auto-Retracción");
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("Lonxitude Retracción");
  LSTR MSG_FILAMENT_SWAP_EXTRA            = _UxGT("Cambio Extra");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("Lonxitude de Purga");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Cambiar Ferramenta");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Levantar Z");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Velocidade prim.");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Vel. de Retracción");
  LSTR MSG_FILAMENT_PARK_ENABLED          = _UxGT("Extrusor Est.");
  LSTR MSG_SINGLENOZZLE_UNRETRACT_SPEED   = _UxGT("Vel. Recuperación");
  LSTR MSG_SINGLENOZZLE_FAN_SPEED         = _UxGT("Vel. Ventilador");
  LSTR MSG_SINGLENOZZLE_FAN_TIME          = _UxGT("Tempo Ventilador");
  LSTR MSG_TOOL_MIGRATION_ON              = _UxGT("Auto ON");
  LSTR MSG_TOOL_MIGRATION_OFF             = _UxGT("Auto OFF");
  LSTR MSG_TOOL_MIGRATION                 = _UxGT("Cambio Ferramenta");
  LSTR MSG_TOOL_MIGRATION_AUTO            = _UxGT("Cambio Automático");
  LSTR MSG_TOOL_MIGRATION_END             = _UxGT("Último Extrusor");
  LSTR MSG_TOOL_MIGRATION_SWAP            = _UxGT("Cambio a *");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Cambiar Filamento");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Cambiar Filamento *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Cargar Filamento");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Cargar Filamento *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Descargar Filamento");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Descargar Filamento *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Descargar Todo");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Iniciar SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Iniciar USB");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Iniciar ") MEDIA_TYPE_GL;
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Cambiar ") MEDIA_TYPE_GL;
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Lanzar ") MEDIA_TYPE_GL;
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Sonda-Z fóra Cama");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Factor de Desviación");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("Auto-Test");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Reiniciar");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("Recoller");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("Estender");
  LSTR MSG_BLTOUCH_SW_MODE                = _UxGT("Modo Software");
  LSTR MSG_BLTOUCH_5V_MODE                = _UxGT("Modo 5V");
  LSTR MSG_BLTOUCH_OD_MODE                = _UxGT("Modo OD");
  LSTR MSG_BLTOUCH_MODE_STORE             = _UxGT("Modo Almacenar");
  LSTR MSG_BLTOUCH_MODE_STORE_5V          = _UxGT("Axustar BLTouch a 5V");
  LSTR MSG_BLTOUCH_MODE_STORE_OD          = _UxGT("Axustar BLTouch a OD");
  LSTR MSG_BLTOUCH_MODE_ECHO              = _UxGT("Modo de Informe");
  LSTR MSG_BLTOUCH_MODE_CHANGE            = _UxGT("PERIGO: Unha mala configuración pode producir daños! Proceder igualmente?");
  LSTR MSG_TOUCHMI_PROBE                  = _UxGT("TouchMI");
  LSTR MSG_TOUCHMI_INIT                   = _UxGT("Iniciar TouchMI");
  LSTR MSG_TOUCHMI_ZTEST                  = _UxGT("Test de Desfase Z");
  LSTR MSG_TOUCHMI_SAVE                   = _UxGT("Gardar");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI          = _UxGT("Estender TouchMI");
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("Estender Sonda Z");
  LSTR MSG_MANUAL_STOW                    = _UxGT("Recoller Sonda Z");
  LSTR MSG_HOME_FIRST                     = _UxGT("Orixe %s Primeiro");
  LSTR MSG_ZPROBE_OFFSETS                 = _UxGT("Desfases Sonda");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Desfase Sonda X");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Desfase Sonda Y");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Desfase Sonda Z");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Desfase Sonda @");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Micropaso Sonda-Z");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Micropaso X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Micropaso Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Micropaso Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Micropaso @");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Total");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Erro FinCarro");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Fallo Quentando");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Erro:Temp Redundante");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("FUGA TÉRMICA");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Erro:TEMP MÁX");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Erro:TEMP MÍN");
  LSTR MSG_HALTED                         = _UxGT("IMPRESORA DETIDA");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Debe reiniciar!");
  LSTR MSG_HEATING                        = _UxGT("Quentando...");
  LSTR MSG_COOLING                        = _UxGT("Arrefriando...");
  LSTR MSG_BED_HEATING                    = _UxGT("Quentando cama...");
  LSTR MSG_BED_COOLING                    = _UxGT("Enfriando Cama...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Quentando Cámara...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Arrefriando Cámara...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Calibracion Delta");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Calibrar X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Calibrar Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Calibrar Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Calibrar Centro");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Configuración Delta");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Auto Calibración");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Barra Diagonal");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Altura");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Radio");
  LSTR MSG_INFO_MENU                      = _UxGT("Acerca de...");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Información");
  LSTR MSG_3POINT_LEVELING                = _UxGT("Nivelación 3puntos");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Nivelación Lineal");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Nivelación Bilineal");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Nivelación UBL");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Nivelación en Malla");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Estatísticas");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Placa nai");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistores");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Extrusores");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Baudios");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protocolo");
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Reloxo Traballo: OFF");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Reloxo Traballo: ON");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Luz da Caixa");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Brillo Luces");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("IMPRESORA INCORRECTA");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Impresións");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Completadas");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Total");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Máis Longa");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Extruido");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Temp Mín");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Temp Máx");
  LSTR MSG_INFO_PSU                       = _UxGT("Fonte Alimentación");
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Forza do Motor");
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ Driver %");
  LSTR MSG_ERROR_TMC                      = _UxGT("ERRO CONEX. TMC");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("Escribe DAC EEPROM");
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("CAMBIAR FILAMENTO");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("IMPRESIÓN PAUSADA");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("CARGAR FILAMENTO");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("DESCARGAR FILAMENTO");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("OPCIÓN DE RETOMAR:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Purgar máis");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Retomar traballo");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Bico: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Sensor Filamento");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Dist mm Sensor Fil");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Fallo ao ir á Orixe");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Fallo ao Sondar");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("ESCOLLE FILAMENTO");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("Actualizar FW MMU!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU Precisa Atención.");
  LSTR MSG_MMU2_RESUME                    = _UxGT("Retomar impr.");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("Retomando...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("Cargar Filamento");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("Cargar Todo");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("Cargar até bico");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("Expulsar Filamento");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("Expulsar Filamento ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("Descargar Filamento");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("Cargando Fil. %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Expulsando Fil. ...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Descargando Fil...");
  LSTR MSG_MMU2_ALL                       = _UxGT("Todo");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("Filamento ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("Reiniciar MMU");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("Reiniciando MMU...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("Expulsar, premer");

  LSTR MSG_MIX                            = _UxGT("Mestura");
  LSTR MSG_MIX_COMPONENT_N                = _UxGT("Compoñente {");
  LSTR MSG_MIXER                          = _UxGT("Mesturadora");
  LSTR MSG_GRADIENT                       = _UxGT("Degradado");
  LSTR MSG_FULL_GRADIENT                  = _UxGT("Degradado Total");
  LSTR MSG_TOGGLE_MIX                     = _UxGT("Mestura Conmutada");
  LSTR MSG_CYCLE_MIX                      = _UxGT("Mestura Cíclica");
  LSTR MSG_GRADIENT_MIX                   = _UxGT("Mestura de Degradado");
  LSTR MSG_REVERSE_GRADIENT               = _UxGT("Degradado Inverso");
  LSTR MSG_ACTIVE_VTOOL                   = _UxGT("Activar Ferr-V");
  LSTR MSG_START_VTOOL                    = _UxGT("Inicio Ferr-V");
  LSTR MSG_END_VTOOL                      = _UxGT("  Fin Ferr-V");
  LSTR MSG_GRADIENT_ALIAS                 = _UxGT("Alias Ferr-V");
  LSTR MSG_RESET_VTOOLS                   = _UxGT("Reiniciar Ferr-V");
  LSTR MSG_COMMIT_VTOOL                   = _UxGT("Commit mest. Ferr-V");
  LSTR MSG_VTOOLS_RESET                   = _UxGT("Ferr-V reiniciadas");
  LSTR MSG_START_Z                        = _UxGT("Inicio Z:");
  LSTR MSG_END_Z                          = _UxGT("  Fin Z:");

  LSTR MSG_GAMES                          = _UxGT("Xogos");
  LSTR MSG_BRICKOUT                       = _UxGT("Brickout");
  LSTR MSG_INVADERS                       = _UxGT("Invaders");
  LSTR MSG_SNAKE                          = _UxGT("Sn4k3");
  LSTR MSG_MAZE                           = _UxGT("Labirinto");

  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_1_LINE("Premer para continuar"));
  LSTR MSG_PAUSE_PRINT_PARKING            = _UxGT(MSG_1_LINE("Estacionando..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Agarde..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Introduza e click"));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_1_LINE("Prema para quentar"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Quentando..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Descargando..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Cargando..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("Purgando..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_1_LINE("Prema para finalizar"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Retomando..."));

  LSTR MSG_TMC_DRIVERS                    = _UxGT("Controladores TMC");
  LSTR MSG_TMC_CURRENT                    = _UxGT("Controlador Actual");
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("Limiar Hibrido");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Orixe sen Sensores");
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("Modo de pasos");
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("StealthChop Habilit.");
  LSTR MSG_SERVICE_RESET                  = _UxGT("Reiniciar");
  LSTR MSG_SERVICE_IN                     = _UxGT(" dentro:");
  LSTR MSG_BACKLASH                       = _UxGT("Reacción");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("Corrección");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("Suavizado");

  LSTR MSG_LEVEL_X_AXIS                   = _UxGT("Nivel Eixe X");
  LSTR MSG_AUTO_CALIBRATE                 = _UxGT("Auto Calibrar");
  LSTR MSG_HEATER_TIMEOUT                 = _UxGT("Tempo exc. Quent.");
  LSTR MSG_REHEAT                         = _UxGT("Requentar");
  LSTR MSG_REHEATING                      = _UxGT("Requentando...");
}

namespace LanguageWide_gl {
  using namespace LanguageNarrow_gl;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Total Impresións");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Completadas");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Tempo Total Imp.");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Impresión máis longa");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Total Extruído");
  #endif
}

namespace LanguageTall_gl {
  using namespace LanguageWide_gl;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Preme o botón para", "continuar impresión"));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Agarde para", "comezar cambio", "de filamento"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Introduza o", "filamento e", "faga click"));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Prema o botón para", "quentar o bico"));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Quentando bico", "Agarde, por favor..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_3_LINE("Agarde pola", "descarga do", "filamento"));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_3_LINE("Agarde pola", "carga do", "filamento"));
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("Agarde para", "purgar o filamento"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_2_LINE("Prema para finalizar", "a purga do filamen."));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_3_LINE("Agarde a que", "se retome", "a impresión"));
  #endif
}

namespace Language_gl {
  using namespace LanguageTall_gl;
}
