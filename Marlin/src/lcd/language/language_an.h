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
 * Aragonese
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define DISPLAY_CHARSET_ISO10646_1
#define NOT_EXTENDED_ISO10646_1_5X7

namespace LanguageNarrow_an {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 1;
  LSTR LANGUAGE                           = _UxGT("Aragonese");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" parada.");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Tarcheta mesa");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Tarcheta sacada");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Endstops"); // Max length 8 characters
  LSTR MSG_MAIN_MENU                      = _UxGT("Menu prencipal");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Inicio automatico");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Amortar motors");
  LSTR MSG_HOMING                         = _UxGT("Orichen");
  LSTR MSG_AUTO_HOME                      = _UxGT("Levar a l'orichen");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Orichen X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Orichen Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Orichen Z");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Orichen XYZ");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Encetar (pretar)");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Vinient punto");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Nivelacion feita!");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Achustar desfases");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Desfase aplicau");

  LSTR MSG_PREHEAT_1                      = _UxGT("Precalentar ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Precalentar ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Precal. ") PREHEAT_1_LABEL _UxGT(" Boquilla");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Precal. ") PREHEAT_1_LABEL _UxGT(" Boquilla ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Tot");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Base");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Conf");

  LSTR MSG_PREHEAT_M                      = _UxGT("Precalentar $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Precalentar $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Precal. $ Boquilla");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Precal. $ Boquilla ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Precalentar $ Tot");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Precalentar $ Base");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Precalentar $ Conf");

  LSTR MSG_COOLDOWN                       = _UxGT("Enfriar");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Enchegar Fuent");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Amortar Fuent");
  LSTR MSG_EXTRUDE                        = _UxGT("Extruir");
  LSTR MSG_RETRACT                        = _UxGT("Retraer");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Mover Eixes");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Sondar y Nivelar");
  LSTR MSG_BED_LEVELING                   = _UxGT("Nivelar base");
  LSTR MSG_LEVEL_BED                      = _UxGT("Nivelar base");
  LSTR MSG_MOVE_X                         = _UxGT("Mover X");
  LSTR MSG_MOVE_Y                         = _UxGT("Mover Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Mover Z");
  LSTR MSG_MOVE_N                         = _UxGT("Mover @");
  LSTR MSG_MOVE_E                         = _UxGT("Extrusor");
  LSTR MSG_MOVE_EN                        = _UxGT("Extrusor *");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Mover $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Mover $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Mover $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Velocidat");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Base Z");
  LSTR MSG_NOZZLE                         = _UxGT("Boquilla");
  LSTR MSG_NOZZLE_N                       = _UxGT("Boquilla ~");
  LSTR MSG_BED                            = _UxGT("Base");
  LSTR MSG_FAN_SPEED                      = _UxGT("Ixoriador");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Ixoriador ~");
  LSTR MSG_FLOW                           = _UxGT("Fluxo");
  LSTR MSG_FLOW_N                         = _UxGT("Fluxo ~");
  LSTR MSG_CONTROL                        = _UxGT("Control");
  LSTR MSG_AUTOTEMP                       = _UxGT("Temperatura Auto.");
  LSTR MSG_SELECT_E                       = _UxGT("Trigar *");
  LSTR MSG_ACC                            = _UxGT("Aceleracion");
  LSTR MSG_VTRAV_MIN                      = _UxGT("Vel. viache min");
  LSTR MSG_ACCELERATION                   = _UxGT("Accel");
  LSTR MSG_AMAX_A                         = _UxGT("Acel. max ") STR_A;
  LSTR MSG_AMAX_B                         = _UxGT("Acel. max ") STR_B;
  LSTR MSG_AMAX_C                         = _UxGT("Acel. max ") STR_C;
  LSTR MSG_AMAX_N                         = _UxGT("Acel. max @");
  LSTR MSG_AMAX_E                         = _UxGT("Acel. max E");
  LSTR MSG_AMAX_EN                        = _UxGT("Acel. max *");
  LSTR MSG_A_RETRACT                      = _UxGT("Acel. retrac.");
  LSTR MSG_A_TRAVEL                       = _UxGT("Acel. Viaje");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Trangos/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" trangos/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" trangos/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" trangos/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ trangos/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E trangos/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* trangos/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperatura");
  LSTR MSG_MOTION                         = _UxGT("Movimiento");
  LSTR MSG_FILAMENT                       = _UxGT("Filamento");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E in mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Fil. Dia.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Fil. Dia. *");
  LSTR MSG_CONTRAST                       = _UxGT("Contraste");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Alzar memoria");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Cargar memoria");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Restaurar memoria");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Tornar a cargar");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Informacion");
  LSTR MSG_PREPARE                        = _UxGT("Preparar");
  LSTR MSG_TUNE                           = _UxGT("Achustar");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pausar impresion");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Contin. impresion");
  LSTR MSG_STOP_PRINT                     = _UxGT("Detener Impresion");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Menu de SD");
  LSTR MSG_NO_MEDIA                       = _UxGT("No i hai tarcheta");
  LSTR MSG_DWELL                          = _UxGT("Reposo...");
  LSTR MSG_USERWAIT                       = _UxGT("Aguardand ordines");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Impres. cancelada");
  LSTR MSG_NO_MOVE                        = _UxGT("Sin movimiento");
  LSTR MSG_KILLED                         = _UxGT("Aturada d'emerch.");
  LSTR MSG_STOPPED                        = _UxGT("Aturada.");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Retraer mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Swap Retraer mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Retraer  F");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Devantar mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("DesRet mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Swap DesRet mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("DesRet F");
  LSTR MSG_AUTORETRACT                    = _UxGT("Retraccion auto.");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Cambear filamento");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Cambear filamento *");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Encetan. SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Encetan. USB");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Encetan. SD");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Cambiar tarcheta");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Sonda Z fuera");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Reset BLTouch");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Desfase X");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Desfase Y");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Desfase Z");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Desfase @");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Probe Z con pas");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Micropaso X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Micropaso Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Micropaso Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Micropaso @");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Cancelado - Endstop");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Error en calentar");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Error temp adicional");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("Error de temperatura");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Error Temp Max");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Error Temp Min");
  LSTR MSG_HALTED                         = _UxGT("IMPRESORA ATURADA");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Per favor reinic.");
  LSTR MSG_HEATING                        = _UxGT("Calentando...");
  LSTR MSG_BED_HEATING                    = _UxGT("Calentando base...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Calibracion Delta");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Calibrar X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Calibrar Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Calibrar Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Calibrar Centro");

  LSTR MSG_INFO_MENU                      = _UxGT("Inf. Impresora");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Inf. Impresora");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Estadisticas Imp.");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Inf. Controlador");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistors");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Extrusors");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Baudios");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protocolo");
  LSTR MSG_CASE_LIGHT                     = _UxGT("Luz");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Impresions");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Completadas");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Total");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Mas larga");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Extrusion");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Temperatura menima");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Temperatura maxima");
  LSTR MSG_INFO_PSU                       = _UxGT("Fuente de aliment");

  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Fuerza d'o driver");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("Escri. DAC EEPROM");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Resumir imp.");

  // Up to 2 lines allowed
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_2_LINE("Aguardand iniciar", "d'o fil. cambear"));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_2_LINE("Meta o filamento", "y prete lo boton"));

  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_2_LINE("Aguardando a", "expulsar filament"));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_2_LINE("Aguardando a", "cargar filamento"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_2_LINE("Aguardando impre.", "pa continar"));
}

namespace LanguageWide_an {
  using namespace LanguageNarrow_an;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Conteo de impresion");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Completadas");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Tiempo total d'imp.");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Impresion mas larga");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Total d'extrusion");
  #endif
}

namespace LanguageTall_an {
  using namespace LanguageWide_an;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Aguardand iniciar", "d'o filamento", "cambear"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Meta o filamento", "y prete lo boton", "pa continar..."));
  #endif
}

namespace Language_an {
  using namespace LanguageTall_an;
}
