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
 * Catalan
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */
namespace LanguageNarrow_ca {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Catalan");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" preparada.");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Targeta detectada.");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Targeta extreta.");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Endstops");
  LSTR MSG_MAIN_MENU                      = _UxGT("Menú principal");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Inici automatic");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Desactiva motors");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Menu de depuracio");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Test barra progres");
  LSTR MSG_HOMING                         = _UxGT("Origen");
  LSTR MSG_AUTO_HOME                      = _UxGT("Ves a l'origen");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("X a origen");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Y a origen");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Z a origen");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Origen XYZ");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Premeu per iniciar");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Següent punt");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Anivellament fet!");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Ajusta decalatge");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Decalatge aplicat");

  LSTR MSG_PREHEAT_1                      = _UxGT("Preescalfa ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Preescalfa ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Preescalfa ") PREHEAT_1_LABEL _UxGT(" End");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Preescalfa ") PREHEAT_1_LABEL _UxGT(" End ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Preescalfa ") PREHEAT_1_LABEL _UxGT(" Tot");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Preescalfa ") PREHEAT_1_LABEL _UxGT(" Llit");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Preescalfa ") PREHEAT_1_LABEL _UxGT(" Conf.");

  LSTR MSG_PREHEAT_M                      = _UxGT("Preescalfa $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Preescalfa $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Preescalfa $ End");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Preescalfa $ End ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Preescalfa $ Tot");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Preescalfa $ Llit");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Preescalfa $ Conf.");

  LSTR MSG_COOLDOWN                       = _UxGT("Refreda");

  LSTR MSG_EXTRUDE                        = _UxGT("Extrudeix");
  LSTR MSG_RETRACT                        = _UxGT("Retreu");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Mou eixos");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Sonda i anivella");
  LSTR MSG_BED_LEVELING                   = _UxGT("Anivella llit");
  LSTR MSG_LEVEL_BED                      = _UxGT("Anivella llit");

  LSTR MSG_MOVING                         = _UxGT("Movent..");
  LSTR MSG_FREE_XY                        = _UxGT("XY lliures");
  LSTR MSG_MOVE_X                         = _UxGT("Mou X");
  LSTR MSG_MOVE_Y                         = _UxGT("Mou Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Mou Z");
  LSTR MSG_MOVE_N                         = _UxGT("Mou @");
  LSTR MSG_MOVE_E                         = _UxGT("Extrusor");
  LSTR MSG_MOVE_EN                        = _UxGT("Extrusor *");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Mou $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Mou $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Mou $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Velocitat");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Llit Z");
  LSTR MSG_NOZZLE                         = _UxGT("Nozzle");
  LSTR MSG_NOZZLE_N                       = _UxGT("Nozzle ~");
  LSTR MSG_BED                            = _UxGT("Llit");
  LSTR MSG_FAN_SPEED                      = _UxGT("Vel. Ventilador");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Vel. Ventilador ~");
  LSTR MSG_FLOW                           = _UxGT("Flux");
  LSTR MSG_FLOW_N                         = _UxGT("Flux ~");
  LSTR MSG_VTRAV_MIN                      = _UxGT("VViatge min");

  LSTR MSG_A_RETRACT                      = _UxGT("Accel. retracc");
  LSTR MSG_A_TRAVEL                       = _UxGT("Accel. Viatge");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Passos/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" passos/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" passos/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" passos/mm");
  LSTR MSG_E_STEPS                        = _UxGT("@ passos/mm");
  LSTR MSG_N_STEPS                        = _UxGT("E passos/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* passos/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperatura");
  LSTR MSG_MOTION                         = _UxGT("Moviment");
  LSTR MSG_FILAMENT                       = _UxGT("Filament");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E en mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Diam. Fil.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Diam. Fil. *");
  LSTR MSG_CONTRAST                       = _UxGT("Contrast de LCD");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Desa memoria");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Carrega memoria");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Restaura valors");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Actualitza");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Pantalla Info.");
  LSTR MSG_PREPARE                        = _UxGT("Prepara");
  LSTR MSG_TUNE                           = _UxGT("Ajusta");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pausa impressio");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Repren impressio");
  LSTR MSG_STOP_PRINT                     = _UxGT("Atura impressio.");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Imprimeix de SD");
  LSTR MSG_NO_MEDIA                       = _UxGT("No hi ha targeta");
  LSTR MSG_DWELL                          = _UxGT("En repos...");
  LSTR MSG_USERWAIT                       = _UxGT("Esperant usuari..");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Imp. cancelada");
  LSTR MSG_NO_MOVE                        = _UxGT("Sense moviment.");
  LSTR MSG_KILLED                         = _UxGT("MATAT.");
  LSTR MSG_STOPPED                        = _UxGT("ATURADA.");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Retreu mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Swap Retreure mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Retreu V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Aixeca mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("DesRet +mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Swap DesRet +mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("DesRet V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Auto retraccio");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Canvia filament");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Canvia filament *");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Inicialitza SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Inicialitza USB");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Inicialitza SD");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Canvia SD");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Sonda Z fora");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Reinicia BLTouch");
  LSTR MSG_HOME_FIRST                     = _UxGT("Home %s primer");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Decalatge X");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Decalatge Y");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Decalatge Z");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Decalatge @");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Pas a pas el Z");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Micropas X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Micropas Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Micropas Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Micropas @");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Cancel. Endstop");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Error al escalfar");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Err: TEMP REDUNDANT");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("THERMAL RUNAWAY");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Err: TEMP MAXIMA");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Err: TEMP MINIMA");
  LSTR MSG_HALTED                         = _UxGT("IMPRESSORA PARADA");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Reinicieu");
  LSTR MSG_HEATING                        = _UxGT("Escalfant...");
  LSTR MSG_BED_HEATING                    = _UxGT("Escalfant llit...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Calibratge Delta");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Calibra X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Calibra Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Calibra Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Calibra el centre");

  LSTR MSG_INFO_MENU                      = _UxGT("Quant a la impr.");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Info Impressora");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Estadistiques");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Info placa");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistors");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Extrusors");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Baud");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protocol");
  LSTR MSG_CASE_LIGHT                     = _UxGT("Llum");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Impressions");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Acabades");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Total");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Mes llarg");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Extrudit");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Temp. mínima");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Temp. màxima");
  LSTR MSG_INFO_PSU                       = _UxGT("Font alimentacio");

  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Força motor");

  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Repren impressió");

  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Impressora incorrecta");

  //
  // Filament Change screens show up to 2 lines on a 3-line display
  //
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Espereu..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Expulsant..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Insereix i prem"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Escalfant..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Carregant..."));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Reprenent..."));
}

namespace LanguageWide_ca {
  using namespace LanguageNarrow_ca;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Total impressions");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Acabades");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Temps imprimint");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Treball mes llarg");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Total extrudit");
  #endif
}

namespace LanguageTall_ca {
  using namespace LanguageWide_ca;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Esperant per", "iniciar el canvi", "de filament"));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_2_LINE("Esperant per", "treure filament"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Poseu filament", "i premeu el boto", "per continuar..."));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Premeu boto per", "escalfar nozzle."));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Escalfant nozzle", "Espereu..."));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_2_LINE("Esperant carrega", "de filament"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_2_LINE("Esperant per", "reprendre"));
  #endif
}

namespace Language_ca {
  using namespace LanguageTall_ca;
}
