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
 * Spanish
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define MEDIA_TYPE_ES "SD/FD"

namespace LanguageNarrow_es {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Spanish");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" Lista");
  LSTR MSG_YES                            = _UxGT("SI");
  LSTR MSG_NO                             = _UxGT("NO");
  LSTR MSG_BACK                           = _UxGT("Atrás");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Cancelando...");
  LSTR MSG_MEDIA_INSERTED                 = MEDIA_TYPE_ES _UxGT(" insertado");
  LSTR MSG_MEDIA_REMOVED                  = MEDIA_TYPE_ES _UxGT(" retirado");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Esperando al ") MEDIA_TYPE_ES;
  LSTR MSG_MEDIA_INIT_FAIL                = _UxGT("Fallo al iniciar ") MEDIA_TYPE_ES;
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Error lectura ") MEDIA_TYPE_ES;
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("Disp. USB retirado");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("Inicio USB fallido");
  LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("Desbordamiento de subllamada");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Endstops"); // Max length 8 characters
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Soft Endstops");
  LSTR MSG_MAIN_MENU                      = _UxGT("Menú principal");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Ajustes avanzados");
  LSTR MSG_CONFIGURATION                  = _UxGT("Configuración");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Inicio automático");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Apagar motores");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Menú depuración");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Prob. barra progreso");
  LSTR MSG_HOMING                         = _UxGT("Origen");
  LSTR MSG_AUTO_HOME                      = _UxGT("Llevar al origen");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Origen X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Origen Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Origen Z");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Auto alineado Z");
  LSTR MSG_ITERATION                      = _UxGT("G34 Iteración: %i");
  LSTR MSG_DECREASING_ACCURACY            = _UxGT("¡Precisión disminuyendo!");
  LSTR MSG_ACCURACY_ACHIEVED              = _UxGT("Precisión conseguida");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Origen XYZ");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Pulsar para comenzar");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Siguiente punto");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("¡Nivelación lista!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Compen. Altura");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Ajustar desfases");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Desfase aplicada");

  LSTR MSG_PREHEAT_1                      = _UxGT("Precal. ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Precal. ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Precal. ") PREHEAT_1_LABEL _UxGT(" Fusor");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Precal. ") PREHEAT_1_LABEL _UxGT(" Fusor ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Precal. ") PREHEAT_1_LABEL _UxGT(" Todo");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Precal. ") PREHEAT_1_LABEL _UxGT(" Cama");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Precal. ") PREHEAT_1_LABEL _UxGT(" Ajuste");

  LSTR MSG_PREHEAT_M                      = _UxGT("Precal. $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Precal. $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Precal. $ Fusor");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Precal. $ Fusor ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Precal. $ Todo");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Precal. $ Cama");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Precal. $ Ajuste");

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Precal. manual");
  LSTR MSG_COOLDOWN                       = _UxGT("Enfriar");
  LSTR MSG_CUTTER_FREQUENCY               = _UxGT("Frecuencia");
  LSTR MSG_LASER_MENU                     = _UxGT("Control Láser");
  LSTR MSG_LASER_POWER                    = _UxGT("Potencia Láser");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("Control Mandrino");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("Potencia Mandrino");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("Invertir giro");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Encender Fuente");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Apagar Fuente");
  LSTR MSG_EXTRUDE                        = _UxGT("Extruir");
  LSTR MSG_RETRACT                        = _UxGT("Retraer");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Mover ejes");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Sondear y Nivelar");
  LSTR MSG_BED_LEVELING                   = _UxGT("Nivelando Cama");
  LSTR MSG_LEVEL_BED                      = _UxGT("Nivelar Cama");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Recorrido Cama");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Siguente Esquina");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Editor Mallado");
  LSTR MSG_EDIT_MESH                      = _UxGT("Editar Mallado");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Ed. Mallado parada");
  LSTR MSG_PROBING_POINT                  = _UxGT("Sondear Punto");
  LSTR MSG_MESH_X                         = _UxGT("Índice X");
  LSTR MSG_MESH_Y                         = _UxGT("Índice Y");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Valor Z");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Com. Personalizados");
  LSTR MSG_M48_TEST                       = _UxGT("M48 Probar Sonda");
  LSTR MSG_M48_POINT                      = _UxGT("M48 Punto");
  LSTR MSG_M48_DEVIATION                  = _UxGT("Desviación");
  LSTR MSG_IDEX_MENU                      = _UxGT("Modo IDEX");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("Desfase Herramienta");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Auto-Aparcado");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Duplicar");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Copia Reflejada");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Control Total");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2ª Fusor Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2ª Fusor @");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("Hacer G29");
  LSTR MSG_UBL_TOOLS                      = _UxGT("Herramientas UBL");
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("Punto de inclinación");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Crear Mallado man.");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Colocar cuña y Medir");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Medir");
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Retirar y Medir Cama");
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Mover al Siguente");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Temp. Cama");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Temp. Cama perso.");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Temp. Fusor");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Temp. Fusor perso.");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Edit. Mallado perso.");
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Ajuste fino Mallado");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Term. edici. Mallado");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Crear Mallado Pers.");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Crear Mallado");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("Crear Mallado ($)");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Valid. Mall. ($)");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Crear Mallado Frío");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Ajustar alt. Mallado");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Cantidad de altura");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Valid. Mallado");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Valid. Mall. perso.");
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26 Calentando Cama");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26 Calent. Boquilla");
  LSTR MSG_G26_MANUAL_PRIME               = _UxGT("Imprimado manual...");
  LSTR MSG_G26_FIXED_LENGTH               = _UxGT("Impri. longit. fija");
  LSTR MSG_G26_PRIME_DONE                 = _UxGT("Imprimación Lista");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26 Cancelado");
  LSTR MSG_G26_LEAVING                    = _UxGT("Dejando G26");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Contin. Mallado cama");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Nivelando Mallado");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("Nivelando 3Puntos");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Niv. Mall. cuadri");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Nivel de Mallado");
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Puntos Laterales");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Tipo de mapa ");
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Salida Mapa mallado");
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Salida para el host");
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("Salida para CSV");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Cópia de seg. ext");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("Salida Info. UBL");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Cantidad de relleno");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Relleno manual");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Relleno inteligente");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Mallado de relleno");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Invalidar todo");
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Invalidar proximos");
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Ajustar Fino Todo");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Ajustar Fino proxi.");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Almacen de Mallado");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Huecos memoria");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Cargar Mall. cama");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Guardar Mall. cama");
  LSTR MSG_MESH_LOADED                    = _UxGT("Malla %i Cargada");
  LSTR MSG_MESH_SAVED                     = _UxGT("Malla %i Guardada");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("Sin guardar");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("Error Guardar UBL");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("Error Restaurar UBL");
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Desfase de Z: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Desfase de Z Parado");
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("UBL Paso a Paso");
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1.Crear Mall. Frío");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2.Relleno intelig.");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3.Valid. Mallado");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4.Ajustar Fino Todo");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5.Valid. Mallado");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6.Ajustar Fino Todo");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7.Guardar Mall. cama");

  LSTR MSG_LED_CONTROL                    = _UxGT("Control LED");
  LSTR MSG_LIGHTS                         = _UxGT("Luces");
  LSTR MSG_LIGHT_N                        = _UxGT("Luce #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("Color predefinido");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Rojo");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Naranja");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Amarillo");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Verde");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Azul");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Índigo");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Violeta");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Blanco");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Por defecto");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Color personalizado");
  LSTR MSG_INTENSITY_R                    = _UxGT("Intensidad Rojo");
  LSTR MSG_INTENSITY_G                    = _UxGT("Intensidad Verde");
  LSTR MSG_INTENSITY_B                    = _UxGT("Intensidad Azul");
  LSTR MSG_INTENSITY_W                    = _UxGT("Intensidad Blanco");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Brillo");

  LSTR MSG_MOVING                         = _UxGT("Moviendo...");
  LSTR MSG_FREE_XY                        = _UxGT("Libre XY");
  LSTR MSG_MOVE_X                         = _UxGT("Mover X");
  LSTR MSG_MOVE_Y                         = _UxGT("Mover Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Mover Z");
  LSTR MSG_MOVE_N                         = _UxGT("Mover @");
  LSTR MSG_MOVE_E                         = _UxGT("Extrusor");
  LSTR MSG_MOVE_EN                        = _UxGT("Extrusor *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Hotend muy frio");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Mover $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Mover $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Mover $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Velocidad");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Cama Z");
  LSTR MSG_NOZZLE                         = _UxGT("Boquilla");
  LSTR MSG_NOZZLE_N                       = _UxGT("Boquilla ~");
  LSTR MSG_NOZZLE_PARKED                  = _UxGT("Boquilla Aparcada");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Boquilla en Espera");
  LSTR MSG_BED                            = _UxGT("Cama");
  LSTR MSG_CHAMBER                        = _UxGT("Recinto");
  LSTR MSG_FAN_SPEED                      = _UxGT("Ventilador");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Ventilador ~");
  LSTR MSG_STORED_FAN_N                   = _UxGT("Vent. almacenado ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Vel. Ext. ventil.");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Vel. Ext. ventil. ~");
  LSTR MSG_CONTROLLER_FAN                 = _UxGT("Controlador Vent.");
  LSTR MSG_CONTROLLER_FAN_IDLE_SPEED      = _UxGT("Velocidad en reposo");
  LSTR MSG_CONTROLLER_FAN_AUTO_ON         = _UxGT("Modo Auto");
  LSTR MSG_CONTROLLER_FAN_SPEED           = _UxGT("Velocidad Activa");
  LSTR MSG_CONTROLLER_FAN_DURATION        = _UxGT("Periodo de reposo");
  LSTR MSG_FLOW                           = _UxGT("Flujo");
  LSTR MSG_FLOW_N                         = _UxGT("Flujo ~");
  LSTR MSG_CONTROL                        = _UxGT("Control");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Factor");
  LSTR MSG_AUTOTEMP                       = _UxGT("Temp. Autom.");
  LSTR MSG_LCD_ON                         = _UxGT("Enc");
  LSTR MSG_LCD_OFF                        = _UxGT("Apg");
  LSTR MSG_PID_AUTOTUNE                   = _UxGT("PID Auto-ajuste");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("PID Auto-ajuste *");
  LSTR MSG_SELECT_E                       = _UxGT("Seleccionar *");
  LSTR MSG_ACC                            = _UxGT("Aceleración");
  LSTR MSG_JERK                           = _UxGT("Jerk");
  LSTR MSG_VA_JERK                        = _UxGT("Max ") STR_A _UxGT(" Jerk");
  LSTR MSG_VB_JERK                        = _UxGT("Max ") STR_B _UxGT(" Jerk");
  LSTR MSG_VC_JERK                        = _UxGT("Max ") STR_C _UxGT(" Jerk");
  LSTR MSG_VN_JERK                        = _UxGT("Max @ Jerk");
  LSTR MSG_VE_JERK                        = _UxGT("Max E Jerk");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Desvi. Unión");
  LSTR MSG_MAX_SPEED                      = _UxGT("Max Velocidad");
  LSTR MSG_VMAX_A                         = _UxGT("Max ") STR_A _UxGT(" Speed");
  LSTR MSG_VMAX_B                         = _UxGT("Max ") STR_B _UxGT(" Speed");
  LSTR MSG_VMAX_C                         = _UxGT("Max ") STR_C _UxGT(" Speed");
  LSTR MSG_VMAX_N                         = _UxGT("Max @ Speed");
  LSTR MSG_VMAX_E                         = _UxGT("Max E Speed");
  LSTR MSG_VMAX_EN                        = _UxGT("Max * Speed");
  LSTR MSG_VMIN                           = _UxGT("Vmin");
  LSTR MSG_VTRAV_MIN                      = _UxGT("Vel. viaje min");
  LSTR MSG_ACCELERATION                   = _UxGT("Acceleración");
  LSTR MSG_AMAX_A                         = _UxGT("Acel. max") STR_A;
  LSTR MSG_AMAX_B                         = _UxGT("Acel. max") STR_B;
  LSTR MSG_AMAX_C                         = _UxGT("Acel. max") STR_C;
  LSTR MSG_AMAX_N                         = _UxGT("Acel. max@");
  LSTR MSG_AMAX_E                         = _UxGT("Acel. maxE");
  LSTR MSG_AMAX_EN                        = _UxGT("Acel. max *");
  LSTR MSG_A_RETRACT                      = _UxGT("Acel. retrac.");
  LSTR MSG_A_TRAVEL                       = _UxGT("Acel. Viaje");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Pasos/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" pasos/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" pasos/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" pasos/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ pasos/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E pasos/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* pasos/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperatura");
  LSTR MSG_MOTION                         = _UxGT("Movimiento");
  LSTR MSG_FILAMENT                       = _UxGT("Filamento");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E en mm") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Diámetro Fil.");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Diámetro Fil. *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Descarga mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Carga mm");
  LSTR MSG_ADVANCE_K                      = _UxGT("Avance K");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("Avance K *");
  LSTR MSG_CONTRAST                       = _UxGT("Contraste LCD");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Guardar EEPROM");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Cargar EEPROM");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Rest. fábrica");
  LSTR MSG_INIT_EEPROM                    = _UxGT("Inicializar EEPROM");
  LSTR MSG_ERR_EEPROM_CRC                 = _UxGT("Err: EEPROM CRC");
  LSTR MSG_ERR_EEPROM_SIZE                = _UxGT("Err: EEPROM Tamaño");
  LSTR MSG_ERR_EEPROM_VERSION             = _UxGT("Err: Versión EEPROM");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("Actualizar ") MEDIA_TYPE_ES;
  LSTR MSG_RESET_PRINTER                  = _UxGT("Resetear Impresora");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Recargar");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Pantalla de Inf.");
  LSTR MSG_PREPARE                        = _UxGT("Preparar");
  LSTR MSG_TUNE                           = _UxGT("Ajustar");
  LSTR MSG_START_PRINT                    = _UxGT("Iniciar impresión");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("Siguinte");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Iniciar");
  LSTR MSG_BUTTON_STOP                    = _UxGT("Parar");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Imprimir");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Reiniciar");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("Cancelar");
  LSTR MSG_BUTTON_DONE                    = _UxGT("Listo");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Retroceder");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Proceder");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Pausar impresión");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Reanudar impresión");
  LSTR MSG_STOP_PRINT                     = _UxGT("Detener impresión");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Imprimiendo Objeto");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Cancelar Objeto");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Cancelar Objeto {");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Rec. Fallo electrico");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Imprim. desde ") MEDIA_TYPE_ES;
  LSTR MSG_NO_MEDIA                       = MEDIA_TYPE_ES _UxGT(" no presente");
  LSTR MSG_DWELL                          = _UxGT("Reposo...");
  LSTR MSG_USERWAIT                       = _UxGT("Pulsar para Reanudar");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Impresión Pausada");
  LSTR MSG_PRINTING                       = _UxGT("Imprimiendo...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Impresión cancelada");
  LSTR MSG_PRINT_DONE                     = _UxGT("Impresión Completada");
  LSTR MSG_NO_MOVE                        = _UxGT("Sin movimiento");
  LSTR MSG_KILLED                         = _UxGT("MUERTA");
  LSTR MSG_STOPPED                        = _UxGT("DETENIDA");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Retraer mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Interc. Retraer mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Retraer  V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Levantar mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("DesRet mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Interc. DesRet mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("DesRet V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("S UnRet V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Retracción Auto.");
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("Inter. longitud");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("Purgar longitud");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Cambiar Herramienta");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Aumentar Z");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Vel. de Cebado");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Vel. de retracción");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Cambiar filamento");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Cambiar filamento *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Cargar filamento");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Cargar filamento *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Descargar filamento");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Descargar fil. *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Descargar todo");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Iniciar SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Iniciar USB");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Iniciar ") MEDIA_TYPE_ES;
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Cambiar ") MEDIA_TYPE_ES;
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Lanzar ") MEDIA_TYPE_ES;
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Sonda Z fuera cama");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Factor de desviación");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("Auto-Prueba");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Reiniciar");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("Subir pistón");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("Bajar pistón");
  LSTR MSG_BLTOUCH_SW_MODE                = _UxGT("Modo Software");
  LSTR MSG_BLTOUCH_5V_MODE                = _UxGT("Modo 5V");
  LSTR MSG_BLTOUCH_OD_MODE                = _UxGT("Modo OD");
  LSTR MSG_BLTOUCH_MODE_STORE             = _UxGT("Modo almacenar");
  LSTR MSG_BLTOUCH_MODE_STORE_5V          = _UxGT("Poner BLTouch a 5V");
  LSTR MSG_BLTOUCH_MODE_STORE_OD          = _UxGT("Poner BLTouch a OD");
  LSTR MSG_BLTOUCH_MODE_ECHO              = _UxGT("Informe de drenaje");
  LSTR MSG_BLTOUCH_MODE_CHANGE            = _UxGT("PELIGRO: ¡Una mala configuración puede producir daños! ¿Proceder igualmente?");
  LSTR MSG_TOUCHMI_PROBE                  = _UxGT("TouchMI");
  LSTR MSG_TOUCHMI_INIT                   = _UxGT("Iniciar TouchMI");
  LSTR MSG_TOUCHMI_ZTEST                  = _UxGT("Test de desfase Z");
  LSTR MSG_TOUCHMI_SAVE                   = _UxGT("Guardar");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI          = _UxGT("Subir TouchMI");
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("Subir Sonda Z");
  LSTR MSG_MANUAL_STOW                    = _UxGT("Bajar Sonda Z");
  LSTR MSG_HOME_FIRST                     = _UxGT("Origen %s Prim.");
  LSTR MSG_ZPROBE_OFFSETS                 = _UxGT("Desf. Sonda");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Desf. Sonda X");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Desf. Sonda Y");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Desf. Sonda Z");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Desf. Sonda @");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Ajuste Z al paso");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Micropaso X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Micropaso Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Micropaso Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Micropaso @");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Total");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Cancelado - Endstop");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Calent. fallido");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Err: TEMP. REDUN.");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("FUGA TÉRMICA");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Err:TEMP. MÁX");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Err:TEMP. MIN");
  LSTR MSG_HALTED                         = _UxGT("IMPRESORA DETENIDA");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Por favor, reinicie");
  LSTR MSG_HEATING                        = _UxGT("Calentando...");
  LSTR MSG_COOLING                        = _UxGT("Enfriando...");
  LSTR MSG_BED_HEATING                    = _UxGT("Calentando Cama...");
  LSTR MSG_BED_COOLING                    = _UxGT("Enfriando Cama...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Calentando Cámara...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Enfriando Cámara...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Calibración Delta");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Calibrar X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Calibrar Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Calibrar Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Calibrar Centro");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Configuración Delta");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Auto Calibración");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Barra Diagonal");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Altura");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Radio");
  LSTR MSG_INFO_MENU                      = _UxGT("Info. Impresora");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Info. Impresora");
  LSTR MSG_3POINT_LEVELING                = _UxGT("Nivelando 3puntos");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Nivelando Lineal");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Nivelando Bilineal");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Nivelando UBL");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Nivelando en Mallado");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Estadísticas Imp.");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Info. Controlador");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistores");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Extrusores");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Baudios");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protocolo");
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Vig. Fuga Térm.: OFF");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Vig. Fuga Térm.: ON");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Luz cabina");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Brillo cabina");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Impresora incorrecta");

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Impresiones");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Completadas");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Total");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Más larga");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Extruido");

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Temp. Mínima");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Temp. Máxima");
  LSTR MSG_INFO_PSU                       = _UxGT("F. Aliment.");
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Fuerza de empuje");
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ Driver %");
  LSTR MSG_ERROR_TMC                      = _UxGT("ERROR CONEX. TMC");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("Escribe DAC EEPROM");
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("CAMBIAR FILAMENTO");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("IMPRESIÓN PAUSADA");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("CARGAR FILAMENTO");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("DESCARGAR FILAMENTO");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("OPC. REINICIO:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Purgar más");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Continuar imp.");
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Boquilla: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Sens. filamento");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Dist. filamento mm");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Ir a origen Fallado");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Sondeo Fallado");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("ELIJE FILAMENTO");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("¡Actu. MMU Firmware!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU Necesita Cuidado");
  LSTR MSG_MMU2_RESUME                    = _UxGT("Continuar imp.");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("Resumiendo...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("Cargar Filamento");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("Cargar Todo");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("Cargar hasta boqui.");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("Expulsar Filamento");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("Expulsar Filamento ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("Descargar Filamento");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("Cargando Fil. %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Expulsando Fil. ...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Descargando Fil....");
  LSTR MSG_MMU2_ALL                       = _UxGT("Todo");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("Filamento ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("Reiniciar MMU");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("Reiniciando MMU...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("Retirar, y pulsar");

  LSTR MSG_MIX                            = _UxGT("Mezcla");
  LSTR MSG_MIX_COMPONENT_N                = _UxGT("Componente {");
  LSTR MSG_MIXER                          = _UxGT("Miezclador");
  LSTR MSG_GRADIENT                       = _UxGT("Degradado");
  LSTR MSG_FULL_GRADIENT                  = _UxGT("Degradado Total");
  LSTR MSG_TOGGLE_MIX                     = _UxGT("Mezcla Conmutada");
  LSTR MSG_CYCLE_MIX                      = _UxGT("Mezcla Cíclica");
  LSTR MSG_GRADIENT_MIX                   = _UxGT("Mezcla de Degradado");
  LSTR MSG_REVERSE_GRADIENT               = _UxGT("Degradado inverso");
  LSTR MSG_ACTIVE_VTOOL                   = _UxGT("Activar Herr.V");
  LSTR MSG_START_VTOOL                    = _UxGT("Inicio Herr.V");
  LSTR MSG_END_VTOOL                      = _UxGT("   Fin Herr.V");
  LSTR MSG_GRADIENT_ALIAS                 = _UxGT("Alias Herr.V");
  LSTR MSG_RESET_VTOOLS                   = _UxGT("Reiniciar Herr.V");
  LSTR MSG_COMMIT_VTOOL                   = _UxGT("Cometer mezc. Herr.V");
  LSTR MSG_VTOOLS_RESET                   = _UxGT("Herr.V reiniciados");
  LSTR MSG_START_Z                        = _UxGT("Inicio Z:");
  LSTR MSG_END_Z                          = _UxGT("   Fin Z:");

  LSTR MSG_GAMES                          = _UxGT("Juegos");
  LSTR MSG_BRICKOUT                       = _UxGT("Brickout");
  LSTR MSG_INVADERS                       = _UxGT("Invaders");
  LSTR MSG_SNAKE                          = _UxGT("Sn4k3");
  LSTR MSG_MAZE                           = _UxGT("Maze");

  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_1_LINE("Pulse para continuar"));
  LSTR MSG_PAUSE_PRINT_PARKING            = _UxGT(MSG_1_LINE("Aparcando..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Por Favor espere..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Inserte y Pulse"));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_1_LINE("Pulse para Calentar"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Calentando..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Liberando..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Cargando..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("Purgando..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_1_LINE("Pulse para finalizar"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Reanudando..."));

  LSTR MSG_TMC_DRIVERS                    = _UxGT("Controladores TMC");
  LSTR MSG_TMC_CURRENT                    = _UxGT("Amperaje Controlador");
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("Límite Hibrido");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Origen sin sensores");
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("Modo de pasos");
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("StealthChop Habilit.");
  LSTR MSG_SERVICE_RESET                  = _UxGT("Reiniciar");
  LSTR MSG_SERVICE_IN                     = _UxGT(" dentro:");
  LSTR MSG_BACKLASH                       = _UxGT("Backlash");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("Corrección");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("Suavizado");

  LSTR MSG_LEVEL_X_AXIS                   = _UxGT("Nivel Eje X");
  LSTR MSG_AUTO_CALIBRATE                 = _UxGT("Auto Calibrar");
  LSTR MSG_HEATER_TIMEOUT                 = _UxGT("T. de esp. Calent.");
  LSTR MSG_REHEAT                         = _UxGT("Recalentar");
  LSTR MSG_REHEATING                      = _UxGT("Recalentando...");
}

namespace LanguageWide_es {
  using namespace LanguageNarrow_es;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Cont. de impresión");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Completadas");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Tiempo total de imp.");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Impresión más larga");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Total Extruido");
  #endif
}

namespace LanguageTall_es {
  using namespace LanguageWide_es;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Pulsar el botón para", "reanudar impresión"));
    LSTR MSG_PAUSE_PRINT_PARKING          = _UxGT(MSG_1_LINE("Aparcando..."));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Esperando para", "iniciar el cambio", "de filamento"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Inserte el filamento", "y pulse el botón", "para continuar..."));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Pulse el botón para", "calentar la boquilla"));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Calentando boquilla", "Espere por favor..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_2_LINE("Espere para", "liberar el filamento"));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_2_LINE("Espere para", "cargar el filamento"));
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("Espere para", "purgar el filamento"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_2_LINE("Pulse para finalizar", "la purga de filamen."));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_2_LINE("Esperando impresora", "para reanudar..."));
  #endif
}

namespace Language_es {
  using namespace LanguageTall_es;
}
