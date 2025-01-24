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
 * German
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

namespace LanguageNarrow_de {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Deutsch");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" bereit");
  LSTR MSG_YES                            = _UxGT("JA");
  LSTR MSG_NO                             = _UxGT("NEIN");
  LSTR MSG_HIGH                           = _UxGT("HOCH");
  LSTR MSG_LOW                            = _UxGT("RUNTER");
  LSTR MSG_BACK                           = _UxGT("Zurück");
  LSTR MSG_ERROR                          = _UxGT("Fehler");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Abbruch...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Medium erkannt");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Medium entfernt");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Warten auf Medium");
  LSTR MSG_MEDIA_INIT_FAIL                = _UxGT("Medium Init fehlgesch.");
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Medium Lesefehler");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("USB Gerät entfernt");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("USB Start fehlge.");
  LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("Subcall überschritten");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Endstopp"); // Max length 8 characters
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Software-Endstopp");
  LSTR MSG_MAIN_MENU                      = _UxGT("Hauptmenü");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Erw. Einstellungen");
  LSTR MSG_CONFIGURATION                  = _UxGT("Konfiguration");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Autostart");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Motoren deaktivieren"); // M84 :: Max length 19 characters
  LSTR MSG_DEBUG_MENU                     = _UxGT("Debug-Menü");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Statusbalken-Test");
  LSTR MSG_HOMING                         = _UxGT("Homing");
  LSTR MSG_AUTO_HOME                      = _UxGT("Auto Home");
  LSTR MSG_AUTO_HOME_N                    = _UxGT("Home @");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Home X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Home Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Home Z");
  LSTR MSG_FILAMENT_SET                   = _UxGT("Fila. Einstellungen");
  LSTR MSG_FILAMENT_MAN                   = _UxGT("Filament Management");
  LSTR MSG_MANUAL_LEVELING                = _UxGT("Manuell Nivellierung");
  LSTR MSG_TRAM_FL                        = _UxGT("Vorne Links");
  LSTR MSG_TRAM_FR                        = _UxGT("Vorne Rechts");
  LSTR MSG_TRAM_C                         = _UxGT("Mitte");
  LSTR MSG_TRAM_BL                        = _UxGT("Hinten Links");
  LSTR MSG_TRAM_BR                        = _UxGT("Hinten Rechts");
  LSTR MSG_MANUAL_MESH                    = _UxGT("Manuelles Netz");
  LSTR MSG_AUTO_MESH                      = _UxGT("Netz auto. erstellen");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Z-Achsen ausgleichen");
  LSTR MSG_ITERATION                      = _UxGT("G34 Iteration: %i");
  LSTR MSG_DECREASING_ACCURACY            = _UxGT("Genauigkeit sinkt!");
  LSTR MSG_ACCURACY_ACHIEVED              = _UxGT("Genauigkeit erreicht");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("XYZ homen");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Klick zum Starten");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Nächste Koordinate");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Nivellieren fertig!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Ausblendhöhe");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Setze Homeversatz");
  LSTR MSG_HOME_OFFSET_X                  = _UxGT("Homeversatz X"); // DWIN
  LSTR MSG_HOME_OFFSET_Y                  = _UxGT("Homeversatz Y"); // DWIN
  LSTR MSG_HOME_OFFSET_Z                  = _UxGT("Homeversatz Z"); // DWIN
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Homeversatz aktiv");
  LSTR MSG_TRAMMING_WIZARD                = _UxGT("Tramming Assistent");
  LSTR MSG_SELECT_ORIGIN                  = _UxGT("Wählen Sie Ursprung");
  LSTR MSG_LAST_VALUE_SP                  = _UxGT("Letzter Wert ");

  LSTR MSG_PREHEAT_1                      = PREHEAT_1_LABEL _UxGT(" Vorwärmen");
  LSTR MSG_PREHEAT_1_H                    = PREHEAT_1_LABEL _UxGT(" Vorwärmen ~");
  LSTR MSG_PREHEAT_1_END                  = PREHEAT_1_LABEL _UxGT(" Extr. Vorwärmen");
  LSTR MSG_PREHEAT_1_END_E                = PREHEAT_1_LABEL _UxGT(" Extr. Vorwärm. ~");
  LSTR MSG_PREHEAT_1_ALL                  = PREHEAT_1_LABEL _UxGT(" Alles Vorwärmen");
  LSTR MSG_PREHEAT_1_BEDONLY              = PREHEAT_1_LABEL _UxGT(" Bett Vorwärmen");
  LSTR MSG_PREHEAT_1_SETTINGS             = PREHEAT_1_LABEL _UxGT(" Einstellungen");

  LSTR MSG_PREHEAT_M                      = _UxGT("$ Vorwärmen");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("$ Vorwärmen") " ~";
  LSTR MSG_PREHEAT_M_END                  = _UxGT("$ Extr. Vorwärmen");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("$ Extr. Vorwärm. ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("$ Alles Vorwärmen");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("$ Bett Vorwärmen");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("$ Einstellungen");

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("benutzerdef. Heizen");
  LSTR MSG_COOLDOWN                       = _UxGT("Abkühlen");

  LSTR MSG_CUTTER_FREQUENCY               = _UxGT("Frequenz");
  LSTR MSG_LASER_MENU                     = _UxGT("Laser");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("Spindel-Steuerung");
  LSTR MSG_LASER_POWER                    = _UxGT("Laserleistung");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("Spindelleistung");
  LSTR MSG_LASER_TOGGLE                   = _UxGT("Laser umschalten");
  LSTR MSG_LASER_EVAC_TOGGLE              = _UxGT("Gebläse umschalten");
  LSTR MSG_LASER_ASSIST_TOGGLE            = _UxGT("Luftunterstützung");
  LSTR MSG_LASER_PULSE_MS                 = _UxGT("Test Impuls ms");
  LSTR MSG_LASER_FIRE_PULSE               = _UxGT("Fire Impuls");
  LSTR MSG_FLOWMETER_FAULT                = _UxGT("Feh. Kühlmittelfluss");
  LSTR MSG_SPINDLE_TOGGLE                 = _UxGT("Spindel umschalten");
  LSTR MSG_SPINDLE_EVAC_TOGGLE            = _UxGT("Vakuum umschalten");
  LSTR MSG_SPINDLE_FORWARD                = _UxGT("Spindel vorwärts");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("Spindelrichtung");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Netzteil ein");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Netzteil aus");
  LSTR MSG_EXTRUDE                        = _UxGT("Extrudieren");
  LSTR MSG_RETRACT                        = _UxGT("Einzug");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Achsen bewegen");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Sonden nivellieren");
  LSTR MSG_BED_LEVELING                   = _UxGT("Bett-Nivellierung");
  LSTR MSG_LEVEL_BED                      = _UxGT("Bett nivellieren");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Bett ausrichten");
  LSTR MSG_BED_TRAMMING_MANUAL            = _UxGT("Manuelles ausrichten");
  LSTR MSG_BED_TRAMMING_RAISE             = _UxGT("Das Bett anpassen, bis zum auslösen.");
  LSTR MSG_BED_TRAMMING_IN_RANGE          = _UxGT("Ecken in der Toleranz. Bett ausger.");
  LSTR MSG_BED_TRAMMING_GOOD_POINTS       = _UxGT("Gute Punkte: ");
  LSTR MSG_BED_TRAMMING_LAST_Z            = _UxGT("Letztes Z: ");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Nächste Ecke");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Netz Editor");
  LSTR MSG_MESH_VIEWER                    = _UxGT("Netzbetrachter");
  LSTR MSG_EDIT_MESH                      = _UxGT("Netz bearbeiten");
  LSTR MSG_MESH_VIEW                      = _UxGT("Netz ansehen");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Netzbearb. angeh.");
  LSTR MSG_NO_VALID_MESH                  = _UxGT("Kein gültiges Netz");
  LSTR MSG_ACTIVATE_MESH                  = _UxGT("Nivellierung aktiv.");
  LSTR MSG_PROBING_POINT                  = _UxGT("Messpunkt");
  LSTR MSG_MESH_X                         = _UxGT("Index X");
  LSTR MSG_MESH_Y                         = _UxGT("Index Y");
  LSTR MSG_MESH_INSET                     = _UxGT("Mesh-Einsatz");
  LSTR MSG_MESH_MIN_X                     = _UxGT("Mesh X Minimum");
  LSTR MSG_MESH_MAX_X                     = _UxGT("Mesh X Maximum");
  LSTR MSG_MESH_MIN_Y                     = _UxGT("Mesh Y Minimum");
  LSTR MSG_MESH_MAX_Y                     = _UxGT("Mesh Y Maximum");
  LSTR MSG_MESH_AMAX                      = _UxGT("Bereich maximieren");
  LSTR MSG_MESH_CENTER                    = _UxGT("Center Area");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Z-Wert");
  LSTR MSG_MESH_CANCEL                    = _UxGT("Mesh abgebrochen");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Benutzer-Menü");
  LSTR MSG_M48_TEST                       = _UxGT("M48 Sondentest");
  LSTR MSG_M48_POINT                      = _UxGT("M48 Punkt");
  LSTR MSG_M48_OUT_OF_BOUNDS              = _UxGT("Zu weit draußen");
  LSTR MSG_M48_DEVIATION                  = _UxGT("Abweichung");
  LSTR MSG_IDEX_MENU                      = _UxGT("IDEX-Modus");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("Werkzeugversätze");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Autom. parken");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Duplizieren");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Spiegelkopie");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("vollstä. Kontrolle");
  LSTR MSG_IDEX_DUPE_GAP                  = _UxGT("X-Lücke duplizieren");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2. Düse Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2. Düse @");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("G29 ausführen");
  LSTR MSG_UBL_TOOLS                      = _UxGT("UBL-Werkzeuge");
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("Berührungspunkt");
  LSTR MSG_UBL_TILT_MESH                  = _UxGT("Tilt Mesh");
  LSTR MSG_UBL_TILTING_GRID               = _UxGT("Tilting Grid Size");
  LSTR MSG_UBL_MESH_TILTED                = _UxGT("Mesh Tilted");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Netz manuell erst.");
  LSTR MSG_UBL_MESH_WIZARD                = _UxGT("UBL Netz Assistent");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Unterlegen & messen");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Messen");
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Entfernen & messen");
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Nächster Punkt...");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Betttemperatur");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Betttemperatur");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Hotend-Temp.");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Hotend-Temp.");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Eigenes Netz bearb.");
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Feineinstellung...");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Bearbeitung beendet");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Eigenes Netz erst.");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Netz erstellen");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("$ Netz erstellen");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Netz erstellen kalt");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Netzhöhe einst.");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Höhe");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Netz validieren");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("$ Netz validieren");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Eig. Netz validieren");
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26 heizt Bett");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26 Düse aufheizen");
  LSTR MSG_G26_MANUAL_PRIME               = _UxGT("Manuell Prime...");
  LSTR MSG_G26_FIXED_LENGTH               = _UxGT("Feste Länge Prime");
  LSTR MSG_G26_PRIME_DONE                 = _UxGT("Priming fertig");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26 abgebrochen");
  LSTR MSG_G26_LEAVING                    = _UxGT("G26 verlassen");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Netzerst. forts.");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Netz-Nivellierung");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("3-Punkt-Nivell.");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Gitternetz-Nivell.");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Netz nivellieren");
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Eckpunkte");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Kartentyp");
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Karte ausgeben");
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Ausgabe für Host");
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("Ausgabe für CSV");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Externe Sicherung");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("UBL-Info ausgeben");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Menge an Füllung");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Manuelles Füllen");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Cleveres Füllen");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Netz Füllen");
  LSTR MSG_UBL_MESH_FILLED                = _UxGT("Fehlende Punkte erg.");
  LSTR MSG_UBL_MESH_INVALID               = _UxGT("Ungültiges Netz");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Alles annullieren");
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Nächstlieg. ann.");
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Feineinst. Alles");
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Feineinst. Nächstl.");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Netz-Speicherplatz");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Speicherort");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Bettnetz laden");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Bettnetz speichern");
  LSTR MSG_UBL_INVALID_SLOT               = _UxGT("Wähle einen Mesh-Slot");
  LSTR MSG_MESH_LOADED                    = _UxGT("Netz %i geladen");
  LSTR MSG_MESH_SAVED                     = _UxGT("Netz %i gespeichert");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("Kein Speicher");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("Err:UBL speichern");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("Err:UBL wiederherst.");
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Z-Versatz: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Z-Versatz angehalten");
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("Schrittweises UBL");
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1.Netz kalt erstellen");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2.Intelligent Füllen");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3.Netz validieren");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4.Alles Feineinst.");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5.Netz validieren");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6.Alles Feineinst.");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7.Bettnetz speichern");

  LSTR MSG_LED_CONTROL                    = _UxGT("Licht-Steuerung");
  LSTR MSG_LIGHTS                         = _UxGT("Licht");
  LSTR MSG_LIGHT_N                        = _UxGT("Lichter #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("Licht-Einstellung");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Rot");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Orange");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Gelb");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Grün");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Blau");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Indigo");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Violett");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Weiß");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Standard");
  LSTR MSG_LED_CHANNEL_N                  = _UxGT("Kanal {");
  LSTR MSG_NEO2_PRESETS                   = _UxGT("Licht #2 Voreinst.");
  LSTR MSG_NEO2_BRIGHTNESS                = _UxGT("Helligkeit");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Benutzerdefiniert");
  LSTR MSG_INTENSITY_R                    = _UxGT("Intensität Rot");
  LSTR MSG_INTENSITY_G                    = _UxGT("Intensität Grün");
  LSTR MSG_INTENSITY_B                    = _UxGT("Intensität Blau");
  LSTR MSG_INTENSITY_W                    = _UxGT("Intensität Weiß");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Helligkeit");

  LSTR MSG_MOVING                         = _UxGT("In Bewegung...");
  LSTR MSG_FREE_XY                        = _UxGT("Abstand XY");
  LSTR MSG_MOVE_X                         = _UxGT("Bewege X");
  LSTR MSG_MOVE_Y                         = _UxGT("Bewege Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Bewege Z");
  LSTR MSG_MOVE_N                         = _UxGT("Bewege @");
  LSTR MSG_MOVE_E                         = _UxGT("Bewege Extruder");
  LSTR MSG_MOVE_EN                        = _UxGT("Bewege Extruder *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Hotend zu kalt");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Bewege $mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Bewege $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Bewege $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Geschw.");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Bett Z");
  LSTR MSG_NOZZLE                         = _UxGT("Düse");
  LSTR MSG_NOZZLE_N                       = _UxGT("Düse ~");
  LSTR MSG_NOZZLE_PARKED                  = _UxGT("Düse geparkt");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Düse bereit");
  LSTR MSG_BED                            = _UxGT("Bett");
  LSTR MSG_CHAMBER                        = _UxGT("Gehäuse");
  LSTR MSG_COOLER                         = _UxGT("Laser-Kühlmittel");
  LSTR MSG_COOLER_TOGGLE                  = _UxGT("Kühler umschalten");
  LSTR MSG_FLOWMETER_SAFETY               = _UxGT("Durchflusssicherheit");
  LSTR MSG_LASER                          = _UxGT("Laser");
  LSTR MSG_FAN_SPEED                      = _UxGT("Lüfter");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Lüfter ~");
  LSTR MSG_STORED_FAN_N                   = _UxGT("Gespeich. Lüfter ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Geschw. Extralüfter");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Geschw. Extralüfter ~");
  LSTR MSG_CONTROLLER_FAN                 = _UxGT("Lüfter Kontroller");
  LSTR MSG_CONTROLLER_FAN_IDLE_SPEED      = _UxGT("Lüfter Leerlauf");
  LSTR MSG_CONTROLLER_FAN_AUTO_ON         = _UxGT("Motorlast Modus");
  LSTR MSG_CONTROLLER_FAN_SPEED           = _UxGT("Lüfter Motorlast");
  LSTR MSG_CONTROLLER_FAN_DURATION        = _UxGT("Ausschalt Delay");
  LSTR MSG_FLOW                           = _UxGT("Flussrate");
  LSTR MSG_FLOW_N                         = _UxGT("Flussrate ~");
  LSTR MSG_CONTROL                        = _UxGT("Einstellungen");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Faktor");
  LSTR MSG_AUTOTEMP                       = _UxGT("Auto Temperatur");
  LSTR MSG_LCD_ON                         = _UxGT("an");
  LSTR MSG_LCD_OFF                        = _UxGT("aus");
  LSTR MSG_PID_AUTOTUNE                   = _UxGT("PID Autotune");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("PID Autotune *");
  LSTR MSG_PID_CYCLE                      = _UxGT("PID Zyklus");
  LSTR MSG_PID_AUTOTUNE_DONE              = _UxGT("PID Tuning fertig");
  LSTR MSG_PID_AUTOTUNE_FAILED            = _UxGT("PID Autotune fehlge.!");
  LSTR MSG_BAD_HEATER_ID                  = _UxGT("ungültiger Extruder.");
  LSTR MSG_TEMP_TOO_HIGH                  = _UxGT("Temperatur zu hoch.");
  LSTR MSG_PID_BAD_HEATER_ID              = _UxGT("Autotune fehlge.! Ungültiger Extruder");
  LSTR MSG_PID_TEMP_TOO_HIGH              = _UxGT("Autotune fehlge.! Temperatur zu hoch.");
  LSTR MSG_PID_TIMEOUT                    = _UxGT("Autotune fehlge.! Timeout.");
  LSTR MSG_MPC_MEASURING_AMBIENT          = _UxGT("teste Wärmeverlust");
  LSTR MSG_MPC_AUTOTUNE                   = _UxGT("Autotune MPC");
  LSTR MSG_MPC_EDIT                       = _UxGT("MPC * bearbeiten");
  LSTR MSG_MPC_POWER_E                    = _UxGT("* Leistung");
  LSTR MSG_MPC_BLOCK_HEAT_CAPACITY_E      = _UxGT("Block C *");
  LSTR MSG_SENSOR_RESPONSIVENESS_E        = _UxGT("Sensor res *");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_E       = _UxGT("Ambient h *");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_FAN_E   = _UxGT("Amb. h fan *");
  LSTR MSG_SELECT_E                       = _UxGT("Auswählen *");
  LSTR MSG_ACC                            = _UxGT("Beschleunigung");
  LSTR MSG_JERK                           = _UxGT("Jerk");
  LSTR MSG_VA_JERK                        = _UxGT("Max ") STR_A _UxGT(" Jerk");
  LSTR MSG_VB_JERK                        = _UxGT("Max ") STR_B _UxGT(" Jerk");
  LSTR MSG_VC_JERK                        = _UxGT("Max ") STR_C _UxGT(" Jerk");
  LSTR MSG_VN_JERK                        = _UxGT("Max @ Jerk");
  LSTR MSG_VE_JERK                        = _UxGT("Max ") STR_E _UxGT(" Jerk");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Junction Dev");
  LSTR MSG_MAX_SPEED                      = _UxGT("Max Geschw. (mm/s)");
  LSTR MSG_VMAX_A                         = _UxGT("V max ") STR_A;
  LSTR MSG_VMAX_B                         = _UxGT("V max ") STR_B;
  LSTR MSG_VMAX_C                         = _UxGT("V max ") STR_C;
  LSTR MSG_VMAX_N                         = _UxGT("V max @");
  LSTR MSG_VMAX_E                         = _UxGT("V max ") STR_E;
  LSTR MSG_VMAX_EN                        = _UxGT("V max *");
  LSTR MSG_VMIN                           = _UxGT("V min ");
  LSTR MSG_VTRAV_MIN                      = _UxGT("V min Leerfahrt");
  LSTR MSG_ACCELERATION                   = _UxGT("Beschleunigung");
  LSTR MSG_AMAX_A                         = _UxGT("A max ") STR_A;
  LSTR MSG_AMAX_B                         = _UxGT("A max ") STR_B;
  LSTR MSG_AMAX_C                         = _UxGT("A max ") STR_C;
  LSTR MSG_AMAX_N                         = _UxGT("A max @");
  LSTR MSG_AMAX_E                         = _UxGT("A max ") STR_E;
  LSTR MSG_AMAX_EN                        = _UxGT("A max *");
  LSTR MSG_A_RETRACT                      = _UxGT("A Einzug");
  LSTR MSG_A_TRAVEL                       = _UxGT("A Leerfahrt");
  LSTR MSG_XY_FREQUENCY_LIMIT             = _UxGT("max. Frequenz");
  LSTR MSG_XY_FREQUENCY_FEEDRATE          = _UxGT("min. Vorschub");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Steps/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" Steps/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" Steps/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" Steps/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ Steps/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E Steps/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* Steps/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Temperatur");
  LSTR MSG_MOTION                         = _UxGT("Bewegung");
  LSTR MSG_FILAMENT                       = _UxGT("Filament");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E in mm") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT               = _UxGT("E Limit in mm") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT_E             = _UxGT("E Limit *");
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Filamentdurchmesser");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Filamentdurchmesser *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Entladen mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Laden mm");
  LSTR MSG_ADVANCE_K                      = _UxGT("Vorschubfaktor");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("Vorschubfaktor *");
  LSTR MSG_CONTRAST                       = _UxGT("LCD-Kontrast");
  LSTR MSG_BRIGHTNESS                     = _UxGT("LCD-Helligkeit");
  LSTR MSG_SCREEN_TIMEOUT                 = _UxGT("LCD Timeout (m)");
  LSTR MSG_BRIGHTNESS_OFF                 = _UxGT("LCD ausschalten");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Konfig. speichern");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Konfig. laden");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Standardwerte laden");
  LSTR MSG_INIT_EEPROM                    = _UxGT("Werkseinstellungen");
  LSTR MSG_ERR_EEPROM_CRC                 = _UxGT("EEPROM CRC Fehler");
  LSTR MSG_ERR_EEPROM_SIZE                = _UxGT("EEPROM Größe Fehler");
  LSTR MSG_ERR_EEPROM_VERSION             = _UxGT("EEPROM Version Fehler");
  LSTR MSG_SETTINGS_STORED                = _UxGT("Einstell. gespei.");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("FW Update vom Medium");
  LSTR MSG_RESET_PRINTER                  = _UxGT("Drucker neustarten");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Aktualisieren");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Info");
  LSTR MSG_INFO_MACHINENAME               = _UxGT("Machine Name");
  LSTR MSG_INFO_SIZE                      = _UxGT("Größe");
  LSTR MSG_INFO_FWVERSION                 = _UxGT("Firmware Version");
  LSTR MSG_INFO_BUILD                     = _UxGT("Build Datum");
  LSTR MSG_PREPARE                        = _UxGT("Vorbereitung");
  LSTR MSG_TUNE                           = _UxGT("Justierung");
  LSTR MSG_POWER_MONITOR                  = _UxGT("Power Monitor");
  LSTR MSG_CURRENT                        = _UxGT("Strom");
  LSTR MSG_VOLTAGE                        = _UxGT("Spannung");
  LSTR MSG_POWER                          = _UxGT("Power");
  LSTR MSG_START_PRINT                    = _UxGT("Starte Druck");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("Weiter");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Bestätigen");
  LSTR MSG_BUTTON_STOP                    = _UxGT("Stop");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Drucken");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Reseten");
  LSTR MSG_BUTTON_IGNORE                  = _UxGT("Ignorieren");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("Abbrechen");
  LSTR MSG_BUTTON_CONFIRM                 = _UxGT("Bestätigen");
  LSTR MSG_BUTTON_CONTINUE                = _UxGT("Fortsetzen");
  LSTR MSG_BUTTON_DONE                    = _UxGT("Fertig");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Zurück");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Weiter");
  LSTR MSG_BUTTON_SKIP                    = _UxGT("Überspringen");
  LSTR MSG_BUTTON_INFO                    = _UxGT("Info");
  LSTR MSG_BUTTON_LEVEL                   = _UxGT("Level");
  LSTR MSG_BUTTON_PAUSE                   = _UxGT("Pause");
  LSTR MSG_BUTTON_RESUME                  = _UxGT("Fortsetzen");
  LSTR MSG_BUTTON_ADVANCED                = _UxGT("Erweitert");
  LSTR MSG_BUTTON_SAVE                    = _UxGT("Speichern");
  LSTR MSG_BUTTON_PURGE                   = _UxGT("Reinigen");
  LSTR MSG_PAUSING                        = _UxGT("Pause...");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("SD-Druck pausieren");
  LSTR MSG_ADVANCED_PAUSE                 = _UxGT("Erweiterte Pause");
  LSTR MSG_RESUME_PRINT                   = _UxGT("SD-Druck fortsetzen");
  LSTR MSG_HOST_START_PRINT               = _UxGT("Host-Druck starten");
  LSTR MSG_STOP_PRINT                     = _UxGT("SD-Druck abbrechen");
  LSTR MSG_END_LOOPS                      = _UxGT("Wiederholung beenden");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Objekt drucken");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Objekt abbrechen");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Objekt abbrechen {");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Wiederh. n. Stroma.");
  LSTR MSG_CONTINUE_PRINT_JOB             = _UxGT("Druckauftrag fortset.");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Druck vom Medium");
  LSTR MSG_NO_MEDIA                       = _UxGT("Kein Medium");
  LSTR MSG_DWELL                          = _UxGT("Warten...");
  LSTR MSG_USERWAIT                       = _UxGT("Klick zum Fortsetzen");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Druck pausiert...");
  LSTR MSG_PRINTING                       = _UxGT("Druckt...");
  LSTR MSG_STOPPING                       = _UxGT("Stoppen...");
  LSTR MSG_REMAINING_TIME                 = _UxGT("Verbleiben");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Druck abgebrochen");
  LSTR MSG_PRINT_DONE                     = _UxGT("Druck fertig");
  LSTR MSG_PRINTER_KILLED                 = _UxGT("Drucker killed!");
  LSTR MSG_TURN_OFF                       = _UxGT("Drucker ausschalten");
  LSTR MSG_NO_MOVE                        = _UxGT("Motoren angeschaltet");
  LSTR MSG_KILLED                         = _UxGT("ABGEBROCHEN");
  LSTR MSG_STOPPED                        = _UxGT("ANGEHALTEN");
  LSTR MSG_FWRETRACT                      = _UxGT("Firmware Retract");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Einzug mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Wechs. Einzug mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("V Einzug");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Z-Sprung mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("Wechs. Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Unretract V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("S UnRet V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Autom. Einzug");
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("Einzugslänge");
  LSTR MSG_FILAMENT_SWAP_EXTRA            = _UxGT("Extra Einzug");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("Entladelänge");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Werkzeugwechsel");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Z anheben");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Prime-Geschwin.");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Einzug-Geschwin.");
  LSTR MSG_FILAMENT_PARK_ENABLED          = _UxGT("Kopf parken");
  LSTR MSG_SINGLENOZZLE_UNRETRACT_SPEED   = _UxGT("Rückzugsgeschwindigkeit");
  LSTR MSG_SINGLENOZZLE_FAN_SPEED         = _UxGT("Lüfter Geschwindigkeit");
  LSTR MSG_SINGLENOZZLE_FAN_TIME          = _UxGT("Lüfter Zeit");
  LSTR MSG_TOOL_MIGRATION_ON              = _UxGT("Auto AN");
  LSTR MSG_TOOL_MIGRATION_OFF             = _UxGT("Auto AUS");
  LSTR MSG_TOOL_MIGRATION                 = _UxGT("Werkzeugmigration");
  LSTR MSG_TOOL_MIGRATION_AUTO            = _UxGT("Auto-Migration");
  LSTR MSG_TOOL_MIGRATION_END             = _UxGT("Letzter Extruder");
  LSTR MSG_TOOL_MIGRATION_SWAP            = _UxGT("Migrieren zu *");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Filament wechseln");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Filament wechseln *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Filament laden");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Filament laden *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Filament entladen");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Filament entladen *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Alles entladen");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("SD initial.");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("USB initial.");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("SD initial.");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Medium getauscht"); // SD-card changed by user. For machines with no autocarddetect. Both send "M21"
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Medium freigeben"); // if Marlin gets confused - M22
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Z-Sonde außerhalb");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Korrekturfaktor");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("Selbsttest");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Zurücksetzen");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("Einfahren");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("Ausfahren");
  LSTR MSG_BLTOUCH_SW_MODE                = _UxGT("SW-Modus");
  LSTR MSG_BLTOUCH_SPEED_MODE             = _UxGT("High Speed");
  LSTR MSG_BLTOUCH_5V_MODE                = _UxGT("5V-Modus");
  LSTR MSG_BLTOUCH_OD_MODE                = _UxGT("OD-Modus");
  LSTR MSG_BLTOUCH_MODE_STORE             = _UxGT("Mode-Store");
  LSTR MSG_BLTOUCH_MODE_STORE_5V          = _UxGT("Setze auf 5V");
  LSTR MSG_BLTOUCH_MODE_STORE_OD          = _UxGT("Setze auf OD");
  LSTR MSG_BLTOUCH_MODE_ECHO              = _UxGT("Modus: ");
  LSTR MSG_BLTOUCH_MODE_CHANGE            = _UxGT("ACHTUNG: Falsche Einstellung - kann zu Beschädigung führen! Fortfahren?");
  LSTR MSG_TOUCHMI_PROBE                  = _UxGT("TouchMI");
  LSTR MSG_TOUCHMI_INIT                   = _UxGT("TouchMI initial.");
  LSTR MSG_TOUCHMI_ZTEST                  = _UxGT("Test Z-Versatz");
  LSTR MSG_TOUCHMI_SAVE                   = _UxGT("Speichern");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI          = _UxGT("TouchMI ausfahren");
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("Z-Sonde ausfahren");
  LSTR MSG_MANUAL_STOW                    = _UxGT("Z-Sonde einfahren");
  LSTR MSG_HOME_FIRST                     = _UxGT("Vorher %s homen");
  LSTR MSG_ZPROBE_SETTINGS                = _UxGT("Sondeneinstellungen");
  LSTR MSG_ZPROBE_OFFSETS                 = _UxGT("Sondenversatz");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Sondenversatz X");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Sondenversatz Y");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Sondenversatz Z");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Sondenversatz @");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Z Offset mit Pas");
  LSTR MSG_ZPROBE_MARGIN                  = _UxGT("Sondenrand");
  LSTR MSG_Z_FEED_RATE                    = _UxGT("Z-Vorschub");
  LSTR MSG_ENABLE_HS_MODE                 = _UxGT("HS-Modus aktivieren");
  LSTR MSG_MOVE_NOZZLE_TO_BED             = _UxGT("Bewege Düse zum Bett");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Babystep X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Babystep Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Babystep Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Babystep @");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Total");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Abbr. mit Endstopp");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("HEIZEN ERFOLGLOS");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("REDUND. TEMP-ABWEI.");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = " " LCD_STR_THERMOMETER _UxGT(" NICHT ERREICHT");
  LSTR MSG_ERR_TEMP_MALFUNCTION           = _UxGT("TEMP-FEHLER");
  LSTR MSG_ERR_COOLING_FAILED             = _UxGT("Kühlung fehlgeschla.");
  LSTR MSG_ERR_MAXTEMP                    = " " LCD_STR_THERMOMETER _UxGT(" ÜBERSCHRITTEN");
  LSTR MSG_ERR_MINTEMP                    = " " LCD_STR_THERMOMETER _UxGT(" UNTERSCHRITTEN");
  LSTR MSG_HALTED                         = _UxGT("DRUCKER GESTOPPT");
  LSTR MSG_PLEASE_WAIT                    = _UxGT("Bitte warten...");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Bitte neustarten");
  LSTR MSG_PREHEATING                     = _UxGT("vorheizen...");
  LSTR MSG_HEATING                        = _UxGT("heizt...");
  LSTR MSG_COOLING                        = _UxGT("kühlt...");
  LSTR MSG_BED_HEATING                    = _UxGT("Bett heizt...");
  LSTR MSG_BED_COOLING                    = _UxGT("Bett kühlt...");
  LSTR MSG_PROBE_HEATING                  = _UxGT("Sonde heizt...");
  LSTR MSG_PROBE_COOLING                  = _UxGT("Sonde kühlt...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Gehäuse heizt...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Gehäuse kühlt...");
  LSTR MSG_LASER_COOLING                  = _UxGT("Laser kühlt...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Delta kalibrieren");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Kalibriere X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Kalibriere Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Kalibriere Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Kalibriere Mitte");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Delta Einst. anzeig.");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Autom. Kalibrierung");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Diag Rod");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Höhe");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Radius");
  LSTR MSG_INFO_MENU                      = _UxGT("Über den Drucker");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Drucker-Info");
  LSTR MSG_3POINT_LEVELING                = _UxGT("3-Punkt-Nivellierung");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Lineare Nivellierung");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("Bilineare Nivell.");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Einheit. Bettnivell.");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Netz-Nivellierung");
  LSTR MSG_MESH_DONE                      = _UxGT("Nivellierung fertig");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Drucker-Statistik");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Board-Info");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Thermistoren");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Extruder");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Baudrate");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protokoll");
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Runaway Watch: AUS");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Runaway Watch: AN");
  LSTR MSG_HOTEND_IDLE_TIMEOUT            = _UxGT("Hotend Idle Timeout");
  LSTR MSG_FAN_SPEED_FAULT                = _UxGT("Fehler Lüftergeschw.");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Beleuchtung");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Helligkeit");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Falscher Drucker");

  LSTR MSG_COLORS_GET                     = _UxGT("Farbe");                 // ProUI
  LSTR MSG_COLORS_SELECT                  = _UxGT("Farben auswählen");      // ProUI
  LSTR MSG_COLORS_APPLIED                 = _UxGT("Farben verwenden");      // ProUI
  LSTR MSG_COLORS_RED                     = _UxGT("Rot");                   // ProUI / JyersUI
  LSTR MSG_COLORS_GREEN                   = _UxGT("Grün");                  // ProUI / JyersUI
  LSTR MSG_COLORS_BLUE                    = _UxGT("Blau");                  // ProUI / JyersUI
  LSTR MSG_COLORS_WHITE                   = _UxGT("Weiß");                  // ProUI
  LSTR MSG_UI_LANGUAGE                    = _UxGT("UI Sprache");            // ProUI
  LSTR MSG_SOUND_ENABLE                   = _UxGT("Ton aktivieren");        // ProUI
  LSTR MSG_LOCKSCREEN                     = _UxGT("Bildschirm sperren");    // ProUI
  LSTR MSG_LOCKSCREEN_LOCKED              = _UxGT("Drucker ist gesperrt,"); // ProUI
  LSTR MSG_LOCKSCREEN_UNLOCK              = _UxGT("Scrollen zum Entsper."); // ProUI
  LSTR MSG_PLEASE_WAIT_REBOOT             = _UxGT("Auf Neustart warten");   // ProUI
  LSTR MSG_PLEASE_PREHEAT                 = _UxGT("Bitte vorheizen");       // ProUI

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Drucke");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Komplette");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Gesamte");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("Längste");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Extrud.");
  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Min Temp");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Max Temp");
  LSTR MSG_INFO_PSU                       = _UxGT("Netzteil");

  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Motorleistung");
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ Treiber %");
  LSTR MSG_ERROR_TMC                      = _UxGT("TMC Verbindungsfehler");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("Werte speichern");
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("FILAMENT WECHSEL");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("DRUCK PAUSIERT");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("FILAMENT LADEN");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("FILAMENT ENTLADEN");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("FORTS. OPTIONEN:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Mehr entladen");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Druck weiter");
  LSTR MSG_FILAMENT_CHANGE_PURGE_CONTINUE = _UxGT("Löschen o. fortfah.?");  // ProUI
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Düse: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Runout-Sensor");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Runout-Weg mm");
  LSTR MSG_EXTRUDER_MIN_TEMP              = _UxGT("Extruder Min Temp.");    // ProUI
  LSTR MSG_FANCHECK                       = _UxGT("Lüftergeschw. prüfen");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Homing gescheitert");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Probing gescheitert");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("FILAMENT WÄHLEN");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("Update MMU Firmware!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU handeln erfor.");
  LSTR MSG_MMU2_RESUME                    = _UxGT("Druck fortsetzen");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("Fortfahren...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("Filament laden");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("Lade alle");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("Düse laden");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("Filament auswerfen");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("Filament ~ auswerfen");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("Filament entladen ");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("Lade Fila. %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Fila. auswerfen...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Fila. entladen...");
  LSTR MSG_MMU2_ALL                       = _UxGT("Alle");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("Filament ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("setze MMU zurück");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("MMU zurücksetzen...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("Entfernen, klicken");

  LSTR MSG_MIX                            = _UxGT("Mix");
  LSTR MSG_MIX_COMPONENT_N                = _UxGT("Komponente {");
  LSTR MSG_MIXER                          = _UxGT("Mixer");
  LSTR MSG_GRADIENT                       = _UxGT("Gradient"); // equal Farbverlauf
  LSTR MSG_FULL_GRADIENT                  = _UxGT("Volle Gradient");
  LSTR MSG_TOGGLE_MIX                     = _UxGT("Mix umschalten");
  LSTR MSG_CYCLE_MIX                      = _UxGT("Zyklus Mix");
  LSTR MSG_GRADIENT_MIX                   = _UxGT("Gradient Mix");
  LSTR MSG_REVERSE_GRADIENT               = _UxGT("Umgekehrte Gradient");
  LSTR MSG_ACTIVE_VTOOL                   = _UxGT("Aktives V-Tool");
  LSTR MSG_START_VTOOL                    = _UxGT("V-Tool Start");
  LSTR MSG_END_VTOOL                      = _UxGT("V-Tool Ende");
  LSTR MSG_GRADIENT_ALIAS                 = _UxGT("V-Tool Alias");
  LSTR MSG_RESET_VTOOLS                   = _UxGT("V-Tools Reseten");
  LSTR MSG_COMMIT_VTOOL                   = _UxGT("V-Tool Mix sichern");
  LSTR MSG_VTOOLS_RESET                   = _UxGT("V-Tools ist resetet");
  LSTR MSG_START_Z                        = _UxGT("Z Start:");
  LSTR MSG_END_Z                          = _UxGT("Z Ende:");

  LSTR MSG_GAMES                          = _UxGT("Spiele");
  LSTR MSG_BRICKOUT                       = _UxGT("Brickout");
  LSTR MSG_INVADERS                       = _UxGT("Invaders");
  LSTR MSG_SNAKE                          = _UxGT("Sn4k3");
  LSTR MSG_MAZE                           = _UxGT("Maze");

  LSTR MSG_BAD_PAGE                       = _UxGT("ungült. Seitenzahl");
  LSTR MSG_BAD_PAGE_SPEED                 = _UxGT("ungült. Seitengeschw.");

  LSTR MSG_EDIT_PASSWORD                  = _UxGT("Passwort bearbeiten");
  LSTR MSG_LOGIN_REQUIRED                 = _UxGT("Login erforderlich");
  LSTR MSG_PASSWORD_SETTINGS              = _UxGT("Passwort Einstellungen");
  LSTR MSG_ENTER_DIGIT                    = _UxGT("PIN eingeben");
  LSTR MSG_CHANGE_PASSWORD                = _UxGT("Passwort ändern");
  LSTR MSG_REMOVE_PASSWORD                = _UxGT("Passwort löschen");
  LSTR MSG_PASSWORD_SET                   = _UxGT("Passwort ist ");
  LSTR MSG_START_OVER                     = _UxGT("von vorn beginnen");
  LSTR MSG_REMINDER_SAVE_SETTINGS         = _UxGT("Bald speichern!");
  LSTR MSG_PASSWORD_REMOVED               = _UxGT("Passwort gelöscht");

  //
  // Die Filament-Change-Bildschirme können bis zu 2 Zeilen auf einem 3-Zeilen-Display.
  //
  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_1_LINE("Klick zum Fortsetzen"));
  LSTR MSG_PAUSE_PRINT_PARKING            = _UxGT(MSG_1_LINE("Pausiert..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Bitte warten..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Laden und Klick"));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_1_LINE("Klick zum Heizen"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Heizen..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Entnehmen..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Laden..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("Spülen..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_2_LINE("Klick zum beenden", "der Düsenspülung"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Fortsetzen..."));

  LSTR MSG_TMC_DRIVERS                    = _UxGT("TMC Treiber"); // Max length 18 characters
  LSTR MSG_TMC_CURRENT                    = _UxGT("Treiber Strom");
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("Hybrid threshold");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Sensorloses Homing");
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("Schrittmodus");
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("StealthChop einsch.");
  LSTR MSG_SERVICE_RESET                  = _UxGT("Reset");
  LSTR MSG_SERVICE_IN                     = _UxGT(" im:");
  LSTR MSG_BACKLASH                       = _UxGT("Spiel");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("Korrektur");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("Glätten");

  LSTR MSG_LEVEL_X_AXIS                   = _UxGT("X Achse leveln");
  LSTR MSG_AUTO_CALIBRATE                 = _UxGT("Auto. Kalibiren");
  LSTR MSG_FTDI_HEATER_TIMEOUT            = _UxGT("Idle Timeout, Temperatur gefallen. Drücke Okay, um erneut aufzuheizen und fortzufahren.");
  LSTR MSG_HEATER_TIMEOUT                 = _UxGT("Heizungs Timeout");
  LSTR MSG_REHEAT                         = _UxGT("Erneut aufheizen");
  LSTR MSG_REHEATING                      = _UxGT("Erneut aufhei. ...");
  LSTR MSG_REHEATDONE                     = _UxGT("Aufwärmen fertig");

  LSTR MSG_PROBE_WIZARD                   = _UxGT("Sonden-Assistent");
  LSTR MSG_PROBE_WIZARD_PROBING           = _UxGT("Sonden-Bezug");
  LSTR MSG_PROBE_WIZARD_MOVING            = _UxGT("Bewege zur Position");

  LSTR MSG_XATC                           = _UxGT("X-Verdreh-Assistent");
  LSTR MSG_XATC_DONE                      = _UxGT("X-Verdreh-Assi fertig!");
  LSTR MSG_XATC_UPDATE_Z_OFFSET           = _UxGT("Z-Versatz Sonde akt. auf ");

  LSTR MSG_SOUND                          = _UxGT("Ton");

  LSTR MSG_TOP_LEFT                       = _UxGT("Oben Links");
  LSTR MSG_BOTTOM_LEFT                    = _UxGT("Unten Links");
  LSTR MSG_TOP_RIGHT                      = _UxGT("Oben Rechts");
  LSTR MSG_BOTTOM_RIGHT                   = _UxGT("Unten Rechts");
  LSTR MSG_CALIBRATION_COMPLETED          = _UxGT("Kalibrierung beendet");
  LSTR MSG_CALIBRATION_FAILED             = _UxGT("Kalibrierung geschei.");

  LSTR MSG_DRIVER_BACKWARD                = _UxGT(" Driver zurück");

  LSTR MSG_SD_CARD                        = _UxGT("SD Karte");
  LSTR MSG_USB_DISK                       = _UxGT("USB Disk");

  LSTR MSG_HOST_SHUTDOWN                  = _UxGT("Host abschalten");

  LSTR MSG_SHORT_DAY                      = _UxGT("t"); // One character only
  LSTR MSG_SHORT_HOUR                     = _UxGT("h"); // One character only
  LSTR MSG_SHORT_MINUTE                   = _UxGT("m"); // One character only
}

namespace LanguageWide_de {
  using namespace LanguageNarrow_de;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_MEDIA_NOT_INSERTED           = _UxGT("Kein Medium eingelegt.");      // ProUI
    LSTR MSG_PLEASE_WAIT_REBOOT           = _UxGT("Bitte auf Neustart warten.");  // ProUI
    LSTR MSG_PLEASE_PREHEAT               = _UxGT("Bitte das Hotend vorheizen."); // ProUI
    LSTR MSG_INFO_PRINT_COUNT_RESET       = _UxGT("Druckzähler zurücksetzen");    // ProUI
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Gesamte Drucke");
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Komplette Drucke");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Gesamte Druckzeit");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Längste Druckzeit");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Gesamt Extrudiert");
  #endif
}

namespace LanguageTall_de {
  using namespace LanguageWide_de;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Knopf drücken um", "Druck fortzusetzen"));
    LSTR MSG_PAUSE_PRINT_PARKING          = _UxGT(MSG_2_LINE("Druck ist", "pausiert..."));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Warte auf den", "Start des", "Filamentwechsels..."));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Filament einlegen", "und Knopf drücken", "um fortzusetzen"));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Knopf drücken um", "Düse aufzuheizen"));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Düse heizt auf", "bitte warten..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_3_LINE("Warte auf", "Entnahme", "des Filaments..."));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_3_LINE("Warte auf", "Laden des", "Filaments..."));
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_3_LINE("Warte auf", "Spülung", "der Düse..."));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_3_LINE("Klicke um", "die Düsenspülung", "zu beenden"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_3_LINE("Warte auf", "Fortsetzen des", "Drucks..."));
  #endif
}

namespace Language_de {
  using namespace LanguageTall_de;
}
