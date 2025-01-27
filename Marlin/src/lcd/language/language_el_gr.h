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
 * Greek (Greece)
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */

#define DISPLAY_CHARSET_ISO10646_GREEK

namespace LanguageNarrow_el_gr {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Greek (Greece)");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" έτοιμο.");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Εισαγωγή κάρτας");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Αφαίρεση κάρτας");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Endstops"); // Max length 8 characters
  LSTR MSG_MAIN_MENU                      = _UxGT("Βασική Οθόνη");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Αυτόματη εκκίνηση");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Απενεργοποίηση βηματιστή");
  LSTR MSG_AUTO_HOME                      = _UxGT("Αυτομ. επαναφορά στο αρχικό σημείο");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Αρχικό σημείο X");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Αρχικό σημείο Y");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Αρχικό σημείο Z");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Επαναφορά στο αρχικό σημείο ΧΥZ");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Κάντε κλικ για να ξεκινήσετε");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Επόμενο σημείο");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Ολοκλήρωση επιπεδοποίησης!");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Ορισμός βασικών μετατοπίσεων");
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Εφαρμόστηκαν οι μετατοπίσεις");

  LSTR MSG_PREHEAT_1                      = _UxGT("Προθέρμανση ") PREHEAT_1_LABEL;
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Προθέρμανση ") PREHEAT_1_LABEL " ~";
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Προθέρμανση ") PREHEAT_1_LABEL _UxGT(" End");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Προθέρμανση ") PREHEAT_1_LABEL _UxGT(" End ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Προθέρμανση ") PREHEAT_1_LABEL _UxGT(" όλα");
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Προθέρμανση ") PREHEAT_1_LABEL _UxGT(" κλίνη");
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Προθέρμανση ") PREHEAT_1_LABEL _UxGT(" επιβεβαίωση");

  LSTR MSG_PREHEAT_M                      = _UxGT("Προθέρμανση $");
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Προθέρμανση $ ~");
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Προθέρμανση $ End");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Προθέρμανση $ End ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Προθέρμανση $ όλα");
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Προθέρμανση $ κλίνη");
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Προθέρμανση $ επιβεβαίωση");

  LSTR MSG_COOLDOWN                       = _UxGT("Μειωση θερμοκρασιας");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Ενεργοποίηση");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Απενεργοποίηση");
  LSTR MSG_EXTRUDE                        = _UxGT("Εξώθηση");
  LSTR MSG_RETRACT                        = _UxGT("Ανάσυρση");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Μετακίνηση άξονα");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Ανιχνευτής/ισοπέδωση");
  LSTR MSG_BED_LEVELING                   = _UxGT("Επιπεδοποίηση κλίνης");
  LSTR MSG_LEVEL_BED                      = _UxGT("Επιπεδοποίηση κλίνης");
  LSTR MSG_MOVE_X                         = _UxGT("Μετακίνηση X");
  LSTR MSG_MOVE_Y                         = _UxGT("Μετακίνηση Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Μετακίνηση Z");
  LSTR MSG_MOVE_N                         = _UxGT("Μετακίνηση @");
  LSTR MSG_MOVE_E                         = _UxGT("Εξωθητήρας");
  LSTR MSG_MOVE_EN                        = _UxGT("Εξωθητήρας *");
  LSTR MSG_MOVE_N_MM                      = _UxGT("Μετακίνηση $μμ");
  LSTR MSG_MOVE_N_IN                      = _UxGT("Μετακίνηση $in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Μετακίνηση $") LCD_STR_DEGREE;
  LSTR MSG_SPEED                          = _UxGT("Ταχύτητα");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Κλίνη Z");
  LSTR MSG_NOZZLE                         = _UxGT("Ακροφύσιο");
  LSTR MSG_NOZZLE_N                       = _UxGT("Ακροφύσιο ~");
  LSTR MSG_BED                            = _UxGT("Κλίνη");
  LSTR MSG_FAN_SPEED                      = _UxGT("Ταχύτητα ανεμιστήρα");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Ταχύτητα ανεμιστήρα ~");
  LSTR MSG_FLOW                           = _UxGT("Ροή");
  LSTR MSG_FLOW_N                         = _UxGT("Ροή ~");
  LSTR MSG_CONTROL                        = _UxGT("Έλεγχος");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Fact");
  LSTR MSG_AUTOTEMP                       = _UxGT("Αυτομ. ρύθμιση θερμοκρασίας");
  LSTR MSG_LCD_ON                         = _UxGT("Ενεργοποιημένο");
  LSTR MSG_LCD_OFF                        = _UxGT("Απενεργοποιημένο");
  LSTR MSG_ACC                            = _UxGT("Επιτάχυνση");
  LSTR MSG_JERK                           = _UxGT("Vαντίδραση");
  LSTR MSG_VA_JERK                        = _UxGT("Vαντίδραση ") STR_A;
  LSTR MSG_VB_JERK                        = _UxGT("Vαντίδραση ") STR_B;
  LSTR MSG_VC_JERK                        = _UxGT("Vαντίδραση ") STR_C;
  LSTR MSG_VN_JERK                        = _UxGT("Vαντίδραση @");
  LSTR MSG_VE_JERK                        = _UxGT("Vαντίδραση E");
  LSTR MSG_VMAX_A                         = _UxGT("Vμεγ ") STR_A;
  LSTR MSG_VMAX_B                         = _UxGT("Vμεγ ") STR_B;
  LSTR MSG_VMAX_C                         = _UxGT("Vμεγ ") STR_C;
  LSTR MSG_VMAX_N                         = _UxGT("Vμεγ @");
  LSTR MSG_VMAX_E                         = _UxGT("Vμεγ E");
  LSTR MSG_VMAX_EN                        = _UxGT("Vμεγ *");
  LSTR MSG_VMIN                           = _UxGT("Vελαχ");
  LSTR MSG_VTRAV_MIN                      = _UxGT("Vελάχ. μετατόπιση");
  LSTR MSG_ACCELERATION                   = _UxGT("Accel");
  LSTR MSG_AMAX_A                         = _UxGT("Aμεγ ") STR_A;
  LSTR MSG_AMAX_B                         = _UxGT("Aμεγ ") STR_B;
  LSTR MSG_AMAX_C                         = _UxGT("Aμεγ ") STR_C;
  LSTR MSG_AMAX_N                         = _UxGT("Aμεγ @");
  LSTR MSG_AMAX_E                         = _UxGT("Aμεγ E");
  LSTR MSG_AMAX_EN                        = _UxGT("Aμεγ *");
  LSTR MSG_A_RETRACT                      = _UxGT("Α-ανάσυρση");
  LSTR MSG_A_TRAVEL                       = _UxGT("Α-μετατόπιση");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Bήματα ανά μμ");
  LSTR MSG_A_STEPS                        = _UxGT("Bήματα ") STR_A _UxGT(" ανά μμ");
  LSTR MSG_B_STEPS                        = _UxGT("Bήματα ") STR_B _UxGT(" ανά μμ");
  LSTR MSG_C_STEPS                        = _UxGT("Bήματα ") STR_C _UxGT(" ανά μμ");
  LSTR MSG_N_STEPS                        = _UxGT("Bήματα @ ανά μμ");
  LSTR MSG_E_STEPS                        = _UxGT("Bήματα Ε ανά μμ");
  LSTR MSG_EN_STEPS                       = _UxGT("Bήματα * ανά μμ");
  LSTR MSG_TEMPERATURE                    = _UxGT("Θερμοκρασία");
  LSTR MSG_MOTION                         = _UxGT("Κίνηση");
  LSTR MSG_FILAMENT                       = _UxGT("Νήμα");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("Ε σε μμ") SUPERSCRIPT_THREE;
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Διάμετρος νήματος");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Διάμετρος νήματος *");
  LSTR MSG_CONTRAST                       = _UxGT("Κοντράστ LCD");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Αποθήκευση");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Φόρτωση");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Επαναφορά ασφαλούς αντιγράφου");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Ανανέωση");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Οθόνη πληροφόρησης");
  LSTR MSG_PREPARE                        = _UxGT("Προετοιμασία");
  LSTR MSG_TUNE                           = _UxGT("Συντονισμός");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Παύση εκτύπωσης");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Συνέχιση εκτύπωσης");
  LSTR MSG_STOP_PRINT                     = _UxGT("Διακοπή εκτύπωσης");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Εκτύπωση από SD");
  LSTR MSG_NO_MEDIA                       = _UxGT("Δεν βρέθηκε SD");
  LSTR MSG_DWELL                          = _UxGT("Αναστολή λειτουργίας…");
  LSTR MSG_USERWAIT                       = _UxGT("Αναμονή για χρήστη…");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Διακόπτεται η εκτύπωση");
  LSTR MSG_NO_MOVE                        = _UxGT("Καμία κίνηση.");
  LSTR MSG_KILLED                         = _UxGT("ΤΕΡΜΑΤΙΣΜΟΣ. ");
  LSTR MSG_STOPPED                        = _UxGT("ΔΙΑΚΟΠΗ. ");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Ανάσυρση μμ");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Εναλλαγή ανάσυρσης μμ");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Ανάσυρση V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Μεταπήδηση μμ");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("S Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Unretract V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Αυτόματη ανάσυρση");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Αλλαγή νήματος");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Αλλαγή νήματος *");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Προετοιμασία SD");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Προετοιμασία USB");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Προετοιμασία SD");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Αλλαγή κάρτας SD");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Διερεύνηση Z εκτός κλίνης");
  LSTR MSG_YX_UNHOMED                     = _UxGT("Επαναφορά Χ/Υ πριν από Z");
  LSTR MSG_XYZ_UNHOMED                    = _UxGT("Επαναφορά ΧΥZ πρώτα");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Μετατόπιση X");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Μετατόπιση Y");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Μετατόπιση Z");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Μετατόπιση @");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Βήμα Z ρύθμιση");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Μικρό βήμα Χ");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Μικρό βήμα Υ");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Μικρό βήμα Z");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Ματαίωση endstop ");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Ανεπιτυχής θέρμανση");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Λάθος: ΠΛΕΟΝΑZΟΥΣΑ ΘΕΡΜΟΤΗΤΑ");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("ΔΙΑΦΥΓΗ ΘΕΡΜΟΤΗΤΑΣ");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Λάθος: ΜΕΓΙΣΤΗ ΘΕΡΜΟΤΗΤΑ");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Λάθος: ΕΛΑΧΙΣΤΗ ΘΕΡΜΟΤΗΤΑ");
  LSTR MSG_HEATING                        = _UxGT("Θερμαίνεται…");
  LSTR MSG_BED_HEATING                    = _UxGT("Θέρμανση κλίνης…");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Βαθμονόμηση Delta");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Βαθμονόμηση X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Βαθμονόμηση Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Βαθμονόμηση Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Βαθμονόμηση κέντρου");

  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("Εσφαλμένος εκτυπωτής");
}

namespace LanguageWide_el_gr {
  using namespace LanguageNarrow_el_gr;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
  #endif
}

namespace LanguageTall_el_gr {
  using namespace LanguageWide_el_gr;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
  #endif
}

namespace Language_el_gr {
  using namespace LanguageTall_el_gr;
}
