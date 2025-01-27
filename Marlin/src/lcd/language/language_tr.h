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
 * Turkish
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 *
 * Substitutions are applied for the following characters when used in menu items titles:
 *
 *   $ displays an inserted string
 *   { displays  '0'....'10' for indexes 0 - 10
 *   ~ displays  '1'....'11' for indexes 0 - 10
 *   * displays 'E1'...'E11' for indexes 0 - 10 (By default. Uses LCD_FIRST_TOOL)
 *   @ displays an axis name such as XYZUVW, or E for an extruder
 */

#define DISPLAY_CHARSET_ISO10646_TR

namespace LanguageNarrow_tr {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Turkish");

  // These strings should be translated
  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" Hazır.");
  LSTR MSG_YES                            = _UxGT("EVET");
  LSTR MSG_NO                             = _UxGT("HAYIR");
  LSTR MSG_HIGH                           = _UxGT("YÜKSEK");
  LSTR MSG_LOW                            = _UxGT("DÜŞÜK");
  LSTR MSG_BACK                           = _UxGT("Geri");
  LSTR MSG_ERROR                          = _UxGT("Hata");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Durduruluyor...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("SD K. Yerleştirildi.");
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("SD Kart Çıkarıldı.");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("SD Kart Bekleniyor");
  LSTR MSG_MEDIA_INIT_FAIL                = _UxGT("SD K. Başlatma Hatası");
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Kart Okuma Hatası");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("USB Çıkarıldı");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("USB Başlat. Hatası");
  LSTR MSG_MEDIA_SORT                     = _UxGT("Medyayı Sırala");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("Medyayı Güncelle");
  LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("Subcall Overflow");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Endstops"); // Max length 8 characters
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Soft Endstops");
  LSTR MSG_MAIN_MENU                      = _UxGT("Ana");
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Gelişmiş Ayarlar");
  LSTR MSG_CONFIGURATION                  = _UxGT("Yapılandırma");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Oto. Başlat");
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Motorları Durdur");
  LSTR MSG_DEBUG_MENU                     = _UxGT("Hata Ayıklama");
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Durum Çubuğu Testi");
  LSTR MSG_ENDSTOP_TEST                   = _UxGT("Endstop Test");
  LSTR MSG_Z_PROBE                        = _UxGT("Z Probe");
  LSTR MSG_HOMING                         = _UxGT("Sıfırlanıyor");
  LSTR MSG_AUTO_HOME                      = _UxGT("Eksenleri Sıfırla");
  LSTR MSG_AUTO_HOME_N                    = _UxGT("@ Sıfırla");
  LSTR MSG_AUTO_HOME_X                    = _UxGT("X Sıfırla");
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Y Sıfırla");
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Z Sıfırla");
  LSTR MSG_Z_AFTER_HOME                   = _UxGT("Z After Homing");
  LSTR MSG_FILAMENT_SET                   = _UxGT("Filament Ayarları");
  LSTR MSG_FILAMENT_MAN                   = _UxGT("Filament Yönetimi");
  LSTR MSG_MANUAL_LEVELING                = _UxGT("Manuel Seviyeleme");
  LSTR MSG_TRAM_FL                        = _UxGT("Sol Ön");
  LSTR MSG_TRAM_FR                        = _UxGT("Ön Sağ");
  LSTR MSG_TRAM_C                         = _UxGT("Orta");
  LSTR MSG_TRAM_BL                        = _UxGT("Arka Sol");
  LSTR MSG_TRAM_BR                        = _UxGT("Arka Sağ");
  LSTR MSG_MANUAL_MESH                    = _UxGT("Manuel Mesh");
  LSTR MSG_AUTO_MESH                      = _UxGT("Oto Mesh Oluştur");
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Oto. Z-Hizalama");
  LSTR MSG_ITERATION                      = _UxGT("G34 Yinelemesi: %i");
  LSTR MSG_DECREASING_ACCURACY            = _UxGT("Doğruluk Düşüyor!");
  LSTR MSG_ACCURACY_ACHIEVED              = _UxGT("Doğruluk Sağlandı");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("XYZ Sıfırlanıyor");
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Başlatmak için tıkla");
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Sonraki Nokta");
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("Seviyeleme Tamam!");
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Kaçınma Yüksekliği");
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Ofset Ayarla");
  LSTR MSG_HOME_OFFSET_X                  = _UxGT("X Ofset Sıfırla"); // DWIN
  LSTR MSG_HOME_OFFSET_Y                  = _UxGT("Y Ofset Sıfırla"); // DWIN
  LSTR MSG_HOME_OFFSET_Z                  = _UxGT("Z Ofset Sıfırla"); // DWIN
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Ofset Tamam");
  LSTR MSG_ERR_M428_TOO_FAR               = _UxGT("MIN/MAX Çok Fazla");
  LSTR MSG_TRAMMING_WIZARD                = _UxGT("Hizalama Sihirbazı");
  LSTR MSG_SELECT_ORIGIN                  = _UxGT("Başlangıç Seç");
  LSTR MSG_LAST_VALUE_SP                  = _UxGT("Son değer ");

  LSTR MSG_PREHEAT_1                      = _UxGT("ÖnIsıtma ") PREHEAT_1_LABEL;                   // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_1_H                    = _UxGT("ÖnIsıtma ") PREHEAT_1_LABEL " ~";              // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_1_END                  = _UxGT("ÖnIsıtma ") PREHEAT_1_LABEL _UxGT(" Nozul");   // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("ÖnIsıtma ") PREHEAT_1_LABEL _UxGT(" Nozul ~"); // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("ÖnIsıtma ") PREHEAT_1_LABEL _UxGT(" Tüm");     // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("ÖnIsıtma ") PREHEAT_1_LABEL _UxGT(" Tabla");   // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("ÖnIsıtma ") PREHEAT_1_LABEL _UxGT(" Ayar");    // Çalışma gerekli Hata

  LSTR MSG_PREHEAT_M                      = _UxGT("ÖnIsıtma $");         // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_M_H                    = _UxGT("ÖnIsıtma $ ~");       // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_M_END                  = _UxGT("ÖnIsıtma $ Nozul");   // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("ÖnIsıtma $ Nozul ~"); // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("ÖnIsıtma $ Tüm");     // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("ÖnIsıtma $ Tabla");   // Çalışma gerekli Hata
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("ÖnIsıtma $ Ayar");    // Çalışma gerekli Hata

  LSTR MSG_PREHEAT_HOTEND                 = _UxGT("Nozul Ön Isıtma");
  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Özel Ön Isıtma");
  LSTR MSG_COOLDOWN                       = _UxGT("Soğut/(Durdur)");

  LSTR MSG_CUTTER_FREQUENCY               = _UxGT("Frekans");
  LSTR MSG_LASER_MENU                     = _UxGT("Lazer Kontrolü");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("Spindle Kontrolü");
  LSTR MSG_LASER_POWER                    = _UxGT("Lazer Gücü");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("Spindle Gücü");
  LSTR MSG_LASER_TOGGLE                   = _UxGT("Lazer Değiştir");
  LSTR MSG_LASER_EVAC_TOGGLE              = _UxGT("Üfleyici Değiştir");
  LSTR MSG_LASER_ASSIST_TOGGLE            = _UxGT("Hava Desteği");
  LSTR MSG_LASER_PULSE_MS                 = _UxGT("Test Darbesi ms");
  LSTR MSG_LASER_FIRE_PULSE               = _UxGT("Atış Darbe");
  LSTR MSG_FLOWMETER_FAULT                = _UxGT("Soğutucu Akış Hatası");
  LSTR MSG_SPINDLE_TOGGLE                 = _UxGT("Spindle Değiştir");
  LSTR MSG_SPINDLE_EVAC_TOGGLE            = _UxGT("Vakum Değiştir");
  LSTR MSG_SPINDLE_FORWARD                = _UxGT("Spindle İleri Yön");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("Spindle Geri Yön");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Gücü Aç");
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Gücü Kapat");
  LSTR MSG_EXTRUDE                        = _UxGT("Ekstrüzyon");
  LSTR MSG_RETRACT                        = _UxGT("Geri Çek");
  LSTR MSG_MOVE_AXIS                      = _UxGT("Eksen Hareketleri");
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Prob ve Seviyeleme");
  LSTR MSG_BED_LEVELING                   = _UxGT("Tabla Seviyeleme");
  LSTR MSG_LEVEL_BED                      = _UxGT("Tablayı Seviyele");
  LSTR MSG_BED_TRAMMING                   = _UxGT("Tabla Hizalama");
  LSTR MSG_BED_TRAMMING_MANUAL            = _UxGT("Manuel Hizalama");
  LSTR MSG_BED_TRAMMING_RAISE             = _UxGT("Prob tetiklenene kadar tablayı ayarlayın.");
  LSTR MSG_BED_TRAMMING_IN_RANGE          = _UxGT("Köşeler tolerans dahilinde. Tabla hizalandı.");
  LSTR MSG_BED_TRAMMING_GOOD_POINTS       = _UxGT("İyi Noktalar: ");
  LSTR MSG_BED_TRAMMING_LAST_Z            = _UxGT("Son Z: ");
  LSTR MSG_NEXT_CORNER                    = _UxGT("Sonraki Köşe");
  LSTR MSG_MESH_EDITOR                    = _UxGT("Mesh Editörü");
  LSTR MSG_MESH_VIEWER                    = _UxGT("Mesh görüntüleyici");
  LSTR MSG_EDIT_MESH                      = _UxGT("Mesh Düzenle");
  LSTR MSG_MESH_VIEW                      = _UxGT("Mesh Göster");
  LSTR MSG_EDITING_STOPPED                = _UxGT("Mesh Düzenleme Durdu");
  LSTR MSG_NO_VALID_MESH                  = _UxGT("Geçerli Mesh yok");
  LSTR MSG_ACTIVATE_MESH                  = _UxGT("Seviyeleme Aç");
  LSTR MSG_PROBING_POINT                  = _UxGT("Prop Noktası");
  LSTR MSG_MESH_X                         = _UxGT("İndeks X");
  LSTR MSG_MESH_Y                         = _UxGT("İndeks Y");
  LSTR MSG_MESH_INSET                     = _UxGT("Mesh Inset");
  LSTR MSG_MESH_MIN_X                     = _UxGT("Mesh X Minimum");
  LSTR MSG_MESH_MAX_X                     = _UxGT("Mesh X Maximum");
  LSTR MSG_MESH_MIN_Y                     = _UxGT("Mesh Y Minimum");
  LSTR MSG_MESH_MAX_Y                     = _UxGT("Mesh Y Maximum");
  LSTR MSG_MESH_AMAX                      = _UxGT("Alanı Büyüt");
  LSTR MSG_MESH_CENTER                    = _UxGT("Merkez Alan");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Z Değeri");
  LSTR MSG_MESH_CANCEL                    = _UxGT("Mesh iptal edildi");
  LSTR MSG_MESH_RESET                     = _UxGT("Mesh reset");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Özel Komutlar");
  LSTR MSG_M48_TEST                       = _UxGT("M48 Prob Testi");
  LSTR MSG_M48_POINT                      = _UxGT("M48 Nokta");
  LSTR MSG_M48_OUT_OF_BOUNDS              = _UxGT("Sınır dışı araştırma");
  LSTR MSG_M48_DEVIATION                  = _UxGT("Sapma");
  LSTR MSG_IDEX_MENU                      = _UxGT("IDEX Modu");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("Takım Ofsetleri");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Oto-Park");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Kopyala");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("Yansıtılmış kopya");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Tam Kontrol");
  LSTR MSG_IDEX_DUPE_GAP                  = _UxGT("Yinelenen X-Boşluğu");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("2. nozul Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("2. nozul @");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("G29 Çalışıyor");
  LSTR MSG_UBL_TOOLS                      = _UxGT("UBL Araçları");
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("Eğim Noktası");
  LSTR MSG_UBL_TILT_MESH                  = _UxGT("Eğimli Mesh");
  LSTR MSG_UBL_TILTING_GRID               = _UxGT("Eğimli Izgara Boyutu");
  LSTR MSG_UBL_MESH_TILTED                = _UxGT("Mesh Eğildi");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Elle Mesh Oluştur");
  LSTR MSG_UBL_MESH_WIZARD                = _UxGT("UBL Mesh Sihirbazı");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Altlık & Ölçü Ver");
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Ölçüm");
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Tablayı Kaldır & Ölç");
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Sonrakine Git");
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Tabla Sıcaklığı");
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Tabla Sıcaklığı");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Nozul Sıcaklığı");
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Nozul Sıcaklığı");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Özel Mesh Düzenleme");
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("İnce Ayar Mesh");
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Mesh Düzenleme Tamam");
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Özel Mesh Oluştur");
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Mesh Oluştur");
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("Mesh Oluştur ($)");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Soğuk Mesh Oluştur");
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Mesh Yükseklik Ayarı");
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Yükseklik miktarı");
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Mesh Doğrulama");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Mesh Doğrulama ($)");
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Özel Mesh Doğrulama");
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26 Isıtma Tablası");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26 Isıtma Memesi");
  LSTR MSG_G26_MANUAL_PRIME               = _UxGT("Manuel çalışma...");
  LSTR MSG_G26_FIXED_LENGTH               = _UxGT("Birincil Sabit Uzunluk");
  LSTR MSG_G26_PRIME_DONE                 = _UxGT("Çalışma Tamamlandı");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26 İptal edildi");
  LSTR MSG_G26_LEAVING                    = _UxGT("Çıkış G26");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Tabla Mesh Devam et");
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Mesh Seviyeleme");
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("3-Nokta Seviyeleme");
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Kafes Mesh Seviyeleme");
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Mesh Seviyesi");
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Yan Noktalar");
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Haritalama Türü");
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Mesh Çikis Haritası"); // Çalışma gerekli Hata
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Host için Çıktı");
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("CSV için Çıktı");
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Yedekleme Kapalı");
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("UBL Çıkış Bilgisi");
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Dolgu Miktarı");
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Manuel Dolgu");
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Akıllı Dolgu");
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Mesh Dolgu");
  LSTR MSG_UBL_MESH_FILLED                = _UxGT("Eksikler Dolduruldu");
  LSTR MSG_UBL_MESH_INVALID               = _UxGT("Geçersiz Mesh");
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Tumunu Gecersiz Kil"); // Çalışma gerekli Hata
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Yakini Gecersiz Kil"); // Çalışma gerekli Hata
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Tumunu İnce Ayarla");  // Çalışma gerekli Hata
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Yakını İnce Ayarla");
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Mesh Depolama");
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Bellek Yuvası");
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Tabla Mesh Yükle");
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Tabla Mesh Kaydet");
  LSTR MSG_UBL_INVALID_SLOT               = _UxGT("Önce Mesh Yuvası Seç");
  LSTR MSG_MESH_LOADED                    = _UxGT("Mesh %i yüklendi");
  LSTR MSG_MESH_SAVED                     = _UxGT("Mesh %i kayıtlandı");
  LSTR MSG_MESH_ACTIVE                    = _UxGT("%i mesh aktif");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("Depolama Yok");
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("UBL Kaydetme Hatası");
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("UBL Geri Yükleme Hatası");
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Z-Ofset: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Z-Ofset Durduruldu");
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("Adım Adım UBL");
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1.Soğuk Mesh Oluştur");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2.Akıllı Dolgu");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3.Mesh Dogrulama");      // Çalışma gerekli Hata
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4.Tumunu ince Ayarla");  // Çalışma gerekli Hata
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5.Mesh Dogrulama");      // Çalışma gerekli Hata
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6.Tumunu ince Ayarla");  // Çalışma gerekli Hata
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7.Tabla Mesh Kaydet");

  LSTR MSG_LED_CONTROL                    = _UxGT("LED Kontrolü");
  LSTR MSG_LIGHTS                         = _UxGT("LEDler");
  LSTR MSG_LIGHT_N                        = _UxGT("Işıklar #{");
  LSTR MSG_LED_PRESETS                    = _UxGT("LED Hazır Ayarları");
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Kırmızı");
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Turuncu");
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Sarı");
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Yeşil");
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Mavi");
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Lacivert");
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Menekşe");
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Beyaz");
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Varsayılan");
  LSTR MSG_LED_CHANNEL_N                  = _UxGT("Kanal {");
  LSTR MSG_NEO2_PRESETS                   = _UxGT("Işık #2 Ön Ayarları");
  LSTR MSG_NEO2_BRIGHTNESS                = _UxGT("Parlaklık");
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Özel Işıklar");
  LSTR MSG_INTENSITY_R                    = _UxGT("Kırmızı Şiddeti");
  LSTR MSG_INTENSITY_G                    = _UxGT("Yeşil Şiddeti");
  LSTR MSG_INTENSITY_B                    = _UxGT("Mavi Şiddeti");
  LSTR MSG_INTENSITY_W                    = _UxGT("Beyaz Şiddeti");
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("Parlaklık");

  LSTR MSG_MOVING                         = _UxGT("Hareket Ediyor..");
  LSTR MSG_FREE_XY                        = _UxGT("Durdur XY");
  LSTR MSG_MOVE_X                         = _UxGT("X Hareketi"); // Used by draw_edit_screen
  LSTR MSG_MOVE_Y                         = _UxGT("Y Hareketi");
  LSTR MSG_MOVE_Z                         = _UxGT("Z Hareketi");
  LSTR MSG_MOVE_N                         = _UxGT("@ Hareketi");
  LSTR MSG_MOVE_E                         = _UxGT("Ekstruder");
  LSTR MSG_MOVE_EN                        = _UxGT("Ekstruder *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Nozul Çok Soğuk");
  LSTR MSG_MOVE_N_MM                      = _UxGT("$mm");
  LSTR MSG_MOVE_N_IN                      = _UxGT("$in");
  LSTR MSG_MOVE_N_DEG                     = _UxGT("$") LCD_STR_DEGREE;
  LSTR MSG_LIVE_MOVE                      = _UxGT("Canlı Hareket");
  LSTR MSG_SPEED                          = _UxGT("Hız");
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Z Mesafesi");
  LSTR MSG_NOZZLE                         = _UxGT("Nozul");
  LSTR MSG_NOZZLE_N                       = _UxGT("Nozul ~");
  LSTR MSG_NOZZLE_PARKED                  = _UxGT("Nozul Park Edildi");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("Nozul Beklemede");
  LSTR MSG_BED                            = _UxGT("Tabla");
  LSTR MSG_CHAMBER                        = _UxGT("Kabin");
  LSTR MSG_COOLER                         = _UxGT("Lazer Soğutucu");
  LSTR MSG_COOLER_TOGGLE                  = _UxGT("Soğutucuyu Değiştir");
  LSTR MSG_FLOWMETER_SAFETY               = _UxGT("Akış Güvenliği");
  LSTR MSG_CUTTER                         = _UxGT("Kesici");
  LSTR MSG_LASER                          = _UxGT("Lazer");
  LSTR MSG_FAN_SPEED                      = _UxGT("Fan Hızı");
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Fan Hızı ~");
  LSTR MSG_STORED_FAN_N                   = _UxGT("Depolanan Fan ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Ekstra Fan Hızı");
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Ekstra Fan Hızı ~");
  LSTR MSG_CONTROLLER_FAN                 = _UxGT("Sistem Fan Kontrol");
  LSTR MSG_CONTROLLER_FAN_IDLE_SPEED      = _UxGT("Rölanti Hızı");
  LSTR MSG_CONTROLLER_FAN_AUTO_ON         = _UxGT("Otomatik Mod");
  LSTR MSG_CONTROLLER_FAN_SPEED           = _UxGT("Aktif Hız");
  LSTR MSG_CONTROLLER_FAN_DURATION        = _UxGT("Rölanti Periyodu");
  LSTR MSG_FLOW                           = _UxGT("Akış");
  LSTR MSG_FLOW_N                         = _UxGT("Akış ~");
  LSTR MSG_CONTROL                        = _UxGT("Kontrol");
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Min");
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Max");
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Çarpan");
  LSTR MSG_AUTOTEMP                       = _UxGT("Oto. Sıcaklık");
  LSTR MSG_TIMEOUT                        = _UxGT("Zaman aşımı");
  LSTR MSG_LCD_ON                         = _UxGT("Açık");
  LSTR MSG_LCD_OFF                        = _UxGT("Kapalı");
  LSTR MSG_PID_AUTOTUNE                   = _UxGT("PID Kalibrasyonu");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("* PID Kalibrasyonu");
  LSTR MSG_PID_CYCLE                      = _UxGT("PID Döngüleri");
  LSTR MSG_PID_AUTOTUNE_DONE              = _UxGT("PID ayarı yapıldı");
  LSTR MSG_PID_AUTOTUNE_FAILED            = _UxGT("Kalibrasyon başarısız!");
  LSTR MSG_BAD_HEATER_ID                  = _UxGT("Kötü ekstruder.");
  LSTR MSG_TEMP_TOO_HIGH                  = _UxGT("Sıcaklık çok yüksek.");
  LSTR MSG_TEMP_TOO_LOW                   = _UxGT("Sıcaklık çok düşük");
  LSTR MSG_PID_BAD_HEATER_ID              = _UxGT("Kalibrasyon başarısız! Kötü ekstruder.");
  LSTR MSG_PID_TEMP_TOO_HIGH              = _UxGT("Kalibrasyon başarısız! Sıcaklık çok yüksek.");
  LSTR MSG_PID_TIMEOUT                    = _UxGT("Kalibrasyon başarısız! Zaman aşımı.");
  LSTR MSG_MPC_MEASURING_AMBIENT          = _UxGT("Isı kaybı testi");
  LSTR MSG_MPC_HEATING_PAST_200           = _UxGT(">200C Isıtma");
  LSTR MSG_MPC_COOLING_TO_AMBIENT         = _UxGT("Ortam ısısına soğutma");
  LSTR MSG_MPC_AUTOTUNE                   = _UxGT("MPC Kalibrasyonu");
  LSTR MSG_MPC_EDIT                       = _UxGT("* MPC Düzenle");
  LSTR MSG_MPC_POWER                      = _UxGT("Isıtıcı Gücü");
  LSTR MSG_MPC_POWER_E                    = _UxGT("* Gücü");
  LSTR MSG_MPC_BLOCK_HEAT_CAPACITY        = _UxGT("Isı Kapasitesi");
  LSTR MSG_MPC_BLOCK_HEAT_CAPACITY_E      = _UxGT("Isı Kap. *");
  LSTR MSG_SENSOR_RESPONSIVENESS          = _UxGT("Sensör Hassasiyeti");
  LSTR MSG_SENSOR_RESPONSIVENESS_E        = _UxGT("Sensör Has. *");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF         = _UxGT("Ortam Kats.");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_E       = _UxGT("Ortam Ka. *");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_FAN     = _UxGT("Fan kats.");
  LSTR MSG_MPC_AMBIENT_XFER_COEFF_FAN_E   = _UxGT("Fan kats. *");
  LSTR MSG_SELECT_E                       = _UxGT("Seç *");
  LSTR MSG_ACC                            = _UxGT("İvme");
  LSTR MSG_JERK                           = _UxGT("Sarsım");
  LSTR MSG_VA_JERK                        = _UxGT("Max ") STR_A _UxGT(" Sarsımı");
  LSTR MSG_VB_JERK                        = _UxGT("Max ") STR_B _UxGT(" Sarsımı");
  LSTR MSG_VC_JERK                        = _UxGT("Max ") STR_C _UxGT(" Sarsımı");
  LSTR MSG_VN_JERK                        = _UxGT("Max @ Sarsımı");
  LSTR MSG_VE_JERK                        = _UxGT("Max E Sarsımı");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Jonksiyon Sapması");
  LSTR MSG_STEP_SMOOTHING                 = _UxGT("Adım Yumuşatma");
  LSTR MSG_MAX_SPEED                      = _UxGT("Max Hız (mm/s)");
  LSTR MSG_VMAX_A                         = _UxGT("Max ") STR_A _UxGT(" Hızı");
  LSTR MSG_VMAX_B                         = _UxGT("Max ") STR_B _UxGT(" Hızı");
  LSTR MSG_VMAX_C                         = _UxGT("Max ") STR_C _UxGT(" Hızı");
  LSTR MSG_VMAX_N                         = _UxGT("Max @ Hızı");
  LSTR MSG_VMAX_E                         = _UxGT("Max E Hızı");
  LSTR MSG_VMAX_EN                        = _UxGT("Max * Hızı");
  LSTR MSG_VMIN                           = _UxGT("Min Hız");
  LSTR MSG_VTRAV_MIN                      = _UxGT("Min Gezme Hızı");
  LSTR MSG_ACCELERATION                   = _UxGT("İvme");
  LSTR MSG_AMAX_A                         = _UxGT("Max ") STR_A _UxGT(" İvmesi");
  LSTR MSG_AMAX_B                         = _UxGT("Max ") STR_B _UxGT(" İvmesi");
  LSTR MSG_AMAX_C                         = _UxGT("Max ") STR_C _UxGT(" İvmesi");
  LSTR MSG_AMAX_N                         = _UxGT("Max @ İvmesi");
  LSTR MSG_AMAX_E                         = _UxGT("Max E İvmesi");
  LSTR MSG_AMAX_EN                        = _UxGT("Max * İvmesi");
  LSTR MSG_A_RETRACT                      = _UxGT("G.Çekme Hızı");
  LSTR MSG_A_TRAVEL                       = _UxGT("Gezinme Hızı");
  LSTR MSG_INPUT_SHAPING                  = _UxGT("Input Shaping");
  LSTR MSG_SHAPING_ENABLE_N               = _UxGT("@ Biçimlemeyi Aç");
  LSTR MSG_SHAPING_DISABLE_N              = _UxGT("@ Biçimlemeyi Kapat");
  LSTR MSG_SHAPING_FREQ_N                 = _UxGT("@ frekans");
  LSTR MSG_SHAPING_ZETA_N                 = _UxGT("@ sönümleme");
  LSTR MSG_SHAPING_A_FREQ                 = STR_A _UxGT(" frekansı");   // ProUI
  LSTR MSG_SHAPING_B_FREQ                 = STR_B _UxGT(" frekansı");   // ProUI
  LSTR MSG_SHAPING_C_FREQ                 = STR_C _UxGT(" frekansı");   // ProUI
  LSTR MSG_SHAPING_A_ZETA                 = STR_A _UxGT(" sönümleme");  // ProUI
  LSTR MSG_SHAPING_B_ZETA                 = STR_B _UxGT(" sönümleme");  // ProUI
  LSTR MSG_SHAPING_C_ZETA                 = STR_C _UxGT(" sönümleme");  // ProUI
  LSTR MSG_XY_FREQUENCY_LIMIT             = _UxGT("XY Frek. Sınırı");
  LSTR MSG_XY_FREQUENCY_FEEDRATE          = _UxGT("Min FR Factörü");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Adım/mm");
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" adım/mm");
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" adım/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" adım/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ adım/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E adım/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("* adım/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Sıcaklık");
  LSTR MSG_MOTION                         = _UxGT("Hareket");
  LSTR MSG_FILAMENT                       = _UxGT("Filaman");
  LSTR MSG_FILAMENT_EN                    = _UxGT("Filaman *");
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("Ekstrüzyon/mm") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT               = _UxGT("E Limit in mm") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT_E             = _UxGT("E Limit *");
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Filaman Çapı");
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Filaman Çapı *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Çıkart mm");
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Yükle mm");
  LSTR MSG_SEGMENTS_PER_SECOND            = _UxGT("Segment/Sn");
  LSTR MSG_DRAW_MIN_X                     = _UxGT("Min X Çiz");
  LSTR MSG_DRAW_MAX_X                     = _UxGT("Max X Çiz");
  LSTR MSG_DRAW_MIN_Y                     = _UxGT("Min Y Çİz");
  LSTR MSG_DRAW_MAX_Y                     = _UxGT("Max Y Çiz");
  LSTR MSG_MAX_BELT_LEN                   = _UxGT("Max Kayış Boyu");
  LSTR MSG_LINEAR_ADVANCE                 = _UxGT("Doğrusal İlerleme");
  LSTR MSG_ADVANCE_K                      = _UxGT("K. Değeri");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("K. Değeri *");
  LSTR MSG_CONTRAST                       = _UxGT("LCD Kontrast");
  LSTR MSG_BRIGHTNESS                     = _UxGT("LCD Parlaklığı");
  LSTR MSG_SCREEN_TIMEOUT                 = _UxGT("LCD Zaman Aşımı (m)");
  LSTR MSG_HOTEND_TEMP_GRAPH              = _UxGT("Nozul Isı Grafiği");
  LSTR MSG_BED_TEMP_GRAPH                 = _UxGT("Tablo Isı Grafiği");
  LSTR MSG_BRIGHTNESS_OFF                 = _UxGT("Arka Işık Kapalı");
  LSTR MSG_STORE_EEPROM                   = _UxGT("Hafızaya Al");
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Hafızadan Yükle");
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Fabrika Ayarları");
  LSTR MSG_INIT_EEPROM                    = _UxGT("EEPROM'u başlat");
  LSTR MSG_ERR_EEPROM_CRC                 = _UxGT("Hata: EEPROM CRC");
  LSTR MSG_ERR_EEPROM_SIZE                = _UxGT("Hata: EEPROM Boyutu");
  LSTR MSG_ERR_EEPROM_VERSION             = _UxGT("Hata: EEPROM Sürümü");
  LSTR MSG_ERR_EEPROM_CORRUPT             = _UxGT("Hata: EEPROM Bozuk");
  LSTR MSG_SETTINGS_STORED                = _UxGT("Ayarlar Kaydedildi");
  LSTR MSG_HAS_PREVIEW                    = _UxGT("Önizleme var");
  LSTR MSG_RESET_PRINTER                  = _UxGT("Yazıcıyı Resetle");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("Yenile");
  LSTR MSG_INFO_SCREEN                    = _UxGT("Bilgi Ekranı");
  LSTR MSG_INFO_MACHINENAME               = _UxGT("Makine Adı");
  LSTR MSG_INFO_SIZE                      = _UxGT("Boyut");
  LSTR MSG_INFO_FWVERSION                 = _UxGT("Yazılım Sürümü");
  LSTR MSG_INFO_BUILD                     = _UxGT("Tarih & Saat Oluştur");
  LSTR MSG_PREPARE                        = _UxGT("Hazırlık");
  LSTR MSG_TUNE                           = _UxGT("Ayar");
  LSTR MSG_POWER_MONITOR                  = _UxGT("Güç monitörü");
  LSTR MSG_CURRENT                        = _UxGT("Akım");
  LSTR MSG_VOLTAGE                        = _UxGT("Voltaj");
  LSTR MSG_POWER                          = _UxGT("Güç");
  LSTR MSG_START_PRINT                    = _UxGT("Yazdırmayı Başlat");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("İleri");
  LSTR MSG_BUTTON_INIT                    = _UxGT("Başlat");
  LSTR MSG_BUTTON_STOP                    = _UxGT("Durdur");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("Yazdır");
  LSTR MSG_BUTTON_RESET                   = _UxGT("Resetle");
  LSTR MSG_BUTTON_IGNORE                  = _UxGT("Yoksay");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("İptal");
  LSTR MSG_BUTTON_CONFIRM                 = _UxGT("Onayla");
  LSTR MSG_BUTTON_CONTINUE                = _UxGT("Sürdür");
  LSTR MSG_BUTTON_DONE                    = _UxGT("Tamamlandı");
  LSTR MSG_BUTTON_BACK                    = _UxGT("Geri");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("Devam Et");
  LSTR MSG_BUTTON_SKIP                    = _UxGT("Atla");
  LSTR MSG_BUTTON_INFO                    = _UxGT("Bilgi");
  LSTR MSG_BUTTON_LEVEL                   = _UxGT("Seviye");
  LSTR MSG_BUTTON_PAUSE                   = _UxGT("Duraklat");
  LSTR MSG_BUTTON_RESUME                  = _UxGT("Devam Et");
  LSTR MSG_BUTTON_ADVANCED                = _UxGT("Gelişmiş");
  LSTR MSG_BUTTON_SAVE                    = _UxGT("Kaydet");
  LSTR MSG_BUTTON_PURGE                   = _UxGT("Temizle");
  LSTR MSG_PAUSING                        = _UxGT("Duraklatılıyor...");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Duraklat");
  LSTR MSG_ADVANCED_PAUSE                 = _UxGT("Gelişmiş Duraklatma");
  LSTR MSG_RESUME_PRINT                   = _UxGT("Baskıyı Sürdür");
  LSTR MSG_STOP_PRINT                     = _UxGT("Baskıyı Durdur");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Kesinti Kurtarma");
  LSTR MSG_RESUME_BED_TEMP                = _UxGT("Tabla Isıtmayı Sürdür");
  LSTR MSG_HOST_START_PRINT               = _UxGT("Host Başlat");
  LSTR MSG_PRINTING_OBJECT                = _UxGT("Nesneyi Yazdır");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("Nesneyi İptal Et");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("Nesneyi İptal Et {");
  LSTR MSG_CONTINUE_PRINT_JOB             = _UxGT("İşe Devam Et");
  LSTR MSG_MEDIA_MENU                     = _UxGT("Medyayı Yazdır");
  LSTR MSG_TURN_OFF                       = _UxGT("Şimdi kapat");
  LSTR MSG_END_LOOPS                      = _UxGT("Son Döngüler");
  LSTR MSG_NO_MEDIA                       = _UxGT("SD Kart Yok!");
  LSTR MSG_DWELL                          = _UxGT("Uyku...");
  LSTR MSG_USERWAIT                       = _UxGT("Devam için tıkla...");
  LSTR MSG_PRINT_PAUSED                   = _UxGT("Baskı Duraklatıldı");
  LSTR MSG_PRINTING                       = _UxGT("Baskı Yapılıyor...");
  LSTR MSG_STOPPING                       = _UxGT("Durduruluyor...");
  LSTR MSG_REMAINING_TIME                 = _UxGT("Kalan");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("Baskı Durduruldu!");
  LSTR MSG_PRINT_DONE                     = _UxGT("Yazdırma Bitti");
  LSTR MSG_PRINTER_KILLED                 = _UxGT("Yazıcı çöktü!");
  LSTR MSG_NO_MOVE                        = _UxGT("İşlem yok.");
  LSTR MSG_KILLED                         = _UxGT("ÇÖKTÜ. ");
  LSTR MSG_STOPPED                        = _UxGT("DURDURULDU. ");
  LSTR MSG_FWRETRACT                      = _UxGT("Yazılım Geri Çekme");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Geri Çek mm");
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("GÇ. Değiş mm");
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Geri Çekme V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Atlama mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("Geri Çek. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("S Unretr. mm");
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("Unretract V");
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("S UnRet V");
  LSTR MSG_AUTORETRACT                    = _UxGT("Oto. Geri Çekme");
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("G.Çekme Boyu");
  LSTR MSG_FILAMENT_SWAP_EXTRA            = _UxGT("Ekstra Değiştir");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("Tasfiye uzunluğu");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Takım Değişimi");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Z Yükselt");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Birincil Hız");
  LSTR MSG_SINGLENOZZLE_WIPE_RETRACT      = _UxGT("Geri Çekme Sil");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Geri Çekme Hızı");
  LSTR MSG_FILAMENT_PARK_ENABLED          = _UxGT("Kafa Park");
  LSTR MSG_PARK_FAILED                    = _UxGT("Kafa park edilemiyor");
  LSTR MSG_SINGLENOZZLE_UNRETRACT_SPEED   = _UxGT("Kurtarma Hızı");
  LSTR MSG_SINGLENOZZLE_FAN_SPEED         = _UxGT("Fan Hızı");
  LSTR MSG_SINGLENOZZLE_FAN_TIME          = _UxGT("Fan Süresi");
  LSTR MSG_TOOL_MIGRATION_ON              = _UxGT("Otomatik AÇIK");
  LSTR MSG_TOOL_MIGRATION_OFF             = _UxGT("Otomatik KAPALI");
  LSTR MSG_TOOL_MIGRATION                 = _UxGT("Takım Taşıma");
  LSTR MSG_TOOL_MIGRATION_AUTO            = _UxGT("Otomatik-taşıma");
  LSTR MSG_TOOL_MIGRATION_END             = _UxGT("Son Ekstruder");
  LSTR MSG_TOOL_MIGRATION_SWAP            = _UxGT("* konumuna taşı");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Filaman Değiştir");
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Filaman Değiştir *");
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Filaman Yükle");
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Filaman Yükle *");
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Filaman Çıkart");
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Filaman Çıkart *");
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Tümünü Çıkart");
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("SD Kartı takın");
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("USB Sürücüyü takın");
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Medyayı Ekle");
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Medyayı Değiştir");
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Medyayı Çıkart");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Z Prob Tablayı Geçti");
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Çarpıklık Faktörü");
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch");
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("BLTouch K. Test");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Sıfırla");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("Kapat");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("Aç");
  LSTR MSG_BLTOUCH_SW_MODE                = _UxGT("SW-Modu");
  LSTR MSG_BLTOUCH_SPEED_MODE             = _UxGT("Yüksek Hız");
  LSTR MSG_BLTOUCH_5V_MODE                = _UxGT("5V-Modu");
  LSTR MSG_BLTOUCH_OD_MODE                = _UxGT("OD-Modu");
  LSTR MSG_BLTOUCH_MODE_STORE             = _UxGT("Mode-Store");
  LSTR MSG_BLTOUCH_MODE_STORE_5V          = _UxGT("BLTouch 5V Ayarla");
  LSTR MSG_BLTOUCH_MODE_STORE_OD          = _UxGT("BLTouch OD Ayarla");
  LSTR MSG_BLTOUCH_MODE_ECHO              = _UxGT("Drenaj Raporu");
  LSTR MSG_BLTOUCH_MODE_CHANGE            = _UxGT("TEHLIKE: Kötü ayarlar zarar verebilir! Yine de devam edilsin mi?");
  LSTR MSG_TOUCHMI_PROBE                  = _UxGT("TouchMI");
  LSTR MSG_TOUCHMI_INIT                   = _UxGT("TouchMI'yi Başlat");
  LSTR MSG_TOUCHMI_ZTEST                  = _UxGT("Z Ofset Testi");
  LSTR MSG_TOUCHMI_SAVE                   = _UxGT("Kaydet");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI          = _UxGT("TouchMI Aç");
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("Z-Probe Aç");
  LSTR MSG_MANUAL_PENUP                   = _UxGT("Kalem yukarı");
  LSTR MSG_MANUAL_PENDOWN                 = _UxGT("Kalem aşağı");
  LSTR MSG_MANUAL_STOW                    = _UxGT("Z-Probe Kapat");
  LSTR MSG_HOME_FIRST                     = _UxGT("Önce %s Sıfırla");
  LSTR MSG_ZPROBE_SETTINGS                = _UxGT("Prob Ayarları");
  LSTR MSG_ZPROBE_OFFSETS                 = _UxGT("Prob Ofsetleri");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("X Prob Ofseti");
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Y Prob Ofseti");
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Z Prob Ofseti");
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("@ Prob Ofseti");
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Z ayarı adımsal");
  LSTR MSG_ZPROBE_MARGIN                  = _UxGT("Prob Payı");
  LSTR MSG_Z_FEED_RATE                    = _UxGT("Z İlerleme Hızı");
  LSTR MSG_ENABLE_HS_MODE                 = _UxGT("HS modunu aç");
  LSTR MSG_MOVE_NOZZLE_TO_BED             = _UxGT("Nozülü Tablaya Taşı");
  LSTR MSG_BABYSTEP_X                     = _UxGT("Miniadım X");
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Miniadım Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Miniadım Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Miniadım @");
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("Toplam");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Endstop iptal");
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Isınma başarısız");
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Hata: ISI AŞIMI");
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("ISI SORUNU");
  LSTR MSG_ERR_TEMP_MALFUNCTION           = _UxGT("SICAKLIK ARIZASI");
  LSTR MSG_ERR_COOLING_FAILED             = _UxGT("Soğutma Başarısız");
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("HATA:MAX.SICAK.");
  LSTR MSG_ERR_MINTEMP                    = _UxGT("HATA:MIN.SICAK.");
  LSTR MSG_HALTED                         = _UxGT("YAZICI DURDURULDU");
  LSTR MSG_PLEASE_WAIT                    = _UxGT("Lütfen bekleyin...");
  LSTR MSG_PLEASE_RESET                   = _UxGT("Lütfen Resetleyin");
  LSTR MSG_PREHEATING                     = _UxGT("ön ısıtma...");
  LSTR MSG_HEATING                        = _UxGT("Isınıyor...");
  LSTR MSG_COOLING                        = _UxGT("Soğuyor...");
  LSTR MSG_BED_HEATING                    = _UxGT("Tabla Isınıyor...");
  LSTR MSG_BED_COOLING                    = _UxGT("Tabla Soğuyor...");
  LSTR MSG_BED_ANNEALING                  = _UxGT("Tavlama...");
  LSTR MSG_PROBE_HEATING                  = _UxGT("Prob ısınıyor...");
  LSTR MSG_PROBE_COOLING                  = _UxGT("Prob Soğuyor...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("Kabin Isınıyor...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("Kabin Soğuyor...");
  LSTR MSG_LASER_COOLING                  = _UxGT("Lazer Soğuyor...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Delta Kalibrasyonu");
  LSTR MSG_DELTA_CALIBRATION_IN_PROGRESS  = _UxGT("Delta Kalibrasyonu sürüyor");
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Ayarla X");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Ayarla Y");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Ayarla Z");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Ayarla Merkez");
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Delta Ayarları");
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Oto Kalibrasyon");
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Çapral Mil");
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Yükseklik");
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Yarıçap");
  LSTR MSG_INFO_MENU                      = _UxGT("Yazıcı Hakkında");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Yazıcı Bilgisi");
  LSTR MSG_3POINT_LEVELING                = _UxGT("3-Nokta Seviyeleme");
  LSTR MSG_LINEAR_LEVELING                = _UxGT("Doğrusal Seviyeleme");
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("İki Yönlü Doğ. Hiza.");
  LSTR MSG_UBL_LEVELING                   = _UxGT("Birleşik Tabla Hiza.");
  LSTR MSG_MESH_LEVELING                  = _UxGT("Mesh Seviyeleme");
  LSTR MSG_MESH_DONE                      = _UxGT("Mesh sondalama tamam");
  LSTR MSG_INFO_STATS_MENU                = _UxGT("İstatistikler");
  LSTR MSG_RESET_STATS                    = _UxGT("Yazdırma istatistik Sıfırla?");
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Kontrolcü Bilgisi");
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Termistörler");
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Ekstruderler");
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("İletişim Hızı");
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Protokol");
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("Kaçak İzleme: KAPALI");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("Kaçak İzleme: AÇIK");
  LSTR MSG_HOTEND_IDLE_TIMEOUT            = _UxGT("Hotend Boşta Zamn Aş");
  LSTR MSG_HOTEND_IDLE_DISABLE            = _UxGT("Zaman Aşımı Pasif");
  LSTR MSG_HOTEND_IDLE_NOZZLE_TARGET      = _UxGT("Meme Boşta Isısı");
  LSTR MSG_HOTEND_IDLE_BED_TARGET         = _UxGT("Tabla Boşta Isısı");
  LSTR MSG_FAN_SPEED_FAULT                = _UxGT("Fan hızı hatası");

  LSTR MSG_CASE_LIGHT                     = _UxGT("Aydınlatma");
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Işık Parlaklğı");
  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("HATALI YAZICI");

  LSTR MSG_COLORS_GET                     = _UxGT("Renk Al");               // ProUI
  LSTR MSG_COLORS_SELECT                  = _UxGT("Renkleri Seç");          // ProUI
  LSTR MSG_COLORS_APPLIED                 = _UxGT("Uygulanan renkler");     // ProUI
  LSTR MSG_COLORS_RED                     = _UxGT("Kırmızı");               // ProUI / JyersUI
  LSTR MSG_COLORS_GREEN                   = _UxGT("Yeşil");                 // ProUI / JyersUI
  LSTR MSG_COLORS_BLUE                    = _UxGT("Mavi");                  // ProUI / JyersUI
  LSTR MSG_COLORS_WHITE                   = _UxGT("Beyaz");                 // ProUI
  LSTR MSG_UI_LANGUAGE                    = _UxGT("UI Dili");               // ProUI
  LSTR MSG_SOUND_ENABLE                   = _UxGT("Sesi etkinleştir");      // ProUI
  LSTR MSG_LOCKSCREEN                     = _UxGT("Kilit Ekranı");          // ProUI
  LSTR MSG_LOCKSCREEN_LOCKED              = _UxGT("Yazıcı Kilitlendi,");    // ProUI
  LSTR MSG_LOCKSCREEN_UNLOCK              = _UxGT("Açmak için kaydırın.");  // ProUI
  LSTR MSG_PLEASE_WAIT_REBOOT             = _UxGT("Y.Başlatma bekleyin.");  // ProUI
  LSTR MSG_MEDIA_NOT_INSERTED             = _UxGT("Medya Yok");             // ProUI
  LSTR MSG_PLEASE_PREHEAT                 = _UxGT("Ön Isıtma Lütfen");      // ProUI

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("Baskı");
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("Süre");
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("En Uzun");
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("Filaman");
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("Tamamlanan");
  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Min Sıc.");
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Max Sıc.");
  LSTR MSG_INFO_PSU                       = _UxGT("Güç Kaynağı");

  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Sürücü Gücü");
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ Sürücü %");
  LSTR MSG_ERROR_TMC                      = _UxGT("TMC BAĞLANTI HATASI");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("DAC EEPROM Yaz");
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("FILAMAN DEGISTIR");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("BASKI DURAKLATILDI");
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("FILAMAN YüKLE");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("FILAMAN ÇIKART");
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("Seçenekler:");
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Daha Fazla Temizle");
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Baskıyı sürdür");
  LSTR MSG_FILAMENT_CHANGE_PURGE_CONTINUE = _UxGT("Temizle veya Devam?"); // ProUI
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Nozul: ");
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("Fil. Sensörü");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("Aşınma Farkı mm");
  LSTR MSG_EXTRUDER_MIN_TEMP              = _UxGT("Ekstruder Min Isı");   // ProUI
  LSTR MSG_FANCHECK                       = _UxGT("Fan Takosu Kontrolü");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Sıfırlama Başarısız");
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("Probing Başarısız");

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("FILAMAN SEÇ");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("MMU Yaz. Güncelle!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU Dikkat Gerektirir.");
  LSTR MSG_MMU2_RESUME                    = _UxGT("Yaz. Devam Et");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("Sürdürülüyor...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("Filaman Yükle");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("Tümünü Yükle");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("Nozula Yükle");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("Filaman Çıkart");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("Filaman Çıkart ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("Filamenti Boşalt");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("%i Filaman Yükle...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("Filaman Çek...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("Filaman Boşalt....");
  LSTR MSG_MMU2_ALL                       = _UxGT("Tümü");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("Filaman ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("MMU Resetle");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("MMU Resetleniyor...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("MMU2 Eject Recover");
  LSTR MSG_MMU2_REMOVE_AND_CLICK          = _UxGT("Kaldır ve tıkla...");

  LSTR MSG_MIX                            = _UxGT("Karışım");
  LSTR MSG_MIX_COMPONENT_N                = _UxGT("Bileşen {");
  LSTR MSG_MIXER                          = _UxGT("Karıştırıcı");
  LSTR MSG_GRADIENT                       = _UxGT("Gradyan");
  LSTR MSG_FULL_GRADIENT                  = _UxGT("Tam Gradyan");
  LSTR MSG_TOGGLE_MIX                     = _UxGT("Karışım Geçişi");
  LSTR MSG_CYCLE_MIX                      = _UxGT("Döngü Karışımı");
  LSTR MSG_GRADIENT_MIX                   = _UxGT("Gradyan Karışımı");
  LSTR MSG_REVERSE_GRADIENT               = _UxGT("Ters Gradyan");
  LSTR MSG_ACTIVE_VTOOL                   = _UxGT("Aktif V-tool");
  LSTR MSG_START_VTOOL                    = _UxGT("Başlat V-tool");
  LSTR MSG_END_VTOOL                      = _UxGT("  Bitir V-tool");
  LSTR MSG_GRADIENT_ALIAS                 = _UxGT("Alias V-tool");
  LSTR MSG_RESET_VTOOLS                   = _UxGT("Reset V-tools");
  LSTR MSG_COMMIT_VTOOL                   = _UxGT("V-tool Karışıö Yap");
  LSTR MSG_VTOOLS_RESET                   = _UxGT("V-tools Resetlendi");
  LSTR MSG_START_Z                        = _UxGT("Başlat Z:");
  LSTR MSG_END_Z                          = _UxGT("  Bitir Z:");

  LSTR MSG_GAMES                          = _UxGT("Oyunlar");
  LSTR MSG_BRICKOUT                       = _UxGT("Brickout");
  LSTR MSG_INVADERS                       = _UxGT("Invaders");
  LSTR MSG_SNAKE                          = _UxGT("Sn4k3");
  LSTR MSG_MAZE                           = _UxGT("Maze");

  LSTR MSG_BAD_PAGE                       = _UxGT("Hatalı sayfa dizini");
  LSTR MSG_BAD_PAGE_SPEED                 = _UxGT("Kötü sayfa hızı");

  LSTR MSG_EDIT_PASSWORD                  = _UxGT("Şifreyi Düzenle");
  LSTR MSG_LOGIN_REQUIRED                 = _UxGT("Giriş Gerekli");
  LSTR MSG_PASSWORD_SETTINGS              = _UxGT("Şifre Ayarları");
  LSTR MSG_ENTER_DIGIT                    = _UxGT("Rakam girin");
  LSTR MSG_CHANGE_PASSWORD                = _UxGT("Şifre Belirle/Düzenle");
  LSTR MSG_REMOVE_PASSWORD                = _UxGT("Şifre Kaldır");
  LSTR MSG_PASSWORD_SET                   = _UxGT("Şifre ");
  LSTR MSG_START_OVER                     = _UxGT("Yeniden Başla");
  LSTR MSG_REMINDER_SAVE_SETTINGS         = _UxGT("Kaydetmeyi Unutma!");
  LSTR MSG_PASSWORD_REMOVED               = _UxGT("Şifre Kaldırıldı");

  // Filament Değişim ekranları 3 satırlı ekranda 2 satıra kadar
  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_1_LINE("Sürdürmek İçin Tıkla"));
  LSTR MSG_PAUSE_PRINT_PARKING            = _UxGT(MSG_1_LINE("Park Ediliyor..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("Lütfen bekleyiniz..."));
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("Yükle ve bas"));
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_1_LINE("Isıtmak için Tıkla"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("Isınıyor..."));
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("Çıkartılıyor..."));
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("Yüklüyor..."));
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("Temizleniyor..."));
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_1_LINE("Bitirmek için Tıkla"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("Sürdürülüyor..."));
  LSTR MSG_TMC_DRIVERS                    = _UxGT("TMC Sürücüleri");
  LSTR MSG_TMC_CURRENT                    = _UxGT("Sürücü Akımı");
  LSTR MSG_TMC_ACURRENT                   = STR_A _UxGT("Sürücü Akımı");
  LSTR MSG_TMC_BCURRENT                   = STR_B _UxGT("Sürücü Akımı");
  LSTR MSG_TMC_CCURRENT                   = STR_C _UxGT("Sürücü Akımı");
  LSTR MSG_TMC_ECURRENT                   = _UxGT("E Sürücü Akımı");
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("Hibrit Eşiği");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Sensörsüz Sıfırlama");
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("Adım Modu");
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("StealthChop Aktif");
  LSTR MSG_SERVICE_RESET                  = _UxGT("Resetle");
  LSTR MSG_SERVICE_IN                     = _UxGT(" içinde:");
  LSTR MSG_BACKLASH                       = _UxGT("Ters Tepki");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("Düzeltme");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("Yumuşatma");

  LSTR MSG_FIXED_TIME_MOTION              = _UxGT("Sabit Zamanlı Hareket");
  LSTR MSG_FTM_CMPN_MODE                  = _UxGT("@ Telafi Modu:");
  LSTR MSG_FTM_ZV                         = _UxGT("ZV");
  LSTR MSG_FTM_ZVD                        = _UxGT("ZVD");
  LSTR MSG_FTM_ZVDD                       = _UxGT("ZVDD");
  LSTR MSG_FTM_ZVDDD                      = _UxGT("ZVDDD");
  LSTR MSG_FTM_EI                         = _UxGT("EI");
  LSTR MSG_FTM_2HEI                       = _UxGT("2HEI");
  LSTR MSG_FTM_3HEI                       = _UxGT("3HEI");
  LSTR MSG_FTM_MZV                        = _UxGT("MZV");
  //LSTR MSG_FTM_ULENDO_FBS               = _UxGT("Ulendo FBS");
  //LSTR MSG_FTM_DISCTF                   = _UxGT("DISCTF");
  LSTR MSG_FTM_DYN_MODE                   = _UxGT("DF Modu:");
  LSTR MSG_FTM_Z_BASED                    = _UxGT("Z-based");
  LSTR MSG_FTM_MASS_BASED                 = _UxGT("Mass-based");
  LSTR MSG_FTM_BASE_FREQ_N                = _UxGT("@ Base Freq.");
  LSTR MSG_FTM_DFREQ_K_N                  = _UxGT("@ Dyn. Freq.");
  LSTR MSG_FTM_ZETA_N                     = _UxGT("@ Damping");
  LSTR MSG_FTM_VTOL_N                     = _UxGT("@ Vib. Level");

  LSTR MSG_LEVEL_X_AXIS                   = _UxGT("Seviye X Ekseni");
  LSTR MSG_AUTO_CALIBRATE                 = _UxGT("Otomatik Kalibre Et");
  LSTR MSG_FTDI_HEATER_TIMEOUT            = _UxGT("Boşta kalma zaman aşımı, sıcaklık düştü. Yeniden ısıtmak ve tekrar devam etmek için için Tamam'a basın.");
  LSTR MSG_HEATER_TIMEOUT                 = _UxGT("Isıtıcı Zaman Aşımı");
  LSTR MSG_REHEAT                         = _UxGT("Yeniden ısıt");
  LSTR MSG_REHEATING                      = _UxGT("Yeniden ısıtılıyor...");
  LSTR MSG_REHEATDONE                     = _UxGT("Y. Isıtma Tamam");

  LSTR MSG_PROBE_WIZARD                   = _UxGT("Z Prob Sihirbazı");
  LSTR MSG_PROBE_WIZARD_PROBING           = _UxGT("Z Referansını Tarama");
  LSTR MSG_PROBE_WIZARD_MOVING            = _UxGT("Prob Konumuna Geçme");

  LSTR MSG_XATC                           = _UxGT("X-Twist Sihirbazı");
  LSTR MSG_XATC_DONE                      = _UxGT("X-Twist Sihirbazı Bitti!");
  LSTR MSG_XATC_UPDATE_Z_OFFSET           = _UxGT("Prob Z-Offset güncelle ");

  LSTR MSG_SOUND                          = _UxGT("Ses");

  LSTR MSG_TOP_LEFT                       = _UxGT("Üst Sol");
  LSTR MSG_BOTTOM_LEFT                    = _UxGT("Alt Sol");
  LSTR MSG_TOP_RIGHT                      = _UxGT("Üst Sağ");
  LSTR MSG_BOTTOM_RIGHT                   = _UxGT("Alt Sağ");
  LSTR MSG_CALIBRATION_COMPLETED          = _UxGT("Kalibrasyon Tamamlandı");
  LSTR MSG_CALIBRATION_FAILED             = _UxGT("Kalibrasyon Başarısız");

  LSTR MSG_DRIVER_BACKWARD                = _UxGT(" driver backward");

  LSTR MSG_SD_CARD                        = _UxGT("SD Kart");
  LSTR MSG_USB_DISK                       = _UxGT("USB Disk");

  LSTR MSG_HOST_SHUTDOWN                  = _UxGT("Host'u Kapat");

  LSTR MSG_SHORT_DAY                      = _UxGT("g"); // One character only
  LSTR MSG_SHORT_HOUR                     = _UxGT("s"); // One character only
  LSTR MSG_SHORT_MINUTE                   = _UxGT("d"); // One character only

  // These strings can be the same in all languages
  LSTR MSG_MARLIN                         = _UxGT("Marlin");
  LSTR MSG_PID_P                          = _UxGT("PID-P");
  LSTR MSG_PID_P_E                        = _UxGT("PID-P *");
  LSTR MSG_PID_I                          = _UxGT("PID-I");
  LSTR MSG_PID_I_E                        = _UxGT("PID-I *");
  LSTR MSG_PID_D                          = _UxGT("PID-D");
  LSTR MSG_PID_D_E                        = _UxGT("PID-D *");
  LSTR MSG_PID_C                          = _UxGT("PID-C");
  LSTR MSG_PID_C_E                        = _UxGT("PID-C *");
  LSTR MSG_PID_F                          = _UxGT("PID-F");
  LSTR MSG_PID_F_E                        = _UxGT("PID-F *");
  LSTR MSG_BACKLASH_N                     = _UxGT("@");

  // DGUS-Specific message strings, not used elsewhere
  LSTR DGUS_MSG_NOT_WHILE_PRINTING        = _UxGT("Yazdırma sırasında izin verilmez");
  LSTR DGUS_MSG_NOT_WHILE_IDLE            = _UxGT("Boştayken izin verilmez");
  LSTR DGUS_MSG_NO_FILE_SELECTED          = _UxGT("Seçili dosya yok");
  LSTR DGUS_MSG_EXECUTING_COMMAND         = _UxGT("Komut yürütülüyor...");
  LSTR DGUS_MSG_BED_PID_DISABLED          = _UxGT("Tabla PID pasif");
  LSTR DGUS_MSG_PID_DISABLED              = _UxGT("PID devre dışı");
  LSTR DGUS_MSG_PID_AUTOTUNING            = _UxGT("PID otomatik ayarlama...");
  LSTR DGUS_MSG_INVALID_RECOVERY_DATA     = _UxGT("Geçersiz kurtarma verileri");

  LSTR DGUS_MSG_HOMING_REQUIRED           = _UxGT("Eve dönüş gerekli");
  LSTR DGUS_MSG_BUSY                      = _UxGT("Meşgul");
  LSTR DGUS_MSG_HOMING                    = _UxGT("Eve dönüş...");
  LSTR DGUS_MSG_FW_OUTDATED               = _UxGT("DWIN GUI/OS güncelleme gerekli");
  LSTR DGUS_MSG_ABL_REQUIRED              = _UxGT("Oto tabla seviyeleme gerekli");
  LSTR DGUS_MSG_PROBING_FAILED            = _UxGT("Sondalama başarısız");
  LSTR DGUS_MSG_PROBING_SUCCESS           = _UxGT("Sondalama başarılı");
  LSTR DGUS_MSG_RESET_EEPROM              = _UxGT("EEPROM sıfırla");
  LSTR DGUS_MSG_WRITE_EEPROM_FAILED       = _UxGT("EEPROM yazımı başarısız");
  LSTR DGUS_MSG_READ_EEPROM_FAILED        = _UxGT("EEPROM okuma başarısız");
  LSTR DGUS_MSG_FILAMENT_RUNOUT           = _UxGT("E%d Filaman Tükenme");
}

namespace LanguageWide_tr {
  using namespace LanguageNarrow_tr;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_LIVE_MOVE                    = _UxGT("Canlı Hareket");
    LSTR MSG_HOST_START_PRINT             = _UxGT("Host Baskıyı başlat");
    LSTR MSG_PRINTING_OBJECT              = _UxGT("Yazdırma Nesnesi");
    LSTR MSG_CANCEL_OBJECT                = _UxGT("Nesneyi İptal Et");
    LSTR MSG_CANCEL_OBJECT_N              = _UxGT("Nesneyi İptal Et {");
    LSTR MSG_CONTINUE_PRINT_JOB           = _UxGT("Yazdırmaya Devam Et");
    LSTR MSG_MEDIA_MENU                   = _UxGT("SD Karttan Yazdır");
    LSTR MSG_TURN_OFF                     = _UxGT("Yazıcıyı kapat");
    LSTR MSG_END_LOOPS                    = _UxGT("Tekrr Döngüler Bitir");
    LSTR MSG_MEDIA_NOT_INSERTED           = _UxGT("Ortam yerleştirilmedi.");        // ProUI
    LSTR MSG_PLEASE_PREHEAT               = _UxGT("Lütfen önce hotend'i ısıtın.");  // ProUI
    LSTR MSG_INFO_PRINT_COUNT_RESET       = _UxGT("Baskı Sayısını Sıfırla");        // ProUI
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Baskı Sayısı");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Toplam Baskı Süresi");
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("En Uzun Baskı Süresi");
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Toplam Filaman");
  #endif
}

namespace LanguageTall_tr {
  using namespace LanguageWide_tr;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Baskıya devam etmek", "için Butona bas"));
    LSTR MSG_PAUSE_PRINT_PARKING          = _UxGT(MSG_1_LINE("Park Ediliyor..."));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Filaman değişimi", "için başlama", "bekleniyor"));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Filamanı yükle", "ve devam için", "tuşa bas..."));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Nozulü Isıtmak için", "Butona Bas."));
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Nozul Isınıyor", "Lütfen Bekleyin..."));
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_2_LINE("Filamanın çıkması", "bekleniyor"));
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_2_LINE("Filamanın yüklenmesi", "bekleniyor.."));
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("Filaman Temizlemesi", "için bekle"));
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_2_LINE("Filaman Temizlemesi", "bitirmek için tıkla"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_2_LINE("Baskının devam ", "etmesi için bekle"));
  #endif
}

namespace Language_tr {
  using namespace LanguageTall_tr;
}
