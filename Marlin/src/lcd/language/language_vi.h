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
 * Vietnamese
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */
namespace LanguageNarrow_vi {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 2;
  LSTR LANGUAGE                           = _UxGT("Vietnamese");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT(" Sẵn sàng.");              // Ready
  LSTR MSG_BACK                           = _UxGT("Trở lại");                              // Back
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("Đang hủy bỏ...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("Phương tiện được cắm vào");             // Media inserted
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("Phương tiện được rút ra");
  LSTR MSG_MEDIA_WAITING                  = _UxGT("Chờ đợi phương tiện");
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("Lỗi đọc phương tiện");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("USB được rút ra");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("USB khởi thất bại");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("Công tắc");                             // Endstops - công tắc hành trình
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("Công tắc mềm");                         // Soft Endstops
  LSTR MSG_MAIN_MENU                      = _UxGT("Chính");                                // Main
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("Thiết lập cấp cao");                    // Advanced Settings
  LSTR MSG_CONFIGURATION                  = _UxGT("Cấu hình");                             // Configuration
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("Khởi chạy tự động");                    // Autostart
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("Tắt động cơ bước");                     // Disable steppers
  LSTR MSG_DEBUG_MENU                     = _UxGT("Menu gỡ lỗi");                          // Debug Menu
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("Kiểm tra tiến độ");                     // Progress bar test
  LSTR MSG_AUTO_HOME                      = _UxGT("Về nhà tự động");                       // Auto home
  LSTR MSG_AUTO_HOME_X                    = _UxGT("Về nhà X");                             // Home X
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("Về nhà Y");                             // Home Y
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("Về nhà Z");                             // Home Z
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("Chỉnh canh Z tự động");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("Đang về nhà XYZ");                      // Homing XYZ
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("Nhấn để bắt đầu");                      // Click to Begin
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("Điểm tiếp theo");                       // Next Point
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("San lấp được hoàn thành");              // Leveling Done!
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("Chiều cao mờ dần");                     // Fade Height
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("Đặt bù đắp nhà");                       // Set home offsets
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("Bù đắp được áp dụng");                  // Offsets applied

  LSTR MSG_PREHEAT_1                      = _UxGT("Làm nóng ") PREHEAT_1_LABEL _UxGT(" trước");      // Preheat
  LSTR MSG_PREHEAT_1_H                    = _UxGT("Làm nóng ") PREHEAT_1_LABEL _UxGT(" trước ~");    // Preheat
  LSTR MSG_PREHEAT_1_END                  = _UxGT("Làm nóng ") PREHEAT_1_LABEL _UxGT(" Đầu");
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("Làm nóng ") PREHEAT_1_LABEL _UxGT(" Đầu ~");
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("Làm nóng ") PREHEAT_1_LABEL _UxGT(" Tất cả");     // All
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("Làm nóng ") PREHEAT_1_LABEL _UxGT(" Bàn");        // Bed -- using vietnamese term for 'table' instead
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("Làm nóng ") PREHEAT_1_LABEL _UxGT(" Cấu hình");   // Conf

  LSTR MSG_PREHEAT_M                      = _UxGT("Làm nóng $ trước");                     // Preheat
  LSTR MSG_PREHEAT_M_H                    = _UxGT("Làm nóng $ trước ~");                   // Preheat
  LSTR MSG_PREHEAT_M_END                  = _UxGT("Làm nóng $ Đầu");
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("Làm nóng $ Đầu ~");
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("Làm nóng $ Tất cả");                    // All
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("Làm nóng $ Bàn");                       // Bed -- using vietnamese term for 'table' instead
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("Làm nóng $ Cấu hình");                  // Conf

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("Sự nóng trước tự chọn");                // Preheat Custom
  LSTR MSG_COOLDOWN                       = _UxGT("Nguội xuống");                          // Cooldown
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("Bật nguồn");                            // Switch power on
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("Tắt nguồn");                            // Switch power off
  LSTR MSG_EXTRUDE                        = _UxGT("Ép đùn");                               // Extrude
  LSTR MSG_RETRACT                        = _UxGT("Rút lại");                              // Retract
  LSTR MSG_MOVE_AXIS                      = _UxGT("Di chuyển trục");                       // Move axis
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("Chạm và san lấp");                      // Probe and Level
  LSTR MSG_BED_LEVELING                   = _UxGT("San Lấp Bàn");                          // Bed Leveling
  LSTR MSG_LEVEL_BED                      = _UxGT("Làm bằng mặt bàn");                     // Level bed
  LSTR MSG_BED_TRAMMING                   = _UxGT("Làm bằng góc bàn");                     // Level corners
  LSTR MSG_NEXT_CORNER                    = _UxGT("Góc tiếp theo");                        // Next corner
  LSTR MSG_EDITING_STOPPED                = _UxGT("Chỉnh lưới đã dừng");                   // Mesh Editing Stopped
  LSTR MSG_MESH_X                         = _UxGT("Mục lục X");                            // Index X
  LSTR MSG_MESH_Y                         = _UxGT("Mục lục Y");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Giá trị Z");                            // Z Value
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("Các lệnh tự chọn");                     // Custom Commands
  LSTR MSG_UBL_DOING_G29                  = _UxGT("Đang chạy G29");                        // Doing G29
  LSTR MSG_UBL_TOOLS                      = _UxGT("Công cụ UBL");                          // UBL tools
  LSTR MSG_IDEX_MENU                      = _UxGT("chế độ IDEX");                          // IDEX Mode
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("Đậu tự động");                          // Auto-Park
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("Sự gấp đôi");                           // Duplication
  LSTR MSG_IDEX_MODE_SCALED_COPY          = _UxGT("Bản sao thu nhỏ");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("Toàn quyền điều khiển");                // Full control
  LSTR MSG_IDEX_X_OFFSET                  = _UxGT("Đầu phun X nhì");                       // 2nd nozzle X
  LSTR MSG_IDEX_Y_OFFSET                  = _UxGT("Đầu phun Y nhì");
  LSTR MSG_IDEX_Z_OFFSET                  = _UxGT("Đầu phun Z nhì");
  LSTR MSG_IDEX_SAVE_OFFSETS              = _UxGT("Lưu bù đắp");                           // Save offsets
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("Tự xây dựng lưới");                     // Manually Build Mesh
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("Đặt chêm và đo");                       // Place shim & measure
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("Đo");                                   // Measure
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("Tháo và đo bàn");                       // Remove & measure bed
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("Chuyển sang tiếp theo");                // Moving to next
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("Nhiệt độ bàn");                         // Bed Temp
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("Bed Temp");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("Nhiệt độ đầu phun");                    // Hotend Temp
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("Hotend Temp");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("Chỉnh sửa lưới tự chọn");               // Edit Custom Mesh
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("Chỉnh lưới chính xác");                 // Fine tuning mesh
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("Chỉnh sửa xong lưới");                  // Done Editing Mesh
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("Xây dựng lưới tự chọn");                // Build Custom Mesh
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("Xây dựng lưới");                        // Build Mesh
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("Xây dựng lưới ($)");
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("Thẩm tra lưới ($)");
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("Xây dựng lưới lạnh");                   // Build cold mesh
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("Điều chỉnh chiều cao lưới");            // Adjust Mesh Height
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("Số lượng chiều cao");                   // Height Amount
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("Thẩm tra lưới");                        // Validate Mesh
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("Thẩm tra lưới tự chọn");                // Validate custom mesh
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("Tiếp tục xây lưới bàn");                // Continue Bed Mesh
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("Đang san lấp lưới");                    // Mesh Leveling
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("Đang san lấp 3-điểm");                  // 3-Point Leveling
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("Đang san lấp lưới phẳng");              // Grid (planar) Mesh Leveling
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("Làm bằng lưới");                        // Level Mesh
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("Điểm bên cạnh");                        // Side Points
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("Loại bản đồ");                          // Map Type
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("Đầu ra bản đồ lưới");                   // Output Mesh Map
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("Đầu ra cho máy chủ");                   // Output for Host
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("Đầu ra cho CSV");                       // Output for CSV
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("Hỗ trợ lưới");                          // Off Printer Backup | backup mesh
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("Đầu ra thông tin UBL");                 // Output UBL Info
  LSTR MSG_EDIT_MESH                      = _UxGT("Chỉnh sửa lưới");                       // Edit mesh
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("Số lượng lấp đầy");                     // Fill-in Amount
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("Tự lấp đầy");                           // Manual Fill-in
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("Lấp đầy thông minh");                   // Smart Fill-in
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("Lưới lấp đầy");                         // Fill-in Mesh
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("Bác bỏ tất cả");                       // Invalidate All
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("Bác bỏ gần nhất");                     // Invalidate Closest
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("Chỉnh chính xác tất cả");               // Fine Tune ALl
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("Chỉnh chính xác gần nhất");             // Fine Tune Closest
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("Lưu trữ lưới");                         // Mesh Storage
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("Khe nhớ");                              // Memory Slot
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("Tải lưới bàn");                         // Load Bed Mesh
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("Lưu lưới bàn");                         // Save Bed Mesh
  LSTR MSG_MESH_LOADED                    = _UxGT("%i lưới được nạp");                     // Mesh %i loaded
  LSTR MSG_MESH_SAVED                     = _UxGT("%i lưới đã lưu");
  LSTR MSG_NO_STORAGE                     = _UxGT("Không lưu trữ");                        // No Storage
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("Điều sai: Lưu UBL");                    // Err: UBL Save
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("Điều Sai: Khôi Phục UBL");              // Err: UBL Restore
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Đầu Dò-Z Đã Ngừng");                    // Z-Offset Stopped
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("Bước-Từng-Bước UBL");                   // Step-By-Step UBL
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1.Xây dựng lưới lạnh");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2.Lấp đầy thông minh");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3.Thẩm tra lưới");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4.Chỉnh chính xác tất cả");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5.Thẩm tra lưới");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6.Chỉnh chính xác tất cả");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7.Lưu lưới bàn");

  LSTR MSG_LED_CONTROL                    = _UxGT("Điều khiển LED");                       // LED Control
  LSTR MSG_LIGHTS                         = _UxGT("Đèn");                                  // Lights
  LSTR MSG_LIGHT_N                        = _UxGT("Đèn #{");                               // Light #{
  LSTR MSG_LED_PRESETS                    = _UxGT("Đèn định sẵn");                         // Light Presets
  LSTR MSG_SET_LEDS_RED                   = _UxGT("Đỏ");                                   // Red
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("Cam");                                  // Orange
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("Vàng");                                 // Yellow
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("Xanh Lá");                              // Green
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("Xanh");                                 // Blue
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("Xanh Đậm");                             // Indigo
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("Tím");                                  // Violet
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("Trắng");                                // White
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("Mặc định");                             // Default
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("Đèn Tự Chọn");                          // Custom Lights
  LSTR MSG_INTENSITY_R                    = _UxGT("Cường Độ Đỏ");                          // Red Intensity
  LSTR MSG_INTENSITY_G                    = _UxGT("Cường Độ Xanh Lá");                     // Green Intensity
  LSTR MSG_INTENSITY_B                    = _UxGT("Cường Độ Xanh");                        // Blue Intensity
  LSTR MSG_INTENSITY_W                    = _UxGT("Cường Độ Trắng");                       // White Intensity
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("độ sáng");                              // Brightness

  LSTR MSG_MOVING                         = _UxGT("Di chuyển...");                         // Moving
  LSTR MSG_FREE_XY                        = _UxGT("Giải phóng XY");                        // Free XY
  LSTR MSG_MOVE_X                         = _UxGT("Di chuyển X");                          // Move X
  LSTR MSG_MOVE_Y                         = _UxGT("Di chuyển Y");
  LSTR MSG_MOVE_Z                         = _UxGT("Di chuyển Z");
  LSTR MSG_MOVE_N                         = _UxGT("Di chuyển @");
  LSTR MSG_MOVE_E                         = _UxGT("Máy đùn");                              // Extruder
  LSTR MSG_MOVE_EN                        = _UxGT("Máy đùn *");
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("Đầu nóng quá lạnh");                    // Hotend too cold
  LSTR MSG_MOVE_N_MM                      = _UxGT("Di chuyển $mm");                        // Move $mm
  LSTR MSG_MOVE_N_IN                      = _UxGT("Di chuyển $in");                        // Move $in
  LSTR MSG_MOVE_N_DEG                     = _UxGT("Di chuyển $") LCD_STR_DEGREE;           // Move $°
  LSTR MSG_SPEED                          = _UxGT("Tốc độ");                               // Speed
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("Z Bàn");
  LSTR MSG_NOZZLE                         = _UxGT("Đầu phun");                             // Nozzle
  LSTR MSG_NOZZLE_N                       = _UxGT("Đầu phun ~");
  LSTR MSG_BED                            = _UxGT("Bàn");                                  // Bed
  LSTR MSG_FAN_SPEED                      = _UxGT("Tốc độ quạt");                          // Fan speed
  LSTR MSG_FAN_SPEED_N                    = _UxGT("Tốc độ quạt ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("Tốc độ quạt phụ");                      // Extra fan speed
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("Tốc độ quạt phụ ~");
  LSTR MSG_FLOW                           = _UxGT("Lưu Lượng");
  LSTR MSG_FLOW_N                         = _UxGT("Lưu Lượng ~");
  LSTR MSG_CONTROL                        = _UxGT("Điều khiển");                           // Control
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" Đa");          // Min
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" Thiểu");       // Max
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" Hệ Số");       // Factor
  LSTR MSG_AUTOTEMP                       = _UxGT("Nhiệt độ tự động");                     // Autotemp
  LSTR MSG_LCD_ON                         = _UxGT("Bật");                                  // On
  LSTR MSG_LCD_OFF                        = _UxGT("Tắt");                                  // Off
  LSTR MSG_SELECT_E                       = _UxGT("Lựa *");
  LSTR MSG_ACC                            = _UxGT("Tăng Tốc");
  LSTR MSG_JERK                           = _UxGT("Giật");
  LSTR MSG_VA_JERK                        = _UxGT("Giật-V") STR_A;
  LSTR MSG_VB_JERK                        = _UxGT("Giật-V") STR_B;
  LSTR MSG_VC_JERK                        = _UxGT("Giật-V") STR_C;
  LSTR MSG_VN_JERK                        = _UxGT("Giật-V@");
  LSTR MSG_VE_JERK                        = _UxGT("Giật-Ve");
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("Độ Lệch Chỗ Giao");                    // Junction Dev
  LSTR MSG_MAX_SPEED                      = _UxGT("Vận tốc");                              // Velocity
  LSTR MSG_VMAX_A                         = _UxGT("Vđa") STR_A;                            // Vmax
  LSTR MSG_VMAX_B                         = _UxGT("Vđa") STR_B;
  LSTR MSG_VMAX_C                         = _UxGT("Vđa") STR_C;
  LSTR MSG_VMAX_N                         = _UxGT("Vđa@");
  LSTR MSG_VMAX_E                         = _UxGT("VđaE");
  LSTR MSG_VMAX_EN                        = _UxGT("Vđa *");
  LSTR MSG_VMIN                           = _UxGT("Vthiểu");                               // Vmin
  LSTR MSG_VTRAV_MIN                      = _UxGT("Vchuyển thiểu");                        // VTrav min
  LSTR MSG_ACCELERATION                   = _UxGT("Sự tăng tốc");                          // Acceleration
  LSTR MSG_AMAX_A                         = _UxGT("Tăng tốc ca") STR_A;                    // Amax
  LSTR MSG_AMAX_B                         = _UxGT("Tăng tốc ca") STR_B;
  LSTR MSG_AMAX_C                         = _UxGT("Tăng tốc ca") STR_C;
  LSTR MSG_AMAX_N                         = _UxGT("Tăng tốc ca@");
  LSTR MSG_AMAX_E                         = _UxGT("Tăng tốc caE");
  LSTR MSG_AMAX_EN                        = _UxGT("Tăng tốc ca *");
  LSTR MSG_A_RETRACT                      = _UxGT("TT-Rút");                              // A-retract
  LSTR MSG_A_TRAVEL                       = _UxGT("TT-Chuyển");                           // A-travel
  LSTR MSG_STEPS_PER_MM                   = _UxGT("Bước/mm");                             // Steps
  LSTR MSG_A_STEPS                        = _UxGT("Bước") STR_A _UxGT("/mm");             // Steps/mm
  LSTR MSG_B_STEPS                        = _UxGT("Bước") STR_B _UxGT("/mm");
  LSTR MSG_C_STEPS                        = _UxGT("Bước") STR_C _UxGT("/mm");
  LSTR MSG_N_STEPS                        = _UxGT("Bước@/mm");
  LSTR MSG_E_STEPS                        = _UxGT("BướcE/mm");
  LSTR MSG_EN_STEPS                       = _UxGT("Bước */mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("Nhiệt độ");                            // Temperature
  LSTR MSG_MOTION                         = _UxGT("Chuyển động");                         // Motion
  LSTR MSG_FILAMENT                       = _UxGT("Vật liệu in");                         // Dây nhựa
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E bằng mm") SUPERSCRIPT_THREE;         // E in mm
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("Đường kính nhựa");                     // Fil. Dai.
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("Đường kính nhựa *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("Dỡ mm");                               // Unload mm
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("Nạp mm");
  LSTR MSG_ADVANCE_K                      = _UxGT("K Cấp Cao");                           // Advance K
  LSTR MSG_ADVANCE_K_E                    = _UxGT("K Cấp Cao *");                         // Advance K
  LSTR MSG_CONTRAST                       = _UxGT("Độ tương phản LCD");                   // LCD contrast
  LSTR MSG_STORE_EEPROM                   = _UxGT("Lưu các thiết lập");                   // Store settings
  LSTR MSG_LOAD_EEPROM                    = _UxGT("Tải các cài đặt");                     // Load settings
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("Khôi phục phòng hư");                  // Restore Defaults
  LSTR MSG_INIT_EEPROM                    = _UxGT("Khởi Tạo EEPROM");                     // Initialize EEPROM
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("Cập Nhật phương tiện");                // Update media
  LSTR MSG_RESET_PRINTER                  = _UxGT("Bặt Lại Máy In");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH  _UxGT("Cập Nhật");           // Refresh
  LSTR MSG_INFO_SCREEN                    = _UxGT("Màn Hình Thông Tin");                  // Info screen
  LSTR MSG_PREPARE                        = _UxGT("Chuẩn bị");                            // Prepare
  LSTR MSG_TUNE                           = _UxGT("Điều Chỉnh");                          // Tune
  LSTR MSG_PAUSE_PRINT                    = _UxGT("Tạm dừng in");                         // Pause print
  LSTR MSG_RESUME_PRINT                   = _UxGT("Tiếp tục in");                         // Resume print
  LSTR MSG_STOP_PRINT                     = _UxGT("Ngừng in");                            // Stop print
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("Phục Hồi Mất Điện");                   // Outage Recovery
  LSTR MSG_MEDIA_MENU                     = _UxGT("In từ phương tiện");                   // Print from media
  LSTR MSG_NO_MEDIA                       = _UxGT("Không có phương tiện");                // No media
  LSTR MSG_DWELL                          = _UxGT("Ngủ...");                              // Sleep
  LSTR MSG_USERWAIT                       = _UxGT("Nhấn để tiếp tục...");                 // Click to resume (same as 'continue')
  LSTR MSG_PRINT_PAUSED                   = _UxGT("In tạm dừng");                         // Print paused
  LSTR MSG_PRINTING                       = _UxGT("Đang in...");                          // Printing
  LSTR MSG_PRINT_ABORTED                  = _UxGT("In đã hủy bỏ");                        // Print aborted
  LSTR MSG_NO_MOVE                        = _UxGT("Không di chuyển.");                    // No move.
  LSTR MSG_KILLED                         = _UxGT("ĐÃ CHẾT. ");
  LSTR MSG_STOPPED                        = _UxGT("ĐÃ NGỪNG. ");
  LSTR MSG_CONTROL_RETRACT                = _UxGT("Rút mm");                              // Retract mm
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("Rút Trao.mm");                         // Swap Re.mm
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("Rút V");
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Nhảy mm");                             // Hop
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("BỏRút mm");                            // Unretr. mm
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("BỏRút T mm");                          // S Unretr. mm
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("BỏRút V");                             // UnRet V
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("BỏRút T V");                           // S UnRet V
  LSTR MSG_AUTORETRACT                    = _UxGT("RútTựĐộng");                           // Auto-Retract
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("Khoảng Cách Rút");                     // Retract Distance
  LSTR MSG_TOOL_CHANGE                    = _UxGT("Thay Đổi Công Cụ");                    // Tool Change
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Đưa Lên Z");                           // Z Raise
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("Tốc Độ Tuôn Ra");                      // Prime Speed
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("Tốc Độ Rút Lại");                      // Retract Speed
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("Thay dây nhựa");                       // Change filament
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("Thay dây nhựa *");                     // Change filament
  LSTR MSG_FILAMENTLOAD                   = _UxGT("Nạp dây nhựa");                        // Load filament
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("Nạp dây nhựa *");                      // Load filament
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("Dỡ dây nhựa");                         // Unload filament
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("Dỡ dây nhựa *");                       // Unload filament
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("Dỡ tất cả");                           // Unload All
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("Khởi tạo thẻ SD");                     // Attach SD Card
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("Khởi tạo thanh USB");                  // Attach USB Drive
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("Khởi tạo phương tiện");                // Attach media
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("Thay phương tiện");                    // Change midea
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("Phát hành phương tiện");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Đầu Dò Z qua bàn");                    // Z Probe past bed
  LSTR MSG_SKEW_FACTOR                    = _UxGT("Hệ số nghiêng");                       // Skew Factor
  LSTR MSG_BLTOUCH                        = _UxGT("BLTOUCH");                             // BLTouch
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("Tự kiểm tra BLTOUCH ");                // BLTouch Self-Test
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("Bặt lại BLTouch");                     // Reset BLTouch
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("Đem BLTouch");                         // Deploy BLTouch
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("Cất BLTouch");                         // Stow BLTouch
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("Đem Đầu Dò-Z");                        // Deploy Z-Probe
  LSTR MSG_MANUAL_STOW                    = _UxGT("Cất Đầu Dò-Z");                        // Stow Z-Probe
  LSTR MSG_HOME_FIRST                     = _UxGT("Về nhà %s Trước");                     // Home %s first
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("Đầu Dò Bù Đắp X");                     // Probe X Offset
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("Đầu Dò Bù Đắp Y");                     // Probe Y Offset
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("Đầu Dò Bù Đắp Z");                     // Probe Z Offset
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("Đầu Dò Bù Đắp @");                     // Probe @ Offset
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("Điều chỉnh Z từng");                   // Babystep Probe Z
  LSTR MSG_BABYSTEP_X                     = _UxGT("Nhít X");                              // Babystep X
  LSTR MSG_BABYSTEP_Y                     = _UxGT("Nhít Y");
  LSTR MSG_BABYSTEP_Z                     = _UxGT("Nhít Z");
  LSTR MSG_BABYSTEP_N                     = _UxGT("Nhít @");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("Hủy bỏ công tắc");                     // Endstop abort
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("Sưởi đầu phun không thành công");      // Heating failed
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("Điều sai: nhiệt độ dư");               // Err: REDUNDANT TEMP
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("Vấn đề nhiệt");                        // THERMAL RUNAWAY | problem
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("Điều sai: nhiệt độ tối đa");           // Err: MAXTEMP
  LSTR MSG_ERR_MINTEMP                    = _UxGT("Điều sai: nhiệt độ tối thiểu");        // Err: MINTEMP
  LSTR MSG_HALTED                         = _UxGT("MÁY IN ĐÃ DỪNG LẠI");                  // PRINTER HALTED
  LSTR MSG_PLEASE_RESET                   = _UxGT("Xin bặt lại");                         // Please reset
  LSTR MSG_HEATING                        = _UxGT("Đang sưởi nóng...");                   // Heating
  LSTR MSG_COOLING                        = _UxGT("Đang làm nguội...");                   // Cooling
  LSTR MSG_BED_HEATING                    = _UxGT("Đang sưởi nong bàn...");               // Bed heating
  LSTR MSG_BED_COOLING                    = _UxGT("Đang làm nguội bàn...");               // Bed cooling
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("Cân Chỉnh Delta");                     // Delta calibration
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("Chỉnh X lại");
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("Chỉnh Y lại");
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("Chỉnh Z lại");
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("Chỉnh Z Center");                      // Calibrate Center
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("Cài Đặt Delta");                       // Delta Settings
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("Cân Chỉnh Tự Động");                   // Auto Calibration
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("Gậy Chéo");                            // Diag Rod
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("Chiều Cao");                           // Height
  LSTR MSG_DELTA_RADIUS                   = _UxGT("Bán Kính");                            // Radius
  LSTR MSG_INFO_MENU                      = _UxGT("Về Máy In");
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("Thông Tin Máy In");                    // Printer Info
  LSTR MSG_3POINT_LEVELING                = _UxGT("San lấp 3-Điểm");                      // 3-Point Leveling
  LSTR MSG_LINEAR_LEVELING                = _UxGT("San Lấp Tuyến Tính");                  // Linear Leveling
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("San Lấp Song Tuyến");                  // Bilinear Leveling
  LSTR MSG_UBL_LEVELING                   = _UxGT("San Lấp Bàn Thống Nhất");              // Unified Bed Leveling
  LSTR MSG_MESH_LEVELING                  = _UxGT("Lưới San Lấp");                        // Mesh Leveling
  LSTR MSG_INFO_STATS_MENU                = _UxGT("Thống Kê Máy In");                     // Printer Stats
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("Thông Tin Bo Mạch");                   // Board Info
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("Điện Trở Nhiệt");                      // Thermistors
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("Máy đùn");                             // Extruders
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("Baud");                                // Baud
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("Giao Thức");                           // Protocol
  LSTR MSG_CASE_LIGHT                     = _UxGT("Đèn Khuông");                          // Case light
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("Độ Sáng");                             // Light Brightness
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("Số In");                               // Print Count
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Đã hoàn thành");
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Tổng số thời gian in");               // Total print time
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Thời gian việc lâu nhất");             // Longest job time
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Tổng số đùn");                         // Extruded total
  #else
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("In");                                  // Prints
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("Đã hoàn thành");                       // Completed
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("Tổng số");                            // Total
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("Dài nhất");                            // Longest
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("Đã ép đùn");
  #endif
  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("Nhiệt độ tối thiểu");                  // Min Temp
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("Nhiệt độ tối đa");                     // Max temp
  LSTR MSG_INFO_PSU                       = _UxGT("Bộ nguồn");                            // PSU
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("Sức mạnh ổ đĩa");                      // Drive Strength
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ % trình điều khiển");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("Ghi DAC EEPROM");                      // DAC EEPROM Write
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("In tạm dừng");                         // PRINT PAUSED
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("Nạp dây nhựa");
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("Dỡ dây nhựa");                         // Unload filament
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("Tùy chọn hồi phục:");                  // RESUME OPTIONS
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("Xả thêm");                             // Purge more
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("Tiếp tục");                            // Continue
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  Đầu Phun: ");                        // Nozzle
  LSTR MSG_RUNOUT_SENSOR_ENABLE           = _UxGT("Cảm Biến Hết");                        // Runout Sensor
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("Sự nhà không thành công");             // Homing failed
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT(" không thành công");                   // Probing failed

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  #if LCD_HEIGHT >= 4
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("Nhấn nút", "để tiếp tục in")); // Press button to resume print
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("Chờ cho sự", "thay đổi dây nhựa", "bắt đầu")); // Wait for filament change to start
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("Đút dây nhựa vào", "và nhấn nút", "để tiếp tục")); // Insert filament and press button to continue                       //
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("Nhấn nút", "để làm nóng đầu phun")); // Press button to heat nozzle
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("Đầu phun đang nóng lên", "Xin chờ...")); // Nozzle heating Please wait
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_2_LINE("Chờ tro", "dây nhựa ra"));  // Wait for filament unload
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_2_LINE("Chờ tro", "dây nhựa vào")); // Wait for filament load
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("Chờ tro", "xả dây nhựa"));  // Wait for filament purge
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_2_LINE("Nhấn nút để kết thúc", "xả dây nhựa")); // Click to finish dây nhựa purge
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_2_LINE("Chờ tro in", "tiếp tục...")); // Wait for print to resume
  #else // LCD_HEIGHT < 4
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_1_LINE("Nhấn nút để tiếp tục"));    // Click to continue
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_1_LINE("Xin chờ..."));
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_1_LINE("Chèn và nhấn"));            // Insert and Click
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_1_LINE("Nhấn để sưởi"));            // Click to heat
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_1_LINE("Đang sưởi nóng"));          // Heating
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_1_LINE("Đang dỡ ra..."));           // Ejecting
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_1_LINE("Đang nạp..."));             // Loading
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_1_LINE("Đang xả..."));              // Purging
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_1_LINE("Nhấn nút để kết thúc"));    // Click to finish
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_1_LINE("Đang tiếp tục..."));        // Resuming
  #endif // LCD_HEIGHT < 4

  LSTR MSG_TMC_DRIVERS                    = _UxGT("Trình điều khiển TMC");                // TMC drivers
  LSTR MSG_TMC_CURRENT                    = _UxGT("Dòng điện trình điều khiển");          // Driver current
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("Ngưỡng Hỗn Hợp");                      // Hybrid threshold
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("Vô cảm biến");                         // Sensorless homing
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("Chế độ từng bước");                    // Stepping mode
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("CắtTàngHình được kích hoạt");          // StealthChop enabled

  LSTR MSG_SHORT_DAY                      = _UxGT("n");                                   // d - ngày - One character only
  LSTR MSG_SHORT_HOUR                     = _UxGT("g");                                   // h - giờ  - One character only
  LSTR MSG_SHORT_MINUTE                   = _UxGT("p");                                   // m - phút - One character only
}

namespace LanguageWide_vi {
  using namespace LanguageNarrow_vi;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
  #endif
}

namespace LanguageTall_vi {
  using namespace LanguageWide_vi;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
  #endif
}

namespace Language_vi {
  using namespace LanguageTall_vi;
}
