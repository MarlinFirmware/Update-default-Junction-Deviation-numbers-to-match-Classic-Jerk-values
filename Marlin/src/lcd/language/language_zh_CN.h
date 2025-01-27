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
 * Simplified Chinese
 *
 * LCD Menu Messages
 * See also https://marlinfw.org/docs/development/lcd_language.html
 */
namespace LanguageNarrow_zh_CN {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE              = 3;
  LSTR LANGUAGE                           = _UxGT("Simplified Chinese");

  LSTR WELCOME_MSG                        = MACHINE_NAME _UxGT("已就绪."); // " ready."
  LSTR MSG_MARLIN                         = _UxGT("马林");
  LSTR MSG_YES                            = _UxGT("是");
  LSTR MSG_NO                             = _UxGT("否");
  LSTR MSG_HIGH                           = _UxGT("高");
  LSTR MSG_LOW                            = _UxGT("低");
  LSTR MSG_BACK                           = _UxGT("返回"); // ”Back“
  LSTR MSG_ERROR                          = _UxGT("错误");
  LSTR MSG_MEDIA_ABORTING                 = _UxGT("存储卡中止...");
  LSTR MSG_MEDIA_INSERTED                 = _UxGT("存储卡已插入"); // "Card inserted"
  LSTR MSG_MEDIA_REMOVED                  = _UxGT("存储卡被拔出"); // "Card removed"
  LSTR MSG_MEDIA_WAITING                  = _UxGT("等待存储器");
  LSTR MSG_MEDIA_READ_ERROR               = _UxGT("卡读卡器错误");
  LSTR MSG_MEDIA_USB_REMOVED              = _UxGT("USB设备已弹出");
  LSTR MSG_MEDIA_USB_FAILED               = _UxGT("USB读取失败");
  LSTR MSG_KILL_SUBCALL_OVERFLOW          = _UxGT("子响应溢出");
  LSTR MSG_LCD_ENDSTOPS                   = _UxGT("挡块"); // "Endstops" // Max length 8 characters
  LSTR MSG_LCD_SOFT_ENDSTOPS              = _UxGT("软挡块");
  LSTR MSG_MAIN_MENU                      = _UxGT("主菜单"); // "Main"
  LSTR MSG_ADVANCED_SETTINGS              = _UxGT("高级设置");
  LSTR MSG_CONFIGURATION                  = _UxGT("配置");
  LSTR MSG_RUN_AUTO_FILES                 = _UxGT("自动开始"); // "Autostart"
  LSTR MSG_DISABLE_STEPPERS               = _UxGT("关闭步进电机"); // "Disable steppers"
  LSTR MSG_DEBUG_MENU                     = _UxGT("调试菜单"); // "Debug Menu"
  LSTR MSG_PROGRESS_BAR_TEST              = _UxGT("进度条测试"); // "Progress Bar Test"
  LSTR MSG_HOMING                         = _UxGT("回原点");
  LSTR MSG_AUTO_HOME                      = _UxGT("自动回原点"); // "Auto home"
  LSTR MSG_AUTO_HOME_X                    = _UxGT("回X原位"); // "Home X"
  LSTR MSG_AUTO_HOME_Y                    = _UxGT("回Y原位"); // "Home Y"
  LSTR MSG_AUTO_HOME_Z                    = _UxGT("回Z原位"); // "Home Z"
  LSTR MSG_AUTO_Z_ALIGN                   = _UxGT("自动Z对齐");
  LSTR MSG_LEVEL_BED_HOMING               = _UxGT("平台调平XYZ归原位"); // "Homing XYZ"
  LSTR MSG_LEVEL_BED_WAITING              = _UxGT("单击开始热床调平"); // "Click to Begin"
  LSTR MSG_LEVEL_BED_NEXT_POINT           = _UxGT("下个热床调平点"); // "Next Point"
  LSTR MSG_LEVEL_BED_DONE                 = _UxGT("完成热床调平"); // "Leveling Done!"
  LSTR MSG_Z_FADE_HEIGHT                  = _UxGT("淡出高度"); // "Fade Height"
  LSTR MSG_SET_HOME_OFFSETS               = _UxGT("设置原点偏移"); // "Set home offsets"
  LSTR MSG_HOME_OFFSETS_APPLIED           = _UxGT("偏移已启用"); // "Offsets applied"

  LSTR MSG_PREHEAT_1                      = _UxGT("预热 ") PREHEAT_1_LABEL; // "Preheat PREHEAT_2_LABEL"
  LSTR MSG_PREHEAT_1_H                    = _UxGT("预热 ") PREHEAT_1_LABEL " ~"; // "Preheat PREHEAT_2_LABEL"
  LSTR MSG_PREHEAT_1_END                  = _UxGT("预热 ") PREHEAT_1_LABEL _UxGT(" 喷嘴"); //MSG_PREHEAT_1 " "
  LSTR MSG_PREHEAT_1_END_E                = _UxGT("预热 ") PREHEAT_1_LABEL _UxGT(" 喷嘴 ~"); //MSG_PREHEAT_1 " "
  LSTR MSG_PREHEAT_1_ALL                  = _UxGT("预热 ") PREHEAT_1_LABEL _UxGT(" 全部"); //MSG_PREHEAT_1 " All"
  LSTR MSG_PREHEAT_1_BEDONLY              = _UxGT("预热 ") PREHEAT_1_LABEL _UxGT(" 热床"); //MSG_PREHEAT_1 " Bed"
  LSTR MSG_PREHEAT_1_SETTINGS             = _UxGT("预热 ") PREHEAT_1_LABEL _UxGT(" 设置"); //MSG_PREHEAT_1 " conf"

  LSTR MSG_PREHEAT_M                      = _UxGT("预热 $"); // "Preheat PREHEAT_2_LABEL"
  LSTR MSG_PREHEAT_M_H                    = _UxGT("预热 $ ~"); // "Preheat PREHEAT_2_LABEL"
  LSTR MSG_PREHEAT_M_END                  = _UxGT("预热 $ 喷嘴"); //MSG_PREHEAT_1 " "
  LSTR MSG_PREHEAT_M_END_E                = _UxGT("预热 $ 喷嘴 ~"); //MSG_PREHEAT_1 " "
  LSTR MSG_PREHEAT_M_ALL                  = _UxGT("预热 $ 全部"); //MSG_PREHEAT_1 " All"
  LSTR MSG_PREHEAT_M_BEDONLY              = _UxGT("预热 $ 热床"); //MSG_PREHEAT_1 " Bed"
  LSTR MSG_PREHEAT_M_SETTINGS             = _UxGT("预热 $ 设置"); //MSG_PREHEAT_1 " conf"

  LSTR MSG_PREHEAT_CUSTOM                 = _UxGT("预热自定义");
  LSTR MSG_COOLDOWN                       = _UxGT("降温"); // "Cooldown"
  LSTR MSG_CUTTER_FREQUENCY               = _UxGT("切割频率");
  LSTR MSG_LASER_MENU                     = _UxGT("激光控制");
  LSTR MSG_LASER_POWER                    = _UxGT("激光电源");
  LSTR MSG_SPINDLE_MENU                   = _UxGT("主轴控制");
  LSTR MSG_SPINDLE_POWER                  = _UxGT("主轴电源");
  LSTR MSG_SPINDLE_REVERSE                = _UxGT("主轴反转");
  LSTR MSG_SWITCH_PS_ON                   = _UxGT("开启电源"); // "Switch power on"
  LSTR MSG_SWITCH_PS_OFF                  = _UxGT("关闭电源"); // "Switch power off"
  LSTR MSG_EXTRUDE                        = _UxGT("挤出"); // "Extrude"
  LSTR MSG_RETRACT                        = _UxGT("回抽"); // "Retract"
  LSTR MSG_MOVE_AXIS                      = _UxGT("移动轴"); // "Move axis"
  LSTR MSG_PROBE_AND_LEVEL                = _UxGT("探测并调平"); // "Probe and Level"
  LSTR MSG_BED_LEVELING                   = _UxGT("调平热床"); // "Bed leveling"
  LSTR MSG_LEVEL_BED                      = _UxGT("调平热床"); // "Level bed"
  LSTR MSG_BED_TRAMMING                   = _UxGT("调平边角"); // "Bed Tramming"
  LSTR MSG_NEXT_CORNER                    = _UxGT("下个边角"); // "Next corner"
  LSTR MSG_MESH_EDITOR                    = _UxGT("网格编辑器");
  LSTR MSG_EDIT_MESH                      = _UxGT("编辑网格"); // "Edit Mesh"
  LSTR MSG_EDITING_STOPPED                = _UxGT("网格编辑已停止"); // "Mesh Editing Stopped"
  LSTR MSG_PROBING_POINT                  = _UxGT("探测点");
  LSTR MSG_MESH_X                         = _UxGT("索引X");
  LSTR MSG_MESH_Y                         = _UxGT("索引Y");
  LSTR MSG_MESH_EDIT_Z                    = _UxGT("Z 值");
  LSTR MSG_CUSTOM_COMMANDS                = _UxGT("定制命令"); // "Custom Commands"
  LSTR MSG_M48_TEST                       = _UxGT("M48探测");
  LSTR MSG_M48_POINT                      = _UxGT("M48点");
  LSTR MSG_M48_OUT_OF_BOUNDS              = _UxGT("探针测试越界");
  LSTR MSG_M48_DEVIATION                  = _UxGT("M48偏差");
  LSTR MSG_IDEX_MENU                      = _UxGT("IDEX模式");
  LSTR MSG_OFFSETS_MENU                   = _UxGT("工具偏移量");
  LSTR MSG_IDEX_MODE_AUTOPARK             = _UxGT("自动停靠");
  LSTR MSG_IDEX_MODE_DUPLICATE            = _UxGT("复制");
  LSTR MSG_IDEX_MODE_MIRRORED_COPY        = _UxGT("镜像复制");
  LSTR MSG_IDEX_MODE_FULL_CTRL            = _UxGT("完全控制");
  LSTR MSG_HOTEND_OFFSET_Z                = _UxGT("第二喷头是Z");
  LSTR MSG_HOTEND_OFFSET_N                = _UxGT("第二喷头是@");
  LSTR MSG_UBL_DOING_G29                  = _UxGT("执行G29"); // "Doing G29"
  LSTR MSG_UBL_TOOLS                      = _UxGT("UBL工具"); // "UBL Tools"
  LSTR MSG_LCD_TILTING_MESH               = _UxGT("倾斜點");
  LSTR MSG_UBL_TILT_MESH                  = _UxGT("倾斜网格");
  LSTR MSG_UBL_TILTING_GRID               = _UxGT("倾斜网格尺寸");
  LSTR MSG_UBL_MESH_TILTED                = _UxGT("网格倾斜");
  LSTR MSG_UBL_MANUAL_MESH                = _UxGT("手工创设网格"); // "Manually Build Mesh"
  LSTR MSG_UBL_MESH_WIZARD                = _UxGT("网格精灵(UBL)");
  LSTR MSG_UBL_BC_INSERT                  = _UxGT("放置垫片并测量"); // "Place shim & measure"
  LSTR MSG_UBL_BC_INSERT2                 = _UxGT("测量"); // "Measure"
  LSTR MSG_UBL_BC_REMOVE                  = _UxGT("移除并测量热床"); // "Remove & measure bed"
  LSTR MSG_UBL_MOVING_TO_NEXT             = _UxGT("移动到下一个"); // "Moving to next"
  LSTR MSG_UBL_SET_TEMP_BED               = _UxGT("热床温度"); // "Bed Temp"
  LSTR MSG_UBL_BED_TEMP_CUSTOM            = _UxGT("热床温度");
  LSTR MSG_UBL_SET_TEMP_HOTEND            = _UxGT("热端温度"); // "Hotend Temp"
  LSTR MSG_UBL_HOTEND_TEMP_CUSTOM         = _UxGT("热端温度");
  LSTR MSG_UBL_EDIT_CUSTOM_MESH           = _UxGT("编辑客户网格"); // "Edit Custom Mesh"
  LSTR MSG_UBL_FINE_TUNE_MESH             = _UxGT("细调网格"); // "Fine Tuning Mesh"
  LSTR MSG_UBL_DONE_EDITING_MESH          = _UxGT("完成编辑网格"); // "Done Editing Mesh"
  LSTR MSG_UBL_BUILD_CUSTOM_MESH          = _UxGT("创设客户网格"); // "Build Custom Mesh"
  LSTR MSG_UBL_BUILD_MESH_MENU            = _UxGT("创设网格"); // "Build Mesh"
  LSTR MSG_UBL_BUILD_MESH_M               = _UxGT("创设 $ 网格"); // "Build PREHEAT_1_LABEL Mesh"
  LSTR MSG_UBL_VALIDATE_MESH_M            = _UxGT("批准 $ 网格"); // "Validate PREHEAT_1_LABEL Mesh"
  LSTR MSG_UBL_BUILD_COLD_MESH            = _UxGT("创设冷网格"); // "Build Cold Mesh"
  LSTR MSG_UBL_MESH_HEIGHT_ADJUST         = _UxGT("调整网格高度"); // "Adjust Mesh Height"
  LSTR MSG_UBL_MESH_HEIGHT_AMOUNT         = _UxGT("高度合计"); // "Height Amount"
  LSTR MSG_UBL_VALIDATE_MESH_MENU         = _UxGT("批准网格"); // "Validate Mesh"
  LSTR MSG_UBL_VALIDATE_CUSTOM_MESH       = _UxGT("批准客户网格"); // "Validate Custom Mesh"
  LSTR MSG_G26_HEATING_BED                = _UxGT("G26加热热床");
  LSTR MSG_G26_HEATING_NOZZLE             = _UxGT("G26加热喷嘴");
  LSTR MSG_G26_MANUAL_PRIME               = _UxGT("手动填装中...");
  LSTR MSG_G26_FIXED_LENGTH               = _UxGT("固定距离填装");
  LSTR MSG_G26_PRIME_DONE                 = _UxGT("填装完成");
  LSTR MSG_G26_CANCELED                   = _UxGT("G26已取消");
  LSTR MSG_G26_LEAVING                    = _UxGT("离开G26");
  LSTR MSG_UBL_CONTINUE_MESH              = _UxGT("继续热床网格"); // "Continue Bed Mesh"
  LSTR MSG_UBL_MESH_LEVELING              = _UxGT("网格调平"); // "Mesh Leveling"
  LSTR MSG_UBL_3POINT_MESH_LEVELING       = _UxGT("三点调平"); // "3-Point Leveling"
  LSTR MSG_UBL_GRID_MESH_LEVELING         = _UxGT("格子网格调平"); // "Grid Mesh Leveling"
  LSTR MSG_UBL_MESH_LEVEL                 = _UxGT("调平网格"); // "Level Mesh"
  LSTR MSG_UBL_SIDE_POINTS                = _UxGT("边点"); // "Side Points"
  LSTR MSG_UBL_MAP_TYPE                   = _UxGT("图类型"); // "Map Type"
  LSTR MSG_UBL_OUTPUT_MAP                 = _UxGT("输出网格图"); // "Output Mesh Map"
  LSTR MSG_UBL_OUTPUT_MAP_HOST            = _UxGT("输出到主机"); // "Output for Host"
  LSTR MSG_UBL_OUTPUT_MAP_CSV             = _UxGT("输出到CSV"); // "Output for CSV"
  LSTR MSG_UBL_OUTPUT_MAP_BACKUP          = _UxGT("输出到备份"); // "Off Printer Backup"
  LSTR MSG_UBL_INFO_UBL                   = _UxGT("输出UBL信息"); // "Output UBL Info"
  LSTR MSG_UBL_FILLIN_AMOUNT              = _UxGT("填充合计"); // "Fill-in Amount"
  LSTR MSG_UBL_MANUAL_FILLIN              = _UxGT("手工填充"); // "Manual Fill-in"
  LSTR MSG_UBL_SMART_FILLIN               = _UxGT("聪明填充"); // "Smart Fill-in"
  LSTR MSG_UBL_FILLIN_MESH                = _UxGT("填充网格"); // "Fill-in Mesh"
  LSTR MSG_UBL_INVALIDATE_ALL             = _UxGT("作废所有的"); // "Invalidate All"
  LSTR MSG_UBL_INVALIDATE_CLOSEST         = _UxGT("作废最近的"); // "Invalidate Closest"
  LSTR MSG_UBL_FINE_TUNE_ALL              = _UxGT("细调所有的"); // "Fine Tune All"
  LSTR MSG_UBL_FINE_TUNE_CLOSEST          = _UxGT("细调最近的"); // "Fine Tune Closest"
  LSTR MSG_UBL_STORAGE_MESH_MENU          = _UxGT("网格存储"); // "Mesh Storage"
  LSTR MSG_UBL_STORAGE_SLOT               = _UxGT("存储槽"); // "Memory Slot"
  LSTR MSG_UBL_LOAD_MESH                  = _UxGT("装载热床网格"); // "Load Bed Mesh"
  LSTR MSG_UBL_SAVE_MESH                  = _UxGT("保存热床网格"); // "Save Bed Mesh"
  LSTR MSG_UBL_INVALID_SLOT               = _UxGT("首先选择一个网格槽");
  LSTR MSG_MESH_LOADED                    = _UxGT("网格 %i 已装载"); // "Mesh %i loaded"
  LSTR MSG_MESH_SAVED                     = _UxGT("网格 %i 已保存"); // "Mesh %i saved"
  LSTR MSG_MESH_ACTIVE                    = _UxGT("网格 %i 激活");
  LSTR MSG_UBL_NO_STORAGE                 = _UxGT("没有存储"); // "No storage"
  LSTR MSG_UBL_SAVE_ERROR                 = _UxGT("错误: UBL保存"); // "Err: UBL Save"
  LSTR MSG_UBL_RESTORE_ERROR              = _UxGT("错误: UBL还原"); // "Err: UBL Restore"
  LSTR MSG_UBL_Z_OFFSET                   = _UxGT("Z偏移量: ");
  LSTR MSG_UBL_Z_OFFSET_STOPPED           = _UxGT("Z偏移已停止"); // "Z-Offset Stopped"
  LSTR MSG_UBL_STEP_BY_STEP_MENU          = _UxGT("一步步UBL"); // "Step-By-Step UBL"
  LSTR MSG_UBL_1_BUILD_COLD_MESH          = _UxGT("1. 创设冷网格");
  LSTR MSG_UBL_2_SMART_FILLIN             = _UxGT("2. 聪明填充");
  LSTR MSG_UBL_3_VALIDATE_MESH_MENU       = _UxGT("3. 批准网格");
  LSTR MSG_UBL_4_FINE_TUNE_ALL            = _UxGT("4. 细调所有的");
  LSTR MSG_UBL_5_VALIDATE_MESH_MENU       = _UxGT("5. 批准网格");
  LSTR MSG_UBL_6_FINE_TUNE_ALL            = _UxGT("6. 细调所有的");
  LSTR MSG_UBL_7_SAVE_MESH                = _UxGT("7. 保存热床网格");

  LSTR MSG_LED_CONTROL                    = _UxGT("LED控制"); // "LED Control"
  LSTR MSG_LIGHTS                         = _UxGT("灯"); // "Lights"
  LSTR MSG_LIGHT_N                        = _UxGT("灯 #{"); // "Light #{"
  LSTR MSG_LED_PRESETS                    = _UxGT("灯预置"); // "Light Presets"
  LSTR MSG_SET_LEDS_RED                   = _UxGT("红"); // "Red"
  LSTR MSG_SET_LEDS_ORANGE                = _UxGT("橙"); // "Orange"
  LSTR MSG_SET_LEDS_YELLOW                = _UxGT("黄"); // "Yellow"
  LSTR MSG_SET_LEDS_GREEN                 = _UxGT("绿"); // "Green"
  LSTR MSG_SET_LEDS_BLUE                  = _UxGT("蓝"); // "Blue"
  LSTR MSG_SET_LEDS_INDIGO                = _UxGT("青"); // "Indigo"
  LSTR MSG_SET_LEDS_VIOLET                = _UxGT("紫"); // "Violet"
  LSTR MSG_SET_LEDS_WHITE                 = _UxGT("白"); // "White"
  LSTR MSG_SET_LEDS_DEFAULT               = _UxGT("缺省"); // "Default"
  LSTR MSG_CUSTOM_LEDS                    = _UxGT("定制灯"); // "Custom Lights"
  LSTR MSG_INTENSITY_R                    = _UxGT("红饱和度"); // "Red Intensity"
  LSTR MSG_INTENSITY_G                    = _UxGT("绿饱和度"); // "Green Intensity"
  LSTR MSG_INTENSITY_B                    = _UxGT("蓝饱和度"); // "Blue Intensity"
  LSTR MSG_INTENSITY_W                    = _UxGT("白饱和度"); // "White Intensity"
  LSTR MSG_LED_BRIGHTNESS                 = _UxGT("亮度"); // "Brightness"

  LSTR MSG_MOVING                         = _UxGT("移动..."); // "Moving..."
  LSTR MSG_FREE_XY                        = _UxGT("释放 XY"); // "Free XY"
  LSTR MSG_MOVE_X                         = _UxGT("移动X"); // "Move X"
  LSTR MSG_MOVE_Y                         = _UxGT("移动Y"); // "Move Y"
  LSTR MSG_MOVE_Z                         = _UxGT("移动Z"); // "Move Z"
  LSTR MSG_MOVE_N                         = _UxGT("移动@"); // "Move @"
  LSTR MSG_MOVE_E                         = _UxGT("挤出机"); // "Extruder"
  LSTR MSG_MOVE_EN                        = _UxGT("挤出机 *"); // "Extruder"
  LSTR MSG_HOTEND_TOO_COLD                = _UxGT("热端太冷");
  LSTR MSG_MOVE_N_MM                      = _UxGT("移动 $mm"); // "Move $mm"
  LSTR MSG_MOVE_N_IN                      = _UxGT("移动 $in"); // "Move $in"
  LSTR MSG_MOVE_N_DEG                     = _UxGT("移动 $") LCD_STR_DEGREE; // "Move $°"
  LSTR MSG_SPEED                          = _UxGT("速率"); // "Speed"
  LSTR MSG_MESH_Z_OFFSET                  = _UxGT("热床Z"); // "Bed Z"
  LSTR MSG_NOZZLE                         = _UxGT("喷嘴"); // "Nozzle" 噴嘴
  LSTR MSG_NOZZLE_N                       = _UxGT("喷嘴 ~"); // "Nozzle" 噴嘴
  LSTR MSG_NOZZLE_PARKED                  = _UxGT("喷嘴已停靠");
  LSTR MSG_NOZZLE_STANDBY                 = _UxGT("喷嘴待命中");
  LSTR MSG_BED                            = _UxGT("热床"); // "Bed"
  LSTR MSG_CHAMBER                        = _UxGT("机箱壳");
  LSTR MSG_FAN_SPEED                      = _UxGT("风扇速率"); // "Fan speed"
  LSTR MSG_FAN_SPEED_N                    = _UxGT("风扇速率 ~"); // "Fan speed"
  LSTR MSG_STORED_FAN_N                   = _UxGT("存储的风扇 ~");
  LSTR MSG_EXTRA_FAN_SPEED                = _UxGT("额外风扇速率"); // "Extra fan speed"
  LSTR MSG_EXTRA_FAN_SPEED_N              = _UxGT("额外风扇速率 ~"); // "Extra fan speed"
  LSTR MSG_CONTROLLER_FAN                 = _UxGT("控制器风扇");
  LSTR MSG_CONTROLLER_FAN_IDLE_SPEED      = _UxGT("空闲速度");
  LSTR MSG_CONTROLLER_FAN_AUTO_ON         = _UxGT("自动模式");
  LSTR MSG_CONTROLLER_FAN_SPEED           = _UxGT("工作速度");
  LSTR MSG_CONTROLLER_FAN_DURATION        = _UxGT("空闲周期");
  LSTR MSG_FLOW                           = _UxGT("挤出速率"); // "Flow"
  LSTR MSG_FLOW_N                         = _UxGT("挤出速率 ~"); // "Flow"
  LSTR MSG_CONTROL                        = _UxGT("控制"); // "Control"
  LSTR MSG_MIN                            = " " LCD_STR_THERMOMETER _UxGT(" 最小"); // " " LCD_STR_THERMOMETER " Min"
  LSTR MSG_MAX                            = " " LCD_STR_THERMOMETER _UxGT(" 最大"); // " " LCD_STR_THERMOMETER " Max"
  LSTR MSG_FACTOR                         = " " LCD_STR_THERMOMETER _UxGT(" 因数"); // " " LCD_STR_THERMOMETER " Fact"
  LSTR MSG_AUTOTEMP                       = _UxGT("自动控温"); // "Autotemp"
  LSTR MSG_LCD_ON                         = _UxGT("开"); // "On"
  LSTR MSG_LCD_OFF                        = _UxGT("关"); // "Off"
  LSTR MSG_PID_AUTOTUNE                   = _UxGT("自动PID");
  LSTR MSG_PID_AUTOTUNE_E                 = _UxGT("自动PID *");
  LSTR MSG_PID_AUTOTUNE_DONE              = _UxGT("PID调整完成");
  LSTR MSG_PID_BAD_HEATER_ID              = _UxGT("自动调失败! 坏的挤出机");
  LSTR MSG_PID_TEMP_TOO_HIGH              = _UxGT("自动调失败! 温度太高");
  LSTR MSG_PID_TIMEOUT                    = _UxGT("自动调失败! 超时");
  LSTR MSG_SELECT_E                       = _UxGT("选择 *");
  LSTR MSG_ACC                            = _UxGT("加速度"); // "Accel" acceleration
  LSTR MSG_JERK                           = _UxGT("抖动速率"); // "Jerk"
  LSTR MSG_VA_JERK                        = _UxGT("轴抖动速率") STR_A; // "Va-jerk"
  LSTR MSG_VB_JERK                        = _UxGT("轴抖动速率") STR_B; // "Vb-jerk"
  LSTR MSG_VC_JERK                        = _UxGT("轴抖动速率") STR_C; // "Vc-jerk"
  LSTR MSG_VN_JERK                        = _UxGT("轴抖动速率@"); // "V@-jerk"
  LSTR MSG_VE_JERK                        = _UxGT("挤出机抖动速率"); // "Ve-jerk"
  LSTR MSG_JUNCTION_DEVIATION             = _UxGT("接点差");
  LSTR MSG_MAX_SPEED                      = _UxGT("速度"); // "Velocity"
  LSTR MSG_VMAX_A                         = _UxGT("最大进料速率") STR_A; // "Vmax " max_feedrate_mm_s
  LSTR MSG_VMAX_B                         = _UxGT("最大进料速率") STR_B;
  LSTR MSG_VMAX_C                         = _UxGT("最大进料速率") STR_C;
  LSTR MSG_VMAX_N                         = _UxGT("最大进料速率@");
  LSTR MSG_VMAX_E                         = _UxGT("最大进料速率E");
  LSTR MSG_VMAX_EN                        = _UxGT("最大进料速率 *");
  LSTR MSG_VMIN                           = _UxGT("最小进料速率"); // "Vmin"  min_feedrate_mm_s
  LSTR MSG_VTRAV_MIN                      = _UxGT("最小移动速率"); // "VTrav min" min_travel_feedrate_mm_s, (target) speed of the move
  LSTR MSG_ACCELERATION                   = _UxGT("加速度"); // "Acceleration"
  LSTR MSG_AMAX_A                         = _UxGT("最大打印加速度") STR_A; // "Amax " max_acceleration_mm_per_s2, acceleration in units/s^2 for print moves
  LSTR MSG_AMAX_B                         = _UxGT("最大打印加速度") STR_B;
  LSTR MSG_AMAX_C                         = _UxGT("最大打印加速度") STR_C;
  LSTR MSG_AMAX_N                         = _UxGT("最大打印加速度@");
  LSTR MSG_AMAX_E                         = _UxGT("最大打印加速度E");
  LSTR MSG_AMAX_EN                        = _UxGT("最大打印加速度 *");
  LSTR MSG_A_RETRACT                      = _UxGT("收进加速度"); // "A-retract" retract_acceleration, E acceleration in mm/s^2 for retracts
  LSTR MSG_A_TRAVEL                       = _UxGT("非打印移动加速度"); // "A-travel" travel_acceleration, X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
  LSTR MSG_XY_FREQUENCY_LIMIT             = _UxGT("频率最大");
  LSTR MSG_XY_FREQUENCY_FEEDRATE          = _UxGT("进给速度");
  LSTR MSG_STEPS_PER_MM                   = _UxGT("轴步数/mm"); // "Steps/mm" axis_steps_per_mm, axis steps-per-unit G92
  LSTR MSG_A_STEPS                        = STR_A _UxGT(" 步数/mm"); // "Asteps/mm"
  LSTR MSG_B_STEPS                        = STR_B _UxGT(" 步数/mm");
  LSTR MSG_C_STEPS                        = STR_C _UxGT(" 步数/mm");
  LSTR MSG_N_STEPS                        = _UxGT("@ 步数/mm");
  LSTR MSG_E_STEPS                        = _UxGT("E 步数/mm"); // "Esteps/mm"
  LSTR MSG_EN_STEPS                       = _UxGT("* 步数/mm");
  LSTR MSG_TEMPERATURE                    = _UxGT("温度"); // "Temperature"
  LSTR MSG_MOTION                         = _UxGT("运动"); // "Motion"
  LSTR MSG_FILAMENT                       = _UxGT("料丝"); // "Filament" menu_advanced_filament
  LSTR MSG_VOLUMETRIC_ENABLED             = _UxGT("E 在 mm") SUPERSCRIPT_THREE; // "E in mm3" volumetric_enabled
  LSTR MSG_VOLUMETRIC_LIMIT               = _UxGT("E 限制 在 mm") SUPERSCRIPT_THREE;
  LSTR MSG_VOLUMETRIC_LIMIT_E             = _UxGT("E 限制 *");
  LSTR MSG_FILAMENT_DIAM                  = _UxGT("丝料直径"); // "Fil. Dia."
  LSTR MSG_FILAMENT_DIAM_E                = _UxGT("丝料直径 *");
  LSTR MSG_FILAMENT_UNLOAD                = _UxGT("卸载 mm"); // "Unload mm"
  LSTR MSG_FILAMENT_LOAD                  = _UxGT("装载 mm"); // "Load mm"
  LSTR MSG_ADVANCE_K                      = _UxGT("Advance K");
  LSTR MSG_ADVANCE_K_E                    = _UxGT("Advance K *");
  LSTR MSG_CONTRAST                       = _UxGT("LCD对比度"); // "LCD contrast"
  LSTR MSG_STORE_EEPROM                   = _UxGT("保存设置"); // "Store memory"
  LSTR MSG_LOAD_EEPROM                    = _UxGT("装载设置"); // "Load memory"
  LSTR MSG_RESTORE_DEFAULTS               = _UxGT("恢复安全值"); // "Restore Defaults"
  LSTR MSG_INIT_EEPROM                    = _UxGT("初始化设置"); // "Initialize EEPROM"
  LSTR MSG_ERR_EEPROM_CRC                 = _UxGT("EEPROM 校验和 错误");
  LSTR MSG_ERR_EEPROM_SIZE                = _UxGT("EEPROM 尺寸 错误");
  LSTR MSG_ERR_EEPROM_VERSION             = _UxGT("EEPROM 版本 错误");
  LSTR MSG_SETTINGS_STORED                = _UxGT("设置已保存");
  LSTR MSG_MEDIA_UPDATE                   = _UxGT("存储器更新");
  LSTR MSG_RESET_PRINTER                  = _UxGT("复位打印机");
  LSTR MSG_REFRESH                        = LCD_STR_REFRESH _UxGT("刷新");
  LSTR MSG_INFO_SCREEN                    = _UxGT("信息屏"); // "Info screen"
  LSTR MSG_PREPARE                        = _UxGT("准备"); // "Prepare"
  LSTR MSG_TUNE                           = _UxGT("调整"); // "Tune"
  LSTR MSG_POWER_MONITOR                  = _UxGT("电源监控");
  LSTR MSG_CURRENT                        = _UxGT("电流");
  LSTR MSG_VOLTAGE                        = _UxGT("电压");
  LSTR MSG_POWER                          = _UxGT("功率");
  LSTR MSG_START_PRINT                    = _UxGT("开始打印");
  LSTR MSG_BUTTON_NEXT                    = _UxGT("下一个");
  LSTR MSG_BUTTON_INIT                    = _UxGT("初始");
  LSTR MSG_BUTTON_STOP                    = _UxGT("停止");
  LSTR MSG_BUTTON_PRINT                   = _UxGT("打印");
  LSTR MSG_BUTTON_RESET                   = _UxGT("复位");
  LSTR MSG_BUTTON_IGNORE                  = _UxGT("忽略");
  LSTR MSG_BUTTON_CANCEL                  = _UxGT("取消");
  LSTR MSG_BUTTON_DONE                    = _UxGT("完成");
  LSTR MSG_BUTTON_BACK                    = _UxGT("返回");
  LSTR MSG_BUTTON_PROCEED                 = _UxGT("继续");
  LSTR MSG_PAUSING                        = _UxGT("暂停中...");
  LSTR MSG_PAUSE_PRINT                    = _UxGT("暂停打印"); // "Pause print"
  LSTR MSG_RESUME_PRINT                   = _UxGT("恢复打印"); // "Resume print"
  LSTR MSG_STOP_PRINT                     = _UxGT("停止打印"); // "Stop print"
  LSTR MSG_PRINTING_OBJECT                = _UxGT("打印物体");
  LSTR MSG_CANCEL_OBJECT                  = _UxGT("取消物体");
  LSTR MSG_CANCEL_OBJECT_N                = _UxGT("取消物体 {");
  LSTR MSG_OUTAGE_RECOVERY                = _UxGT("中断恢复");
  LSTR MSG_MEDIA_MENU                     = _UxGT("从存储卡上打印"); // "Print from SD"
  LSTR MSG_NO_MEDIA                       = _UxGT("无存储卡"); // "No SD card"
  LSTR MSG_DWELL                          = _UxGT("休眠中 ..."); // "Sleep..."
  LSTR MSG_USERWAIT                       = _UxGT("点击继续 ..."); // "Click to resume..."
  LSTR MSG_PRINT_PAUSED                   = _UxGT("暫停打印"); // "Print paused"
  LSTR MSG_PRINTING                       = _UxGT("打印中...");
  LSTR MSG_PRINT_ABORTED                  = _UxGT("已取消打印"); // "Print aborted"
  LSTR MSG_PRINT_DONE                     = _UxGT("打印已完成");
  LSTR MSG_NO_MOVE                        = _UxGT("无移动"); // "No move."
  LSTR MSG_KILLED                         = _UxGT("已杀掉"); // "KILLED. "
  LSTR MSG_STOPPED                        = _UxGT("已停止"); // "STOPPED. "
  LSTR MSG_CONTROL_RETRACT                = _UxGT("回抽长度mm"); // "Retract mm" retract_length, retract length (positive mm)
  LSTR MSG_CONTROL_RETRACT_SWAP           = _UxGT("换手回抽长度mm"); // "Swap Re.mm" swap_retract_length, swap retract length (positive mm), for extruder change
  LSTR MSG_CONTROL_RETRACTF               = _UxGT("回抽速率mm/s"); // "Retract V" retract_feedrate_mm_s, feedrate for retracting (mm/s)
  LSTR MSG_CONTROL_RETRACT_ZHOP           = _UxGT("Hop mm"); // "Hop mm" retract_zraise, retract Z-lift
  LSTR MSG_CONTROL_RETRACT_RECOVER        = _UxGT("回抽恢复长度mm"); // "UnRet +mm" retract_recover_extra, additional recover length (mm, added to retract length when recovering)
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAP   = _UxGT("换手回抽恢复长度mm"); // "S UnRet+mm" swap_retract_recover_extra, additional swap recover length (mm, added to retract length when recovering from extruder change)
  LSTR MSG_CONTROL_RETRACT_RECOVERF       = _UxGT("回抽恢复后进料速率mm/s"); // "Unretract V" retract_recover_feedrate_mm_s, feedrate for recovering from retraction (mm/s)
  LSTR MSG_CONTROL_RETRACT_RECOVER_SWAPF  = _UxGT("S UnRet V"); // "S UnRet V"
  LSTR MSG_AUTORETRACT                    = _UxGT("自动抽回"); // "Auto-Retract" autoretract_enabled,
  LSTR MSG_FILAMENT_SWAP_LENGTH           = _UxGT("交换长度");
  LSTR MSG_FILAMENT_SWAP_EXTRA            = _UxGT("额外的交换");
  LSTR MSG_FILAMENT_PURGE_LENGTH          = _UxGT("清洗长度");
  LSTR MSG_TOOL_CHANGE                    = _UxGT("换工具");
  LSTR MSG_TOOL_CHANGE_ZLIFT              = _UxGT("Z抬起");
  LSTR MSG_SINGLENOZZLE_PRIME_SPEED       = _UxGT("进给速度");
  LSTR MSG_SINGLENOZZLE_RETRACT_SPEED     = _UxGT("回抽速度");
  LSTR MSG_FILAMENT_PARK_ENABLED          = _UxGT("停靠头");
  LSTR MSG_SINGLENOZZLE_UNRETRACT_SPEED   = _UxGT("恢复速度");
  LSTR MSG_SINGLENOZZLE_FAN_SPEED         = _UxGT("风扇速度");
  LSTR MSG_SINGLENOZZLE_FAN_TIME          = _UxGT("风扇时间");
  LSTR MSG_TOOL_MIGRATION_ON              = _UxGT("自动开");
  LSTR MSG_TOOL_MIGRATION_OFF             = _UxGT("自动关");
  LSTR MSG_TOOL_MIGRATION                 = _UxGT("工具迁移");
  LSTR MSG_TOOL_MIGRATION_AUTO            = _UxGT("自动迁移");
  LSTR MSG_TOOL_MIGRATION_END             = _UxGT("上一个挤出机");
  LSTR MSG_TOOL_MIGRATION_SWAP            = _UxGT("迁移至 *");
  LSTR MSG_FILAMENTCHANGE                 = _UxGT("更换丝料"); // "Change filament"
  LSTR MSG_FILAMENTCHANGE_E               = _UxGT("更换丝料 *"); // "Change filament"
  LSTR MSG_FILAMENTLOAD                   = _UxGT("装载丝料"); // "Load filament"
  LSTR MSG_FILAMENTLOAD_E                 = _UxGT("装载丝料 *"); // "Load filament"
  LSTR MSG_FILAMENTUNLOAD                 = _UxGT("卸载丝料"); // "Unload filament"
  LSTR MSG_FILAMENTUNLOAD_E               = _UxGT("卸载丝料 *"); // "Unload filament"
  LSTR MSG_FILAMENTUNLOAD_ALL             = _UxGT("卸载全部"); // "Unload All"
  #if HAS_MULTI_VOLUME
    LSTR MSG_ATTACH_SD_MEDIA              = _UxGT("挂载存储卡"); // "Attach SD Card"
    LSTR MSG_ATTACH_USB_MEDIA             = _UxGT("挂载U盘"); // "Attach USB Drive"
  #else
    LSTR MSG_ATTACH_MEDIA                 = _UxGT("挂载存储卡"); // "Attach SD Card"
  #endif
  LSTR MSG_CHANGE_MEDIA                   = _UxGT("更换存储卡"); // "Change SD card"
  LSTR MSG_RELEASE_MEDIA                  = _UxGT("释放存储卡");
  LSTR MSG_ZPROBE_OUT                     = _UxGT("Z探针在热床之外"); // "Z probe out. bed" Z probe is not within the physical limits
  LSTR MSG_SKEW_FACTOR                    = _UxGT("偏斜因数"); // "Skew Factor"
  LSTR MSG_BLTOUCH                        = _UxGT("BLTouch"); // "BLTouch"
  LSTR MSG_BLTOUCH_SELFTEST               = _UxGT("自检");
  LSTR MSG_BLTOUCH_RESET                  = _UxGT("重置");
  LSTR MSG_BLTOUCH_STOW                   = _UxGT("装载");
  LSTR MSG_BLTOUCH_DEPLOY                 = _UxGT("部署");
  LSTR MSG_BLTOUCH_SW_MODE                = _UxGT("SW模式");
  LSTR MSG_BLTOUCH_5V_MODE                = _UxGT("5V模式");
  LSTR MSG_BLTOUCH_OD_MODE                = _UxGT("OD模式");
  LSTR MSG_BLTOUCH_MODE_STORE             = _UxGT("模式保存");
  LSTR MSG_BLTOUCH_MODE_STORE_5V          = _UxGT("设置BLTouch为5V");
  LSTR MSG_BLTOUCH_MODE_STORE_OD          = _UxGT("设置BLTouch为OD");
  LSTR MSG_BLTOUCH_MODE_ECHO              = _UxGT("报告开漏");
  LSTR MSG_BLTOUCH_MODE_CHANGE            = _UxGT("危险: 错误的设置将引起损坏! 是否继续?");
  LSTR MSG_TOUCHMI_PROBE                  = _UxGT("TouchMI");
  LSTR MSG_TOUCHMI_INIT                   = _UxGT("初始化TouchMI");
  LSTR MSG_TOUCHMI_ZTEST                  = _UxGT("Z偏移量测试");
  LSTR MSG_TOUCHMI_SAVE                   = _UxGT("保存");
  LSTR MSG_MANUAL_DEPLOY_TOUCHMI          = _UxGT("部署TouchMI");
  LSTR MSG_MANUAL_DEPLOY                  = _UxGT("部署Z探针");
  LSTR MSG_MANUAL_STOW                    = _UxGT("收好Z探针");
  LSTR MSG_HOME_FIRST                     = _UxGT("归位 %s 先"); // "Home ... first"
  LSTR MSG_ZPROBE_OFFSETS                 = _UxGT("探针偏移量");
  LSTR MSG_ZPROBE_XOFFSET                 = _UxGT("探针X偏移"); // "X Offset"
  LSTR MSG_ZPROBE_YOFFSET                 = _UxGT("探针Y偏移"); // "Y Offset"
  LSTR MSG_ZPROBE_ZOFFSET                 = _UxGT("探针Z偏移"); // "Z Offset"
  LSTR MSG_ZPROBE_OFFSET_N                = _UxGT("探针@偏移"); // "@ Offset"
  LSTR MSG_BABYSTEP_PROBE_Z               = _UxGT("逐步调整Z"); // "Babystep Probe Z"
  LSTR MSG_BABYSTEP_X                     = _UxGT("微量调整X轴"); // "Babystep X" lcd_babystep_x, Babystepping enables the user to control the axis in tiny amounts
  LSTR MSG_BABYSTEP_Y                     = _UxGT("微量调整Y轴"); // "Babystep Y"
  LSTR MSG_BABYSTEP_Z                     = _UxGT("微量调整Z轴"); // "Babystep Z"
  LSTR MSG_BABYSTEP_TOTAL                 = _UxGT("总计");
  LSTR MSG_ENDSTOP_ABORT                  = _UxGT("挡块终止"); // "Endstop abort"
  LSTR MSG_ERR_HEATING_FAILED             = _UxGT("加热失败"); // "Heating failed"
  LSTR MSG_ERR_REDUNDANT_TEMP             = _UxGT("错误：冗余温度"); // "Err: REDUNDANT TEMP"
  LSTR MSG_ERR_THERMAL_RUNAWAY            = _UxGT("温控失控"); // "THERMAL RUNAWAY"
  LSTR MSG_ERR_MAXTEMP                    = _UxGT("错误：最高温度"); // "Err: MAXTEMP"
  LSTR MSG_ERR_MINTEMP                    = _UxGT("错误：最低温度"); // "Err: MINTEMP"
  LSTR MSG_HALTED                         = _UxGT("打印停机"); // "PRINTER HALTED"
  LSTR MSG_PLEASE_RESET                   = _UxGT("请重置"); // "Please reset"
  LSTR MSG_HEATING                        = _UxGT("加热中 ..."); // "Heating..."
  LSTR MSG_COOLING                        = _UxGT("冷却中 ...");
  LSTR MSG_BED_HEATING                    = _UxGT("加热热床中 ..."); // "Bed Heating..."
  LSTR MSG_BED_COOLING                    = _UxGT("热床冷却中 ...");
  LSTR MSG_CHAMBER_HEATING                = _UxGT("机箱加热中 ...");
  LSTR MSG_CHAMBER_COOLING                = _UxGT("机箱冷却中 ...");
  LSTR MSG_DELTA_CALIBRATE                = _UxGT("⊿校准"); // "Delta Calibration"
  LSTR MSG_DELTA_CALIBRATE_X              = _UxGT("⊿校准X"); // "Calibrate X"
  LSTR MSG_DELTA_CALIBRATE_Y              = _UxGT("⊿校准Y"); // "Calibrate Y"
  LSTR MSG_DELTA_CALIBRATE_Z              = _UxGT("⊿校准Z"); // "Calibrate Z"
  LSTR MSG_DELTA_CALIBRATE_CENTER         = _UxGT("⊿校准中心"); // "Calibrate Center"
  LSTR MSG_DELTA_SETTINGS                 = _UxGT("⊿设置"); // "Delta Settings"
  LSTR MSG_DELTA_AUTO_CALIBRATE           = _UxGT("⊿自动校准"); // "Auto Calibration"
  LSTR MSG_DELTA_DIAG_ROD                 = _UxGT("⊿斜柱"); // "Diag Rod"
  LSTR MSG_DELTA_HEIGHT                   = _UxGT("⊿高度"); // "Height"
  LSTR MSG_DELTA_RADIUS                   = _UxGT("⊿半径"); // "Radius"
  LSTR MSG_INFO_MENU                      = _UxGT("关于打印机"); // "About Printer"
  LSTR MSG_INFO_PRINTER_MENU              = _UxGT("打印机信息"); // "Printer Info"
  LSTR MSG_3POINT_LEVELING                = _UxGT("三点调平"); // "3-Point Leveling"
  LSTR MSG_LINEAR_LEVELING                = _UxGT("线性调平"); // "Linear Leveling"
  LSTR MSG_BILINEAR_LEVELING              = _UxGT("双线性调平"); // "Bilinear Leveling"
  LSTR MSG_UBL_LEVELING                   = _UxGT("统一热床调平(UBL)"); // "Unified Bed Leveling"
  LSTR MSG_MESH_LEVELING                  = _UxGT("网格调平"); // "Mesh Leveling"
  LSTR MSG_INFO_STATS_MENU                = _UxGT("打印机统计"); // "Printer Stats"
  LSTR MSG_INFO_BOARD_MENU                = _UxGT("主板信息"); // "Board Info"
  LSTR MSG_INFO_THERMISTOR_MENU           = _UxGT("温度计"); // "Thermistors"
  LSTR MSG_INFO_EXTRUDERS                 = _UxGT("挤出机"); // "Extruders"
  LSTR MSG_INFO_BAUDRATE                  = _UxGT("波特率"); // "Baud"
  LSTR MSG_INFO_PROTOCOL                  = _UxGT("协议"); // "Protocol"
  LSTR MSG_INFO_RUNAWAY_OFF               = _UxGT("监控温度失控:关");
  LSTR MSG_INFO_RUNAWAY_ON                = _UxGT("监控温度失控:开");
  LSTR MSG_HOTEND_IDLE_TIMEOUT            = _UxGT("热端空闲超时");

  LSTR MSG_CASE_LIGHT                     = _UxGT("外壳灯"); // "Case light"
  LSTR MSG_CASE_LIGHT_BRIGHTNESS          = _UxGT("灯亮度"); // "Light BRIGHTNESS"

  LSTR MSG_KILL_EXPECTED_PRINTER          = _UxGT("打印机不正确"); // "The printer is incorrect"

  LSTR MSG_INFO_PRINT_COUNT               = _UxGT("打印数"); // "Prints"
  LSTR MSG_INFO_COMPLETED_PRINTS          = _UxGT("完成"); // "Completed"
  LSTR MSG_INFO_PRINT_TIME                = _UxGT("总共"); // "Total"
  LSTR MSG_INFO_PRINT_LONGEST             = _UxGT("最长"); // "Longest"
  LSTR MSG_INFO_PRINT_FILAMENT            = _UxGT("已挤出"); // "Extruded"

  LSTR MSG_INFO_MIN_TEMP                  = _UxGT("最低温度"); // "Min Temp"
  LSTR MSG_INFO_MAX_TEMP                  = _UxGT("最高温度"); // "Max Temp"
  LSTR MSG_INFO_PSU                       = _UxGT("电源供应"); // "Power Supply"
  LSTR MSG_DRIVE_STRENGTH                 = _UxGT("驱动力度"); // "Drive Strength"
  LSTR MSG_DAC_PERCENT_N                  = _UxGT("@ 驱动 %"); // "E Driver %"
  LSTR MSG_ERROR_TMC                      = _UxGT("TMC 连接错误");
  LSTR MSG_DAC_EEPROM_WRITE               = _UxGT("保存驱动设置"); // "DAC EEPROM Write"
  LSTR MSG_FILAMENT_CHANGE_HEADER         = _UxGT("更换料");
  LSTR MSG_FILAMENT_CHANGE_HEADER_PAUSE   = _UxGT("打印已暂停"); // "PRINT PAUSED"
  LSTR MSG_FILAMENT_CHANGE_HEADER_LOAD    = _UxGT("装载丝料"); // "LOAD FILAMENT"
  LSTR MSG_FILAMENT_CHANGE_HEADER_UNLOAD  = _UxGT("卸载丝料"); // "UNLOAD FILAMENT"
  LSTR MSG_FILAMENT_CHANGE_OPTION_HEADER  = _UxGT("恢复选项:"); // "RESUME OPTIONS:"
  LSTR MSG_FILAMENT_CHANGE_OPTION_PURGE   = _UxGT("清除更多"); // "Purge more"
  LSTR MSG_FILAMENT_CHANGE_OPTION_RESUME  = _UxGT("恢复打印"); // "Resume print"
  LSTR MSG_FILAMENT_CHANGE_NOZZLE         = _UxGT("  喷嘴: "); // "  Nozzle: "
  LSTR MSG_RUNOUT_SENSOR                  = _UxGT("断料传感器");
  LSTR MSG_RUNOUT_DISTANCE_MM             = _UxGT("断料距离mm");
  LSTR MSG_KILL_HOMING_FAILED             = _UxGT("归原位失败"); // "Homing failed"
  LSTR MSG_LCD_PROBING_FAILED             = _UxGT("探针探测失败"); // "Probing failed"

  LSTR MSG_MMU2_CHOOSE_FILAMENT_HEADER    = _UxGT("选择料");
  LSTR MSG_MMU2_MENU                      = _UxGT("MMU");
  LSTR MSG_KILL_MMU2_FIRMWARE             = _UxGT("升级MMU固件!");
  LSTR MSG_MMU2_NOT_RESPONDING            = _UxGT("MMU需要专注.");
  LSTR MSG_MMU2_RESUME                    = _UxGT("MMU恢复");
  LSTR MSG_MMU2_RESUMING                  = _UxGT("MMU恢复中...");
  LSTR MSG_MMU2_LOAD_FILAMENT             = _UxGT("MMU加载");
  LSTR MSG_MMU2_LOAD_ALL                  = _UxGT("MMU加载全部");
  LSTR MSG_MMU2_LOAD_TO_NOZZLE            = _UxGT("MMU加载到喷嘴");
  LSTR MSG_MMU2_EJECT_FILAMENT            = _UxGT("MMU弹出");
  LSTR MSG_MMU2_EJECT_FILAMENT_N          = _UxGT("MMU弹出 ~");
  LSTR MSG_MMU2_UNLOAD_FILAMENT           = _UxGT("MMU卸载");
  LSTR MSG_MMU2_LOADING_FILAMENT          = _UxGT("加载填充. %i...");
  LSTR MSG_MMU2_EJECTING_FILAMENT         = _UxGT("弹出填充. ...");
  LSTR MSG_MMU2_UNLOADING_FILAMENT        = _UxGT("卸载填充....");
  LSTR MSG_MMU2_ALL                       = _UxGT("全部");
  LSTR MSG_MMU2_FILAMENT_N                = _UxGT("料 ~");
  LSTR MSG_MMU2_RESET                     = _UxGT("复位MMU");
  LSTR MSG_MMU2_RESETTING                 = _UxGT("MMU复位中...");
  LSTR MSG_MMU2_EJECT_RECOVER             = _UxGT("移出, 按下");

  LSTR MSG_MIX                            = _UxGT("混合");
  LSTR MSG_MIX_COMPONENT_N                = _UxGT("器件  {");
  LSTR MSG_MIXER                          = _UxGT("混合器");
  LSTR MSG_GRADIENT                       = _UxGT("梯度");
  LSTR MSG_FULL_GRADIENT                  = _UxGT("全梯度");
  LSTR MSG_TOGGLE_MIX                     = _UxGT("开关混合");
  LSTR MSG_CYCLE_MIX                      = _UxGT("循环混合");
  LSTR MSG_GRADIENT_MIX                   = _UxGT("梯度混合");
  LSTR MSG_REVERSE_GRADIENT               = _UxGT("反向梯度");
  LSTR MSG_ACTIVE_VTOOL                   = _UxGT("激活 V-tool");
  LSTR MSG_START_VTOOL                    = _UxGT("开始 V-tool");
  LSTR MSG_END_VTOOL                      = _UxGT("  结束 V-tool");
  LSTR MSG_GRADIENT_ALIAS                 = _UxGT("别名 V-tool");
  LSTR MSG_RESET_VTOOLS                   = _UxGT("复位 V-tools");
  LSTR MSG_COMMIT_VTOOL                   = _UxGT("提交 V-tool Mix");
  LSTR MSG_VTOOLS_RESET                   = _UxGT("V-tools 已复位");
  LSTR MSG_START_Z                        = _UxGT("开始 Z:");
  LSTR MSG_END_Z                          = _UxGT("  结束 Z:");

  LSTR MSG_GAMES                          = _UxGT("游戏");
  LSTR MSG_BRICKOUT                       = _UxGT("敲方块");
  LSTR MSG_INVADERS                       = _UxGT("入侵者");
  LSTR MSG_SNAKE                          = _UxGT("贪吃蛇");
  LSTR MSG_MAZE                           = _UxGT("迷宫");

  LSTR MSG_BAD_PAGE                       = _UxGT("错误页面索引");
  LSTR MSG_BAD_PAGE_SPEED                 = _UxGT("错误页面速度");

  //
  // Filament Change screens show up to 2 lines on a 3-line display
  //
  LSTR MSG_ADVANCED_PAUSE_WAITING         = _UxGT(MSG_1_LINE("按下继续"));
  LSTR MSG_PAUSE_PRINT_PARKING            = _UxGT(MSG_1_LINE("停靠中..."));
  LSTR MSG_FILAMENT_CHANGE_INIT           = _UxGT(MSG_1_LINE("请等待 ...")); // "Please wait..."
  LSTR MSG_FILAMENT_CHANGE_INSERT         = _UxGT(MSG_1_LINE("插入并单击")); // "Insert and Click"
  LSTR MSG_FILAMENT_CHANGE_HEAT           = _UxGT(MSG_1_LINE("按下加热"));
  LSTR MSG_FILAMENT_CHANGE_HEATING        = _UxGT(MSG_1_LINE("加热中 ...")); // "Heating..."
  LSTR MSG_FILAMENT_CHANGE_UNLOAD         = _UxGT(MSG_1_LINE("退出中 ...")); // "Ejecting..."
  LSTR MSG_FILAMENT_CHANGE_LOAD           = _UxGT(MSG_1_LINE("装载中 ...")); // "Loading..."
  LSTR MSG_FILAMENT_CHANGE_PURGE          = _UxGT(MSG_1_LINE("清除中 ...")); // "Purging..."
  LSTR MSG_FILAMENT_CHANGE_CONT_PURGE     = _UxGT(MSG_1_LINE("按下完成"));
  LSTR MSG_FILAMENT_CHANGE_RESUME         = _UxGT(MSG_1_LINE("恢复中 ...")); // "Resuming..."

  LSTR MSG_TMC_DRIVERS                    = _UxGT("TMC驱动器");
  LSTR MSG_TMC_CURRENT                    = _UxGT("驱动电流");
  LSTR MSG_TMC_HYBRID_THRS                = _UxGT("混合阈值");
  LSTR MSG_TMC_HOMING_THRS                = _UxGT("无感回零");
  LSTR MSG_TMC_STEPPING_MODE              = _UxGT("步进模式");
  LSTR MSG_TMC_STEALTH_ENABLED            = _UxGT("StealthChop已使能");
  LSTR MSG_SERVICE_RESET                  = _UxGT("复位");
  LSTR MSG_SERVICE_IN                     = _UxGT(" 在:");
  LSTR MSG_BACKLASH                       = _UxGT("回差");
  LSTR MSG_BACKLASH_CORRECTION            = _UxGT("校正");
  LSTR MSG_BACKLASH_SMOOTHING             = _UxGT("平滑的");

  LSTR MSG_LEVEL_X_AXIS                   = _UxGT("X轴调平");
  LSTR MSG_AUTO_CALIBRATE                 = _UxGT("自动校准");
  LSTR MSG_HEATER_TIMEOUT                 = _UxGT("加热器超时");
  LSTR MSG_REHEAT                         = _UxGT("重新加热");
  LSTR MSG_REHEATING                      = _UxGT("重新加热中...");

  LSTR MSG_SHORT_DAY                      = _UxGT("天"); // "d" // One character only
  LSTR MSG_SHORT_HOUR                     = _UxGT("时"); // "h" // One character only
  LSTR MSG_SHORT_MINUTE                   = _UxGT("分"); // "m" // One character only
}

namespace LanguageWide_zh_CN {
  using namespace LanguageNarrow_zh_CN;
  #if LCD_WIDTH >= 20 || HAS_DWIN_E3V2
    LSTR MSG_INFO_PRINT_COUNT             = _UxGT("打印计数"); // "Print Count"
    LSTR MSG_INFO_COMPLETED_PRINTS        = _UxGT("完成了"); // "Completed"
    LSTR MSG_INFO_PRINT_TIME              = _UxGT("总打印时间"); // "Total print time"
    LSTR MSG_INFO_PRINT_LONGEST           = _UxGT("最长工作时间"); // "Longest job time"
    LSTR MSG_INFO_PRINT_FILAMENT          = _UxGT("总计挤出"); // "Extruded total"
  #endif
}

namespace LanguageTall_zh_CN {
  using namespace LanguageWide_zh_CN;
  #if LCD_HEIGHT >= 4
    // Filament Change screens show up to 3 lines on a 4-line display
    LSTR MSG_ADVANCED_PAUSE_WAITING       = _UxGT(MSG_2_LINE("按下按钮", "恢复打印"));
    LSTR MSG_PAUSE_PRINT_PARKING          = _UxGT(MSG_1_LINE("停靠中..."));
    LSTR MSG_FILAMENT_CHANGE_INIT         = _UxGT(MSG_3_LINE("等待开始", "丝料", "变更")); // "Wait for start of the filament change"
    LSTR MSG_FILAMENT_CHANGE_INSERT       = _UxGT(MSG_3_LINE("插入料", "并按下", "以继续"));
    LSTR MSG_FILAMENT_CHANGE_HEAT         = _UxGT(MSG_2_LINE("按下按钮来", "加热喷嘴.")); // "Press button to heat nozzle."
    LSTR MSG_FILAMENT_CHANGE_HEATING      = _UxGT(MSG_2_LINE("加热喷嘴", "请等待 ...")); // "Heating nozzle Please wait..."
    LSTR MSG_FILAMENT_CHANGE_UNLOAD       = _UxGT(MSG_2_LINE("等待", "卸下丝料")); // "Wait for filament unload"
    LSTR MSG_FILAMENT_CHANGE_LOAD         = _UxGT(MSG_2_LINE("等待", "进料")); // "Wait for filament load"
    LSTR MSG_FILAMENT_CHANGE_PURGE        = _UxGT(MSG_2_LINE("等待", "丝料清除")); // "Wait for filament purge"
    LSTR MSG_FILAMENT_CHANGE_CONT_PURGE   = _UxGT(MSG_2_LINE("按下已完成", "料的清洗"));
    LSTR MSG_FILAMENT_CHANGE_RESUME       = _UxGT(MSG_2_LINE("等待打印", "恢复")); // "Wait for print to resume"
  #endif
}

namespace Language_zh_CN {
  using namespace LanguageTall_zh_CN;
}
