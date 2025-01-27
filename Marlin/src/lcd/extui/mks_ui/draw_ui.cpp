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

#include "../../../inc/MarlinConfigPre.h"

#if HAS_TFT_LVGL_UI

#include "SPI_TFT.h"

#include "tft_lvgl_configuration.h"

#include "pic_manager.h"

#include "draw_ui.h"

#include <SPI.h>

#include "../../../MarlinCore.h" // for marlin_state
#include "../../../sd/cardreader.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../inc/MarlinConfig.h"

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#if ENABLED(PARK_HEAD_ON_PAUSE)
  #include "../../../feature/pause.h"
#endif

#if ENABLED(TOUCH_SCREEN_CALIBRATION)
  #include "draw_touch_calibration.h"
#endif

#if ENABLED(MKS_TEST)
  #include "mks_hardware.h"
#endif

CFG_ITMES gCfgItems;
UI_CFG uiCfg;
DISP_STATE_STACK disp_state_stack;
DISP_STATE disp_state = MAIN_UI;
DISP_STATE last_disp_state;
PRINT_TIME print_time;
num_key_value_state value;
keyboard_value_state keyboard_value;

uint32_t To_pre_view;
bool gcode_preview_over, flash_preview_begin, default_preview_flg;
uint32_t size = 809;
uint16_t row;
bool temps_update_flag;
uint8_t printing_rate_update_flag;

extern bool once_flag;
extern uint8_t sel_id;
extern lv_group_t *g;

void LCD_IO_WriteData(uint16_t RegValue);

static const char custom_gcode_command[][100] = {
  "G29N\nM500",
  "G28",
  "G28",
  "G28",
  "G28"
};

lv_point_t line_points[4][2] = {
  {{PARA_UI_POS_X, PARA_UI_POS_Y + PARA_UI_SIZE_Y}, {TFT_WIDTH, PARA_UI_POS_Y + PARA_UI_SIZE_Y}},
  {{PARA_UI_POS_X, PARA_UI_POS_Y*2 + PARA_UI_SIZE_Y}, {TFT_WIDTH, PARA_UI_POS_Y*2 + PARA_UI_SIZE_Y}},
  {{PARA_UI_POS_X, PARA_UI_POS_Y*3 + PARA_UI_SIZE_Y}, {TFT_WIDTH, PARA_UI_POS_Y*3 + PARA_UI_SIZE_Y}},
  {{PARA_UI_POS_X, PARA_UI_POS_Y*4 + PARA_UI_SIZE_Y}, {TFT_WIDTH, PARA_UI_POS_Y*4 + PARA_UI_SIZE_Y}}
};
enum CfgLang : uint8_t {
  LANG_en      = LANG_ENGLISH,
  LANG_zh_CN   = LANG_SIMPLE_CHINESE,
  LANG_zh_TW   = LANG_COMPLEX_CHINESE,
  LANG_jp_kana = LANG_JAPAN,
  LANG_de      = LANG_GERMAN,
  LANG_fr      = LANG_FRENCH,
  LANG_ru      = LANG_RUSSIAN,
  LANG_ko_KR   = LANG_KOREAN,
  LANG_tr      = LANG_TURKISH,
  LANG_es      = LANG_SPANISH,
  LANG_el      = LANG_GREEK,
  LANG_it      = LANG_ITALY,
  LANG_pt      = LANG_PORTUGUESE
};
void gCfgItems_init() {
  gCfgItems.multiple_language = MULTI_LANGUAGE_ENABLE;
  gCfgItems.language = CAT(LANG_, LCD_LANGUAGE);
  gCfgItems.leveling_mode     = 0;
  gCfgItems.from_flash_pic    = false;
  gCfgItems.curFilesize       = 0;
  gCfgItems.finish_power_off  = false;
  gCfgItems.pause_reprint     = false;
  gCfgItems.pausePosX         = -1;
  gCfgItems.pausePosY         = -1;
  gCfgItems.pausePosZ         = 5;
  gCfgItems.trammingPos[0].x  = X_MIN_POS + 30;
  gCfgItems.trammingPos[0].y  = Y_MIN_POS + 30;
  gCfgItems.trammingPos[1].x  = X_MAX_POS - 30;
  gCfgItems.trammingPos[1].y  = Y_MIN_POS + 30;
  gCfgItems.trammingPos[2].x  = X_MAX_POS - 30;
  gCfgItems.trammingPos[2].y  = Y_MAX_POS - 30;
  gCfgItems.trammingPos[3].x  = X_MIN_POS + 30;
  gCfgItems.trammingPos[3].y  = Y_MAX_POS - 30;
  gCfgItems.trammingPos[4].x  = X_BED_SIZE / 2;
  gCfgItems.trammingPos[4].y  = Y_BED_SIZE / 2;
  gCfgItems.cloud_enable      = false;
  gCfgItems.wifi_mode_sel = STA_MODEL;
  gCfgItems.fileSysType   = FILE_SYS_SD;
  gCfgItems.wifi_type     = ESP_WIFI;
  gCfgItems.filamentchange_load_length   = 200;
  gCfgItems.filamentchange_load_speed    = 1000;
  gCfgItems.filamentchange_unload_length = 200;
  gCfgItems.filamentchange_unload_speed  = 1000;
  gCfgItems.filament_limit_temp          = 200;

  gCfgItems.encoder_enable = true;

  W25QXX.SPI_FLASH_BufferRead((uint8_t *)&gCfgItems.spi_flash_flag, VAR_INF_ADDR, sizeof(gCfgItems.spi_flash_flag));
  if (gCfgItems.spi_flash_flag == FLASH_INF_VALID_FLAG) {
    W25QXX.SPI_FLASH_BufferRead((uint8_t *)&gCfgItems, VAR_INF_ADDR, sizeof(gCfgItems));
  }
  else {
    gCfgItems.spi_flash_flag = FLASH_INF_VALID_FLAG;
    W25QXX.SPI_FLASH_SectorErase(VAR_INF_ADDR);
    W25QXX.SPI_FLASH_BufferWrite((uint8_t *)&gCfgItems, VAR_INF_ADDR, sizeof(gCfgItems));
    // Init G-code command
    W25QXX.SPI_FLASH_BufferWrite((uint8_t *)&custom_gcode_command[0], AUTO_LEVELING_COMMAND_ADDR, 100);
    W25QXX.SPI_FLASH_BufferWrite((uint8_t *)&custom_gcode_command[1], OTHERS_COMMAND_ADDR_1, 100);
    W25QXX.SPI_FLASH_BufferWrite((uint8_t *)&custom_gcode_command[2], OTHERS_COMMAND_ADDR_2, 100);
    W25QXX.SPI_FLASH_BufferWrite((uint8_t *)&custom_gcode_command[3], OTHERS_COMMAND_ADDR_3, 100);
    W25QXX.SPI_FLASH_BufferWrite((uint8_t *)&custom_gcode_command[4], OTHERS_COMMAND_ADDR_4, 100);
  }

  const byte rot = (TFT_ROTATION & TFT_ROTATE_180) ? 0xEE : 0x00;
  if (gCfgItems.disp_rotation_180 != rot) {
    gCfgItems.disp_rotation_180 = rot;
    update_spi_flash();
  }

  uiCfg.F[0] = 'N';
  uiCfg.F[1] = 'A';
  uiCfg.F[2] = 'N';
  uiCfg.F[3] = 'O';
  W25QXX.SPI_FLASH_BlockErase(REFLSHE_FLGA_ADD + 32 - 64*1024);
  W25QXX.SPI_FLASH_BufferWrite(uiCfg.F,REFLSHE_FLGA_ADD,4);
}

void ui_cfg_init() {
  uiCfg.curTempType         = 0;
  uiCfg.extruderIndex       = 0;
  uiCfg.stepHeat            = 10;
  uiCfg.leveling_first_time = false;
  uiCfg.para_ui_page        = false;
  uiCfg.extruStep           = uiCfg.eStepMed;
  uiCfg.extruSpeed          = uiCfg.eSpeedN;
  uiCfg.move_dist           = 1;
  uiCfg.moveSpeed           = 1000;
  uiCfg.stepPrintSpeed      = 10;
  uiCfg.command_send        = false;
  uiCfg.dialogType          = 0;
  uiCfg.filament_heat_completed_load = false;
  uiCfg.filament_rate                = 0;
  uiCfg.filament_loading_completed   = false;
  uiCfg.filament_unloading_completed = false;
  uiCfg.filament_loading_time_flg    = false;
  uiCfg.filament_loading_time_cnt    = 0;
  uiCfg.filament_unloading_time_flg  = false;
  uiCfg.filament_unloading_time_cnt  = 0;

  #if ENABLED(MKS_WIFI_MODULE)
    memset(&wifiPara, 0, sizeof(wifiPara));
    memset(&ipPara, 0, sizeof(ipPara));
    strcpy_P(wifiPara.ap_name, PSTR(WIFI_AP_NAME));
    strcpy_P(wifiPara.keyCode, PSTR(WIFI_KEY_CODE));
    // client
    strcpy_P(ipPara.ip_addr, PSTR(IP_ADDR));
    strcpy_P(ipPara.mask, PSTR(IP_MASK));
    strcpy_P(ipPara.gate, PSTR(IP_GATE));
    strcpy_P(ipPara.dns, PSTR(IP_DNS));

    ipPara.dhcp_flag = IP_DHCP_FLAG;

    // AP
    strcpy_P(ipPara.dhcpd_ip, PSTR(AP_IP_ADDR));
    strcpy_P(ipPara.dhcpd_mask, PSTR(AP_IP_MASK));
    strcpy_P(ipPara.dhcpd_gate, PSTR(AP_IP_GATE));
    strcpy_P(ipPara.dhcpd_dns, PSTR(AP_IP_DNS));
    strcpy_P(ipPara.start_ip_addr, PSTR(IP_START_IP));
    strcpy_P(ipPara.end_ip_addr, PSTR(IP_END_IP));

    ipPara.dhcpd_flag = AP_IP_DHCP_FLAG;

    strcpy_P((char*)uiCfg.cloud_hostUrl, PSTR("baizhongyun.cn"));
    uiCfg.cloud_port = 10086;
  #endif

  uiCfg.filament_loading_time = (uint32_t)((gCfgItems.filamentchange_load_length * 60.0f / gCfgItems.filamentchange_load_speed) + 0.5f);
  uiCfg.filament_unloading_time = (uint32_t)((gCfgItems.filamentchange_unload_length * 60.0f / gCfgItems.filamentchange_unload_speed) + 0.5f);
}

void update_spi_flash() {
  uint8_t command_buf[512];

  W25QXX.init(SPI_QUARTER_SPEED);
  // read back the G-code command before erase spi flash
  W25QXX.SPI_FLASH_BufferRead((uint8_t *)&command_buf, GCODE_COMMAND_ADDR, sizeof(command_buf));
  W25QXX.SPI_FLASH_SectorErase(VAR_INF_ADDR);
  W25QXX.SPI_FLASH_BufferWrite((uint8_t *)&gCfgItems, VAR_INF_ADDR, sizeof(gCfgItems));
  W25QXX.SPI_FLASH_BufferWrite((uint8_t *)&command_buf, GCODE_COMMAND_ADDR, sizeof(command_buf));
}

void update_gcode_command(int addr, uint8_t *s) {
  uint8_t command_buf[512];

  W25QXX.init(SPI_QUARTER_SPEED);
  // read back the G-code command before erase spi flash
  W25QXX.SPI_FLASH_BufferRead((uint8_t *)&command_buf, GCODE_COMMAND_ADDR, sizeof(command_buf));
  W25QXX.SPI_FLASH_SectorErase(VAR_INF_ADDR);
  W25QXX.SPI_FLASH_BufferWrite((uint8_t *)&gCfgItems, VAR_INF_ADDR, sizeof(gCfgItems));
  switch (addr) {
    case AUTO_LEVELING_COMMAND_ADDR: memcpy(&command_buf[0 * 100], s, 100); break;
    case OTHERS_COMMAND_ADDR_1: memcpy(&command_buf[1 * 100], s, 100); break;
    case OTHERS_COMMAND_ADDR_2: memcpy(&command_buf[2 * 100], s, 100); break;
    case OTHERS_COMMAND_ADDR_3: memcpy(&command_buf[3 * 100], s, 100); break;
    case OTHERS_COMMAND_ADDR_4: memcpy(&command_buf[4 * 100], s, 100); break;
    default: break;
  }
  W25QXX.SPI_FLASH_BufferWrite((uint8_t *)&command_buf, GCODE_COMMAND_ADDR, sizeof(command_buf));
}

void get_gcode_command(int addr, uint8_t *d) {
  W25QXX.init(SPI_QUARTER_SPEED);
  W25QXX.SPI_FLASH_BufferRead((uint8_t *)d, addr, 100);
}

lv_style_t tft_style_scr;
lv_style_t tft_style_label_pre;
lv_style_t tft_style_label_rel;
lv_style_t style_line;
lv_style_t style_para_value_pre;
lv_style_t style_para_value_rel;

lv_style_t style_num_key_pre;
lv_style_t style_num_key_rel;

lv_style_t style_num_text;
lv_style_t style_sel_text;

lv_style_t style_para_value;
lv_style_t style_para_back;

lv_style_t lv_bar_style_indic;

lv_style_t style_btn_pr;
lv_style_t style_btn_rel;

void tft_style_init() {
  lv_style_copy(&tft_style_scr, &lv_style_scr);
  tft_style_scr.body.main_color   = LV_COLOR_BACKGROUND;
  tft_style_scr.body.grad_color   = LV_COLOR_BACKGROUND;
  tft_style_scr.text.color        = LV_COLOR_TEXT;
  tft_style_scr.text.sel_color    = LV_COLOR_TEXT;
  tft_style_scr.line.width        = 0;
  tft_style_scr.text.letter_space = 0;
  tft_style_scr.text.line_space   = 0;

  lv_style_copy(&tft_style_label_pre, &lv_style_scr);
  lv_style_copy(&tft_style_label_rel, &lv_style_scr);
  tft_style_label_pre.body.main_color = LV_COLOR_BACKGROUND;
  tft_style_label_pre.body.grad_color = LV_COLOR_BACKGROUND;
  tft_style_label_pre.text.color      = LV_COLOR_TEXT;
  tft_style_label_pre.text.sel_color  = LV_COLOR_TEXT;
  tft_style_label_rel.body.main_color = LV_COLOR_BACKGROUND;
  tft_style_label_rel.body.grad_color = LV_COLOR_BACKGROUND;
  tft_style_label_rel.text.color      = LV_COLOR_TEXT;
  tft_style_label_rel.text.sel_color  = LV_COLOR_TEXT;
  tft_style_label_pre.text.font       = TERN(HAS_SPI_FLASH_FONT, &gb2312_puhui32, LV_FONT_DEFAULT);
  tft_style_label_rel.text.font       = TERN(HAS_SPI_FLASH_FONT, &gb2312_puhui32, LV_FONT_DEFAULT);
  tft_style_label_pre.line.width        = 0;
  tft_style_label_rel.line.width        = 0;
  tft_style_label_pre.text.letter_space = 0;
  tft_style_label_rel.text.letter_space = 0;
  tft_style_label_pre.text.line_space   = -5;
  tft_style_label_rel.text.line_space   = -5;

  lv_style_copy(&style_para_value_pre, &lv_style_scr);
  lv_style_copy(&style_para_value_rel, &lv_style_scr);
  style_para_value_pre.body.main_color = LV_COLOR_BACKGROUND;
  style_para_value_pre.body.grad_color = LV_COLOR_BACKGROUND;
  style_para_value_pre.text.color      = LV_COLOR_TEXT;
  style_para_value_pre.text.sel_color  = LV_COLOR_TEXT;
  style_para_value_rel.body.main_color = LV_COLOR_BACKGROUND;
  style_para_value_rel.body.grad_color = LV_COLOR_BACKGROUND;
  style_para_value_rel.text.color      = LV_COLOR_BLACK;
  style_para_value_rel.text.sel_color  = LV_COLOR_BLACK;
  style_para_value_pre.text.font       = TERN(HAS_SPI_FLASH_FONT, &gb2312_puhui32, LV_FONT_DEFAULT);
  style_para_value_rel.text.font       = TERN(HAS_SPI_FLASH_FONT, &gb2312_puhui32, LV_FONT_DEFAULT);
  style_para_value_pre.line.width        = 0;
  style_para_value_rel.line.width        = 0;
  style_para_value_pre.text.letter_space = 0;
  style_para_value_rel.text.letter_space = 0;
  style_para_value_pre.text.line_space   = -5;
  style_para_value_rel.text.line_space   = -5;

  lv_style_copy(&style_num_key_pre, &lv_style_scr);
  lv_style_copy(&style_num_key_rel, &lv_style_scr);
  style_num_key_pre.body.main_color = LV_COLOR_KEY_BACKGROUND;
  style_num_key_pre.body.grad_color = LV_COLOR_KEY_BACKGROUND;
  style_num_key_pre.text.color      = LV_COLOR_TEXT;
  style_num_key_pre.text.sel_color  = LV_COLOR_TEXT;
  style_num_key_rel.body.main_color = LV_COLOR_KEY_BACKGROUND;
  style_num_key_rel.body.grad_color = LV_COLOR_KEY_BACKGROUND;
  style_num_key_rel.text.color      = LV_COLOR_TEXT;
  style_num_key_rel.text.sel_color  = LV_COLOR_TEXT;
  style_num_key_pre.text.font       = TERN(HAS_SPI_FLASH_FONT, &gb2312_puhui32, LV_FONT_DEFAULT);
  style_num_key_rel.text.font       = TERN(HAS_SPI_FLASH_FONT, &gb2312_puhui32, LV_FONT_DEFAULT);

  style_num_key_pre.line.width        = 0;
  style_num_key_rel.line.width        = 0;
  style_num_key_pre.text.letter_space = 0;
  style_num_key_rel.text.letter_space = 0;
  style_num_key_pre.text.line_space   = -5;
  style_num_key_rel.text.line_space   = -5;
  lv_style_copy(&style_num_text, &lv_style_scr);

  style_num_text.body.main_color   = LV_COLOR_WHITE;
  style_num_text.body.grad_color   = LV_COLOR_WHITE;
  style_num_text.text.color        = LV_COLOR_BLACK;
  style_num_text.text.sel_color    = LV_COLOR_BLACK;
  style_num_text.text.font         = TERN(HAS_SPI_FLASH_FONT, &gb2312_puhui32, LV_FONT_DEFAULT);
  style_num_text.line.width        = 0;
  style_num_text.text.letter_space = 0;
  style_num_text.text.line_space   = -5;

  lv_style_copy(&style_sel_text, &lv_style_scr);
  style_sel_text.body.main_color   = LV_COLOR_BACKGROUND;
  style_sel_text.body.grad_color   = LV_COLOR_BACKGROUND;
  style_sel_text.text.color        = LV_COLOR_YELLOW;
  style_sel_text.text.sel_color    = LV_COLOR_YELLOW;
  style_sel_text.text.font         = TERN(HAS_SPI_FLASH_FONT, &gb2312_puhui32, LV_FONT_DEFAULT);
  style_sel_text.line.width        = 0;
  style_sel_text.text.letter_space = 0;
  style_sel_text.text.line_space   = -5;
  lv_style_copy(&style_line, &lv_style_plain);
  style_line.line.color   = LV_COLOR_MAKE(0x49, 0x54, 0xFF);
  style_line.line.width   = 1;
  style_line.line.rounded = 1;

  lv_style_copy(&style_para_value, &lv_style_plain);
  style_para_value.body.border.color = LV_COLOR_BACKGROUND;
  style_para_value.body.border.width = 1;
  style_para_value.body.main_color   = LV_COLOR_WHITE;
  style_para_value.body.grad_color   = LV_COLOR_WHITE;
  style_para_value.body.shadow.width = 0;
  style_para_value.body.radius       = 3;
  style_para_value.text.color        = LV_COLOR_BLACK;
  style_para_value.text.font         = &TERN(HAS_SPI_FLASH_FONT, gb2312_puhui32, lv_font_roboto_22);

  lv_style_copy(&style_para_back, &lv_style_plain);
  style_para_back.body.border.color = LV_COLOR_BACKGROUND;
  style_para_back.body.border.width = 1;
  style_para_back.body.main_color   = TFT_LV_PARA_BACK_BODY_COLOR;
  style_para_back.body.grad_color   = TFT_LV_PARA_BACK_BODY_COLOR;
  style_para_back.body.shadow.width = 0;
  style_para_back.body.radius       = 3;
  style_para_back.text.color        = LV_COLOR_WHITE;
  style_para_back.text.font         = &TERN(HAS_SPI_FLASH_FONT, gb2312_puhui32, lv_font_roboto_22);

  lv_style_copy(&style_btn_rel, &lv_style_plain);
  style_btn_rel.body.border.color = lv_color_hex3(0x269);
  style_btn_rel.body.border.width = 1;
  style_btn_rel.body.main_color   = lv_color_hex3(0xADF);
  style_btn_rel.body.grad_color   = lv_color_hex3(0x46B);
  style_btn_rel.body.shadow.width = 4;
  style_btn_rel.body.shadow.type  = LV_SHADOW_BOTTOM;
  style_btn_rel.body.radius       = LV_RADIUS_CIRCLE;
  style_btn_rel.text.color        = lv_color_hex3(0xDEF);
  style_btn_rel.text.font         = &TERN(HAS_SPI_FLASH_FONT, gb2312_puhui32, lv_font_roboto_22);

  lv_style_copy(&style_btn_pr, &style_btn_rel);
  style_btn_pr.body.border.color = lv_color_hex3(0x46B);
  style_btn_pr.body.main_color   = lv_color_hex3(0x8BD);
  style_btn_pr.body.grad_color   = lv_color_hex3(0x24A);
  style_btn_pr.body.shadow.width = 2;
  style_btn_pr.text.color        = lv_color_hex3(0xBCD);
  style_btn_pr.text.font         = &TERN(HAS_SPI_FLASH_FONT, gb2312_puhui32, lv_font_roboto_22);

  lv_style_copy(&lv_bar_style_indic, &lv_style_pretty_color);
  lv_bar_style_indic.text.color        = lv_color_hex3(0xADF);
  lv_bar_style_indic.image.color       = lv_color_hex3(0xADF);
  lv_bar_style_indic.line.color        = lv_color_hex3(0xADF);
  lv_bar_style_indic.body.main_color   = lv_color_hex3(0xADF);
  lv_bar_style_indic.body.grad_color   = lv_color_hex3(0xADF);
  lv_bar_style_indic.body.border.color = lv_color_hex3(0xADF);
}

#define MAX_TITLE_LEN 28

char public_buf_m[100] = {0};
char public_buf_l[30];

void titleText_cat(char *str, int strSize, char *addPart) {
  if (str == 0 || addPart == 0) return;
  if ((int)(strlen(str) + strlen(addPart)) >= strSize) return;
  strcat(str, addPart);
}

char *getDispText(int index) {

  ZERO(public_buf_l);

  switch (disp_state_stack._disp_state[index]) {
    case PRINT_READY_UI:      strcpy(public_buf_l, main_menu.title); break;
    case PRINT_FILE_UI:       strcpy(public_buf_l, file_menu.title); break;
    case PRINTING_UI:
      switch (disp_state_stack._disp_state[disp_state_stack._disp_index]) {
        IF_DISABLED(TFT35, case OPERATE_UI: case PAUSE_UI:)
        case PRINTING_UI:     strcpy(public_buf_l, common_menu.print_special_title); break;
        default:              strcpy(public_buf_l, printing_menu.title); break;
      }
      break;
    case MOVE_MOTOR_UI:       strcpy(public_buf_l, move_menu.title); break;

    #if ENABLED(PROBE_OFFSET_WIZARD)
      case Z_OFFSET_WIZARD_UI: break;
    #endif
    case OPERATE_UI:
      switch (disp_state_stack._disp_state[disp_state_stack._disp_index]) {
        IF_DISABLED(TFT35, case OPERATE_UI: case PAUSE_UI:)
        case PRINTING_UI:     strcpy(public_buf_l, common_menu.operate_special_title); break;
        default:              strcpy(public_buf_l, operation_menu.title); break;
      }
      break;

    case PAUSE_UI:
      switch (disp_state_stack._disp_state[disp_state_stack._disp_index]) {
        case OPERATE_UI:
        case PAUSE_UI:
        case PRINTING_UI:     strcpy(public_buf_l, common_menu.pause_special_title); break;
        default:              strcpy(public_buf_l, pause_menu.title); break;
      }
      break;
    case EXTRUSION_UI:        strcpy(public_buf_l, extrude_menu.title); break;
    case CHANGE_SPEED_UI:     strcpy(public_buf_l, speed_menu.title); break;
    case FAN_UI:              strcpy(public_buf_l, fan_menu.title); break;
    case PREHEAT_UI:
      switch (disp_state_stack._disp_state[disp_state_stack._disp_index]) {
        case OPERATE_UI:      strcpy(public_buf_l, preheat_menu.adjust_title);
        default:              strcpy(public_buf_l, preheat_menu.title); break;
      }
      break;
    case SET_UI:              strcpy(public_buf_l, set_menu.title); break;
    case ZERO_UI:             strcpy(public_buf_l, home_menu.title); break;
    case SPRAYER_UI:          break;
    case MACHINE_UI:          break;
    case LANGUAGE_UI:         strcpy(public_buf_l, language_menu.title); break;
    case ABOUT_UI:            strcpy(public_buf_l, about_menu.title); break;
    case LOG_UI:              break;
    case DISK_UI:             strcpy(public_buf_l, filesys_menu.title); break;
    case DIALOG_UI:           strcpy(public_buf_l, common_menu.dialog_confirm_title); break;
    case WIFI_UI:             strcpy(public_buf_l, wifi_menu.title); break;
    case MORE_UI:
    case PRINT_MORE_UI:       strcpy(public_buf_l, more_menu.title); break;
    case FILAMENTCHANGE_UI:   strcpy(public_buf_l, filament_menu.title); break;
    case LEVELING_UI:
    case MESHLEVELING_UI:     strcpy(public_buf_l, leveling_menu.title); break;
    case BIND_UI:             strcpy(public_buf_l, cloud_menu.title); break;
    case TOOL_UI:             strcpy(public_buf_l, tool_menu.title); break;
    case WIFI_LIST_UI:        TERN_(MKS_WIFI_MODULE, strcpy(public_buf_l, list_menu.title)); break;
    case MACHINE_PARA_UI:     strcpy(public_buf_l, MachinePara_menu.title); break;
    case BABYSTEP_UI:         strcpy(public_buf_l, operation_menu.babystep); break;
    case EEPROM_SETTINGS_UI:  strcpy(public_buf_l, eeprom_menu.title); break;
    case MEDIA_SELECT_UI:     strcpy(public_buf_l, media_select_menu.title); break;
    default: break;
  }

  return public_buf_l;
}

char *creat_title_text() {
  int index     = 0;
  char *tmpText = 0;
  char tmpCurFileStr[20];

  ZERO(tmpCurFileStr);

  cutFileName(list_file.long_name[sel_id], 16, 16, tmpCurFileStr);

  ZERO(public_buf_m);

  while (index <= disp_state_stack._disp_index) {
    tmpText = getDispText(index);
    if ((*tmpText == 0) || (tmpText == 0)) {
      index++;
      continue;
    }

    titleText_cat(public_buf_m, sizeof(public_buf_m), tmpText);
    if (index < disp_state_stack._disp_index) titleText_cat(public_buf_m, sizeof(public_buf_m), (char *)">");

    index++;
  }

  if (disp_state_stack._disp_state[disp_state_stack._disp_index] == PRINTING_UI) {
    titleText_cat(public_buf_m, sizeof(public_buf_m), (char *)":");
    titleText_cat(public_buf_m, sizeof(public_buf_m), tmpCurFileStr);
  }

  if (strlen(public_buf_m) > MAX_TITLE_LEN) {
    ZERO(public_buf_m);
    tmpText = 0;
    for (index = 0; index <= disp_state_stack._disp_index && (!tmpText || *tmpText == 0); index++)
      tmpText = getDispText(index);
    if (*tmpText != 0) {
      titleText_cat(public_buf_m, sizeof(public_buf_m), tmpText);
      titleText_cat(public_buf_m, sizeof(public_buf_m), (char *)">...>");
      tmpText = getDispText(disp_state_stack._disp_index);
      if (*tmpText != 0) titleText_cat(public_buf_m, sizeof(public_buf_m), tmpText);
    }
  }

  return public_buf_m;
}

#if HAS_GCODE_PREVIEW

  uintptr_t gPicturePreviewStart = 0;

  void preview_gcode_prehandle(char *path) {
    #if HAS_MEDIA
      uintptr_t pre_read_cnt = 0;
      uint32_t *p1;
      char *cur_name;

      gPicturePreviewStart = 0;
      cur_name             = strrchr(path, '/');
      card.openFileRead(cur_name);
      card.read(public_buf, 512);
      p1 = (uint32_t *)strstr((char *)public_buf, ";simage:");

      if (p1) {
        pre_read_cnt = (uintptr_t)p1 - (uintptr_t)((uint32_t *)(&public_buf[0]));

        To_pre_view              = pre_read_cnt;
        gcode_preview_over       = true;
        gCfgItems.from_flash_pic = true;
        update_spi_flash();
      }
      else {
        gcode_preview_over       = false;
        default_preview_flg      = true;
        gCfgItems.from_flash_pic = false;
        update_spi_flash();
      }
      card.closefile();
    #endif
  }

  void gcode_preview(char *path, int xpos_pixel, int ypos_pixel) {
    #if HAS_MEDIA
      volatile uint32_t i, j;
      volatile uint16_t *p_index;
      char *cur_name;

      cur_name = strrchr(path, '/');
      card.openFileRead(cur_name);

      if (gPicturePreviewStart <= 0) {
        while (1) {
          uint32_t br  = card.read(public_buf, 400);
          uint32_t *p1 = (uint32_t *)strstr((char *)public_buf, ";gimage:");
          if (p1) {
            gPicturePreviewStart += (uintptr_t)p1 - (uintptr_t)((uint32_t *)(&public_buf[0]));
            break;
          }
          else
            gPicturePreviewStart += br;

          if (br < 400) break;
        }
      }

      card.setIndex(gPicturePreviewStart + size * row + 8);
      SPI_TFT.setWindow(xpos_pixel, ypos_pixel + row, 200, 1);

      j = i = 0;

      while (1) {
        card.read(public_buf, 400);
        for (i = 0; i < 400; i += 2, j++)
          bmp_public_buf[j] = ascii2dec_test((char*)&public_buf[i]) << 4 | ascii2dec_test((char*)&public_buf[i + 1]);
        if (j >= 400) break;
      }
      for (i = 0; i < 400; i += 2) {
        p_index = (uint16_t *)(&bmp_public_buf[i]);
        if (*p_index == 0x0000) *p_index = LV_COLOR_BACKGROUND.full;
      }
      SPI_TFT.tftio.writeSequence((uint16_t*)bmp_public_buf, 200);
      #if HAS_BAK_VIEW_IN_FLASH
        W25QXX.init(SPI_QUARTER_SPEED);
        if (row < 20) W25QXX.SPI_FLASH_SectorErase(BAK_VIEW_ADDR_TFT35 + row * 4096);
        W25QXX.SPI_FLASH_BufferWrite(bmp_public_buf, BAK_VIEW_ADDR_TFT35 + row * 400, 400);
      #endif
      row++;
      card.abortFilePrintNow();
      if (row >= 200) {
        size = 809;
        row  = 0;

        gcode_preview_over = false;

        char *cur_name = strrchr(list_file.file_name[sel_id], '/');

        MediaFile file;
        MediaFile *curDir;
        const char * const fname = card.diveToFile(false, curDir, cur_name);
        if (!fname) return;
        if (file.open(curDir, fname, O_READ)) {
          gCfgItems.curFilesize = file.fileSize();
          file.close();
          update_spi_flash();
        }

        card.openFileRead(cur_name);
        if (card.isFileOpen()) {
          feedrate_percentage = 100;
          TERN_(HAS_EXTRUDERS, planner.set_flow(0, 100));
          TERN_(HAS_MULTI_EXTRUDER, planner.set_flow(1, 100));
          card.startOrResumeFilePrinting();
          TERN_(POWER_LOSS_RECOVERY, recovery.prepare());
          once_flag = false;
        }
        return;
      }
    #endif // HAS_MEDIA
  }

  void draw_default_preview(int xpos_pixel, int ypos_pixel, uint8_t sel) {
    int index;
    int y_off = 0;
    W25QXX.init(SPI_QUARTER_SPEED);
    for (index = 0; index < 10; index++) { // 200*200
      #if HAS_BAK_VIEW_IN_FLASH
        if (sel == 1) {
          flash_view_Read(bmp_public_buf, 8000); // 20k
        }
        else {
          default_view_Read(bmp_public_buf, DEFAULT_VIEW_MAX_SIZE / 10); // 8k
        }
      #else
        default_view_Read(bmp_public_buf, DEFAULT_VIEW_MAX_SIZE / 10); // 8k
      #endif

      SPI_TFT.setWindow(xpos_pixel, y_off * 20 + ypos_pixel, 200, 20); // 200*200
      SPI_TFT.tftio.writeSequence((uint16_t*)(bmp_public_buf), DEFAULT_VIEW_MAX_SIZE / 20);

      y_off++;
    }
    W25QXX.init(SPI_QUARTER_SPEED);
  }

  void disp_pre_gcode(int xpos_pixel, int ypos_pixel) {
    if (gcode_preview_over) gcode_preview(list_file.file_name[sel_id], xpos_pixel, ypos_pixel);
    #if HAS_BAK_VIEW_IN_FLASH
      if (flash_preview_begin) {
        flash_preview_begin = false;
        draw_default_preview(xpos_pixel, ypos_pixel, 1);
      }
    #endif
    #if HAS_GCODE_DEFAULT_VIEW_IN_FLASH
      if (default_preview_flg) {
        draw_default_preview(xpos_pixel, ypos_pixel, 0);
        default_preview_flg = false;
      }
    #endif
  }
#endif // HAS_GCODE_PREVIEW

void print_time_run() {
  static uint8_t lastSec = 0;

  if (print_time.seconds >= 60) {
    print_time.seconds = 0;
    print_time.minutes++;
    if (print_time.minutes >= 60) {
      print_time.minutes = 0;
      print_time.hours++;
    }
  }
  if (disp_state == PRINTING_UI) {
    if (lastSec != print_time.seconds) disp_print_time();
    lastSec = print_time.seconds;
  }
}

void GUI_RefreshPage() {
  if ((systick_uptime_millis % 1000) == 0) temps_update_flag = true;
  if ((systick_uptime_millis % 3000) == 0) printing_rate_update_flag = true;

  switch (disp_state) {
    case MAIN_UI:
      break;
    case EXTRUSION_UI:
      if (temps_update_flag) {
        temps_update_flag = false;
        disp_hotend_temp();
      }
      break;
    case PREHEAT_UI:
      if (temps_update_flag) {
        temps_update_flag = false;
        disp_desire_temp();
      }
      break;
    case PRINT_READY_UI:
      if (temps_update_flag) {
        temps_update_flag = false;
        lv_temp_refr();
      }
      break;

    case PRINT_FILE_UI: break;

    case PRINTING_UI:
      if (temps_update_flag) {
        temps_update_flag = false;
        disp_ext_temp();
        disp_bed_temp();
        disp_fan_speed();
        disp_print_time();
        disp_fan_Zpos();
      }
      if (printing_rate_update_flag || marlin_state == MarlinState::MF_SD_COMPLETE) {
        printing_rate_update_flag = false;
        if (!gcode_preview_over) setProBarRate();
      }
      break;

    case OPERATE_UI: break;

    case PAUSE_UI: break;

    case FAN_UI:
      if (temps_update_flag) {
        temps_update_flag = false;
        disp_fan_value();
      }
      break;

    case MOVE_MOTOR_UI: break;

    #if ENABLED(PROBE_OFFSET_WIZARD)
      case Z_OFFSET_WIZARD_UI: break;
    #endif

    #if ENABLED(MKS_WIFI_MODULE)
      case WIFI_UI:
        if (temps_update_flag) {
          temps_update_flag = false;
          disp_wifi_state();
        }
        break;

      case BIND_UI: refresh_bind_ui(); break;
    #endif

    case FILAMENTCHANGE_UI:
      if (temps_update_flag) {
        temps_update_flag = false;
        disp_filament_temp();
      }
      break;
    case DIALOG_UI:
      filament_dialog_handle();
      TERN_(MKS_WIFI_MODULE, wifi_scan_handle());
      break;
    case MESHLEVELING_UI: break;
    case HARDWARE_TEST_UI: break;
    case WIFI_LIST_UI:
      #if ENABLED(MKS_WIFI_MODULE)
        if (printing_rate_update_flag) {
          disp_wifi_list();
          printing_rate_update_flag = false;
        }
      #endif
      break;
    case KEYBOARD_UI: break;

    #if ENABLED(MKS_WIFI_MODULE)
      case WIFI_TIPS_UI:
        switch (wifi_tips_type) {
          case TIPS_TYPE_JOINING:
            if (wifi_link_state == WIFI_CONNECTED && strcmp((const char *)wifi_list.wifiConnectedName, (const char *)wifi_list.wifiName[wifi_list.nameIndex]) == 0) {
              tips_disp.timer = TIPS_TIMER_STOP;
              tips_disp.timer_count = 0;

              lv_clear_wifi_tips();
              wifi_tips_type = TIPS_TYPE_WIFI_CONECTED;
              lv_draw_wifi_tips();

            }
            if (tips_disp.timer_count >= SEC_TO_MS(30)) {
              tips_disp.timer = TIPS_TIMER_STOP;
              tips_disp.timer_count = 0;
              lv_clear_wifi_tips();
              wifi_tips_type = TIPS_TYPE_TAILED_JOIN;
              lv_draw_wifi_tips();
            }
            break;
          case TIPS_TYPE_TAILED_JOIN:
            if (tips_disp.timer_count >= SEC_TO_MS(3)) {
              tips_disp.timer = TIPS_TIMER_STOP;
              tips_disp.timer_count = 0;

              last_disp_state = WIFI_TIPS_UI;
              lv_clear_wifi_tips();
              lv_draw_wifi_list();
            }
            break;
          case TIPS_TYPE_WIFI_CONECTED:
            if (tips_disp.timer_count >= SEC_TO_MS(3)) {
              tips_disp.timer = TIPS_TIMER_STOP;
              tips_disp.timer_count = 0;

              last_disp_state = WIFI_TIPS_UI;
              lv_clear_wifi_tips();
              lv_draw_wifi();
            }
            break;
          default: break;
        }
        break;
    #endif

    case BABYSTEP_UI:
      if (temps_update_flag) {
        temps_update_flag = false;
        disp_z_offset_value();
      }
      break;

    default: break;
  }

  print_time_run();
}

void clear_cur_ui() {
  last_disp_state = disp_state_stack._disp_state[disp_state_stack._disp_index];

  switch (disp_state_stack._disp_state[disp_state_stack._disp_index]) {
    case PRINT_READY_UI:              lv_clear_ready_print(); break;
    case PRINT_FILE_UI:               lv_clear_print_file(); break;
    case PRINTING_UI:                 lv_clear_printing(); break;
    case MOVE_MOTOR_UI:               lv_clear_move_motor(); break;
    #if ENABLED(PROBE_OFFSET_WIZARD)
      case Z_OFFSET_WIZARD_UI:        lv_clear_z_offset_wizard(); break;
    #endif
    case OPERATE_UI:                  lv_clear_operation(); break;
    case PAUSE_UI:                    break;
    case EXTRUSION_UI:                lv_clear_extrusion(); break;
    case PREHEAT_UI:                  lv_clear_preHeat(); break;
    case CHANGE_SPEED_UI:             lv_clear_change_speed(); break;
    case FAN_UI:                      lv_clear_fan(); break;
    case SET_UI:                      lv_clear_set(); break;
    case ZERO_UI:                     lv_clear_home(); break;
    case SPRAYER_UI:                  break;
    case MACHINE_UI:                  break;
    case LANGUAGE_UI:                 lv_clear_language(); break;
    case ABOUT_UI:                    lv_clear_about(); break;
    case LOG_UI:                      break;
    case DISK_UI:                     break;
    #if ENABLED(MKS_WIFI_MODULE)
      case WIFI_UI:                   lv_clear_wifi(); break;
    #endif
    case MORE_UI:                     lv_clear_more(); break;
    case FILETRANSFER_UI:             break;
    case DIALOG_UI:                   lv_clear_dialog(); break;
    case FILETRANSFERSTATE_UI:        break;
    case PRINT_MORE_UI:               break;
    case FILAMENTCHANGE_UI:           lv_clear_filament_change(); break;
    case LEVELING_UI:                 lv_clear_manualLevel(); break;
    #if ENABLED(MKS_WIFI_MODULE)
      case BIND_UI:                   lv_clear_cloud_bind(); break;
    #endif
    #if HAS_BED_PROBE
      case NOZZLE_PROBE_OFFSET_UI:    lv_clear_auto_level_offset_settings(); break;
    #endif
    case TOOL_UI:                     lv_clear_tool(); break;
    case MESHLEVELING_UI:             break;
    case HARDWARE_TEST_UI:            break;
    #if ENABLED(MKS_WIFI_MODULE)
      case WIFI_LIST_UI:              lv_clear_wifi_list(); break;
    #endif
    case KEYBOARD_UI:                 lv_clear_keyboard(); break;
    #if ENABLED(MKS_WIFI_MODULE)
      case WIFI_TIPS_UI:              lv_clear_wifi_tips(); break;
    #endif
    case MACHINE_PARA_UI:             lv_clear_machine_para(); break;
    case MACHINE_SETTINGS_UI:         lv_clear_machine_settings(); break;
    case TEMPERATURE_SETTINGS_UI:     break;
    case MOTOR_SETTINGS_UI:           lv_clear_motor_settings(); break;
    case MACHINE_TYPE_UI:             break;
    case STROKE_UI:                   break;
    case HOME_DIR_UI:                 break;
    case ENDSTOP_TYPE_UI:             break;
    case FILAMENT_SETTINGS_UI:        break;
    case LEVELING_PARA_UI:            lv_clear_level_settings(); break;
    case DELTA_LEVELING_PARA_UI:      break;
    case MANUAL_LEVELING_POSITION_UI: lv_clear_tramming_pos_settings(); break;
    case MAXFEEDRATE_UI:              lv_clear_max_feedrate_settings(); break;
    case STEPS_UI:                    lv_clear_step_settings(); break;
    case ACCELERATION_UI:             lv_clear_acceleration_settings(); break;
    case JERK_UI:                     TERN_(CLASSIC_JERK, lv_clear_jerk_settings()); break;
    case MOTORDIR_UI:                 break;
    case HOMESPEED_UI:                break;
    case NOZZLE_CONFIG_UI:            break;
    case HOTBED_CONFIG_UI:            break;
    case ADVANCED_UI:                 lv_clear_advance_settings(); break;
    case DOUBLE_Z_UI:                 break;
    case ENABLE_INVERT_UI:            break;
    case NUMBER_KEY_UI:               lv_clear_number_key(); break;
    case BABYSTEP_UI:                 lv_clear_baby_stepping(); break;
    case PAUSE_POS_UI:                lv_clear_pause_position(); break;
    #if HAS_TRINAMIC_CONFIG
      case TMC_CURRENT_UI:            lv_clear_tmc_current_settings(); break;
    #endif
    case EEPROM_SETTINGS_UI:          lv_clear_eeprom_settings(); break;
    #if HAS_STEALTHCHOP
      case TMC_MODE_UI:               lv_clear_tmc_step_mode_settings(); break;
    #endif
    #if ENABLED(MKS_WIFI_MODULE)
      case WIFI_SETTINGS_UI:          lv_clear_wifi_settings(); break;
    #endif
    #if USE_SENSORLESS
      case HOMING_SENSITIVITY_UI:     lv_clear_homing_sensitivity_settings(); break;
    #endif
    #if HAS_ROTARY_ENCODER
      case ENCODER_SETTINGS_UI:       lv_clear_encoder_settings(); break;
    #endif
    #if ENABLED(TOUCH_SCREEN_CALIBRATION)
      case TOUCH_CALIBRATION_UI:      lv_clear_touch_calibration_screen(); break;
    #endif
    #if HAS_MULTI_VOLUME
      case MEDIA_SELECT_UI:           lv_clear_media_select(); break;
    #endif
    default: break;
  }
}

void draw_return_ui() {
  if (disp_state_stack._disp_index > 0) {
    disp_state_stack._disp_index--;

    switch (disp_state_stack._disp_state[disp_state_stack._disp_index]) {
      case PRINT_READY_UI:              lv_draw_ready_print(); break;
      case PRINT_FILE_UI:               lv_draw_print_file(); break;

      case PRINTING_UI:                 if (gCfgItems.from_flash_pic)
                                          flash_preview_begin = true;
                                        else
                                          default_preview_flg = true;
                                        lv_draw_printing();
                                        break;

      case MOVE_MOTOR_UI:               lv_draw_move_motor(); break;
      #if ENABLED(PROBE_OFFSET_WIZARD)
        case Z_OFFSET_WIZARD_UI:        lv_draw_z_offset_wizard(); break;
      #endif
      case OPERATE_UI:                  lv_draw_operation(); break;
      case PAUSE_UI:                    break;
      case EXTRUSION_UI:                lv_draw_extrusion(); break;
      case PREHEAT_UI:                  lv_draw_preHeat(); break;
      case CHANGE_SPEED_UI:             lv_draw_change_speed(); break;
      case FAN_UI:                      lv_draw_fan(); break;
      case SET_UI:                      lv_draw_set(); break;
      case ZERO_UI:                     lv_draw_home(); break;
      case SPRAYER_UI:                  break;
      case MACHINE_UI:                  break;
      case LANGUAGE_UI:                 lv_draw_language(); break;
      case ABOUT_UI:                    lv_draw_about(); break;

      case CALIBRATE_UI:                break;
      case DISK_UI:                     break;
      #if ENABLED(MKS_WIFI_MODULE)
        case WIFI_UI:                   lv_draw_wifi(); break;
      #endif
      case MORE_UI:                     break;
      case PRINT_MORE_UI:               lv_draw_more(); break;
      case FILAMENTCHANGE_UI:           lv_draw_filament_change(); break;
      case LEVELING_UI:                 lv_draw_manualLevel(); break;
      #if ENABLED(MKS_WIFI_MODULE)
        case BIND_UI:                   lv_draw_cloud_bind(); break;
      #endif
      #if HAS_BED_PROBE
        case NOZZLE_PROBE_OFFSET_UI:    lv_draw_auto_level_offset_settings(); break;
      #endif
      case TOOL_UI:                     lv_draw_tool(); break;
      case GCODE_UI:                    lv_draw_gcode(); break;
      case MESHLEVELING_UI:             break;
      case HARDWARE_TEST_UI:            break;
      #if ENABLED(MKS_WIFI_MODULE)
        case WIFI_LIST_UI:              lv_draw_wifi_list(); break;
      #endif
      case KEYBOARD_UI:                 lv_draw_keyboard(); break;
      #if ENABLED(MKS_WIFI_MODULE)
        case WIFI_TIPS_UI:              lv_draw_wifi_tips(); break;
      #endif
      case MACHINE_PARA_UI:             lv_draw_machine_para(); break;
      case MACHINE_SETTINGS_UI:         lv_draw_machine_settings(); break;
      case TEMPERATURE_SETTINGS_UI:     break;
      case MOTOR_SETTINGS_UI:           lv_draw_motor_settings(); break;
      case MACHINE_TYPE_UI:             break;
      case STROKE_UI:                   break;
      case HOME_DIR_UI:                 break;
      case ENDSTOP_TYPE_UI:             break;
      case FILAMENT_SETTINGS_UI:        lv_draw_filament_settings(); break;
      case LEVELING_PARA_UI:            lv_draw_level_settings(); break;
      case DELTA_LEVELING_PARA_UI:      break;
      case MANUAL_LEVELING_POSITION_UI: lv_draw_tramming_pos_settings(); break;
      case MAXFEEDRATE_UI:              lv_draw_max_feedrate_settings(); break;
      #if ENABLED(EDITABLE_STEPS_PER_UNIT)
        case STEPS_UI:                  lv_draw_step_settings(); break;
      #endif
      case ACCELERATION_UI:             lv_draw_acceleration_settings(); break;
      #if ENABLED(CLASSIC_JERK)
        case JERK_UI:                   lv_draw_jerk_settings(); break;
      #endif
      case MOTORDIR_UI:                 break;
      case HOMESPEED_UI:                break;
      case NOZZLE_CONFIG_UI:            break;
      case HOTBED_CONFIG_UI:            break;
      case ADVANCED_UI:                 lv_draw_advance_settings(); break;
      case DOUBLE_Z_UI:                 break;
      case ENABLE_INVERT_UI:            break;
      case NUMBER_KEY_UI:               lv_draw_number_key(); break;
      case DIALOG_UI:                   break;
      case BABYSTEP_UI:                 lv_draw_baby_stepping(); break;
      case PAUSE_POS_UI:                lv_draw_pause_position(); break;
      #if HAS_TRINAMIC_CONFIG
        case TMC_CURRENT_UI:            lv_draw_tmc_current_settings(); break;
      #endif
      case EEPROM_SETTINGS_UI:          lv_draw_eeprom_settings(); break;
        #if HAS_STEALTHCHOP
          case TMC_MODE_UI:             lv_draw_tmc_step_mode_settings(); break;
        #endif
      #if ENABLED(MKS_WIFI_MODULE)
        case WIFI_SETTINGS_UI:          lv_draw_wifi_settings(); break;
      #endif
      #if USE_SENSORLESS
        case HOMING_SENSITIVITY_UI:     lv_draw_homing_sensitivity_settings(); break;
      #endif
      #if HAS_ROTARY_ENCODER
        case ENCODER_SETTINGS_UI:       lv_draw_encoder_settings(); break;
      #endif
      default: break;
    }
  }
}

void goto_previous_ui() {
  clear_cur_ui();
  draw_return_ui();
}

// Set the same image for both Released and Pressed
void lv_imgbtn_set_src_both(lv_obj_t *imgbtn, const void *src) {
  lv_imgbtn_set_src(imgbtn, LV_BTN_STATE_REL, src);
  lv_imgbtn_set_src(imgbtn, LV_BTN_STATE_PR,  src);
}

// Use label style for the image button
void lv_imgbtn_use_label_style(lv_obj_t *imgbtn) {
  lv_imgbtn_set_style(imgbtn, LV_BTN_STATE_REL, &tft_style_label_rel);
  lv_imgbtn_set_style(imgbtn, LV_BTN_STATE_PR,  &tft_style_label_pre);
}

// Use label style for the button
void lv_btn_use_label_style(lv_obj_t *btn) {
  lv_btn_set_style(btn, LV_BTN_STYLE_REL, &tft_style_label_rel);
  lv_btn_set_style(btn, LV_BTN_STYLE_PR,  &tft_style_label_pre);
}

// Use button style for the button
void lv_btn_use_button_style(lv_obj_t *btn) {
  lv_btn_set_style(btn, LV_BTN_STYLE_REL, &style_btn_rel);
  lv_btn_set_style(btn, LV_BTN_STYLE_PR,  &style_btn_pr);
}

// Use a single style for both Released and Pressed
void lv_btn_set_style_both(lv_obj_t *btn, lv_style_t *style) {
  lv_btn_set_style(btn, LV_BTN_STYLE_REL, style);
  lv_btn_set_style(btn, LV_BTN_STYLE_PR,  style);
}

// Create a screen
lv_obj_t* lv_screen_create(DISP_STATE newScreenType, const char *title) {
  lv_obj_t *scr = lv_obj_create(nullptr, nullptr);
  lv_obj_set_style(scr, &tft_style_scr);
  lv_scr_load(scr);
  lv_obj_clean(scr);

  // breadcrumbs
  if (disp_state_stack._disp_state[disp_state_stack._disp_index] != newScreenType) {
    disp_state_stack._disp_index++;
    disp_state_stack._disp_state[disp_state_stack._disp_index] = newScreenType;
  }
  disp_state = newScreenType;

  // title
  lv_obj_t *titleLabel = nullptr;
  if (!title)
    titleLabel = lv_label_create(scr, TITLE_XPOS, TITLE_YPOS, creat_title_text());
  else if (title[0] != '\0')
    titleLabel = lv_label_create(scr, TITLE_XPOS, TITLE_YPOS, title);
  if (titleLabel)
    lv_obj_set_style(titleLabel, &tft_style_label_rel);

  lv_refr_now(lv_refr_get_disp_refreshing());

  return scr;
}

// Create an empty label
lv_obj_t* lv_label_create_empty(lv_obj_t *par) {
  lv_obj_t *label = lv_label_create(par, (lv_obj_t*)nullptr);
  return label;
}

// Create a label with style and text
lv_obj_t* lv_label_create(lv_obj_t *par, const char *text) {
  lv_obj_t *label = lv_label_create_empty(par);
  if (text) lv_label_set_text(label, text);
  lv_obj_set_style(label, &tft_style_label_rel);
  return label;
}

// Create a label with style, position, and text
lv_obj_t* lv_label_create(lv_obj_t *par, lv_coord_t x, lv_coord_t y, const char *text) {
  lv_obj_t *label = lv_label_create(par, text);
  lv_obj_set_pos(label, x, y);
  return label;
}

// Create a button with callback, ID, and Style.
lv_obj_t* lv_btn_create(lv_obj_t *par, lv_event_cb_t cb, const int id/*=0*/, lv_style_t *style/*=&style_para_value*/) {
  lv_obj_t *btn = lv_btn_create(par, nullptr);
  if (id)
    lv_obj_set_event_cb_mks(btn, cb, id, "", 0);
  else
    lv_obj_set_event_cb(btn, cb);
  lv_btn_set_style_both(btn, style);
  return btn;
}

// Create a button with callback and ID, with label style.
lv_obj_t* lv_label_btn_create(lv_obj_t *par, lv_event_cb_t cb, const int id/*=0*/) {
  lv_obj_t *btn = lv_btn_create(par, cb, id, nullptr);
  lv_btn_use_label_style(btn);
  return btn;
}

// Create a button with callback and ID, with button style.
lv_obj_t* lv_button_btn_create(lv_obj_t *par, lv_event_cb_t cb, const int id/*=0*/) {
  lv_obj_t *btn = lv_btn_create(par, cb, id, nullptr);
  lv_btn_use_button_style(btn);
  return btn;
}

// Create a button with position, size, callback, ID, and style.
lv_obj_t* lv_btn_create(lv_obj_t *par, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h, lv_event_cb_t cb, const int id, lv_style_t *style) {
  lv_obj_t *btn = lv_btn_create(par, cb, id, style);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  return btn;
}

// Create a button with position, size, callback, and ID. Style set to style_para_value.
lv_obj_t* lv_btn_create(lv_obj_t *par, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h, lv_event_cb_t cb, const int id/*=0*/) {
  lv_obj_t *btn = lv_btn_create(par, x, y, w, h, cb, id, &style_para_value);
  return btn;
}

// Create a button with position, size, callback, and ID, with label style.
lv_obj_t* lv_label_btn_create(lv_obj_t *par, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h, lv_event_cb_t cb, const int id/*=0*/) {
  lv_obj_t *btn = lv_label_btn_create(par, cb, id);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  return btn;
}

// Create a button with position, size, callback, and ID, with label style.
lv_obj_t* lv_button_btn_create(lv_obj_t *par, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h, lv_event_cb_t cb, const int id/*=0*/) {
  lv_obj_t *btn = lv_button_btn_create(par, cb, id);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  return btn;
}

// Create a button with callback and ID. Style set to style_para_back.
lv_obj_t* lv_btn_create_back(lv_obj_t *par, lv_event_cb_t cb, const int id/*=0*/) {
  return lv_btn_create(par, cb, id, &style_para_back);
}
// Create a button with position, size, callback, and ID. Style set to style_para_back.
lv_obj_t* lv_btn_create_back(lv_obj_t *par, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h, lv_event_cb_t cb, const int id/*=0*/) {
  lv_obj_t *btn = lv_btn_create_back(par, cb, id);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  return btn;
}

// Create an image button with image, callback, and ID. Use label style.
lv_obj_t* lv_imgbtn_create(lv_obj_t *par, const char *img, lv_event_cb_t cb, const int id/*=0*/) {
  lv_obj_t *btn = lv_imgbtn_create(par, nullptr);
  if (img) lv_imgbtn_set_src_both(btn, img);
  if (id)
    lv_obj_set_event_cb_mks(btn, cb, id, "", 0);
  else
    lv_obj_set_event_cb(btn, cb);
  lv_imgbtn_use_label_style(btn);
  lv_btn_set_layout(btn, LV_LAYOUT_OFF);
  return btn;
}

// Create an image button with image, position, callback, and ID. Use label style.
lv_obj_t* lv_imgbtn_create(lv_obj_t *par, const char *img, lv_coord_t x, lv_coord_t y, lv_event_cb_t cb, const int id/*=0*/) {
  lv_obj_t *btn = lv_imgbtn_create(par, img, cb, id);
  lv_obj_set_pos(btn, x, y);
  return btn;
}

lv_obj_t* lv_big_button_create(lv_obj_t *par, const char *img, const char *text, lv_coord_t x, lv_coord_t y, lv_event_cb_t cb, const int id, bool centerLabel) {
  lv_obj_t *btn = lv_imgbtn_create(par, img, cb, id);
  lv_obj_set_pos(btn, x, y);
  lv_obj_t *label = lv_label_create_empty(btn);
  if (gCfgItems.multiple_language) {
    lv_label_set_text(label, text);
    if (centerLabel)
      lv_obj_align(label, btn, LV_ALIGN_CENTER, 0, 0);
    else
      lv_obj_align(label, btn, LV_ALIGN_IN_BOTTOM_MID, 0, BUTTON_TEXT_Y_OFFSET);
  }
  if (TERN0(HAS_ROTARY_ENCODER, gCfgItems.encoder_enable))
    lv_group_add_obj(g, btn);
  return btn;
}

lv_obj_t* lv_screen_menu_item(lv_obj_t *par, const char *text, lv_coord_t x, lv_coord_t y, lv_event_cb_t cb, const int id, const int index, bool drawArrow) {
  lv_obj_t *btn = lv_btn_create(par, nullptr);   /*Add a button the current screen*/
  lv_obj_set_pos(btn, x, y);                         /*Set its position*/
  lv_obj_set_size(btn, PARA_UI_SIZE_X, PARA_UI_SIZE_Y);                       /*Set its size*/
  if (id > -1) lv_obj_set_event_cb_mks(btn, cb, id, "", 0);
  lv_btn_use_label_style(btn);
  lv_btn_set_layout(btn, LV_LAYOUT_OFF);
  lv_obj_t *label = lv_label_create_empty(btn);        /*Add a label to the button*/
  if (gCfgItems.multiple_language) {
    lv_label_set_text(label, text);
    lv_obj_align(label, btn, LV_ALIGN_IN_LEFT_MID, 0, 0);
  }
  if (TERN0(HAS_ROTARY_ENCODER, gCfgItems.encoder_enable))
    lv_group_add_obj(g, btn);

  if (drawArrow) (void)lv_imgbtn_create(par, "F:/bmp_arrow.bin", x + PARA_UI_SIZE_X, y + PARA_UI_ARROW_V, cb, id);

  lv_obj_t *line1 = lv_line_create(par, nullptr);
  lv_ex_line(line1, line_points[index]);

  return btn;
}

lv_obj_t* lv_screen_menu_item_1_edit(lv_obj_t *par, const char *text, lv_coord_t x, lv_coord_t y, lv_event_cb_t cb, const int id, const int index, const char *editValue) {
  lv_obj_t *btn = lv_screen_menu_item(par, text, x, y, cb, -1, index, false);
  lv_obj_t *btnValue = lv_btn_create(par, PARA_UI_VALUE_POS_X, y + PARA_UI_VALUE_V, PARA_UI_VALUE_BTN_X_SIZE, PARA_UI_VALUE_BTN_Y_SIZE, cb, id);
  lv_obj_t *labelValue = lv_label_create_empty(btnValue);
  lv_label_set_text(labelValue, editValue);
  lv_obj_align(labelValue, btnValue, LV_ALIGN_CENTER, 0, 0);
  return btn;
}

lv_obj_t* lv_screen_menu_item_2_edit(lv_obj_t *par, const char *text, lv_coord_t x, lv_coord_t y, lv_event_cb_t cb, const int id, const int index, const char *editValue, const int idEdit2, const char *editValue2) {
  lv_obj_t *btn = lv_screen_menu_item(par, text, x, y, cb, -1, index, false);

  lv_obj_t *btnValue = lv_btn_create(par, PARA_UI_VALUE_POS_X_2, y + PARA_UI_VALUE_V_2, PARA_UI_VALUE_BTN_X_SIZE, PARA_UI_VALUE_BTN_Y_SIZE, cb, idEdit2);
  lv_obj_t *labelValue = lv_label_create_empty(btnValue);
  lv_label_set_text(labelValue, editValue2);
  lv_obj_align(labelValue, btnValue, LV_ALIGN_CENTER, 0, 0);

  btnValue = lv_btn_create(par, PARA_UI_VALUE_POS_X, y + PARA_UI_VALUE_V, PARA_UI_VALUE_BTN_X_SIZE, PARA_UI_VALUE_BTN_Y_SIZE, cb, id);
  labelValue = lv_label_create_empty(btnValue);
  lv_label_set_text(labelValue, editValue);
  lv_obj_align(labelValue, btnValue, LV_ALIGN_CENTER, 0, 0);

  return btn;
}

lv_obj_t* lv_screen_menu_item_onoff(lv_obj_t *par, const char *text, lv_coord_t x, lv_coord_t y, lv_event_cb_t cb, const int id, const int index, const bool curValue) {
  lv_screen_menu_item(par, text, x, y, cb, -1, index, false);
  lv_obj_t *btnValue = lv_imgbtn_create(par, curValue ? "F:/bmp_enable.bin" : "F:/bmp_disable.bin", PARA_UI_STATE_POS_X, y + PARA_UI_STATE_V, cb, id);
  lv_obj_t *labelValue = lv_label_create_empty(btnValue);
  lv_label_set_text(labelValue, curValue ? machine_menu.enable : machine_menu.disable);
  lv_obj_align(labelValue, btnValue, LV_ALIGN_CENTER, 0, 0);
  return btnValue;
}

void lv_screen_menu_item_onoff_update(lv_obj_t *btn, const bool curValue) {
  lv_imgbtn_set_src_both(btn, curValue ? "F:/bmp_enable.bin" : "F:/bmp_disable.bin");
  lv_label_set_text((lv_obj_t*)btn->child_ll.head, curValue ? machine_menu.enable : machine_menu.disable);
}

#if HAS_MEDIA

  void sd_detection() {
    static bool last_sd_status;
    const bool sd_status = IS_SD_INSERTED();
    if (sd_status != last_sd_status) {
      last_sd_status = sd_status;
      if (sd_status) card.mount(); else card.release();
    }
  }

#endif

void lv_ex_line(lv_obj_t *line, lv_point_t *points) {
  // Copy the previous line and apply the new style
  lv_line_set_points(line, points, 2);     // Set the points
  lv_line_set_style(line, LV_LINE_STYLE_MAIN, &style_line);
  lv_obj_align(line, nullptr, LV_ALIGN_IN_TOP_MID, 0, 0);
}

extern volatile uint32_t systick_uptime_millis;

void print_time_count() {
  if ((systick_uptime_millis % 1000) == 0)
    if (print_time.start == 1) print_time.seconds++;
}

void LV_TASK_HANDLER() {

  if (TERN1(USE_SPI_DMA_TC, !get_lcd_dma_lock()))
    lv_task_handler();

  #if ALL(MKS_TEST, HAS_MEDIA)
    if (mks_test_flag == 0x1E) mks_hardware_test();
  #endif

  TERN_(HAS_GCODE_PREVIEW, disp_pre_gcode(2, 36));

  GUI_RefreshPage();

  TERN_(MKS_WIFI_MODULE, get_wifi_commands());

  #if HAS_ROTARY_ENCODER
    if (gCfgItems.encoder_enable) lv_update_encoder();
  #endif
}

#endif // HAS_TFT_LVGL_UI
