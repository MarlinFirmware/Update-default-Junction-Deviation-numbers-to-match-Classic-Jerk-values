/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/**
 * Resistive Touch Screen
 * Copyright (c) 2023 Creality3D
 */
#include "../../inc/MarlinConfig.h"

#if ENABLED(CREALITY_RTS)

#include "lcd_rts.h"

#include "../marlinui.h" // For material_preset
#include "../../MarlinCore.h"
#include "../../feature/powerloss.h"
#include "../../feature/babystep.h"
#include "../../module/temperature.h"
#include "../../module/printcounter.h"
#include "../../module/planner.h"
#include "../../gcode/queue.h"
#include "../../gcode/gcode.h"

#if HAS_BED_PROBE
  #include "../../module/probe.h"
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #include "../../feature/runout.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #include "../../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(EEPROM_SETTINGS)
  #include "../../HAL/shared/eeprom_api.h"
  #include "../../module/settings.h"
#endif

//#define DEBUG_OUT 1
#include "../../core/debug_out.h"

#if ENABLED(MENU_RESET_WIFI)
  wifi_state_t rts_wifi_state = INITIAL;
#endif

char errorway = 0;
char errornum = 0;
char home_errornum = 0;
char error_sd_num = 0;

#if ENABLED(BABYSTEPPING)
  float zprobe_zoffset;
  float last_zoffset = 0.0;
#endif

bool power_off_type_yes = false;

int8_t g_cloudPLRStatusValue = 0; // Cloud printing status flag

const float manual_feedrate_mm_m[] = { 50 * 60, 50 * 60, 4 * 60, 60 };
constexpr float default_max_feedrate[] = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_acceleration[] = DEFAULT_MAX_ACCELERATION;
constexpr float default_max_jerk[] = { DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK };
constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

float default_nozzle_ptemp = DEFAULT_Kp,
      default_nozzle_itemp = DEFAULT_Ki,
      default_nozzle_dtemp = DEFAULT_Kd;

float default_hotbed_ptemp = DEFAULT_bedKp,
      default_hotbed_itemp = DEFAULT_bedKi,
      default_hotbed_dtemp = DEFAULT_bedKd;

int startprogress = 0;
CRec CardRecbuf;
int16_t temphot = 0;
int16_t tempbed = 0;
float temp_bed_display = 0;
uint8_t afterprobe_fan0_speed = 0;
float pause_e = 0;
bool sdcard_pause_check = true;
bool pause_action_flag = false;
bool print_preheat_check = false;
bool probe_offset_flag = false;

float ChangeFilamentTemp = 200;
millis_t next_rts_update_ms = 0;
millis_t next_shutdown_update_ms = 0;
millis_t next_wifireset_update_ms = 0;
unsigned int count_ms = 0;
unsigned int wifiresetcount_ms = 0;
unsigned long count_lcd_down = 0;
bool flag_lcd_down = false;

int last_target_temperature[4] = { 0 };
int last_target_temperature_bed;
uint8_t waitway = 0;
int change_page_font = 1;
unsigned char Percentrecord = 0;
// represents to update file list
bool CardUpdate = false;

uint8_t fileCnt = 0;
uint8_t file_current_page = 1;
uint8_t file_total_page = 1;
uint8_t page_total_file = 0;

float RTS::zCoordinateOffset; // Z-axis space coordinate difference
DB RTS::recdat;
DB RTS::snddat;
unsigned char RTS::databuf[RTS_DATA_SIZE];

RTS rts;

// represents SD-card status, true means SD is available, false means opposite.
bool lcd_sd_status;

char Checkfilenum = 0;
char cmdbuf[20] = { 0 };
float FilamentLOAD = 10;
float FilamentUnLOAD = 10;

// 1 for 10mm, 2 for 1mm, 3 for 0.1mm
unsigned char AxisUnitMode;
float axis_unit = 10.0;
bool LEDStatus = false;

bool wifi_enable_flag;

int update_time_value = 0;

bool PoweroffContinue = false;
char commandbuf[30];

uint16_t remain_time = 0;

bool home_flag = false;
bool G29_flag = false;
bool G29_finished = false;
float z_values_value[4][4] = { 0 };
bool leveling_flag = false;
bool flag_counter_wifi_reset = false;
HMI_LCD_Flag_t HMI_lcd_flag{0};

static bool temp_remove_card_flag = false;
//static bool temp_cutting_line_flag = false;
//static bool temp_wifi_print_flag = false; // 0 No interruption, 1 Suspension of material breaking 2 Suspension of card pulling

BedNozzleHeightCalSt st_bedNozzleHeightCal = { 0 }; // Platform and Nozzle Height Measuring Structure
float bedNozzleHeightCalZ = 0; // Platform nozzle Z-axis measurement variable
bool g_heaterLoadTempAdd = false; // Inlet and out material temperature heating
bool g_uiXYAxisEnable = false; // Axis movement interface, YX axis motor enable flag bit
bool g_uiZAxisEnable = false; // Axis movement interface, Z-axis motor enable flag

/**
 * Set the difference between the two coordinates
 * (the lower limit of z and the coordinate system established by BLTouch)
 * @Author Creality
 * @Time   2021-12-13
 */
void RTS::setZCoordinateOffset(const_float_t _offset) {
  zCoordinateOffset = _offset;
}

// card pull monitoring
static void Remove_card_window_check() {
  static bool lSDCardStatus = false;
  // SD card inserted
  if (!lSDCardStatus && IS_SD_INSERTED()) { // have card
    lSDCardStatus = true;
  }
  // SD card removed
  else if (lSDCardStatus && !IS_SD_INSERTED()) { // pull card
    lSDCardStatus = false;
    // Remove SD card when printing
    if (IS_SD_PRINTING()) {  // printing
      HMI_lcd_flag.remove_card_flag = true;
      temp_remove_card_flag = true;

      TERN_(POWER_LOSS_RECOVERY, if (recovery.enabled) recovery.save(true, false)); // rock_20211016

      //SERIAL_ECHO_MSG("ROCK_MOVE_CARD2222\n");
      rts.sendData(ExchangePageBase + 47, ExchangepageAddr);
      change_page_font = 47;
      queue.inject(F("M25"));    // M25: Pause the SD print in progress.
    }
  }
  // Refresh SD card status
  else
    lSDCardStatus = IS_SD_INSERTED();
}

inline void RTS_line_to_current(AxisEnum axis) {
  if (!planner.is_full())
    planner.buffer_line(current_position, MMM_TO_MMS(manual_feedrate_mm_m[(int8_t)axis]), active_extruder);
}

static void RTS_line_to_filelist() {
  // clean filename Icon
  for (int j = 0; j < 4; j++)
    for (int i = 0; i < TEXTBYTELEN; i++)
      rts.sendData(0, CardRecbuf.addr[j] + i);

  memset(&CardRecbuf, 0, sizeof(CardRecbuf));

  int num = 0;
  for (uint16_t i = (file_current_page - 1) * 4; i < (file_current_page * 4); i++) {
    card.selectFileByIndex(fileCnt - 1 - i);
    char *pointFilename = card.longFilename;
    int filenamelen = strlen(card.longFilename);
    int j = 1;
    while ((strncmp(&pointFilename[j], ".gcode", 6) && strncmp(&pointFilename[j], ".GCODE", 6)) && ((j++) < filenamelen));

    if (j >= TEXTBYTELEN) {
      strncpy(&card.longFilename[TEXTBYTELEN - 3], "..", 2);
      card.longFilename[TEXTBYTELEN - 1] = '\0';
      j = TEXTBYTELEN - 1;
    }
    strncpy(CardRecbuf.Cardshowfilename[num], card.longFilename, j);

    strcpy(CardRecbuf.Cardfilename[num], card.filename);
    CardRecbuf.addr[num] = FILE1_TEXT_VP + (num * 20);
    rts.sendData(CardRecbuf.Cardshowfilename[num], CardRecbuf.addr[num]);
    CardRecbuf.Filesum = (++num);
  }
  page_total_file = CardRecbuf.Filesum;
  CardRecbuf.Filesum = ((file_total_page - 1) * 4) + page_total_file;
}

RTS::RTS() {
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  ZERO(databuf);
}

void RTS::sdCardInit() {
  if (sdDetected()) // 如果检测到有卡插入
    card.mount();

  DEBUG_ECHOLNPGM(" card.flag.mounted=: ", card.flag.mounted);

  if (card.flag.mounted) { // 挂载成功
    fileCnt = card.get_num_items();
    card.getWorkDirName();
    if (card.filename[0] != '/') card.cdup();
    hal.watchdog_refresh(); // 解决拔插卡重启的问题，Rock_20220915

    if (fileCnt > 4) {
      file_total_page = (fileCnt / 4) + 1;
      if (file_total_page > 5) file_total_page = 5; // rock_20221412
    }
    else
      file_total_page = 1;

    sendData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
    file_current_page = 1;
    sendData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);
    RTS_line_to_filelist(); // rock_20220915
    CardRecbuf.selectFlag = false;

    // rock_20220707
    if (PoweroffContinue /*|| print_job_timer.isRunning()*/) return;

    // clean print file
    for (int j = 0; j < 20; j++) sendData(0, PRINT_FILE_TEXT_VP + j);
    lcd_sd_status = IS_SD_INSERTED();
  }
  else {
    if (PoweroffContinue) return;

    // Clean filename Icon
    for (int j = 0; j < MaxFileNumber; j++)
      for (int i = 0; i < TEXTBYTELEN; i++)
        sendData(0, CardRecbuf.addr[j] + i);

    memset(&CardRecbuf, 0, sizeof(CardRecbuf));
  }
}

bool RTS::sdDetected() {
  static bool last, state;
  static bool flag_stable;
  static uint32_t stable_point_time;

  bool tmp = IS_SD_INSERTED();

  if (tmp != last)
    flag_stable = false;
  else if (!flag_stable) {
    flag_stable = true;
    stable_point_time = millis() + 30;
  }

  if (flag_stable && ELAPSED(millis(), stable_point_time))
    state = tmp;

  last = tmp;

  return state;
}

void RTS::sdCardUpate() {
  const bool sd_status = sdDetected();

  if (sd_status != lcd_sd_status) {
    if (sd_status) {
      // SD card power on
      sdCardInit();
    }
    else {

      if (PoweroffContinue /*|| print_job_timer.isRunning()*/) return;

      card.release();
      for (int i = 0; i < CardRecbuf.Filesum; i++) {
        for (int j = 0; j < 20; j++) sendData(0, CardRecbuf.addr[i] + j);
        sendData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
      }
      for (int j = 0; j < 20; j++) {
        // clean screen.
        sendData(0, PRINT_FILE_TEXT_VP + j);
        sendData(0, SELECT_FILE_TEXT_VP + j);
      }
      memset(&CardRecbuf, 0, sizeof(CardRecbuf));
      sendData(1, PRINT_COUNT_PAGE_DATA_VP);
      file_total_page = 1;
      sendData(1, PRINT_CURRENT_PAGE_DATA_VP);
      file_current_page = 1;
    }
    lcd_sd_status = sd_status;
  }

  // represents to update file list
  if (CardUpdate && lcd_sd_status && sdDetected()) {
    RTS_line_to_filelist();
    for (uint16_t i = 0; i < 5; i++) {
      delay(1);
      sendData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
    }
    hal.watchdog_refresh();
    CardUpdate = false;
  }
}

void RTS::init() {
  #if ENABLED(MENU_RESET_WIFI)
    SET_OUTPUT(RESET_WIFI_PIN);
    OUT_WRITE(RESET_WIFI_PIN, HIGH);
  #endif

  #if PIN_EXISTS(LED_CONTROL)
    SET_OUTPUT(LED_CONTROL_PIN);
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    if (!IS_SD_INSERTED()) { delay(500); card.mount(); }
    if (IS_SD_INSERTED()) recovery.check();
  #endif

  delay(500);

  AxisUnitMode = 1;

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    bool zig = true;
    int8_t inStart, inStop, inInc, showcount;
    showcount = 0;
    settings.load();
    st_bedNozzleHeightCal.zCoordinateOffset = bedNozzleHeightCalZ;// caixiaoliang add 20210807
    for (int y = 0; y < GRID_MAX_POINTS_Y; y++) {
      // away from origin
      if (zig) {
        inStart = 0;
        inStop = GRID_MAX_POINTS_X;
        inInc = 1;
      }
      else {
        // towards origin
        inStart = GRID_MAX_POINTS_X - 1;
        inStop = -1;
        inInc = -1;
      }
      zig ^= true;
      for (int x = inStart; x != inStop; x += inInc) {
        sendData(bedlevel.z_values[x][y] * 1000, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
        showcount++;
      }
    }

  #endif // AUTO_BED_LEVELING_BILINEAR

  last_zoffset = zprobe_zoffset = probe.offset.z;
  sendData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

  for (int i = 0; i < 9; i++) sendData(0, LANGUAGE_CHINESE_TITLE_VP + i);
  sendData(1, LANGUAGE_CHINESE_TITLE_VP + ui.language);
  updateLanguageDisplay();
  delay(500);

  last_target_temperature[0] = thermalManager.degTargetHotend(0);
  last_target_temperature_bed = thermalManager.degTargetBed();
  feedrate_percentage = 100;
  sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);

  /***************turn off motor*****************/
  sendData(1, MOTOR_FREE_ICON_VP);

  /***************transmit temperature to screen*****************/
  sendData(0, HEAD_SET_TEMP_VP);
  sendData(0, BED_SET_TEMP_VP);
  sendData(thermalManager.degHotend(0), HEAD_CURRENT_TEMP_VP);
  delay(20);
  #if ENABLED(BED_TEMP_COMP)
    float bedTempDisp = thermalManager.degBed();
    if (thermalManager.degTargetBed() > 65 && thermalManager.degTargetBed() <= 127) {
      if (bedTempDisp > 65 && bedTempDisp <= 86)
        bedTempDisp -= (5 * bedTempDisp) / thermalManager.degTargetBed();
      else if (bedTempDisp > 86 && bedTempDisp <= 127)
        bedTempDisp -= (7 * bedTempDisp) / thermalManager.degTargetBed();
    }
    sendData(bedTempDisp, BED_CURRENT_TEMP_VP);
  #else
    sendData(thermalManager.degBed(), BED_CURRENT_TEMP_VP);
  #endif

  delay(20);
  sendData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
  sendData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
  sendData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
  sendData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);

  sendData(planner.settings.max_feedrate_mm_s[0], MAX_VELOCITY_XAXIS_DATA_VP);
  sendData(planner.settings.max_feedrate_mm_s[1], MAX_VELOCITY_YAXIS_DATA_VP);
  sendData(planner.settings.max_feedrate_mm_s[2], MAX_VELOCITY_ZAXIS_DATA_VP);
  sendData(planner.settings.max_feedrate_mm_s[3], MAX_VELOCITY_EAXIS_DATA_VP);

  sendData(planner.settings.max_acceleration_mm_per_s2[0], MAX_ACCEL_XAXIS_DATA_VP);
  sendData(planner.settings.max_acceleration_mm_per_s2[1], MAX_ACCEL_YAXIS_DATA_VP);
  sendData(planner.settings.max_acceleration_mm_per_s2[2], MAX_ACCEL_ZAXIS_DATA_VP);
  sendData(planner.settings.max_acceleration_mm_per_s2[3], MAX_ACCEL_EAXIS_DATA_VP);

  sendData(planner.max_jerk.x * 100, MAX_JERK_XAXIS_DATA_VP);
  sendData(planner.max_jerk.y * 100, MAX_JERK_YAXIS_DATA_VP);
  sendData(planner.max_jerk.z * 100, MAX_JERK_ZAXIS_DATA_VP);
  sendData(planner.max_jerk.e * 100, MAX_JERK_EAXIS_DATA_VP);

  sendData(planner.settings.axis_steps_per_mm[0] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
  sendData(planner.settings.axis_steps_per_mm[1] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
  sendData(planner.settings.axis_steps_per_mm[2] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
  sendData(planner.settings.axis_steps_per_mm[3] * 10, MAX_STEPSMM_EAXIS_DATA_VP);

  sendData(thermalManager.temp_hotend[0].pid.p() * 100, NOZZLE_TEMP_P_DATA_VP);
  sendData(thermalManager.temp_hotend[0].pid.i() * 100, NOZZLE_TEMP_I_DATA_VP);
  sendData(thermalManager.temp_hotend[0].pid.d() * 100, NOZZLE_TEMP_D_DATA_VP);
  sendData(thermalManager.temp_bed.pid.p() * 100, HOTBED_TEMP_P_DATA_VP);
  sendData(thermalManager.temp_bed.pid.i() * 100, HOTBED_TEMP_I_DATA_VP);
  sendData(thermalManager.temp_bed.pid.d() * 10, HOTBED_TEMP_D_DATA_VP);

  /***************transmit Fan speed to screen*****************/
  // turn off fans
  thermalManager.set_fan_speed(0, 0);
  sendData(1, PRINTER_FANOPEN_TITLE_VP);
  sendData(0, PRINTER_LEDOPEN_TITLE_VP);
  LEDStatus = false;
  delay(5);

  /*********transmit SD card filename to screen***************/
  sdCardInit();

  /***************transmit Printer information to screen*****************/
  sendData(MACHINE_NAME, MACHINE_NAME_ABOUT_TEXT_VP);
  sendData(FIRMWARE_VERSION, FIRMWARE_VERSION_ABOUT_TEXT_VP);
  //sendData(SCREEN_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);
  sendData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
  sendData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);
  delay(5);
  sendData(ui.language == 0 ? CORP_WEBSITE_C : CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);

  RTS_UpdateWifi();

  sendData(recovery.enabled ? 0 : 1, POWERCONTINUE_CONTROL_ICON_VP);

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    sendData(runout.enabled ? 0 : 1, FILAMENT_CONTROL_ICON_VP);
  #endif

  /**************************some info init*******************************/
  sendData(0, PRINT_PROCESS_ICON_VP);
  sendData(1, PREHAEAT_NOZZLE_ICON_VP);
  sendData(1, PREHAEAT_HOTBED_ICON_VP);

  sendData(ExchangePageBase, ExchangepageAddr);
  change_page_font = 0;

  for (startprogress = 0; startprogress <= 80; startprogress++) {
    sendData(startprogress, START_PROCESS_ICON_VP);
    delay(100);
    hal.watchdog_refresh();
  }
  delay(200);
  hal.watchdog_refresh();

  update_time_value = RTS_UPDATE_VALUE;
}

int RTS::receiveData() {
  static int recnum = 0;
  #define DWINBUF_MAX 256
  static bool recvflag = false;
  static uint8_t databuf[DWINBUF_MAX];

  // Parse data frame
  if (LCD_SERIAL.available() > 0 && recnum < (signed)sizeof(databuf)) {
    databuf[recnum++] = LCD_SERIAL.read();

    //#define RECV_DEBUG
    #ifdef RECV_DEBUG
      char buf[16];
      sprintf_P(buf, PSTR("%02x "), databuf[recnum - 1]);
      serial_print_P(buf);
    #endif

    // Verify the frame head
    if ((recnum == 1) && (databuf[0] != 0x5A)) {
      recnum = 0;
    }
    else if ((recnum == 2) && (databuf[1] != 0xA5)) {
      // Verify the frame head
      recnum = 0;
    }
    else if ((recnum == 4) && (databuf[3] != 0x83)) {
      // Parse only the read command 0x83 and filter out the reply command 0x82
      recnum = 0;
    }
    else if ((recnum >= 3) && (databuf[2] == (recnum - 3))) {
      // Complete resolution
      recvflag = true;

      #ifdef RECV_DEBUG
        serial_print_P("\n");
        SERIAL_ECHO_MSG("dwin rx ok");
      #endif
    }
    else if ((recnum >= 3) && ((recnum - 3) > databuf[2])) {
      // The actual received data bytes were larger than the frame data bytes, parsing failed
      recnum = 0;
    }
  }

  if (!recvflag) return -1;

  recvflag = false;

  // receive nothing
  if (recnum < 1) return -1;

  if (recdat.head[0] == databuf[0] && recdat.head[1] == databuf[1] && recnum > 2) {
    recdat.len = databuf[2];
    recdat.command = databuf[3];
    // response for writing byte
    if ((recdat.len == 0x03) && ((recdat.command == 0x82) || (recdat.command == 0x80)) && (databuf[4] == 0x4F) && (databuf[5] == 0x4B)) {
      ZERO(databuf);
      recnum = 0;
      return -1;
    }
    else if (recdat.command == 0x83) {
      // response for reading the data from the variate
      recdat.addr = databuf[4];
      recdat.addr = (recdat.addr << 8) | databuf[5];
      recdat.bytelen = databuf[6];
      for (unsigned int i = 0; i < recdat.bytelen; i += 2) {
        recdat.data[i / 2] = databuf[7 + i];
        recdat.data[i / 2] = (recdat.data[i / 2] << 8) | databuf[8 + i];
      }
    }
    else if (recdat.command == 0x81) {
      // response for reading the page from the register
      recdat.addr = databuf[4];
      recdat.bytelen = databuf[5];
      for (unsigned int i = 0; i < recdat.bytelen; i++) {
        recdat.data[i] = databuf[6 + i];
        //recdat.data[i] = (recdat.data[i] << 8 )| databuf[7 + i];
      }
    }
  }
  else {
    ZERO(databuf);
    recnum = 0;
    // receive the wrong data
    return -1;
  }
  ZERO(databuf);
  recnum = 0;
  return 2;
}

void RTS::sendData() {
  if (snddat.head[0] == FHONE && snddat.head[1] == FHTWO && snddat.len >= 3) {
    databuf[0] = snddat.head[0];
    databuf[1] = snddat.head[1];
    databuf[2] = snddat.len;
    databuf[3] = snddat.command;

    // to write data to the register
    if (snddat.command == 0x80) {
      databuf[4] = snddat.addr;
      for (int i = 0; i < (snddat.len - 2); i++) databuf[5 + i] = snddat.data[i];
    }
    else if ((snddat.len == 3) && (snddat.command == 0x81)) {
      // to read data from the register
      databuf[4] = snddat.addr;
      databuf[5] = snddat.bytelen;
    }
    else if (snddat.command == 0x82) {
      // to write data to the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      for (int i = 0; i < (snddat.len - 3); i += 2) {
        databuf[6 + i] = snddat.data[i / 2] >> 8;
        databuf[7 + i] = snddat.data[i / 2] & 0xFF;
      }
    }
    else if (snddat.len == 4 && snddat.command == 0x83) {
      // to read data from the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      databuf[6] = snddat.bytelen;
    }
    //usart_tx(LCD_SERIAL.c_dev(), databuf, snddat.len + 3);
    //LCD_SERIAL.flush();
    for (int i = 0; i < (snddat.len + 3); i++) {
      LCD_SERIAL.write(databuf[i]);
      delayMicroseconds(1);
    }

    memset(&snddat, 0, sizeof(snddat));
    ZERO(databuf);
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }
}

//void RTS::sendData(const String &s, unsigned long addr, unsigned char cmd/*=VarAddr_W*/) {
//  if (s.length() < 1) return;
//  sendData(s.c_str(), addr, cmd);
//}

void RTS::sendData(const char *str, unsigned long addr, unsigned char cmd/*=VarAddr_W*/) {
  int len = strlen(str);
  if (len > 0) {
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3 + len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for (int i = 0; i < len; i++) databuf[6 + i] = str[i];

    for (int i = 0; i < (len + 6); i++) {
      LCD_SERIAL.write(databuf[i]);
      delayMicroseconds(1);
    }
    ZERO(databuf);
  }
}

void RTS::sendData(char c, unsigned long addr, unsigned char cmd/*=VarAddr_W*/) {
  snddat.command = cmd;
  snddat.addr = addr;
  snddat.data[0] = (unsigned long)c;
  snddat.data[0] = snddat.data[0] << 8;
  snddat.len = 5;
  sendData();
}

void RTS::sendData(unsigned char *str, unsigned long addr, unsigned char cmd) { sendData((char *)str, addr, cmd); }

void RTS::sendData(int n, unsigned long addr, unsigned char cmd/*=VarAddr_W*/) {
  if (cmd == VarAddr_W) {
    if (n > 0xFFFF) {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == RegAddr_W) {
    snddat.data[0] = n;
    snddat.len = 3;
  }
  else if (cmd == VarAddr_R) {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  sendData();
}

void RTS::sendData(unsigned int n, unsigned long addr, unsigned char cmd) { sendData((int)n, addr, cmd); }

void RTS::sendData(float n, unsigned long addr, unsigned char cmd) { sendData((int)n, addr, cmd); }

void RTS::sendData(long n, unsigned long addr, unsigned char cmd) { sendData((unsigned long)n, addr, cmd); }

void RTS::sendData(unsigned long n, unsigned long addr, unsigned char cmd/*=VarAddr_W*/) {
  if (cmd == VarAddr_W) {
    if (n > 0xFFFF) {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == VarAddr_R) {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  sendData();
}

void RTS::sendText(const char string[], unsigned long addr, uint8_t textSize) {
  for (int8_t i = 0; i < textSize; i++) sendData(0, addr + i);
  sendData(string, addr);
}

void RTS::sdCardStop() {
  if (PoweroffContinue) {
    planner.synchronize();
    //card.endFilePrint();
    card.abortFilePrintSoon();
    queue.clear();
    quickstop_stepper();
    print_job_timer.stop();
    IF_DISABLED(SD_ABORT_NO_COOLDOWN, thermalManager.disable_all_heaters());
    print_job_timer.reset();
    wait_for_heatup = wait_for_user = false;
    PoweroffContinue = false;
    sd_printing_autopause = false;
    TERN_(POWER_LOSS_RECOVERY, if (card.flag.mounted) card.removeJobRecoveryFile());
    CardRecbuf.recordcount = -1;
  }
  SERIAL_ECHOLNPGM("M79 S4"); // 4:cloud print stop

  // Shut down the stepper motor.
  //queue.enqueue_now(F("M84"));
  sendData(1, MOTOR_FREE_ICON_VP);

  sendData(0, PRINT_PROCESS_ICON_VP);
  sendData(0, PRINT_PROCESS_VP);
  delay(2);
  for (int j = 0; j < 20; j++) {
    // clean screen.
    sendData(0, PRINT_FILE_TEXT_VP + j);
    // clean filename
    sendData(0, SELECT_FILE_TEXT_VP + j);
  }
  //waitway = 0;
  //sendData(ExchangePageBase + 1, ExchangepageAddr);
  //change_page_font = 1;
}

void RTS::handleData() {
  proc_command_t Checkkey = NoKey;
  // for waiting
  if (waitway > 0) {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  for (int i = 0; Addrbuf[i] != 0; i++)
    if (recdat.addr == Addrbuf[i]) {
      Checkkey = proc_command_t(i);
      break;
    }

  if (Checkkey < 0) {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }

  switch (Checkkey) {
    case MainEnterKey:

      if (recdat.data[0] == 1) {
        CardUpdate = true;
        CardRecbuf.recordcount = -1;
        if (true == g_heaterLoadTempAdd) break;
        for (int j = 0; j < 20; j++) {
          // clean screen.
          sendData(0, PRINT_FILE_TEXT_VP + j);
          // clean filename
          sendData(0, SELECT_FILE_TEXT_VP + j);
        }
        for (int j = 0; j < 20; j++) sendData(0, FILE1_SELECT_ICON_VP + j);
        sdCardUpate();

        sendData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
        file_current_page = 1;
        sendData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);

        sendData(ExchangePageBase + 2, ExchangepageAddr);
        change_page_font = 2;
        if (IS_SD_INSERTED()) RTS_line_to_filelist();
      }
      else if (recdat.data[0] == 2) {
        AxisUnitMode = 3;
        axis_unit = 0.1;
        RTS_UpdatePosition();
        sendData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
      }
      else if (recdat.data[0] == 3) {
        if (true == g_heaterLoadTempAdd) break;
        sendData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if (recdat.data[0] == 4) {
        leveling_flag = true; // Enter the leveling page
        //gcode.process_subcommands_now(F("M420 S0")); // Enter the leveling page and turn off the automatic compensation first
        sendData(ExchangePageBase + 25, ExchangepageAddr);
        change_page_font = 25;
        gcode.process_subcommands_now(F("G28"));
        gcode.process_subcommands_now(F("M420 S0")); // Enter the leveling page and turn off the automatic compensation first
        gcode.process_subcommands_now(F("G1 X110 Y110 F3000"));
        gcode.process_subcommands_now(F("G1 F600 Z0"));
      }
      else if (recdat.data[0] == 5) {
        queue.clear();
        quickstop_stepper();
        print_job_timer.stop();
        sendData(1, MOTOR_FREE_ICON_VP);
        sendData(0, PRINT_PROCESS_ICON_VP);
        sendData(0, PRINT_PROCESS_VP);

        sendData(0, PRINT_TIME_HOUR_VP);
        sendData(0, PRINT_TIME_MIN_VP);
        print_job_timer.reset();
        sd_printing_autopause = false;
        for (int j = 0; j < 20; j++) {
          // clean screen.
          sendData(0, PRINT_FILE_TEXT_VP + j);
          // clean filename
          sendData(0, SELECT_FILE_TEXT_VP + j);
        }
        CardRecbuf.recordcount = -1; //不选中
        sendData(ExchangePageBase + 1, ExchangepageAddr);// rock_20220708
        change_page_font = 1;
      }
      else if (recdat.data[0] == 6) {
        #if ENABLED(BLTOUCH)
          waitway = 3;
          sendData(ui.language + 1, BEDLEVELING_WAIT_TITLE_VP);
          sendData(ExchangePageBase + 26, ExchangepageAddr);
          change_page_font = 26;

          //RTS_ProbingPauseHotend();
          //RTS_ProbingPauseFans();

          queue.enqueue_now(F("G28\nG29"));
          sendData(0, MOTOR_FREE_ICON_VP);
        #endif
      }
      else if (recdat.data[0] == 7) {
        if (errorway == 1) {
        }
        else if (errorway == 2) {
          // auto home fail
        }
        else if (errorway == 3) {
          // bed leveling fail
        }
        else if (errorway == 4) {
        }
      }
      else if (recdat.data[0] == 8) {
        if (true == g_heaterLoadTempAdd) break;
        sendData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
      }
      break;

    case AdjustEnterKey:
      if (recdat.data[0] == 1) {
        sendData(thermalManager.fan_speed[0] ? 0 : 1, PRINTER_FANOPEN_TITLE_VP);
        sendData(ExchangePageBase + 14, ExchangepageAddr);
        change_page_font = 14;
      }
      else if (recdat.data[0] == 2) {
        if (PoweroffContinue) {
          if (card.isPrinting() || (wait_for_heatup)) {
            sendData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;
          }
          else {
            sendData(ExchangePageBase + 12, ExchangepageAddr);
            change_page_font = 12;
          }
        }
        else {
          sendData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        settings.save();
      }
      else if (recdat.data[0] == 3) {
        const bool fanOn = thermalManager.fan_speed[0];
        sendData(fanOn ? 1 : 0, PRINTER_FANOPEN_TITLE_VP);
        thermalManager.set_fan_speed(0, fanOn ? 0 : 255);
      }
      else if (recdat.data[0] == 4) {
        if (LEDStatus) {
          sendData(0, PRINTER_LEDOPEN_TITLE_VP);
          //WRITE(LED_CONTROL_PIN, HIGH);
          LEDStatus = false;
        }
        else {
          sendData(1, PRINTER_LEDOPEN_TITLE_VP);
          //WRITE(LED_CONTROL_PIN, LOW);
          LEDStatus = true;
        }
      }
      else if (recdat.data[0] == 5) {
        sendData(ExchangePageBase + 15, ExchangepageAddr);
        change_page_font = 15;
      }
      else if (recdat.data[0] == 6) {
      }
      else if (recdat.data[0] == 7) {
        sendData(ExchangePageBase + 14, ExchangepageAddr);
        change_page_font = 14;
        settings.save();
      }
      else if (recdat.data[0] == 8) {
        if (runout.enabled) {
          sendData(1, FILAMENT_CONTROL_ICON_VP);
          runout.enabled = false;
        }
        else {
          sendData(0, FILAMENT_CONTROL_ICON_VP);
          runout.enabled = true;
        }
      }
      else if (recdat.data[0] == 9) {
        if (recovery.enabled) {
          sendData(1, POWERCONTINUE_CONTROL_ICON_VP);
          recovery.enabled = false;
          if (card.flag.mounted) { // rock_20220701 Fix the bug that the switch is always on when the power is off
            #if ENABLED(POWER_LOSS_RECOVERY)
              //card.removeJobRecoveryFile();
              if (card.jobRecoverFileExists())
                //recovery.init(); // Do not clear power-off information
                card.removeFile(recovery.filename);
            #endif
          }
        }
        else {
          sendData(0, POWERCONTINUE_CONTROL_ICON_VP);
          recovery.enabled = true;
          recovery.save(true);
        }
      }
      else if (recdat.data[0] == 10) {
        sendData(ExchangePageBase + 32, ExchangepageAddr);
        change_page_font = 32;
        settings.save();
      }
      break;

    case PrintSpeedEnterKey:
      feedrate_percentage = recdat.data[0];
      sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      SERIAL_ECHOLNPGM("M220 S", feedrate_percentage);
      break;

    case StopPrintKey:
      if (recdat.data[0] == 1) { // Stop printing pop-up window entry
        if (home_flag) break;
        sendData(ExchangePageBase + 13, ExchangepageAddr);
        change_page_font = 13;
      }
      else if (recdat.data[0] == 2) { // Stop printing popup confirmation
        if (!PoweroffContinue) { // rock_20220708 Cloud printing
          SERIAL_ECHOLNPGM("ok");
          while ((planner.movesplanned() < 2) && (destination != current_position)) idle();
          queue.clear();
          delay(20);

          thermalManager.setTargetHotend(0, 0);
          sendData(0, HEAD_SET_TEMP_VP);
          thermalManager.setTargetBed(0);
          sendData(0, BED_SET_TEMP_VP);
          SERIAL_ECHOLNPGM("M79 S4");
          queue.enqueue_now(F("M79 S4")); // 4:cloud print stop
          g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
          sendData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;
        }

        if (G29_flag || home_flag) break;

        if (PoweroffContinue) {
          sendData(ExchangePageBase + 40, ExchangepageAddr);
          change_page_font = 40;
          waitway = 7;
          sendData(0, PRINT_TIME_HOUR_VP);
          sendData(0, PRINT_TIME_MIN_VP);
          thermalManager.setTargetHotend(0, 0);
          sendData(0, HEAD_SET_TEMP_VP);
          thermalManager.setTargetBed(0);
          sendData(0, BED_SET_TEMP_VP);
          temphot = 0;
          thermalManager.zero_fan_speeds();
          update_time_value = 0;
          sdCardStop();
          SERIAL_ECHOLNPGM("M79 S4");
          SERIAL_ECHOLNPGM("ok");
          queue.clear();
          queue.enqueue_now(F("G28 XY\nM84"));
          g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
        }

      }
      else if (recdat.data[0] == 3) { // Stop printing the popup and return
        if (card.isPrinting() || wait_for_heatup || home_flag) { // 暂停
          sendData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else { // continue to print
          sendData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
      }
      else if (recdat.data[0] == 4) {
        if (!planner.has_blocks_queued()) { // rock_20220401
          if (PoweroffContinue) {

            SERIAL_ECHOLNPGM("ok");

            runout.filament_ran_out = false;
            sendData(ExchangePageBase + 40, ExchangepageAddr);
            change_page_font = 40;
            waitway = 7;
            #if ENABLED(FILAMENT_RUNOUT_SENSOR)
              if (runout.enabled) {
                pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
                ui.pause_show_message(PAUSE_MESSAGE_RESUME);
                queue.inject(F("M108"));
              }
            #endif
            sendData(0, PRINT_TIME_HOUR_VP);
            sendData(0, PRINT_TIME_MIN_VP);
            update_time_value = 0;
            temphot = 0;
            card.abortFilePrintSoon();
            queue.clear();
            quickstop_stepper();

            print_job_timer.abort();
            // delay(10);
            while (planner.has_blocks_queued()) idle();
            thermalManager.setTargetHotend(0, 0);
            thermalManager.setTargetBed(0);
            thermalManager.zero_fan_speeds();
            //while (thermalManager.degTargetHotend(0) > 0) {
            //  thermalManager.setTargetHotend(0, 0);
            //  idle();
            //}
            sdCardStop();

            SERIAL_ECHOLNPGM("M79 S4"); // 4:cloud print stop
            //sendData(ExchangePageBase + 1, ExchangepageAddr);
            // change_page_font = 1;
          }
          else if (!PoweroffContinue) {
            SERIAL_ECHOLNPGM("ok");
            PoweroffContinue = true;
            runout.filament_ran_out = false;
            sendData(ExchangePageBase + 40, ExchangepageAddr);
            change_page_font = 40;
            waitway = 7;
            #if ENABLED(FILAMENT_RUNOUT_SENSOR)
              if (runout.enabled) {
                pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
                ui.pause_show_message(PAUSE_MESSAGE_RESUME);
                queue.inject(F("M108"));
              }
            #endif
            sendData(0, PRINT_TIME_HOUR_VP);
            sendData(0, PRINT_TIME_MIN_VP);
            update_time_value = 0;
            temphot = 0;
            card.abortFilePrintSoon();
            queue.clear();
            quickstop_stepper();

            print_job_timer.abort();
            // delay(10);

            while (planner.has_blocks_queued()) idle();

            thermalManager.setTargetHotend(0, 0);
            thermalManager.setTargetBed(0);
            thermalManager.zero_fan_speeds();
            //while (thermalManager.degTargetHotend(0) > 0) {
            //  thermalManager.setTargetHotend(0, 0);
            //  idle();
            //}
            sdCardStop();

            PoweroffContinue = false;
            SERIAL_ECHOLNPGM("M79 S4"); // 4:cloud print stop
            sendData(ExchangePageBase + 1, ExchangepageAddr);
            change_page_font = 1;
          }
        }
      }
      else if (recdat.data[0] == 5) { // 拔卡打印弹窗停止打印
        if (PoweroffContinue) {
          sendData(ExchangePageBase + 40, ExchangepageAddr);
          change_page_font = 40;
          waitway = 7;
          sendData(0, PRINT_TIME_HOUR_VP);
          sendData(0, PRINT_TIME_MIN_VP);
          thermalManager.setTargetHotend(0, 0);
          sendData(0, HEAD_SET_TEMP_VP);
          thermalManager.setTargetBed(0);
          sendData(0, BED_SET_TEMP_VP);
          temphot = 0;
          thermalManager.zero_fan_speeds();
          update_time_value = 0;
          sdCardStop();
        }
      }
      break;

    case PausePrintKey:
      if (recdat.data[0] == 1) {
        // 云打印功能
        if (!PoweroffContinue) {
          sendData(ExchangePageBase + 11, ExchangepageAddr);
          change_page_font = 11;
        }
        // rock_20220714 解决打印开始前反复点击暂停会重启的问题
        if (G29_flag || home_flag || wait_for_heatup) break;
        sendData(ExchangePageBase + 11, ExchangepageAddr);
        change_page_font = 11;
      }
      else if (recdat.data[0] == 2) {
        if (!PoweroffContinue) {
          sendData(ExchangePageBase + 40, ExchangepageAddr);
          SERIAL_ECHOLNPGM("M79 S2"); // 3:cloud print resume
          queue.enqueue_now(F("M79 S2")); // 2:cloud print pause
          update_time_value = 0;
          print_job_timer.pause();
          sendData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }

        if (G29_flag || home_flag || wait_for_heatup) break;

        if (PoweroffContinue && !wait_for_heatup && card.isPrinting()) {
          while (thermalManager.still_heating(0)) idle();
          sendData(ExchangePageBase + 40, ExchangepageAddr);
          waitway = 1;
          pause_e = current_position.e - 3;
          if (!temphot) temphot = thermalManager.degTargetHotend(0);
          card.pauseSDPrint();
          print_job_timer.pause();

          pause_action_flag = true;
          update_time_value = 0;
          planner.synchronize();
          sdcard_pause_check = false;
        }
      }
      else if (recdat.data[0] == 3) {
        if (card.isPrinting() || wait_for_heatup) {
          sendData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else {
          sendData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
      }
      break;

    case ResumePrintKey:
      if (recdat.data[0] == 1) {

        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if (runout.enabled && runout.filament_ran_out) {
            sendData(ExchangePageBase + 7, ExchangepageAddr);
            change_page_font = 7;
            break;
          }
        #endif

        if (PoweroffContinue) {
          sendData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;

          #if ENABLED(HAS_RESUME_CONTINUE)
            const bool was_wait = wait_for_user;
            if (was_wait) wait_for_user = false;
          #else
            constexpr bool was_wait = false;
          #endif

          if (!was_wait) {
            memset(commandbuf, 0, sizeof(commandbuf));
            sprintf_P(commandbuf, PSTR("M109 S%i"), temphot);
            queue.enqueue_one_now(commandbuf);

            card.startOrResumeFilePrinting();
            print_job_timer.start();
            update_time_value = 0;
            sdcard_pause_check = true;
          }
          SERIAL_ECHOLNPGM("M79 S3"); // 3:cloud print resume
        }
        else {
          SERIAL_ECHOLNPGM("M79 S3"); // 3:cloud print resume
          //sendData(ExchangePageBase + 40, ExchangepageAddr);
          queue.enqueue_now(F("M79 S3")); // 3:cloud print resume
        }
      }
      else if (recdat.data[0] == 2) {
        SERIAL_ECHOLNPGM("ok");
        if (G29_flag || home_flag) break;

        if (sdcard_pause_check) {
          if (thermalManager.degTargetHotend(0) >= EXTRUDE_MINTEMP) {
            //thermalManager.setTargetHotend(thermalManager.degTargetHotend(0), 0);
          }
          else if (temphot >= EXTRUDE_MINTEMP)
            thermalManager.setTargetHotend(temphot, 0);
          else
            thermalManager.setTargetHotend(200, 0);

          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if (runout.enabled && runout.filament_ran_out) {
              sendData(ExchangePageBase + 7, ExchangepageAddr);
              change_page_font = 7;
              break;
            }
            else {
              if (!planner.has_blocks_queued()) {
                sendData(ExchangePageBase + 8, ExchangepageAddr);
                change_page_font = 8;
              }
              else {
                sendData(ExchangePageBase + 7, ExchangepageAddr);
                change_page_font = 7;
                break;
              }
            }
          #endif
        }
        else if (!sdcard_pause_check) {
          if (card.flag.mounted) {
            if (runout.enabled && runout.filament_ran_out) {
              sendData(ExchangePageBase + 7, ExchangepageAddr);
              change_page_font = 7;
              break;
            }
            else {
              PoweroffContinue = true;
              char cmd[30];
              char *c;
              sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
              for (c = &cmd[4]; *c; c++) *c = tolower(*c);

              memset(cmdbuf, 0, sizeof(cmdbuf));
              strcpy(cmdbuf, cmd);
              queue.enqueue_one_now(cmd);
              queue.enqueue_now(F("M24"));

              // clean screen.
              for (int j = 0; j < 20; j++) sendData(0, PRINT_FILE_TEXT_VP + j);

              sendData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

              delay(2);

              TERN_(BABYSTEPPING, sendData(0, AUTO_BED_LEVEL_ZOFFSET_VP));

              feedrate_percentage = 100;
              sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
              sendData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
              sendData(ExchangePageBase + 10, ExchangepageAddr);
              update_time_value = 0;
              // 1:cloud print satrt
              SERIAL_ECHOLNPGM("M79 S1");
            }
          }
          else
            break;
        }
      }
      else if (recdat.data[0] == 3) {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if (runout.enabled && runout.filament_ran_out) {
            sendData(ExchangePageBase + 7, ExchangepageAddr);
            change_page_font = 7;
            break;
          }
        #endif
        if (PoweroffContinue) {
          runout.filament_ran_out = false;
          sendData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
          SERIAL_ECHOLNPGM("M79 S1");
          SERIAL_ECHOLNPGM("M79 S3");
          if (temphot > 0) {
            sendData(temphot, HEAD_SET_TEMP_VP);
            thermalManager.setTargetHotend(temphot, 0);
            if (temphot > thermalManager.degHotend(0) - 5) thermalManager.wait_for_hotend(false);
            while (thermalManager.still_heating(0)) {
              idle();
              if (card.flag.abort_sd_printing) break; // rock_20220913
            }
          }
          // Move XY to starting position, then Z
          prepare_internal_move_to_destination(NOZZLE_PARK_XY_FEEDRATE);

          // Move Z_AXIS to saved position
          prepare_internal_move_to_destination(NOZZLE_PARK_Z_FEEDRATE);
          pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
          ui.pause_show_message(PAUSE_MESSAGE_RESUME);
          queue.inject(F("M108"));

          card.startOrResumeFilePrinting();
          print_job_timer.start();
          update_time_value = 0;
          sdcard_pause_check = true;
        }
        else if (!PoweroffContinue) {
          if (CardRecbuf.recordcount > 0) {
            char cmd[30];
            char *c;
            sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
            for (c = &cmd[4]; *c; c++) *c = tolower(*c);

            PoweroffContinue = true;
            memset(cmdbuf, 0, sizeof(cmdbuf));
            strcpy(cmdbuf, cmd);
            queue.enqueue_one_now(cmd);
            delay(20);
            queue.enqueue_now(F("M24"));
            // clean screen.
            for (int j = 0; j < 20; j++) sendData(0, PRINT_FILE_TEXT_VP + j);

            sendData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

            delay(2);

            TERN_(BABYSTEPPING, sendData(0, AUTO_BED_LEVEL_ZOFFSET_VP));
            feedrate_percentage = 100;
            sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
            sendData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
            sendData(ExchangePageBase + 10, ExchangepageAddr);
            update_time_value = 0;
            // 1:cloud print satrt
            SERIAL_ECHOLNPGM("M79 S1");
            SERIAL_ECHOLNPGM("M79 S3");
          }
          else {
            runout.filament_ran_out = false;
            // Move XY to starting position, then Z
            prepare_internal_move_to_destination(NOZZLE_PARK_XY_FEEDRATE);

            // Move Z_AXIS to saved position
            prepare_internal_move_to_destination(NOZZLE_PARK_Z_FEEDRATE);
            pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
            ui.pause_show_message(PAUSE_MESSAGE_RESUME);
            //queue.inject(F("M108"));
            sendData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;

            //card.startOrResumeFilePrinting();
            print_job_timer.start();
            update_time_value = 0;
            sdcard_pause_check = true;
            SERIAL_ECHOLNPGM("ok");
            if (temphot > 0) {
              sendData(temphot, HEAD_SET_TEMP_VP);
              thermalManager.setTargetHotend(temphot, 0);
              if (temphot > thermalManager.degHotend(0) - 5) thermalManager.wait_for_hotend(false);
              while (thermalManager.still_heating(0)) idle();
            }
            SERIAL_ECHOLNPGM("M79 S3");
          }
        }
        else {
          runout.filament_ran_out = false;
          // Move XY to starting position, then Z
          prepare_internal_move_to_destination(NOZZLE_PARK_XY_FEEDRATE);

          // Move Z_AXIS to saved position
          prepare_internal_move_to_destination(NOZZLE_PARK_Z_FEEDRATE);

          pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
          ui.pause_show_message(PAUSE_MESSAGE_RESUME);

          sendData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;

          print_job_timer.start();
          update_time_value = 0;
          sdcard_pause_check = true;
        }
      }
      else if (recdat.data[0] == 4) {
        //if (card.flag.mounted)
        if (IS_SD_INSERTED()) { //有卡
          SD_Card_status = true;
          card.startOrResumeFilePrinting();
          print_job_timer.start();
          update_time_value = 0;
          sdcard_pause_check = true;
          sd_printing_autopause = false;
          HMI_lcd_flag.remove_card_flag = false;
          temp_remove_card_flag = false;

          sendData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
          gcode.process_subcommands_now(F("M24"));
        }
        else {
          CardUpdate = true;
          sdCardUpate();
          //card.mount();
          //SERIAL_ECHO_MSG("ROCK_MOVE_CARD1111\n");
          sendData(ExchangePageBase + 47, ExchangepageAddr);
          change_page_font = 47;
        }
      }
      break;

    case ZoffsetEnterKey:
      last_zoffset = zprobe_zoffset;
      if (recdat.data[0] >= 32768) {
        zprobe_zoffset = (float(recdat.data[0]) - 65536) / 100;
        zprobe_zoffset -= 0.001;
      }
      else {
        zprobe_zoffset = float(recdat.data[0]) / 100;
        zprobe_zoffset += 0.001;
      }
      if (WITHIN((zprobe_zoffset), PROBE_OFFSET_ZMIN, PROBE_OFFSET_ZMAX)) {
        babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
        SERIAL_ECHO_MSG("babystep.add_mm():", zprobe_zoffset - last_zoffset);
      }
      hal.watchdog_refresh();
      probe.offset.z = zprobe_zoffset;
      //settings.save();
      break;

    case TempControlKey:
      if (recdat.data[0] == 2) {
        sendData(ExchangePageBase + 20, ExchangepageAddr);
        change_page_font = 20;
      }
      else if (recdat.data[0] == 3) {
        sendData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
        sendData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
        delay(2);
        sendData(ExchangePageBase + 22, ExchangepageAddr);
        change_page_font = 22;
      }
      else if (recdat.data[0] == 4) {
        sendData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
        sendData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
        delay(2);
        sendData(ExchangePageBase + 23, ExchangepageAddr);
        change_page_font = 23;
      }
      else if (recdat.data[0] == 5) {
        thermalManager.setTargetHotend(ui.material_preset[0].hotend_temp, 0);
        //thermalManager.setTargetHotend(thermalManager.degTargetHotend(0), 0);
        sendData(thermalManager.degTargetHotend(0), HEAD_SET_TEMP_VP);
        thermalManager.setTargetBed(ui.material_preset[0].bed_temp);
        //thermalManager.setTargetBed(thermalManager.degTargetBed());
        #if ENABLED(BED_TEMP_COMP)
          if (WITHIN(thermalManager.degTargetBed(), 66, 85))
            sendData(thermalManager.degTargetBed() - 5, BED_SET_TEMP_VP);
          else if (WITHIN(thermalManager.degTargetBed(), 86, 127))
            sendData(thermalManager.degTargetBed() - 7, BED_SET_TEMP_VP);
        #else
          sendData(thermalManager.degTargetBed(), BED_SET_TEMP_VP);
        #endif
        //sendData(thermalManager.degTargetBed(), BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 6) {
        thermalManager.setTargetHotend(ui.material_preset[1].hotend_temp, 0);
        //thermalManager.setTargetHotend(thermalManager.degTargetHotend(0), 0);
        sendData(thermalManager.degTargetHotend(0), HEAD_SET_TEMP_VP);
        thermalManager.setTargetBed(ui.material_preset[1].bed_temp);
        //thermalManager.setTargetBed(thermalManager.degTargetBed());
        #if ENABLED(BED_TEMP_COMP)
          if (WITHIN(thermalManager.degTargetBed(), 66, 85))
            sendData(thermalManager.degTargetBed() - 5, BED_SET_TEMP_VP);
          else if (WITHIN(thermalManager.degTargetBed(), 86, 127))
            sendData(thermalManager.degTargetBed() - 7, BED_SET_TEMP_VP);
        #else
          sendData(thermalManager.degTargetBed(), BED_SET_TEMP_VP);
        #endif
      }
      else if (recdat.data[0] == 7) {
        sendData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if (recdat.data[0] == 8) {
        if (true == g_heaterLoadTempAdd) break;
        sendData(ExchangePageBase + 20, ExchangepageAddr);
        change_page_font = 20;
      }
      break;

    case CoolDownKey:
      if (recdat.data[0] == 1) {
        thermalManager.setTargetHotend(0, 0);
        sendData(0, HEAD_SET_TEMP_VP);
        thermalManager.setTargetBed(0);
        sendData(0, BED_SET_TEMP_VP);
        thermalManager.fan_speed[0] = 255;
        sendData(0, PRINTER_FANOPEN_TITLE_VP);
      }
      else if (recdat.data[0] == 2) {
        sendData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      break;

    case HeaterTempEnterKey: // rock_20220624
      temphot = recdat.data[0];
      thermalManager.setTargetHotend(temphot, 0);
      sendData(thermalManager.degTargetHotend(0), HEAD_SET_TEMP_VP);
      break;

    case HotBedTempEnterKey:
      tempbed = recdat.data[0];
      temp_bed_display = recdat.data[0];
      #if ENABLED(BED_TEMP_COMP)
        if (WITHIN(tempbed, 61, 80))
          tempbed += 5;
        else if (WITHIN(tempbed, 81, 120))
          tempbed += 7;
      #endif
      thermalManager.setTargetBed(tempbed);
      sendData(temp_bed_display, BED_SET_TEMP_VP);
      break;

    //case HeaterTempEnterKey:
    //  temphot = recdat.data[0];
    //  thermalManager.setTargetHotend(temphot, 0);
    //  //thermalManager.setTargetHotend(thermalManager.degTargetHotend(0), 0);
    //  sendData(thermalManager.degTargetHotend(0), HEAD_SET_TEMP_VP);
    //  break;

    //case HotBedTempEnterKey:
    //  thermalManager.setTargetBed(recdat.data[0]);
    //  //thermalManager.setTargetBed(thermalManager.degTargetBed());
    //  sendData(thermalManager.degTargetBed(), BED_SET_TEMP_VP);
    //  break;

    case PrepareEnterKey:
      if (recdat.data[0] == 1) {
        sendData(ExchangePageBase + 28, ExchangepageAddr);
        change_page_font = 28;
      }
      else if (recdat.data[0] == 2) {
        sendData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
      }
      else if (recdat.data[0] == 3) {
        if (true == g_heaterLoadTempAdd) break;
        RTS_UpdatePosition();
        delay(2);
        sendData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
      }
      else if (recdat.data[0] == 4) {
        // OUT_WRITE(SHUTIDOWN_PIN, LOW);
        delay(2000);
      }
      else if (recdat.data[0] == 5) {
        sendData(MACHINE_NAME, MACHINE_NAME_ABOUT_TEXT_VP);
        sendData(FIRMWARE_VERSION, FIRMWARE_VERSION_ABOUT_TEXT_VP);
        //sendData(SCREEN_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);
        sendData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
        sendData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);
        delay(5);
        sendData(ui.language == 0 ? CORP_WEBSITE_C : CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
        sendData(ExchangePageBase + 24, ExchangepageAddr);
        change_page_font = 24;
      }
      else if (recdat.data[0] == 6) {
        queue.enqueue_now(F("M84"));
        sendData(1, MOTOR_FREE_ICON_VP);
        g_uiXYAxisEnable = false;
        g_uiZAxisEnable = false;
      }
      else if (recdat.data[0] == 7) {
        sendData(ExchangePageBase + 43, ExchangepageAddr);
        change_page_font = 43;
      }
      else if (recdat.data[0] == 8) {
        settings.save();
        sendData(ExchangePageBase + 21, ExchangepageAddr);
        change_page_font = 21;
      }
      else if (recdat.data[0] == 9) {
        sendData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
      }
      else if (recdat.data[0] == 10) {
        sendData(ExchangePageBase + 42, ExchangepageAddr);
        change_page_font = 42;
      }
      else if (recdat.data[0] == 11) {
        #if ENABLED(MENU_RESET_WIFI)
          rts_wifi_state = PRESSED;
          OUT_WRITE(RESET_WIFI_PIN, LOW);
        #endif
        flag_counter_wifi_reset = true;
        sendData(ExchangePageBase + 45, ExchangepageAddr);
        change_page_font = 45;
      }
      else if (recdat.data[0] == 12) {
        sendData(ExchangePageBase + 44, ExchangepageAddr);
        change_page_font = 44;
      }
      else if (recdat.data[0] == 13) {
        settings.reset();
        settings.save();
        updateLanguageDisplay();
        for (int i = 0; i < 9; i++) sendData(0, LANGUAGE_CHINESE_TITLE_VP + i);
        sendData(1, LANGUAGE_CHINESE_TITLE_VP + ui.language);
        sendData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;

        sendData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
        sendData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
        sendData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
        sendData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
        delay(20);

        sendData(default_max_feedrate[X_AXIS], MAX_VELOCITY_XAXIS_DATA_VP);
        sendData(default_max_feedrate[Y_AXIS], MAX_VELOCITY_YAXIS_DATA_VP);
        sendData(default_max_feedrate[Z_AXIS], MAX_VELOCITY_ZAXIS_DATA_VP);
        sendData(default_max_feedrate[E_AXIS], MAX_VELOCITY_EAXIS_DATA_VP);
        delay(20);

        sendData(default_max_acceleration[X_AXIS], MAX_ACCEL_XAXIS_DATA_VP);
        sendData(default_max_acceleration[Y_AXIS], MAX_ACCEL_YAXIS_DATA_VP);
        sendData(default_max_acceleration[Z_AXIS], MAX_ACCEL_ZAXIS_DATA_VP);
        sendData(default_max_acceleration[E_AXIS], MAX_ACCEL_EAXIS_DATA_VP);
        delay(20);

        sendData(default_max_jerk[X_AXIS] * 100, MAX_JERK_XAXIS_DATA_VP);
        sendData(default_max_jerk[Y_AXIS] * 100, MAX_JERK_YAXIS_DATA_VP);
        sendData(default_max_jerk[Z_AXIS] * 100, MAX_JERK_ZAXIS_DATA_VP);
        sendData(default_max_jerk[E_AXIS] * 100, MAX_JERK_EAXIS_DATA_VP);
        delay(20);

        sendData(default_axis_steps_per_unit[X_AXIS] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
        sendData(default_axis_steps_per_unit[Y_AXIS] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
        sendData(default_axis_steps_per_unit[Z_AXIS] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
        sendData(default_axis_steps_per_unit[E_AXIS] * 10, MAX_STEPSMM_EAXIS_DATA_VP);
        delay(20);

        sendData(default_nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
        sendData(default_nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
        sendData(default_nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
        delay(20);

        sendData(default_hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
        sendData(default_hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
        sendData(default_hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
        delay(1000);
      }
      else if (recdat.data[0] == 14) {
        if (!planner.has_blocks_queued()) {
          sendData(ExchangePageBase + 33, ExchangepageAddr);
          change_page_font = 33;
        }
      }
      else if (recdat.data[0] == 15) {
        leveling_flag = false;

        gcode.process_subcommands_now(F("M420 S1")); // 退出调平界面
        if (!planner.has_blocks_queued()) {
          sendData(10 * current_position.x, AXIS_X_COORD_VP);
          sendData(10 * current_position.y, AXIS_Y_COORD_VP);
          sendData(10 * current_position.z, AXIS_Z_COORD_VP);
          sendData(ExchangePageBase + 21, ExchangepageAddr);
          change_page_font = 21;
        }
        settings.save();
        st_bedNozzleHeightCal.isBedLeveling = false;
      }
      break;

    case BedLevelKey:

      if (recdat.data[0] == 1) {
        gcode.process_subcommands_now(F("M420 S0"));
        planner.synchronize();
        queue.enqueue_now(F("G28 Z0"));
        queue.enqueue_now(F("M420 S0"));
        queue.enqueue_one(F("G1 X110 Y110 F3000"));
        queue.enqueue_now(F("G1 F200 Z0.0"));
      }
      else if (recdat.data[0] == 2) {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset + 0.05), -5.02, 5.02)) {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset + 0.05);
            zprobe_zoffset = zprobe_zoffset - 0.0001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
          sendData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        }
      }
      else if (recdat.data[0] == 3) {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset - 0.05), -5.02, 5.02)) {
          TERN_(HAS_LEVELING, zprobe_zoffset -= 0.0499);
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        sendData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if (recdat.data[0] == 4) {
        gcode.process_subcommands_now(F("M420 S0")); // Rock_20220817 进入调平页面先关掉自动补偿
        if (!planner.has_blocks_queued()) {
          sendData(ExchangePageBase + 25, ExchangepageAddr);
          change_page_font = 25;
          queue.enqueue_one(F("G1 F200 Z3.0"));
          queue.enqueue_one(F("G1 X110 Y110 F3000"));
          queue.enqueue_one(F("G1 F200 Z0.0"));
        }
      }
      else if (recdat.data[0] == 5) {
        if (home_flag) break;
        gcode.process_subcommands_now(F("M420 S0")); // Rock_20220817 进入调平页面先关掉自动补偿
        // Bed Tramming,  Centre 1
        if (!planner.has_blocks_queued()) {
          waitway = 4;
          queue.enqueue_now(F("G1 F600 Z3"));
          queue.enqueue_now(F("G1 X110 Y110 F3000"));
          queue.enqueue_now(F("G1 F600 Z0"));
          waitway = 0;
        }

      }
      else if (recdat.data[0] == 6) {
        // Bed Tramming, Front Left 2
        gcode.process_subcommands_now(F("M420 S0")); // Rock_20220817 进入调平页面先关掉自动补偿
        if (!planner.has_blocks_queued()) {
          waitway = 4;
          queue.enqueue_now(F("G1 F600 Z3"));
          queue.enqueue_now(F("G1 X25 Y25 F3000"));
          queue.enqueue_now(F("G1 F600 Z0"));
          waitway = 0;
        }
      }
      else if (recdat.data[0] == 7) {
        // Bed Tramming, Front Right 3
        gcode.process_subcommands_now(F("M420 S0"));// Rock_20220817 进入调平页面先关掉自动补偿
        if (!planner.has_blocks_queued()) {
          waitway = 4;
          queue.enqueue_now(F("G1 F600 Z3"));
          queue.enqueue_now(F("G1 X195 Y25 F3000"));
          queue.enqueue_now(F("G1 F600 Z0"));
          waitway = 0;
        }

      }
      else if (recdat.data[0] == 8) {
        if (home_flag) break;
        gcode.process_subcommands_now(F("M420 S0"));// Rock_20220817 进入调平页面先关掉自动补偿
        // Bed Tramming, Back Right 4
        if (!planner.has_blocks_queued()) {
          waitway = 4;
          queue.enqueue_now(F("G1 F600 Z3"));
          queue.enqueue_now(F("G1 X195 Y195 F3000"));
          queue.enqueue_now(F("G1 F200 Z0"));
          waitway = 0;
        }
      }
      else if (recdat.data[0] == 9) {
        // Bed Tramming, Back Left 5
        gcode.process_subcommands_now(F("M420 S0"));// Rock_20220817 进入调平页面先关掉自动补偿
        if (!planner.has_blocks_queued()) {
          waitway = 4;
          queue.enqueue_now(F("G1 F600 Z3"));
          queue.enqueue_now(F("G1 X30 Y195 F3000"));
          queue.enqueue_now(F("G1 F600 Z0"));
          waitway = 0;
        }
      }
      else if (recdat.data[0] == 10) {
        gcode.process_subcommands_now(F("M420 S0"));// Rock_20220817 进入调平页面先关掉自动补偿
        sendData(0, AUTO_BED_LEVEL_TITLE_VP);
        sendData(0, AUTO_LEVELING_PERCENT_DATA_VP);
        if (!planner.has_blocks_queued()) {
          sendData(ExchangePageBase + 26, ExchangepageAddr);
          change_page_font = 26;
        }
      }
      sendData(0, MOTOR_FREE_ICON_VP);
      break;

    case AutoHomeKey:
      if (recdat.data[0] == 1) {
        AxisUnitMode = 1;
        axis_unit = 10.0;
        sendData(ExchangePageBase + 16, ExchangepageAddr);
        change_page_font = 16;
        sendData(3, MOVEAXIS_UNIT_ICON_VP);
      }
      else if (recdat.data[0] == 2) {
        AxisUnitMode = 2;
        axis_unit = 1.0;
        sendData(ExchangePageBase + 17, ExchangepageAddr);
        change_page_font = 17;
        sendData(2, MOVEAXIS_UNIT_ICON_VP);
      }
      else if (recdat.data[0] == 3) {
        AxisUnitMode = 3;
        axis_unit = 0.1;
        sendData(ExchangePageBase + 18, ExchangepageAddr);
        change_page_font = 18;
        sendData(1, MOVEAXIS_UNIT_ICON_VP);
      }
      else if (recdat.data[0] == 4) {
        waitway = 4;
        queue.enqueue_now(F("G28 X Y"));
        g_uiXYAxisEnable = true;
        update_time_value = 0;
        sendData(0, MOTOR_FREE_ICON_VP);
      }
      else if (recdat.data[0] == 5) {
        waitway = 4;
        if ((axis_is_trusted(X_AXIS)) && (axis_is_trusted(Y_AXIS)) && (current_position.x >= 50) && (current_position.y >= 50)) {
          queue.enqueue_now(F("G28 Z"));
          g_uiZAxisEnable = true;
        }
        else {
          queue.enqueue_now(F("G28"));
          g_uiXYAxisEnable = true;
          g_uiZAxisEnable = true;
        }
        sendData(0, MOTOR_FREE_ICON_VP);
        update_time_value = 0;
      }

      break;

    case XaxismoveKey:
      if (!planner.has_blocks_queued()) {
        float x_min, x_max;
        waitway = 4;
        x_min = 0;
        x_max = X_MAX_POS; // rock_20220615 Change the X axis position not in the center
        //x_max = X_MAX_POS - 8;
        if (!g_uiXYAxisEnable) { // If the XYZ axis of the motor is not enabled, return to zero first
          g_uiXYAxisEnable = true;
          gcode.process_subcommands_now(F("G28 XY"));
          if (g_uiZAxisEnable)
            sendData(0, MOTOR_FREE_ICON_VP);
        }
        current_position.x = 0.1f * recdat.data[0];

        LIMIT(current_position.x, x_min, x_max);

        RTS_line_to_current(X_AXIS);
        sendData(10 * current_position.x, AXIS_X_COORD_VP);
        delay(1);
        sendData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      else
        sendData(10 * current_position.x, AXIS_X_COORD_VP);
      break;

    case YaxismoveKey:
      if (!planner.has_blocks_queued()) {
        float y_min, y_max;
        waitway = 4;
        y_min = 0;
        y_max = Y_MAX_POS;
        if (!g_uiXYAxisEnable) { // If the XYZ axis of the motor is not enabled, return to zero first
          g_uiXYAxisEnable = true;
          gcode.process_subcommands_now(F("G28XY"));
          if (g_uiZAxisEnable) sendData(0, MOTOR_FREE_ICON_VP);
        }

        current_position.y = constrain(0.1f * recdat.data[0], y_min, y_max);

        RTS_line_to_current(Y_AXIS);
        sendData(10 * current_position.y, AXIS_Y_COORD_VP);
        delay(1);
        sendData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      else
        sendData(10 * current_position.y, AXIS_Y_COORD_VP);

      break;

    case ZaxismoveKey:
      if (!planner.has_blocks_queued()) {
        waitway = 4;
        float z_min = Z_MIN_POS, z_max = Z_MAX_POS - probe.offset.z;
        if (!g_uiZAxisEnable) { // If the XYZ axis of the motor is not enabled, return to zero first
          g_uiZAxisEnable = true;
          g_uiXYAxisEnable = true;
          gcode.process_subcommands_now(F("G28Z"));
          if (g_uiXYAxisEnable)
            sendData(0, MOTOR_FREE_ICON_VP);
        }
        current_position.z = constrain(0.1f * recdat.data[0], z_min, z_max);
        RTS_line_to_current(Z_AXIS);
        sendData(10 * current_position.z, AXIS_Z_COORD_VP);
        delay(1);
        sendData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      else
        sendData(10 * current_position.z, AXIS_Z_COORD_VP);
      break;

    case HeaterLoadEnterKey:
      if (!planner.has_blocks_queued()) {
        queue.enqueue_now(F("G92 E0"));
        FilamentLOAD = ((float)recdat.data[0]) / 10;
        sendData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if (runout.enabled && runout.filament_ran_out) {
            sendData(ExchangePageBase + 46, ExchangepageAddr);
            change_page_font = 46;
            sendData(0, HEAD_FILAMENT_LOAD_DATA_VP);
            break;
          }
        #endif
        current_position.e += FilamentLOAD;

        if (WITHIN(thermalManager.degTargetHotend(0), EXTRUDE_MINTEMP, thermalManager.degHotend(0) - 6)) {
          //thermalManager.setTargetHotend(thermalManager.degTargetHotend(0), 0);
          sendData(thermalManager.degTargetHotend(0), HEAD_SET_TEMP_VP);
        }
        else if (WITHIN(thermalManager.degTargetHotend(0), EXTRUDE_MINTEMP, ChangeFilamentTemp - 6)) {
          thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
          sendData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
        }
        g_heaterLoadTempAdd = true;
        while (thermalManager.still_heating(0)) idle();
        g_heaterLoadTempAdd = false;
        RTS_line_to_current(E_AXIS);
        sendData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
        planner.synchronize();
      }
      else
        sendData(0, HEAD_FILAMENT_LOAD_DATA_VP);
      break;

    case HeaterUnLoadEnterKey:
      if (!planner.has_blocks_queued()) {
        queue.enqueue_now(F("G92 E0"));
        FilamentUnLOAD = ((float)recdat.data[0]) / 10;
        sendData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if (runout.enabled && runout.filament_ran_out) {
            sendData(ExchangePageBase + 46, ExchangepageAddr);
            change_page_font = 46;
            sendData(0, HEAD_FILAMENT_UNLOAD_DATA_VP);
            break;
          }
        #endif
        current_position.e -= FilamentUnLOAD;

        if (WITHIN(thermalManager.degTargetHotend(0), EXTRUDE_MINTEMP, thermalManager.degHotend(0) - 6)) {
          //thermalManager.setTargetHotend(thermalManager.degTargetHotend(0), 0);
          sendData(thermalManager.degTargetHotend(0), HEAD_SET_TEMP_VP);
        }
        else if (WITHIN(thermalManager.degTargetHotend(0), EXTRUDE_MINTEMP, ChangeFilamentTemp - 6)) {
          thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
          sendData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
        }
        g_heaterLoadTempAdd = true;
        while (thermalManager.still_heating(0)) idle();
        g_heaterLoadTempAdd = false;
        RTS_line_to_current(E_AXIS);
        sendData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
        planner.synchronize();
      }
      else
        sendData(0, HEAD_FILAMENT_UNLOAD_DATA_VP);
      break;

    case HeaterLoadStartKey:
      if (recdat.data[0] == 1) {
        if (!planner.has_blocks_queued()) {
          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if (runout.enabled && runout.filament_ran_out) {
              sendData(ExchangePageBase + 46, ExchangepageAddr);
              change_page_font = 46;
              break;
            }
          #endif

          if (WITHIN(thermalManager.degTargetHotend(0), EXTRUDE_MINTEMP, thermalManager.degHotend(0) - 6)) {
            //thermalManager.setTargetHotend(thermalManager.degTargetHotend(0), 0);
            sendData(thermalManager.degTargetHotend(0), HEAD_SET_TEMP_VP);
            break;
          }
          else if (WITHIN(thermalManager.degTargetHotend(0), EXTRUDE_MINTEMP, ChangeFilamentTemp - 6)) {
            thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
            sendData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
            break;
          }
          else {
            RTS_line_to_current(E_AXIS);
            planner.synchronize();
          }
          sendData(ExchangePageBase + 19, ExchangepageAddr);
          change_page_font = 19;
        }
      }
      else if (recdat.data[0] == 2) {
        if (!planner.has_blocks_queued()) {
          sendData(ExchangePageBase + 19, ExchangepageAddr);
          change_page_font = 19;
        }
      }
      else if (recdat.data[0] == 3) {
        queue.enqueue_now(F("G92 E0")); // rock_20210927 liuxu  Reset Extruder
        sendData(ExchangePageBase + 19, ExchangepageAddr);
        change_page_font = 19;
        sendData(0, HEAD_FILAMENT_LOAD_DATA_VP);
        sendData(0, HEAD_FILAMENT_UNLOAD_DATA_VP);
      }
      break;

    case SelectLanguageKey:
      if (recdat.data[0] > 0) ui.set_language(recdat.data[0] - 1);
      for (int i = 0; i < 9; i++) sendData(0, LANGUAGE_CHINESE_TITLE_VP + i);
      sendData(1, LANGUAGE_CHINESE_TITLE_VP + ui.language);
      updateLanguageDisplay();
      settings.save();
      break;

    case PowerContinuePrintKey:
      if (recdat.data[0] == 1) {
        if (recovery.recovery_flag && PoweroffContinue) {
          power_off_type_yes = true;
          update_time_value = 0;
          sendData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
          //recovery.resume();
          queue.enqueue_now(F("M1000"));

          sdcard_pause_check = true;
          zprobe_zoffset = probe.offset.z;
          sendData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          g_cloudPLRStatusValue = CLOUD_PLR_YES_STATE;
          // 3:cloud print resume
          SERIAL_ECHOLNPGM("M79 S3");
        }
        else if (!PoweroffContinue) {
          SERIAL_ECHOLNPGM("M79 S3");
          queue.enqueue_now(F("M79 S3")); // 3:cloud print resume
        }
      }
      else if (recdat.data[0] == 2) {
        if (PoweroffContinue) {
          update_time_value = RTS_UPDATE_VALUE;
          sendData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;

          sendData(0, PRINT_TIME_HOUR_VP);
          sendData(0, PRINT_TIME_MIN_VP);
          update_time_value = 0;
          sdCardStop();
          g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
        }
        else if (!PoweroffContinue) {
          SERIAL_ECHOLNPGM("M79 S4");
          queue.enqueue_now(F("M79 S4")); // 4:cloud print stop
        }
      }
      break;

    case PLAHeadSetEnterKey:
      ui.material_preset[0].hotend_temp = recdat.data[0];
      sendData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
      break;

    case PLABedSetEnterKey:
      ui.material_preset[0].bed_temp = recdat.data[0];
      sendData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
      break;

    case ABSHeadSetEnterKey:
      ui.material_preset[1].hotend_temp = recdat.data[0];
      sendData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
      break;

    case ABSBedSetEnterKey:
      ui.material_preset[1].bed_temp = recdat.data[0];
      sendData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
      break;

    case StoreMemoryKey:
      if (recdat.data[0] == 1) {
        sendData(ExchangePageBase + 37, ExchangepageAddr);
        change_page_font = 37;
      }
      if (recdat.data[0] == 2) {
        //queue.enqueue_now(F("M502"));
        ui.set_language(1);
        updateLanguageDisplay();
        for (int i = 0; i < 9; i++) sendData(0, LANGUAGE_CHINESE_TITLE_VP + i);
        sendData(1, LANGUAGE_CHINESE_TITLE_VP + ui.language);
        last_zoffset = zprobe_zoffset = probe.offset.z = 0;
        sendData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        sendData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        // settings.save();
        ui.material_preset[0].hotend_temp = PREHEAT_1_TEMP_HOTEND;
        ui.material_preset[0].bed_temp = PREHEAT_1_TEMP_BED;
        ui.material_preset[1].hotend_temp = PREHEAT_2_TEMP_HOTEND;
        ui.material_preset[1].bed_temp = PREHEAT_2_TEMP_BED;
        sendData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
        sendData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
        sendData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
        sendData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
        delay(20);

        sendData(default_max_feedrate[X_AXIS], MAX_VELOCITY_XAXIS_DATA_VP);
        sendData(default_max_feedrate[Y_AXIS], MAX_VELOCITY_YAXIS_DATA_VP);
        sendData(default_max_feedrate[Z_AXIS], MAX_VELOCITY_ZAXIS_DATA_VP);
        sendData(default_max_feedrate[E_AXIS], MAX_VELOCITY_EAXIS_DATA_VP);
        delay(20);

        sendData(default_max_acceleration[X_AXIS], MAX_ACCEL_XAXIS_DATA_VP);
        sendData(default_max_acceleration[Y_AXIS], MAX_ACCEL_YAXIS_DATA_VP);
        sendData(default_max_acceleration[Z_AXIS], MAX_ACCEL_ZAXIS_DATA_VP);
        sendData(default_max_acceleration[E_AXIS], MAX_ACCEL_EAXIS_DATA_VP);
        delay(20);

        sendData(default_max_jerk[X_AXIS] * 100, MAX_JERK_XAXIS_DATA_VP);
        sendData(default_max_jerk[Y_AXIS] * 100, MAX_JERK_YAXIS_DATA_VP);
        sendData(default_max_jerk[Z_AXIS] * 100, MAX_JERK_ZAXIS_DATA_VP);
        sendData(default_max_jerk[E_AXIS] * 100, MAX_JERK_EAXIS_DATA_VP);
        delay(20);

        sendData(default_axis_steps_per_unit[X_AXIS] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
        sendData(default_axis_steps_per_unit[Y_AXIS] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
        sendData(default_axis_steps_per_unit[Z_AXIS] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
        sendData(default_axis_steps_per_unit[E_AXIS] * 10, MAX_STEPSMM_EAXIS_DATA_VP);
        delay(20);

        sendData(default_nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
        sendData(default_nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
        sendData(default_nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
        delay(20);
        sendData(default_hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
        sendData(default_hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
        sendData(default_hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
        delay(20);
        settings.reset(); // rock_20220816
        delay(50);
        settings.save();
        delay(200);
      }
      else if (recdat.data[0] == 3) {
        sendData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
      }
      else if (recdat.data[0] == 4) {
        sendData(ExchangePageBase + 34, ExchangepageAddr);
        change_page_font = 34;
      }
      else if (recdat.data[0] == 5) {
        sendData(ExchangePageBase + 39, ExchangepageAddr);
        change_page_font = 39;
      }
      else if (recdat.data[0] == 6) {
        wifi_enable_flag ^= true;
        queue.inject(F("M115")); // TODO: Report new WIFI state (not all of M115)
        RTS_UpdateWifi();
        settings.save();
      }
      else if (recdat.data[0] == 7) {
        sendData(ExchangePageBase + 38, ExchangepageAddr);
        change_page_font = 38;
      }
      else if (recdat.data[0] == 8) {
        sendData(ExchangePageBase + 36, ExchangepageAddr);
        change_page_font = 36;
      }
      else if (recdat.data[0] == 9) {
        sendData(ExchangePageBase + 37, ExchangepageAddr);
        change_page_font = 37;
      }
      else if (recdat.data[0] == 10) {
        sendData(ExchangePageBase + 35, ExchangepageAddr);
        change_page_font = 35;
      }
      else if (recdat.data[0] == 11) {
        sendData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        settings.save();
        delay(1000);
      }
      else if (recdat.data[0] == 12) {
        sendData(ExchangePageBase + 33, ExchangepageAddr);
        change_page_font = 33;
        settings.save();
        delay(1000);
      }
      break;

    case FanSpeedEnterKey:
      thermalManager.fan_speed[0] = recdat.data[0];
      sendData(thermalManager.fan_speed[0], FAN_SPEED_CONTROL_DATA_VP);
      break;

    case VelocityXaxisEnterKey: {
      const float velocity_xaxis = recdat.data[0];
      sendData(velocity_xaxis, MAX_VELOCITY_XAXIS_DATA_VP);
      planner.set_max_feedrate(X_AXIS, velocity_xaxis);
    } break;

    case VelocityYaxisEnterKey: {
      const float velocity_yaxis = recdat.data[0];
      sendData(velocity_yaxis, MAX_VELOCITY_YAXIS_DATA_VP);
      planner.set_max_feedrate(Y_AXIS, velocity_yaxis);
    } break;

    case VelocityZaxisEnterKey: {
      const float velocity_zaxis = recdat.data[0];
      sendData(velocity_zaxis, MAX_VELOCITY_ZAXIS_DATA_VP);
      planner.set_max_feedrate(Z_AXIS, velocity_zaxis);
    } break;

    case VelocityEaxisEnterKey: {
      const float velocity_eaxis = recdat.data[0];
      sendData(velocity_eaxis, MAX_VELOCITY_EAXIS_DATA_VP);
      planner.set_max_feedrate(E_AXIS, velocity_eaxis);
    } break;

    case AccelXaxisEnterKey: {
      const float accel_xaxis = recdat.data[0];
      sendData(accel_xaxis, MAX_ACCEL_XAXIS_DATA_VP);
      planner.set_max_acceleration(X_AXIS, accel_xaxis);
    } break;

    case AccelYaxisEnterKey: {
      const float accel_yaxis = recdat.data[0];
      sendData(accel_yaxis, MAX_ACCEL_YAXIS_DATA_VP);
      planner.set_max_acceleration(Y_AXIS, accel_yaxis);
    } break;

    case AccelZaxisEnterKey: {
      const float accel_zaxis = recdat.data[0];
      sendData(accel_zaxis, MAX_ACCEL_ZAXIS_DATA_VP);
      planner.set_max_acceleration(Z_AXIS, accel_zaxis);
    } break;

    case AccelEaxisEnterKey: {
      const float accel_eaxis = recdat.data[0];
      sendData(accel_eaxis, MAX_ACCEL_EAXIS_DATA_VP);
      planner.set_max_acceleration(E_AXIS, accel_eaxis);
    } break;

    case JerkXaxisEnterKey: {
      const float jerk_xaxis = (float)recdat.data[0] / 100;
      sendData(jerk_xaxis * 100, MAX_JERK_XAXIS_DATA_VP);
      planner.set_max_jerk(X_AXIS, jerk_xaxis);
    } break;

    case JerkYaxisEnterKey: {
      const float jerk_yaxis = (float)recdat.data[0] / 100;
      sendData(jerk_yaxis * 100, MAX_JERK_YAXIS_DATA_VP);
      planner.set_max_jerk(Y_AXIS, jerk_yaxis);
    } break;

    case JerkZaxisEnterKey: {
      const float jerk_zaxis = (float)recdat.data[0] / 100;
      sendData(jerk_zaxis * 100, MAX_JERK_ZAXIS_DATA_VP);
      planner.set_max_jerk(Z_AXIS, jerk_zaxis);
    } break;

    case JerkEaxisEnterKey: {
      const float jerk_eaxis = (float)recdat.data[0] / 100;
      sendData(jerk_eaxis * 100, MAX_JERK_EAXIS_DATA_VP);
      planner.set_max_jerk(E_AXIS, jerk_eaxis);
    } break;

    case StepsmmXaxisEnterKey: {
      const float stepsmm_xaxis = (float)recdat.data[0] / 10;
      sendData(stepsmm_xaxis * 10, MAX_STEPSMM_XAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[X_AXIS] = stepsmm_xaxis;
    } break;

    case StepsmmYaxisEnterKey: {
      const float stepsmm_yaxis = (float)recdat.data[0] / 10;
      sendData(stepsmm_yaxis * 10, MAX_STEPSMM_YAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[Y_AXIS] = stepsmm_yaxis;
    } break;

    case StepsmmZaxisEnterKey: {
      const float stepsmm_zaxis = (float)recdat.data[0] / 10;
      sendData(stepsmm_zaxis * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[Z_AXIS] = stepsmm_zaxis;
    } break;

    case StepsmmEaxisEnterKey: {
      const float stepsmm_eaxis = (float)recdat.data[0] / 10;
      sendData(stepsmm_eaxis * 10, MAX_STEPSMM_EAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[E_AXIS] = stepsmm_eaxis;
    } break;

    case NozzlePTempEnterKey: {
      const float nozzle_ptemp = (float)recdat.data[0] / 100;
      sendData(nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
      thermalManager.temp_hotend[0].pid.set_Kp(nozzle_ptemp);
    } break;

    case NozzleITempEnterKey: {
      const float nozzle_itemp = (float)recdat.data[0] / 100;
      sendData(nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
      thermalManager.temp_hotend[0].pid.set_Ki(nozzle_itemp);
    } break;

    case NozzleDTempEnterKey: {
      const float nozzle_dtemp = (float)recdat.data[0] / 100;
      sendData(nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
      thermalManager.temp_hotend[0].pid.set_Kd(nozzle_dtemp);
    } break;

    case HotbedPTempEnterKey: {
      const float hotbed_ptemp = (float)recdat.data[0] / 100;
      sendData(hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
      thermalManager.temp_bed.pid.set_Kp(hotbed_ptemp);
    } break;

    case HotbedITempEnterKey: {
      const float hotbed_itemp = (float)recdat.data[0] / 100;
      sendData(hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
      thermalManager.temp_bed.pid.set_Ki(hotbed_itemp);
    } break;

    case HotbedDTempEnterKey: {
      const float hotbed_dtemp = (float)recdat.data[0] / 10;
      sendData(hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
      thermalManager.temp_bed.pid.set_Kd(hotbed_dtemp);
    } break;

    case SelectFileKey:
      if (sdDetected()) {
        if (recdat.data[0] > CardRecbuf.Filesum) break;
        CardRecbuf.selectFlag = true;
        CardRecbuf.recordcount = recdat.data[0] - 1;

        for (int j = 0; j < 20; j++) sendData(0, SELECT_FILE_TEXT_VP + j);
        delay(2);
        for (int j = 1; j <= CardRecbuf.Filesum; j++) {
          sendData((unsigned long)0x073F, FilenameNature + j * 16);
          sendData(0, FILE1_SELECT_ICON_VP - 1 + j);
        }
        sendData((unsigned long)0xFFFF, FilenameNature + recdat.data[0] * 16);
        sendData(1, FILE1_SELECT_ICON_VP + (recdat.data[0] - 1));
      }

      sendData(ExchangePageBase + 1, ExchangepageAddr);
      change_page_font = 1;
      delay(20);
      sendData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], SELECT_FILE_TEXT_VP);
      break;

    case StartFileKey:
      if ((recdat.data[0] == 1) && sdDetected()) {

        if (CardRecbuf.recordcount < 0 || !CardRecbuf.selectFlag) break;

        if (card.flag.mounted) {
          char cmd[30];
          char *c;
          sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
          for (c = &cmd[4]; *c; c++) *c = tolower(*c);

          memset(cmdbuf, 0, sizeof(cmdbuf));
          strcpy(cmdbuf, cmd);
          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if (runout.enabled && runout.filament_ran_out) {
              sendData(ExchangePageBase + 7, ExchangepageAddr);
              change_page_font = 7;
              sdcard_pause_check = false;
              break;
            }
          #endif
          PoweroffContinue = true;
          queue.enqueue_one_now(cmd);
          delay(20);
          queue.enqueue_now(F("M24"));
          // clean screen.
          for (int j = 0; j < 20; j++) sendData(0, PRINT_FILE_TEXT_VP + j);

          sendData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

          delay(2);
          //DEBUG_ECHO_MSG("\nrock_20220411_check_filament\n");
          wait_for_heatup = wait_for_user = false;
          TERN_(BABYSTEPPING, sendData(0, AUTO_BED_LEVEL_ZOFFSET_VP));
          feedrate_percentage = 100;
          sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          sendData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          sendData(ExchangePageBase + 10, ExchangepageAddr);
          queue.enqueue_now(F("G28 X Y"));// rock_20220707 解决选中文件打印加热烫平台的问题
          update_time_value = 0;
          // 1:cloud print start
          SERIAL_ECHOLNPGM("M79 S1");
        }
        else
          break;
      }
      else if (recdat.data[0] == 2) {
        if (!planner.has_blocks_queued()) {
          sendData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          if ((file_total_page > file_current_page) && (file_current_page < (MaxFileNumber / 4)))
            file_current_page++;
          else
            break;
          sendData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);
          sendData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;

          if (card.flag.mounted)
            RTS_line_to_filelist();
        }
      }
      else if (recdat.data[0] == 3) {
        if (!planner.has_blocks_queued()) {
          sendData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          if (file_current_page > 1)
            file_current_page--;
          else
            break;
          sendData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);
          sendData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;

          if (card.flag.mounted)
            RTS_line_to_filelist();
        }
      }
      else if (recdat.data[0] == 4) {
        if (!planner.has_blocks_queued()) {
          sendData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          file_current_page = 1;
          sendData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);

          RTS_line_to_filelist();

          sendData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;
        }
      }
      else if (recdat.data[0] == 5) {
        if (!planner.has_blocks_queued()) {
          sendData(file_total_page, PRINT_COUNT_PAGE_DATA_VP);
          file_current_page = file_total_page;
          sendData(file_current_page, PRINT_CURRENT_PAGE_DATA_VP);

          RTS_line_to_filelist();

          sendData(ExchangePageBase + 2, ExchangepageAddr);
          change_page_font = 2;
        }
      }
      break;

    case ChangePageKey:
      sendData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

      // represents to update file list
      if (CardUpdate && lcd_sd_status && IS_SD_INSERTED()) {
        RTS_line_to_filelist();
        for (uint16_t i = 0; i < 5; i++) {
          delay(1);
          sendData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
          sendData(0, FILE1_SELECT_ICON_VP + i);
        }
      }

      sendData(MACHINE_NAME, MACHINE_NAME_ABOUT_TEXT_VP);
      sendData(FIRMWARE_VERSION, FIRMWARE_VERSION_ABOUT_TEXT_VP);
      //sendData(SCREEN_VERSION, PRINTER_DISPLAY_VERSION_TEXT_VP);
      sendData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
      sendData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);

      sendData(ui.language == 0 ? CORP_WEBSITE_C : CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);

      sendData(thermalManager.fan_speed[0] ? 1 : 0, PRINTER_FANOPEN_TITLE_VP);
      sendData(LEDStatus ? 1 : 0, PRINTER_LEDOPEN_TITLE_VP);

      //Percentrecord = card.percentDone() + 1;
      //if (Percentrecord <= 100) {
      //  sendData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
      //}
      sendData(card.percentDone(), PRINT_PROCESS_VP);

      sendData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

      sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      sendData(thermalManager.degTargetHotend(0), HEAD_SET_TEMP_VP);
      #if ENABLED(BED_TEMP_COMP)
        if (WITHIN(thermalManager.degTargetBed(), 66, 85))
          sendData(thermalManager.degTargetBed() - 5, BED_SET_TEMP_VP);
        else if (WITHIN(thermalManager.degTargetBed(), 86, 127))
          sendData(thermalManager.degTargetBed() - 7, BED_SET_TEMP_VP);
      #else
        sendData(thermalManager.degTargetBed(), BED_SET_TEMP_VP);
      #endif
      //sendData(thermalManager.degTargetBed(), BED_SET_TEMP_VP);
      updateLanguageDisplay();

      sendData(change_page_font + ExchangePageBase, ExchangepageAddr);
      break;

    case ErrorKey:
      if (recdat.data[0] == 1) {
        if (printingIsActive()) {
          sendData(ExchangePageBase + 10, ExchangepageAddr);
          change_page_font = 10;
        }
        else if (printingIsPaused()) {
          sendData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
        else {
          sendData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;
        }

        if (errorway == 4) hal.reboot(); // reboot
      }
      break;

    default: break;
  }
  memset(&recdat, 0, sizeof(recdat));
  recdat.head[0] = FHONE;
  recdat.head[1] = FHTWO;
}

/**
 * @brief Update the state of the UI
 * @details Called frequently from RTS_Update()
 */
void RTS::eachMomentUpdate() {
  const millis_t ms = millis();

  if (flag_counter_wifi_reset) {
    millis_t ms_3 = millis();
    if (ms_3 > next_wifireset_update_ms) {
      next_wifireset_update_ms = ms_3 + 1000;
      wifiresetcount_ms++;
      sendData((TIME_WIFI_RESET_BACKPAGE - wifiresetcount_ms), WIFI_RESET_REMAIN_TIME_DATA_VP);
    }

    if (wifiresetcount_ms > TIME_WIFI_RESET_BACKPAGE) {
      flag_counter_wifi_reset = false;
      sendData(0, WIFI_CONNECTED_DISPLAY_ICON_VP);
      sendData(ExchangePageBase + 33, ExchangepageAddr);
      change_page_font = 33;
    }
  }
  else
    wifiresetcount_ms = 0;

  if (ms > next_rts_update_ms) {
    // print the file before the power is off.
    if (!power_off_type_yes && lcd_sd_status && recovery.recovery_flag) {
      sendData(ExchangePageBase, ExchangepageAddr);
      if (startprogress < 80) sendData(startprogress, START_PROCESS_ICON_VP);
      delay(30);
      if (++startprogress > 80) {
        sendData(StartSoundSet, SoundAddr);
        power_off_type_yes = true;

        fileCnt = card.get_num_items();
        card.getWorkDirName();
        if (card.filename[0] != '/') card.cdup();

        for (uint16_t i = 0; (i < fileCnt) && (i < MaxFileNumber); i++) {
          card.selectFileByIndex(fileCnt - 1 - i);
          char *pointFilename = card.longFilename;
          int filenamelen = strlen(card.longFilename);
          int j = 1;
          while ((strncmp(&pointFilename[j], ".gcode", 6) && strncmp(&pointFilename[j], ".GCODE", 6)) && ((j++) < filenamelen));

          for (int j = 0; j < 20; j++) sendData(0, PRINT_FILE_TEXT_VP + j);

          if (j >= TEXTBYTELEN) {
            strncpy(&card.longFilename[TEXTBYTELEN - 3], "..", 2);
            card.longFilename[TEXTBYTELEN - 1] = '\0';
            j = TEXTBYTELEN - 1;
          }

          strncpy(CardRecbuf.Cardshowfilename[i], card.longFilename, j);

          strcpy(CardRecbuf.Cardfilename[i], card.filename);
          CardRecbuf.addr[i] = PRINT_FILE_TEXT_VP + 20;
          sendData(CardRecbuf.Cardshowfilename[i], CardRecbuf.addr[i]);
          if (!strcmp(CardRecbuf.Cardfilename[i], &recovery.info.sd_filename[1])) {
            sendData(CardRecbuf.Cardshowfilename[i], PRINT_FILE_TEXT_VP);
            break;
          }
        }
        sendData(ExchangePageBase + 27, ExchangepageAddr);
        change_page_font = 27;
        PoweroffContinue = true;
        g_cloudPLRStatusValue = CLOUD_PLR_CHOOSE_STATE; // 云打印断电续打的选择界面
        SERIAL_ECHOLNPGM("M79 S6"); // 6:cloud print power continue
      }
      return;
    }

    if (!power_off_type_yes && !recovery.recovery_flag) {
      sendData(ExchangePageBase, ExchangepageAddr);
      if (startprogress < 80) sendData(startprogress, START_PROCESS_ICON_VP);
      delay(30);
      if (++startprogress > 80) {
        sendData(StartSoundSet, SoundAddr);
        power_off_type_yes = true;
        update_time_value = RTS_UPDATE_VALUE;
        sendData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
      }
      return;
    }

    static unsigned char last_cardpercentValue = 100;
    #if HAS_MEDIA
      if (PoweroffContinue) {
        duration_t elapsed = print_job_timer.duration();
        sendData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
        sendData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
      }
    #endif

    const uint8_t pct = card.percentDone();

    if (card.isPrinting() && last_cardpercentValue != pct) {
      if (pct > 0) {
        Percentrecord = pct;
        if (Percentrecord <= 100)
          sendData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
      }
      else
        sendData(0, PRINT_PROCESS_ICON_VP);

      sendData(pct, PRINT_PROCESS_VP);
      last_cardpercentValue = pct;
      sendData(10.0f * current_position.z, AXIS_Z_COORD_VP);
    }

    if (pause_action_flag && !sdcard_pause_check && printingIsPaused() && !planner.has_blocks_queued()) {
      pause_action_flag = false;
      queue.enqueue_now(F("G0 F3000 X220 Y220")); // rock_20220704
      thermalManager.setTargetHotend(0, 0);
      sendData(0, HEAD_SET_TEMP_VP);
    }

    sendData(thermalManager.degHotend(0), HEAD_CURRENT_TEMP_VP);

    #if ENABLED(BED_TEMP_COMP)
      float bedTempDisp = thermalManager.degBed();
      if (WITHIN(thermalManager.degTargetBed(), 66, 127)) {
        if (WITHIN(bedTempDisp, 66, 86))
          bedTempDisp -= (5 * bedTempDisp) / thermalManager.degTargetBed();
        else if (WITHIN(bedTempDisp, 87, 127))
          bedTempDisp -= (7 * bedTempDisp) / thermalManager.degTargetBed();
      }
      sendData(bedTempDisp, BED_CURRENT_TEMP_VP);
    #else
      sendData(thermalManager.degBed(), BED_CURRENT_TEMP_VP);
    #endif

    #if HAS_MEDIA

      if (sd_printing_autopause && PoweroffContinue) {
        if (sdcard_pause_check) {
          sendData(0, CHANGE_SDCARD_ICON_VP);
          sdcard_pause_check = false;
        }
      }

      //if (!sdcard_pause_check && !card.isPrinting() && !planner.has_blocks_queued())
      if (HMI_lcd_flag.remove_card_flag) {
        //if (card.flag.mounted)
        if (temp_remove_card_flag && !planner.has_blocks_queued()) {
          queue.inject(F("G1 F1200 X220 Y220"));
          temp_remove_card_flag = false;
          //queue.enqueue_now(F("G28 X220 Y220"));
        }
        sendData(IS_SD_INSERTED() ? 1 : 0, CHANGE_SDCARD_ICON_VP);
      }

    #endif // HAS_MEDIA

    if (last_target_temperature[0] != thermalManager.degTargetHotend(0) || last_target_temperature_bed != thermalManager.degTargetBed()) {
      //thermalManager.setTargetHotend(thermalManager.degTargetHotend(0), 0);
      //thermalManager.setTargetBed(thermalManager.degTargetBed());
      sendData(thermalManager.degTargetHotend(0), HEAD_SET_TEMP_VP);
      #if ENABLED(BED_TEMP_COMP)
        if (WITHIN(thermalManager.degTargetBed(), 66, 85))
          sendData(thermalManager.degTargetBed() - 5, BED_SET_TEMP_VP);
        else if (WITHIN(thermalManager.degTargetBed(), 86, 127))
          sendData(thermalManager.degTargetBed() - 7, BED_SET_TEMP_VP);
      #else
        sendData(thermalManager.degTargetBed(), BED_SET_TEMP_VP);
      #endif
      //sendData(thermalManager.degTargetBed(), BED_SET_TEMP_VP);
      last_target_temperature[0] = thermalManager.degTargetHotend(0);
      last_target_temperature_bed = thermalManager.degTargetBed();
    }

    //static bool heatway = false;
    //if (heatway && thermalManager.degHotend(0) >= thermalManager.degTargetHotend(0)) {
    //  sendData(ExchangePageBase + 19, ExchangepageAddr);
    //  change_page_font = 19;
    //  heatway = false;
    //  sendData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
    //  sendData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
    //}

    TERN_(FILAMENT_RUNOUT_SENSOR, sendData(runout.filament_ran_out ? 0 : 1, FILAMENT_LOAD_ICON_VP));

    next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + update_time_value;
  }
}

void RTS::updateLanguageDisplay() {
  auto RTS_SndLangData = [](const uint16_t v) {
    sendData(ui.language + 1, v);
  };
  RTS_SndLangData(MAIN_PAGE_BLUE_TITLE_VP);
  RTS_SndLangData(SELECT_FILE_BLUE_TITLE_VP);
  RTS_SndLangData(PREPARE_PAGE_BLUE_TITLE_VP);
  RTS_SndLangData(SETTING_PAGE_BLUE_TITLE_VP);
  RTS_SndLangData(MAIN_PAGE_BLACK_TITLE_VP);
  RTS_SndLangData(SELECT_FILE_BLACK_TITLE_VP);
  RTS_SndLangData(PREPARE_PAGE_BLACK_TITLE_VP);
  RTS_SndLangData(SETTING_PAGE_BLACK_TITLE_VP);

  RTS_SndLangData(PRINT_ADJUST_MENT_TITLE_VP);
  RTS_SndLangData(PRINT_SPEED_TITLE_VP);
  RTS_SndLangData(HEAD_SET_TITLE_VP);
  RTS_SndLangData(BED_SET_TITLE_VP);
  RTS_SndLangData(LEVEL_ZOFFSET_TITLE_VP);
  RTS_SndLangData(FAN_CONTROL_TITLE_VP);
  RTS_SndLangData(LED_CONTROL_TITLE_VP);

  RTS_SndLangData(MOVE_AXIS_ENTER_GREY_TITLE_VP);
  RTS_SndLangData(CHANGE_FILAMENT_GREY_TITLE_VP);
  RTS_SndLangData(PREHEAT_PAGE_GREY_TITLE_VP);
  RTS_SndLangData(MOVE_AXIS_ENTER_BLACK_TITLE_VP);
  RTS_SndLangData(CHANGE_FILAMENT_BLACK_TITLE_VP);
  RTS_SndLangData(PREHEAT_PAGE_BLACK_TITLE_VP);

  RTS_SndLangData(PREHEAT_PLA_BUTTON_TITLE_VP);
  RTS_SndLangData(PREHEAT_ABS_BUTTON_TITLE_VP);
  RTS_SndLangData(COOL_DOWN_BUTTON_TITLE_VP);

  RTS_SndLangData(FILAMENT_LOAD_BUTTON_TITLE_VP);
  RTS_SndLangData(FILAMENT_UNLOAD_BUTTON_TITLE_VP);

  RTS_SndLangData(LANGUAGE_SELECT_ENTER_VP);
  RTS_SndLangData(FACTORY_DEFAULT_ENTER_TITLE_VP);
  RTS_SndLangData(LEVELING_PAGE_TITLE_VP);

  RTS_SndLangData(PRINTER_DEVICE_GREY_TITLE_VP);
  RTS_SndLangData(PRINTER_ADVINFO_GREY_TITLE_VP);
  RTS_SndLangData(PRINTER_INFO_ENTER_GREY_TITLE_VP);
  RTS_SndLangData(PRINTER_DEVICE_BLACK_TITLE_VP);
  RTS_SndLangData(PRINTER_ADVINFO_BLACK_TITLE_VP);
  RTS_SndLangData(PRINTER_INFO_ENTER_BLACK_TITLE_VP);

  RTS_SndLangData(PREHEAT_PLA_SET_TITLE_VP);
  RTS_SndLangData(PREHEAT_ABS_SET_TITLE_VP);

  RTS_SndLangData(STORE_MEMORY_CONFIRM_TITLE_VP);
  RTS_SndLangData(STORE_MEMORY_CANCEL_TITLE_VP);

  RTS_SndLangData(FILAMENT_UNLOAD_IGNORE_TITLE_VP);
  RTS_SndLangData(FILAMENT_USEUP_TITLE_VP);
  RTS_SndLangData(BUTTON_CHECK_CONFIRM_TITLE_VP);
  RTS_SndLangData(BUTTON_CHECK_CANCEL_TITLE_VP);
  RTS_SndLangData(FILAMENT_LOAD_TITLE_VP);
  RTS_SndLangData(FILAMENT_LOAD_RESUME_TITLE_VP);
  RTS_SndLangData(PAUSE_PRINT_POP_TITLE_VP);
  RTS_SndLangData(STOP_PRINT_POP_TITLE_VP);
  RTS_SndLangData(POWERCONTINUE_POP_TITLE_VP);
  RTS_SndLangData(AUTO_HOME_WAITING_POP_TITLE_VP);

  RTS_SndLangData(BEDLEVELING_WAIT_TITLE_VP);
  RTS_SndLangData(RESTORE_FACTORY_TITLE_VP);
  RTS_SndLangData(RESET_WIFI_SETTING_TITLE_VP);
  RTS_SndLangData(KILL_THERMAL_RUNAWAY_TITLE_VP);
  RTS_SndLangData(KILL_HEATING_FAIL_TITLE_VP);
  RTS_SndLangData(KILL_THERMISTOR_ERROR_TITLE_VP);
  RTS_SndLangData(WIND_AUTO_SHUTDOWN_TITLE_VP);
  RTS_SndLangData(RESET_WIFI_SETTING_BUTTON_VP);
  RTS_SndLangData(PRINTER_AUTO_SHUTDOWN_TITLE_VP);
  RTS_SndLangData(WIND_AUTO_SHUTDOWN_PAGE_VP);
  RTS_SndLangData(AUTO_LEVELING_START_TITLE_VP);
  RTS_SndLangData(AUX_LEVELING_GREY_TITLE_VP);
  RTS_SndLangData(AUTO_LEVELING_GREY_TITLE_VP);
  RTS_SndLangData(AUX_LEVELING_BLACK_TITLE_VP);
  RTS_SndLangData(AUTO_LEVELING_BLACK_TITLE_VP);
  RTS_SndLangData(LANGUAGE_SELECT_PAGE_TITLE_VP);
  RTS_SndLangData(ADV_SETTING_MOTION_TITLE_VP);
  RTS_SndLangData(ADV_SETTING_PID_TITLE_VP);
  RTS_SndLangData(ADV_SETTING_WIFI_TITLE_VP);

  RTS_SndLangData(MOTION_SETTING_TITLE_VP);
  RTS_SndLangData(MOTION_SETTING_STEPSMM_TITLE_VP);
  RTS_SndLangData(MOTION_SETTING_ACCEL_TITLE_VP);
  RTS_SndLangData(MOTION_SETTING_JERK_TITLE_VP);
  RTS_SndLangData(MOTION_SETTING_VELOCITY_TITLE_VP);

  RTS_SndLangData(MAX_VELOCITY_SETTING_TITLE_VP);
  RTS_SndLangData(MAX_VELOCITY_XAXIS_TITLE_VP);
  RTS_SndLangData(MAX_VELOCITY_YAXIS_TITLE_VP);
  RTS_SndLangData(MAX_VELOCITY_ZAXIS_TITLE_VP);
  RTS_SndLangData(MAX_VELOCITY_EAXIS_TITLE_VP);

  RTS_SndLangData(MAX_ACCEL_SETTING_TITLE_VP);
  RTS_SndLangData(MAX_ACCEL_XAXIS_TITLE_VP);
  RTS_SndLangData(MAX_ACCEL_YAXIS_TITLE_VP);
  RTS_SndLangData(MAX_ACCEL_ZAXIS_TITLE_VP);
  RTS_SndLangData(MAX_ACCEL_EAXIS_TITLE_VP);

  RTS_SndLangData(MAX_JERK_SETTING_TITLE_VP);
  RTS_SndLangData(MAX_JERK_XAXIS_TITLE_VP);
  RTS_SndLangData(MAX_JERK_YAXIS_TITLE_VP);
  RTS_SndLangData(MAX_JERK_ZAXIS_TITLE_VP);
  RTS_SndLangData(MAX_JERK_EAXIS_TITLE_VP);

  RTS_SndLangData(MAX_STEPSMM_SETTING_TITLE_VP);
  RTS_SndLangData(MAX_STEPSMM_XAXIS_TITLE_VP);
  RTS_SndLangData(MAX_STEPSMM_YAXIS_TITLE_VP);
  RTS_SndLangData(MAX_STEPSMM_ZAXIS_TITLE_VP);
  RTS_SndLangData(MAX_STEPSMM_EAXIS_TITLE_VP);

  RTS_SndLangData(TEMP_PID_SETTING_TITLE_VP);
  RTS_SndLangData(NOZZLE_TEMP_P_TITLE_VP);
  RTS_SndLangData(NOZZLE_TEMP_I_TITLE_VP);
  RTS_SndLangData(NOZZLE_TEMP_D_TITLE_VP);
  RTS_SndLangData(HOTBED_TEMP_P_TITLE_VP);
  RTS_SndLangData(HOTBED_TEMP_I_TITLE_VP);
  RTS_SndLangData(HOTBED_TEMP_D_TITLE_VP);

  RTS_SndLangData(FILAMENT_CONTROL_TITLE_VP);
  RTS_SndLangData(POWERCONTINUE_CONTROL_TITLE_VP);

  RTS_SndLangData(MACHINE_TYPE_ABOUT_CHAR_VP);
  RTS_SndLangData(FIRMWARE_VERSION_ABOUT_TEXT_VP);
  RTS_SndLangData(PRINTER_DISPLAY_VERSION_TITLE_VP);
  RTS_SndLangData(HARDWARE_VERSION_ABOUT_TITLE_VP);
  RTS_SndLangData(WIFI_DN_CODE_CHAR_VP);
  RTS_SndLangData(WEBSITE_ABOUT_CHAR_VP);
  RTS_SndLangData(PRINTER_PRINTSIZE_TITLE_VP);
  RTS_SndLangData(PLA_SETTINGS_TITLE_VP);
  RTS_SndLangData(ABS_SETTINGS_TITLE_VP);
  RTS_SndLangData(LEVELING_WAY_TITLE_VP);
}

/**
 * @brief Update the SD Card state, UI, etc.
 * @details Called from idle() to manage the RTS UI
 */
void RTS_Update() {
  // Check the status of card
  if (!home_flag && !G29_flag) // Cards are not detected during homing and leveling
    rts.sdCardUpate();

  rts.eachMomentUpdate();

  // Wait to receive message and response
  if (rts.receiveData() > 0) rts.handleData();
  if (!wait_for_heatup && !home_flag && !G29_flag) Remove_card_window_check(); // rock_20220709
  hal.watchdog_refresh();

  //AutoUIBedNozzleHeightCali(); // caixiaoliang 20210806 Call the nozzle platform height measurement function
}

void RTS_PauseMoveAxisPage() {
  if (waitway == 1) {
    rts.sendData(ExchangePageBase + 12, ExchangepageAddr);
    change_page_font = 12;
    waitway = 0;
    SERIAL_ECHOLNPGM("M79 S2"); // 2:cloud print pause
  }
  else if (waitway == 5) {
    rts.sendData(ExchangePageBase + 7, ExchangepageAddr);
    change_page_font = 7;
    waitway = 0;
  }
  else if (waitway == 7) {
    // Click stop print
    rts.sendData(ExchangePageBase + 1, ExchangepageAddr);
    change_page_font = 1;
    waitway = 0;
  }
}

void RTS_AutoBedLevelPage() {
  if (waitway == 3) {
    rts.sendData(ui.language + 1, BEDLEVELING_WAIT_TITLE_VP);
    rts.sendData(ExchangePageBase + 26, ExchangepageAddr);
    change_page_font = 26;
    waitway = 0;
  }
}

void RTS_MoveAxisHoming() {
  if (waitway == 4) {
    rts.sendData(ExchangePageBase + 16 + (AxisUnitMode - 1), ExchangepageAddr);
    change_page_font = 16;
    waitway = 0;
  }
  else if (waitway == 6) {
    rts.sendData(ExchangePageBase + 25, ExchangepageAddr);
    change_page_font = 25;
    waitway = 0;
  }
  else if (waitway == 7) {
    // Click Print finish
    rts.sendData(ExchangePageBase + 1, ExchangepageAddr);
    change_page_font = 1;
    waitway = 0;
  }

  RTS_UpdatePosition();
}

void RTS_CommandPause() {
  if (printingIsActive()) {
    rts.sendData(ExchangePageBase + 12, ExchangepageAddr);
    change_page_font = 12;
    // card.pauseSDPrint();
    // print_job_timer.pause();
    // pause_action_flag = true;
  }
}

void ErrorHanding() {
  // No more operations
  if (errorway == 1) {
    errorway = errornum = 0;
  }
  else if (errorway == 2) {
    // Z axis home failed
    home_errornum++;
    if (home_errornum <= 3) {
      errorway = 0;
      waitway = 4;
      queue.enqueue_now(F("G28"));
      rts.sendData(0, MOTOR_FREE_ICON_VP);
      update_time_value = 0;
    }
    else {
      // After three failed returns to home, it will report the failure interface
      home_errornum = 0;
      errorway = 0;
      RTS_Error(Error_202); // Z axis home failed

      if (printingIsActive()) {
        rts.sendData(0, PRINT_TIME_HOUR_VP);
        rts.sendData(0, PRINT_TIME_MIN_VP);
        update_time_value = 0;

        rts.sdCardStop();
      }
    }
  }
  else if (errorway == 3) {
    // No more operations
    reset_bed_level();
    errorway = 0;
    errornum = 0;
  }
  else if (errorway == 4) {

  }
}

/**
 * [调平步骤--开始找限位开关到CRTouch的距离]
 * @Author Creality
 * @Time   2021-09-22
 */
bool RTS::levelStepHeightMeasure() {
  #ifdef BLTOUCH
    float coordinateOffset = VALUE_INVALID_8BIT;
    // if (gLcdSermoonV2UI.GetStaGoingHome() != GO_HOME_DONE) return false;

    // set homing status - 'GO_HOME_IDLE'
    // SetStaGoingHome(GO_HOME_IDLE);  //此处用 home_flag 代替
    // bltouch+lower limit enabled
    #if ENABLED(BLTOUCH_AND_Z_LIMIT)
      //constexpr xy_float_t test_offset_xy = {LEVEL_HEIGHT_MEASURE_X, LEVEL_HEIGHT_MEASURE_Y };
      //destination.set(test_offset_xy, current_position.z);
      ////destination.set(safe_homing_xy, current_position.z);
      //TERN_(1, destination -= probe.offset_xy);
      //if (position_is_reachable(destination))
      //  do_blocking_move_to_xy(destination);  // Move bltouch to the middle
      //else
      //  SERIAL_ECHO_MSG(STR_ZPROBE_OUT_SER);
      //// The interface changes to "Operating"
      //lcdDisplay.GotoScreen(DGUSLCD_SCREEN_MSG_OPERATE);
      sendData(ExchangePageBase + 40, ExchangepageAddr);
      // Start to measure the difference between two coordinates
      //coordinateOffset = homeaxis_bl(Z_AXIS, Z_MEASURE_FEEDRATE_FAST, Z_MEASURE_FEEDRATE_SLOW);
      if (coordinateOffset != VALUE_INVALID_8BIT)
        setZCoordinateOffset(coordinateOffset);
      else {
        DEBUG_ECHOLNPGM(" coordinateOffset err!!! ");
        //setZCoordinateOffset(0.0);
      }
      settings.save();
    #endif

    // After measuring the difference between the two coordinates, the
    // nozzle moves to the detection point for height adjustment.
    //MoveXYBlock(float(LEVEL_HEIGHT_MEASURE_X), float(LEVEL_HEIGHT_MEASURE_Y));

    // Get to the base height, block
    //MoveZBlock(0.0);

    // Show Z-axis compensation
    sendData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
    //sendData(probe.offset.z * 1000, VP_DATA_Z_OFFSET);
    //uiShow.UI_ShowText(probe.offset.z, 3, lcdDisplay.current_screen, VP_DATA_Z_OFFSET);
  #endif

  return true;
}

void AutoUIBedNozzleHeightCali() {
  if (st_bedNozzleHeightCal.bedNozzleHeightCalFinishFlag) return;

  switch (st_bedNozzleHeightCal.bedNozzleHeightState) {
    case 0:
      st_bedNozzleHeightCal.isBedLeveling = true;
      st_bedNozzleHeightCal.goHomeSta = GO_HOME_IDLE;
      queue.enqueue_now(F("M420 S0"));
      #ifdef BLTOUCH
        //queue.inject(F("G28"));
        //queue.inject(F("G0 Z0"));
        queue.enqueue_now(F("G28"));
        //queue.enqueue_now(F("G0 Z0"));
      #else
        queue.inject(F("G29 S1"));
      #endif
      st_bedNozzleHeightCal.bedNozzleHeightState++;
      break;

    case 1:
      // wait for homing
      if (st_bedNozzleHeightCal.goHomeSta == GO_HOME_DONE) {
        // set homing status - 'GO_HOME_IDLE'
        st_bedNozzleHeightCal.goHomeSta = GO_HOME_IDLE;
        DEBUG_ECHOLNPGM("\n----------------------current_position.z!!!!!!= ", current_position.z * 100);

        // The coordinates at this point should be Z_MAX_POS
        DEBUG_ECHOLN(current_position.z * 100);
        destination.set(safe_homing_xy, current_position.z);

        destination -= probe.offset_xy; // TODO:OPTIONAL?

        if (position_is_reachable(destination)) {
          // This causes the carriage on Dual X to unpark
          TERN_(DUAL_X_CARRIAGE, active_extruder_parked = false);
          TERN_(SENSORLESS_HOMING, safe_delay(500));         // Short delay needed to settle
          do_blocking_move_to_xy(destination);
        }
        else
          DEBUG_ECHO_MSG(STR_ZPROBE_OUT_SER);

        homeaxis_bl(Z_AXIS);

        // Get to the center(x,y), block
        LcdAutoUIMoveXYBlock(X_CENTER, Y_CENTER);

        // Get to the base height block
        LcdAutoUIMoveZBlock(0.0f);
        bedNozzleHeightCalZ = st_bedNozzleHeightCal.zCoordinateOffset;
        settings.save();      // Call save data to save the measured height data
        st_bedNozzleHeightCal.bedNozzleHeightState++;
      }
      break;

    case 2:
      st_bedNozzleHeightCal.bedNozzleHeightState = 0;
      st_bedNozzleHeightCal.bedNozzleHeightCalFinishFlag = true;
      break;

    default: break;
  }

  //if (st_bedNozzleHeightCal.bedNozzleHeightState == 2 && st_bedNozzleHeightCal.bedNozzleHeightCalFinishFlag) {
  //// Probing is completed and the save button or return button is received
  //  st_bedNozzleHeightCal.bedNozzleHeightState=0;
  //  //DEBUG_ECHOPGM("\n..bedNozzleHeightCalFinish");
  //}
}

/**
 * [LcdAutoUIMoveXY :get to the given position(x, y), block program running]
 * @Author Creality
 * @Time   2021-06-08
 * @param  _posX      [X coordinate]
 * @param  _posY      [Y coordinate]
 */
void LcdAutoUIMoveXYBlock(float _posX, float _posY) {
  do_blocking_move_to_xy(_MIN(_posX, X_BED_SIZE), _MIN(_posY, Y_BED_SIZE));
}

/**
 * [LcdAutoUIMoveZBlock :get to the given position(z), block program running]
 * @Author Creality
 * @Time   2021-06-08
 * @param  _posZ      [Z coordinate]
 */
void LcdAutoUIMoveZBlock(float _posZ) {
  do_blocking_move_to_z(_MIN(_posZ, Z_MAX_POS));
}

///////////////////////

void RTS_Error(const char * const err) {
  rts.sendData(ExchangePageBase + 41, ExchangepageAddr);
  change_page_font = 41;
  rts.sendText(err, ABNORMAL_PAGE_TEXT_VP);
}

void RTS_OpenFileCloud() {
  TERN_(FILAMENT_RUNOUT_SENSOR, if (runout.enabled) runout.filament_ran_out = false);

  if (PoweroffContinue) return;

  // clean print file
  for (uint8_t i = 0; i < 20; ++i) rts.sendData(0, PRINT_FILE_TEXT_VP + i);

  char *cloudfilename = parser.string_arg;
  int cloudnamelen = strlen(parser.string_arg);
  int fileselect;
  for (fileselect = 0; fileselect < CardRecbuf.Filesum; fileselect++)
    if (0 == strncmp(CardRecbuf.Cardshowfilename[fileselect], cloudfilename, cloudnamelen)) break;

  int j = 1;
  while (strncmp(&cloudfilename[j], ".gcode", 6) && strncmp(&cloudfilename[j], ".GCODE", 6) && j++ < cloudnamelen) { /* nada */ }
  NOMORE(j, TEXTBYTELEN - 1);
  strncpy(CardRecbuf.Cardshowfilename[fileselect], parser.string_arg, TEXTBYTELEN);
  parser.string_arg[j] = '\0';

  rts.sendData(CardRecbuf.Cardshowfilename[fileselect], PRINT_FILE_TEXT_VP);
}

void RTS_PausedPrint() {
  rts.sendData(ExchangePageBase + 7, ExchangepageAddr);
  change_page_font = 7;
  sdcard_pause_check = true;
}

void RTS_PrintStartedSD() {
  if (PoweroffContinue) return;
  update_time_value = 0;
  print_job_timer.start();
  rts.sendData(ExchangePageBase + 10, ExchangepageAddr);
  change_page_font = 10;
}

void RTS_SDFileCompleted() {
  PoweroffContinue = false;
}

void RTS_UpdatePosition() {
  rts.sendData(10 * current_position.x, AXIS_X_COORD_VP);
  rts.sendData(10 * current_position.y, AXIS_Y_COORD_VP);
  rts.sendData(10 * current_position.z, AXIS_Z_COORD_VP);
}

void RTS_UpdateFeedrate(const int16_t frp) {
  rts.sendData(frp, PRINT_SPEED_RATE_VP);
  DEBUG_ECHOLNPGM("M220 S", frp);
}

void RTS_UpdateWifi() {
  rts.sendData(wifi_enable_flag ? 0 : 1, ADV_SETTING_WIFI_ICON_VP);
  rts.sendData(wifi_enable_flag ? 1 : 0, WIFI_CONNECTED_DISPLAY_ICON_VP);
}

void RTS_StepperTimeout() {
  waitway = 0;
  RTS_Error(Error_201);
  errorway = 1;
}

void RTS_ReheatHotend(const celsius_t c/*=170*/) {
  if (thermalManager.degTargetHotend(0) > 0) temphot = thermalManager.degTargetHotend(0);
  thermalManager.setTargetHotend(c, 0);
  rts.sendData(c, HEAD_SET_TEMP_VP);
}

void RTS_FilamentRanOut() {
  temphot = thermalManager.degTargetHotend(0);
  rts.sendData(ExchangePageBase + 7, ExchangepageAddr);
  change_page_font = 7;
  sdcard_pause_check = true;
}

void RTS_FileHasFinished() {
  rts.sendData(100, PRINT_PROCESS_VP);      delay(1);
  rts.sendData(100, PRINT_PROCESS_ICON_VP); delay(1);
  rts.sendData(ExchangePageBase + 9, ExchangepageAddr);
  DEBUG_ECHOLNPGM("M79 S5");   // 5:cloud print complete
}

void RTS_ProbingPauseHotend() {
  #if ENABLED(PROBING_HEATERS_OFF)
    if (thermalManager.degTargetHotend(0) > 0) {
      temphot = thermalManager.degTargetHotend(0);
      thermalManager.setTargetHotend(0, 0);
      rts.sendData(0, HEAD_SET_TEMP_VP);
    }
  #endif
}

void RTS_ProbingPauseFans() {
  #if ENABLED(PROBING_FANS_OFF)
    afterprobe_fan0_speed = thermalManager.fan_speed[0];
    if (thermalManager.fan_speed[0] > 0) {
      thermalManager.set_fan_speed(0, 0);
      rts.sendData(0, PRINTER_FANOPEN_TITLE_VP);
    }
  #endif
}

void RTS_ProbingResumeHotend() {
  #if ENABLED(PROBING_HEATERS_OFF)
    if (temphot > 0 && card.isPrinting()) {
      rts.sendData(temphot, HEAD_SET_TEMP_VP);
      thermalManager.setTargetHotend(temphot, 0);
      if (temphot > thermalManager.degHotend(0) - 5) thermalManager.wait_for_hotend(false);
      while (thermalManager.still_heating(0)) idle();
    }
  #endif
}

void RTS_ProbingResumeFans() {
  #if ENABLED(PROBING_FANS_OFF)
    if (afterprobe_fan0_speed > 0 && card.isPrinting()) {
      thermalManager.set_fan_speed(0, afterprobe_fan0_speed);
      rts.sendData(1, PRINTER_FANOPEN_TITLE_VP);
    }
  #endif
}

void RTS_LevelingUpdate(uint8_t &count, const uint8_t total) {
  if (IS_SD_PRINTING()) return;
  if (count >= total) return;
  count++;
  rts.sendData(count * 50 / total, AUTO_BED_LEVEL_TITLE_VP);
  rts.sendData(count * 100 / total, AUTO_LEVELING_PERCENT_DATA_VP);
  rts.sendData(thermalManager.degHotend(0), HEAD_CURRENT_TEMP_VP);
  rts.sendData(ExchangePageBase + 26, ExchangepageAddr);
  change_page_font = 26;
  // TODO: Send the Z value from the in-progress leveling
  //rts.sendData(bedlevel.z_values[meshCount.x][meshCount.y]*1000, AUTO_BED_LEVEL_1POINT_VP + (count - 1) * 2);
}

void RTS_LevelingDone() {
  G29_flag = false;
  //G29_finished = true;  // G29 has been completed rock_20220902
  RTS_AutoBedLevelPage();
}

void RTS_ProbingFailed() {
  waitway = 0;
  RTS_Error(Error_203);
  errorway = 3;
}

// Cloud Print Status Flags
int32_t g_couldPrintState = COULD_PRINT_STATE_OFF;

void RTS_CloudCommand(const cloudCommand_t cloud_cmd) {
  switch (cloud_cmd) {
    case 0:
      // 0:cloud connect
      rts.sendData(1, WIFI_CONNECTED_DISPLAY_ICON_VP);
      break;

    case 1:
      // 1:cloud print start
      if (!PoweroffContinue) {
        update_time_value = 0;
        print_job_timer.start();
        rts.sendData(ExchangePageBase + 10, ExchangepageAddr);
        change_page_font = 10;
        g_couldPrintState = COULD_PRINT_STATE_RUNNING;
      }
      break;

    case 2:
      // 2:cloud print pause
      if (!PoweroffContinue) {  // Cloud printing
        update_time_value = 0;
        print_job_timer.pause();
        if (!runout.filament_ran_out) {
          rts.sendData(ExchangePageBase + 12, ExchangepageAddr);
          change_page_font = 12;
        }
        g_couldPrintState = COULD_PRINT_STATE_PAUSE;
      }
      else if (PoweroffContinue) {  // Print locally
        if (!wait_for_heatup) {     // No hydronic heating
          rts.sendData(ExchangePageBase + 40, ExchangepageAddr);
          waitway = 1;
          pause_e = current_position.e - 3;
          if (!temphot) temphot = thermalManager.temp_hotend[0].target;
          card.pauseSDPrint();
          print_job_timer.pause();
          pause_action_flag = true;
          update_time_value = 0;
          planner.synchronize();
          sdcard_pause_check = false;
        }
        else { // If it is heating in circulation, it will continue to print
          SERIAL_ECHOLNPGM("M79 S3");
          break;
        }
      }
      break;

    case 3:
      // 3: Cloud print resume
      if (!PoweroffContinue) { // Cloud printing
        update_time_value = 0;
        print_job_timer.start();
        rts.sendData(ExchangePageBase + 10, ExchangepageAddr);
        change_page_font = 10;
        if (g_cloudPLRStatusValue == CLOUD_PLR_CHOOSE_STATE) // There is a power failure to continue playing
          g_cloudPLRStatusValue = CLOUD_PLR_YES_STATE;
        g_couldPrintState = COULD_PRINT_STATE_RUNNING;
      }
      else if (PoweroffContinue) { // Print locally
        if (CardReader::flag.mounted) {
          if (CLOUD_PLR_NULL_STATE == g_cloudPLRStatusValue) { // Continuous play without power failure
            rts.sendData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;
            #if ENABLED(HAS_RESUME_CONTINUE)
              const bool was_wait = wait_for_user;
              if (was_wait) wait_for_user = false;
            #else
              constexpr bool was_wait = false;
            #endif
            if (!was_wait) {
              memset(commandbuf, 0, sizeof(commandbuf));
              sprintf_P(commandbuf, PSTR("M109 S%i"), temphot);
              queue.enqueue_one_now(commandbuf);

              card.startOrResumeFilePrinting();
              print_job_timer.start();
              update_time_value = 0;
              sdcard_pause_check = true;
            }
          }
          else if (g_cloudPLRStatusValue == CLOUD_PLR_CHOOSE_STATE) { // 有断电续打
            g_cloudPLRStatusValue = CLOUD_PLR_YES_STATE;
            PoweroffContinue = true;
            power_off_type_yes = true;
            update_time_value = 0;
            rts.sendData(ExchangePageBase + 10, ExchangepageAddr);
            change_page_font = 10;
            // recovery.resume();
            queue.enqueue_now(F("M1000"));
            sdcard_pause_check = true;
            rts.sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          }
        }
      }
      break;

    case 4:
      // 4:cloud print stop
      if (!PoweroffContinue) {
        update_time_value = 0;
        rts.sendData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
        rts.sendData(0, PRINT_PROCESS_VP);
        rts.sendData(0, PRINT_PROCESS_ICON_VP);
        print_job_timer.stop();
        g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
      }
      else if (PoweroffContinue) {
        if (27 == change_page_font) { // 有本地断电续打
          update_time_value = RTS_UPDATE_VALUE;
          rts.sendData(ExchangePageBase + 1, ExchangepageAddr);
          change_page_font = 1;
          rts.sendData(0, PRINT_TIME_HOUR_VP);
          rts.sendData(0, PRINT_TIME_MIN_VP);
          update_time_value = 0;
          rts.sdCardStop();
          g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
          break;
        }
        rts.sendData(0, PRINT_TIME_HOUR_VP);
        rts.sendData(0, PRINT_TIME_MIN_VP);
        update_time_value = 0;
        temphot = 0;
        rts.sendData(ExchangePageBase + 40, ExchangepageAddr);
        waitway = 7;
        rts.sdCardStop();
        gcode.process_subcommands_now(F("G28 XY\nM84"));
        g_cloudPLRStatusValue = CLOUD_PLR_NULL_STATE;
      }
      break;

    case 5:
      // 5:cloud print complete
      if (!PoweroffContinue) {
        update_time_value = 0;
        print_job_timer.stop();
        rts.sendData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_font = 1;
        rts.sendData(0, PRINT_PROCESS_VP);
        rts.sendData(0, PRINT_PROCESS_ICON_VP);
      }
      break;

    case 6:
      // 6:cloud print power continue
      if (!PoweroffContinue) {
        rts.sendData(ExchangePageBase + 27, ExchangepageAddr);
        change_page_font = 27;
        power_off_type_yes = true;
        g_cloudPLRStatusValue = CLOUD_PLR_CHOOSE_STATE;
      }
      break;

    default: break;
  }
}

void RTS_SetStatsFR(const feedRate_t fr_mm_s) {
  rts.sendData(fr_mm_s, PRINT_PROCESS_VP);
  rts.sendData(fr_mm_s, PRINT_PROCESS_ICON_VP);
}

void RTS_SetStatsTime(const millis_t time) {
  rts.sendData(time / 3600, PRINT_TIME_HOUR_VP);
  rts.sendData((time % 3600) / 60, PRINT_TIME_MIN_VP);
}

void RTS_SetStatsRemain(const millis_t remain) {
  rts.sendData(remain / 3600, PRINT_REMAIN_TIME_HOUR_VP);
  rts.sendData((remain % 3600) / 60, PRINT_REMAIN_TIME_MIN_VP);
}

void RTS_SetLED(const bool on) {
  rts.sendData(on ? 0 : 1, PRINTER_LEDOPEN_TITLE_VP);
  #if PIN_EXISTS(LED_CONTROL)
    WRITE(LED_CONTROL_PIN, on ? HIGH : LOW);
  #endif
}

#endif // CREALITY_RTS
