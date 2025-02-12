/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include "../../inc/MarlinConfig.h"

#if ENABLED(SOVOL_SV06_RTS)

#include "sovol_rts.h"

DB RTS::recdat;
DB RTS::snddat;
uint8_t RTS::databuf[DATA_BUF_SIZE];

uint8_t RTS::print_state = 0;
bool RTS::start_print_flag = false;
bool RTS::dark_mode = false;

RTS rts;

#include <WString.h>
#include <stdio.h>
#include <string.h>
#include "../../MarlinCore.h"
#include "../../sd/cardreader.h"
#include "../../module/temperature.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"
#include "../../module/settings.h"
#include "../../module/motion.h"
#include "../../module/printcounter.h"
#include "../../libs/duration_t.h"
#include "../../feature/babystep.h"
#include "../../feature/powerloss.h"
#include "../../feature/bedlevel/bedlevel.h"
#include "../../feature/tmc_util.h"
#include "../../gcode/queue.h"
#include "../../gcode/gcode.h"
#include "../marlinui.h"
#include "../../libs/BL24CXX.h"

#if ENABLED(FIX_MOUNTED_PROBE)
  #include "../../module/endstops.h"
#endif
#if HAS_BED_PROBE
  #include "../../module/probe.h"
#endif

#if HAS_FILAMENT_SENSOR
  //#define CHECKFILAMENT
  #include "../../feature/runout.h"
#endif

float zprobe_zoffset;
float last_zoffset = 0.0;

int16_t startprogress = 0;
CRec cardRec;
bool sdcard_pause_check = true;

float change_filament_temp_0 = 200;

int16_t heatway = 0;
millis_t next_rts_update_ms = 0;

int8_t waitway = 0;
int16_t recnum = 0;

uint8_t job_percent = 0;

bool pause_action_flag = false;
bool pause_flag = false;
bool power_off_type_yes = false;

bool update_sd = false;  // Flag to update the file list

#if HAS_HOTEND
  int16_t last_target_temperature[1] = { 0 };
#endif
#if HAS_HEATED_BED
  int16_t last_target_temperature_bed;
#endif

bool lcd_sd_status;   // SD-card status. true = SD available

int16_t FilenamesCount = 0;
char cmdbuf[20] = { 0 };
float filament_load_0 = 10.0f;
float XoffsetValue;

// 0 for 10mm, 1 for 1mm, 2 for 0.1mm
uint8_t AxisUnitMode;
float axis_unit = 10;
int16_t update_time_value = 0;

bool poweroff_continue = false;
char commandbuf[30];

static SovolPage change_page_number = ID_Startup;

uint32_t remain_time = 0;

static bool last_card_insert_st;
bool card_insert_st;
bool sd_printing;

int16_t fan_speed;
char cmd[MAX_CMD_SIZE + 16];

inline void RTS_line_to_current(const AxisEnum axis) {
  if (!planner.is_full())
    planner.buffer_line(current_position, MMM_TO_MMS(manual_feedrate_mm_m[axis]), active_extruder);
}

RTS::RTS() {
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  ZERO(databuf);
}

void RTS::sdCardInit() {
  if (sdDetected()) card.mount();
  if (card.flag.mounted) {
    const int16_t fileCnt = card.get_num_items();
    card.getWorkDirName();
    if (card.filename[0] != '/') card.cdup();

    int16_t addrnum = 0, num = 0;
    for (uint16_t i = 0; i < fileCnt && i < (MAX_NUM_FILES) + addrnum; i++) {
      card.selectFileByIndex(fileCnt - 1 - i);
      char * const pFilename = card.longest_filename();
      const int16_t filenamelen = strlen(pFilename);
      int16_t j = 1;
      while (strncmp(&pFilename[j], ".gco", 4) && strncmp(&pFilename[j], ".GCO", 4) && j++ < filenamelen);
      if (j >= filenamelen) { addrnum++; continue; }

      if (j >= FILENAME_LEN) {
        strncpy(&pFilename[FILENAME_LEN - 3], "..", 2);
        pFilename[FILENAME_LEN - 1] = '\0';
        j = FILENAME_LEN - 1;
      }

      strncpy(cardRec.display_filename[num], pFilename, j);

      strcpy(cardRec.filename[num], card.filename);
      cardRec.addr[num] = FILE1_TEXT_VP + (num * 20);
      sendData(cardRec.display_filename[num], cardRec.addr[num]);
      cardRec.Filesum = (++num);
    }
    for (uint16_t j = cardRec.Filesum; j < MAX_NUM_FILES; j++) {
      cardRec.addr[j] = FILE1_TEXT_VP + (j * 20);
      sendData(0, cardRec.addr[j]);
    }
    for (uint8_t j = 0; j < 20; j++) {
      // Clear the file name displayed in the print interface
      sendData(0, PRINT_FILE_TEXT_VP + j);
    }
    lcd_sd_status = IS_SD_INSERTED();
  }
  else {
    // Clean all filename Icons
    for (uint8_t j = 0; j < MAX_NUM_FILES; j++)
      for (uint8_t i = 0; i < FILENAME_LEN; i++)
        sendData(0, cardRec.addr[j] + i);
    ZERO(&cardRec);
  }
}

bool RTS::sdDetected() {
  static bool state = false, stable = false, was_present = false;
  static millis_t stable_ms = 0;

  const bool present = IS_SD_INSERTED();
  if (present != was_present)
    stable = false;
  else if (!stable) {
    stable = true;
    stable_ms = millis() + 30;
  }

  if (stable && ELAPSED(millis(), stable_ms))
    state = present;

  was_present = present;
  return state;
}

void RTS::sdCardUpdate() {
  const bool sd_status = sdDetected();
  if (sd_status != lcd_sd_status) {
    if (sd_status) {
      // SD card power on
      card.mount();
      sdCardInit();
    }
    else {
      card.release();

      for (uint8_t i = 0; i < cardRec.Filesum; i++) {
        for (uint8_t j = 0; j < 20; j++) sendData(0, cardRec.addr[i] + j);
        sendData(uint32_t(0x738E), FilenameNature + (i + 1) * 16);
      }

      // Clean screen
      for (uint8_t j = 0; j < 20; j++) {
        sendData(0, PRINT_FILE_TEXT_VP + j);
        sendData(0, SELECT_FILE_TEXT_VP + j);
      }
      ZERO(&cardRec);
    }
    lcd_sd_status = sd_status;
  }

  // Represents to update file list
  if (update_sd && lcd_sd_status && sdDetected()) {
    for (uint16_t i = 0; i < cardRec.Filesum; i++) {
      delay(1);
      sendData(cardRec.display_filename[i], cardRec.addr[i]);
      sendData(uint32_t(0x738E), FilenameNature + (i + 1) * 16);
    }
    update_sd = false;
  }
}

void RTS::init() {
  dark_mode = BL24CXX::readOneByte(FONT_EEPROM);
  AxisUnitMode = 1;
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    bool zig = false;
    int8_t inStart, inStop, inInc, showcount;
    showcount = 0;
    for (int8_t y = 0; y < GRID_MAX_POINTS_Y; y++) {
      // Away from origin
      if (zig) {
        inStart = 0;
        inStop = GRID_MAX_POINTS_X;
        inInc = 1;
      }
      else {
        // Towards origin
        inStart = GRID_MAX_POINTS_X - 1;
        inStop = -1;
        inInc = -1;
      }
      FLIP(zig);
      for (int8_t x = inStart; x != inStop; x += inInc) {
        sendData(bedlevel.z_values[x][y] * 100, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
        showcount++;
      }
    }
    queue.enqueue_now(F("M420 S1"));
  #endif
  last_zoffset = zprobe_zoffset = probe.offset.z;
  sendData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

  TERN_(HAS_HOTEND, last_target_temperature[0] = thermalManager.degTargetHotend(0));
  TERN_(HAS_HEATED_BED, last_target_temperature_bed = thermalManager.degTargetBed());

  feedrate_percentage = 100;
  sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);

  /***************turn off motor*****************/
  sendData(1, MOTOR_FREE_ICON_VP);

  /***************transmit temperature to screen*****************/
  updateTempE0();
  updateTempBed();

  /***************transmit Fan speed to screen*****************/
  // Turn off fans
  thermalManager.zero_fan_speeds();
  //queue.enqueue_now(F("M107"));
  //sendData(1, HEAD0_FAN_ICON_VP);
  updateFan0();

  delay(5);

  /*********transmit SD card filename to screen***************/
  sdCardInit();

  /**************************some info init*******************************/
  sendData(0, PRINT_PROCESS_ICON_VP);
  change_page_number = card.flag.mounted ? (dark_mode ? ID_Home_D : ID_Home_L) : ID_Startup;
}

int16_t RTS::receiveData() {
  int16_t frame_index = 0, timeout = 0, framelen = 0;
  bool frame_flag = false;
  if (LCD_SERIAL.available() <= 0) return -1;
  do {
    if (LCD_SERIAL.available() > 0) {
      databuf[frame_index] = LCD_SERIAL.read();
      timeout = 0;
      // 0x5A
      if (frame_index == 0 && databuf[frame_index] == FHONE) {
        frame_index++;
        continue;
      }
      // 0xA5
      else if (frame_index == 1) {
        if (databuf[frame_index] == FHTWO) frame_index++; else frame_index = 0;
        continue;
      }
      // 长度
      else if (frame_index == 2) {
        framelen = databuf[frame_index];
        frame_index++;
        continue;
      }
      else if (frame_index != 0) {
        frame_index++;
        // After a frame of data is extracted, the remaining serial data will be processed next time the function will be processed
        if (frame_index == (framelen + 3)) {
          frame_flag = true;
          break;
        }
      }
    }
    else {
      timeout++;
      delay(1);
    }
  } while (timeout < 50); // Timeout function

  if (frame_flag) {
    recdat.head[0] = databuf[0];
    recdat.head[1] = databuf[1];
    recdat.len = databuf[2];
    recdat.command = databuf[3];
    for (uint8_t idx = 0; idx < frame_index; idx++) { }
  }
  else
    return -1;

  // Response for writing byte
  if (recdat.len == 0x03
    && (recdat.command == 0x82 || recdat.command == 0x80)
    && databuf[4] == 0x4F && databuf[5] == 0x4B
  ) {
    ZERO(databuf);
    recnum = 0;
    return -1;
  }
  else if (recdat.command == 0x83) {
    // Response for reading the data from the variate
    recdat.addr = databuf[4];
    recdat.addr = (recdat.addr << 8) | databuf[5];
    recdat.bytelen = databuf[6];
    for (uint16_t i = 0; i < recdat.bytelen; i += 2) {
      recdat.data[i / 2] = databuf[7 + i];
      recdat.data[i / 2] = (recdat.data[i / 2] << 8) | databuf[8 + i];
    }
  }
  else if (recdat.command == 0x81) {
    // Response for reading the page from the register
    recdat.addr = databuf[4];
    recdat.bytelen = databuf[5];
    for (uint16_t i = 0; i < recdat.bytelen; i++) {
      recdat.data[i] = databuf[6 + i];
      //recdat.data[i] = (recdat.data[i] << 8 )| databuf[7 + i];
    }
  }
  else {
    ZERO(databuf);
    recnum = 0;
    // Receive the wrong data
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
    // To write data to the register
    if (snddat.command == 0x80) {
      databuf[4] = snddat.addr;
      for (uint16_t i = 0; i < snddat.len - 2; i++)
        databuf[5 + i] = snddat.data[i];
    }
    else if (snddat.len == 3 && snddat.command == 0x81) {
      // To read data from the register
      databuf[4] = snddat.addr;
      databuf[5] = snddat.bytelen;
    }
    else if (snddat.command == 0x82) {
      // To write data to the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      for (uint16_t i = 0; i < snddat.len - 3; i += 2) {
        databuf[6 + i] = snddat.data[i/2] >> 8;
        databuf[7 + i] = snddat.data[i/2] & 0xFF;
      }
    }
    else if (snddat.len == 4 && snddat.command == 0x83) {
      // To read data from the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      databuf[6] = snddat.bytelen;
    }
    for (uint16_t i = 0; i < snddat.len + 3; i++)
      LCD_SERIAL.write(databuf[i]);

    ZERO(&snddat);
    ZERO(databuf);
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }
}

void RTS::sendData(const String &s, const uint32_t addr, const uint8_t cmd/*=VarAddr_W*/) {
  if (s.length() < 1) return;
  sendData(s.c_str(), addr, cmd);
}

void RTS::sendData(const char str[], const uint32_t addr, const uint8_t cmd/*=VarAddr_W*/) {
  int16_t len = strlen(str);
  if (len > 0) {
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3 + len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for (int16_t i = 0; i < len; i++) databuf[6 + i] = str[i];
    for (int16_t i = 0; i < len + 6; i++) LCD_SERIAL.write(databuf[i]);
    ZERO(databuf);
  }
}

void RTS::sendData(const char c, const uint32_t addr, const uint8_t cmd/*=VarAddr_W*/) {
  snddat.command = cmd;
  snddat.addr = addr;
  snddat.data[0] = uint32_t(c);
  snddat.data[0] = snddat.data[0] << 8;
  snddat.len = 5;
  sendData();
}

void RTS::sendData(const int16_t n, const uint32_t addr, const uint8_t cmd/*=VarAddr_W*/) {
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

void RTS::sendData(const uint32_t n, const uint32_t addr, const uint8_t cmd/*=VarAddr_W*/) {
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

void RTS::sdCardStop() {
  waitway = 7;
  change_page_number = ID_Home_D;
  card.flag.abort_sd_printing = true;

  IF_DISABLED(SD_ABORT_NO_COOLDOWN, thermalManager.disable_all_heaters());
  print_job_timer.reset();

  thermalManager.setTargetHotend(0, 0);
  thermalManager.setTargetBed(0);
  updateTempE0();
  updateTempBed();
  thermalManager.zero_fan_speeds();
  wait_for_heatup = wait_for_user = false;
  poweroff_continue = false;
  #if ALL(HAS_MEDIA, POWER_LOSS_RECOVERY)
    if (card.flag.mounted) card.removeJobRecoveryFile();
  #endif
  #ifdef EVENT_GCODE_SD_STOP
    queue.inject(F(EVENT_GCODE_SD_STOP));
  #endif

  // Shut down the stepper motor.
  //queue.enqueue_now(F("M84"));
  sendData(0, MOTOR_FREE_ICON_VP);

  sendData(0, PRINT_PROCESS_ICON_VP);
  sendData(0, PRINT_PROCESS_VP);
  delay(2);
  for (uint8_t j = 0; j < 20; j++) {
    sendData(0, PRINT_FILE_TEXT_VP + j);  // Clean screen
    sendData(0, SELECT_FILE_TEXT_VP + j); // Clean filename
  }
}

void RTS::handleData() {
  int16_t checkKey = -1;
  // For waiting
  if (waitway > 0) {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  for (uint16_t i = 0; Addrbuf[i] != 0; i++) {
    if (recdat.addr == Addrbuf[i]) {
      if (Addrbuf[i] >= ChangePageKey) checkKey = i;
      break;
    }
  }

  if (checkKey < 0) {
    ZERO(&recdat);
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }

  switch (checkKey) {
    case MainPageKey: // Front page
      if (recdat.data[0] == 1) { // Select print file
        update_sd = true;
        cardRec.recordcount = -1;
        if (card.flag.mounted) {
          for (uint8_t j = 0; j < 20; j++) sendData(0, SELECT_FILE_TEXT_VP + j);
          gotoPage(ID_Page1_L, ID_Page1_D);
        }
        else
          gotoPage(ID_BrowseNoSd_L, ID_BrowseNoSd_D);
      }
      else if (recdat.data[0] == 2) { // Complete printing
        waitway = 7;
        card.flag.abort_sd_printing = true;
        quickstop_stepper();
        print_job_timer.reset();
        queue.clear();
        sendData(0, MOTOR_FREE_ICON_VP);
        sendData(0, PRINT_PROCESS_ICON_VP);
        sendData(0, PRINT_PROCESS_VP);
        delay(2);
        sendData(0, PRINT_TIME_HOUR_VP);
        sendData(0, PRINT_TIME_MIN_VP);
        sendData(0, PRINT_SURPLUS_TIME_HOUR_VP);
        sendData(0, PRINT_SURPLUS_TIME_MIN_VP);

        change_page_number = ID_Home_D;
      }
      else if (recdat.data[0] == 3) { // Enter the tone interface
        waitway = 6;
        queue.enqueue_now(F("G28\nG1 F200 Z0.0"));
        updateFan0();
        sendData(1, Wait_VP);
        gotoPage(ID_AutoHome_L, ID_AutoHome_D);
      }
      else if (recdat.data[0] == 4) { // Enter the settings interface
        gotoPage(ID_Settings_L, ID_Settings_D);
      }
      else if (recdat.data[0] == 5) { // Enter the temperature interface
        updateFan0();
        gotoPage(ID_TempChange_L, ID_TempChange_D);
      }
      else if (recdat.data[0] == 6) { // Light mode
        dark_mode = false;
        BL24CXX::writeOneByte(FONT_EEPROM, dark_mode);
        gotoPage(ID_Home_L);
        change_page_number = ID_Home_L;
        settings.save();
      }
      else if (recdat.data[0] == 7) { // Dark mode
        dark_mode = true;
        BL24CXX::writeOneByte(FONT_EEPROM, dark_mode);
        gotoPage(ID_Home_D);
        change_page_number = ID_Home_D;
        settings.save();
      }
      break;

    case AdjustmentKey:  // Adjust the interface
      if (recdat.data[0] == 1) { // Enter the adjustment interface
        updateFan0();
        sendData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        gotoPage(ID_PrintAdjust5_L, ID_PrintAdjust5_D);
      }
      else if (recdat.data[0] == 2) { // Return to print interface
        if (start_print_flag) {
          gotoPage(ID_PrintHeating_L, ID_PrintHeating_D);
        }
        else if (!sdcard_pause_check) {
          gotoPage(ID_PrintResume_L, ID_PrintResume_D);
        }
        else {
          refreshTime();
          start_print_flag = false;
        }
      }
      else if (recdat.data[0] == 4) {
        updateTempE0();
        gcode.process_subcommands_now(F("M600"));
      }
      break;

    case FanSpeedKey: // Set fan speed
      fan_speed = recdat.data[0];
      thermalManager.set_fan_speed(0, fan_speed);
      updateFan0();
      break;

    case PrintSpeedKey: // Set the print speed
      feedrate_percentage = recdat.data[0];
      sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      break;

    case StopPrintKey: // Stop printing
      if ((recdat.data[0] == 1) || (recdat.data[0] == 0xF1)) {
        sendData(1, Wait_VP);
        gotoPage(ID_Processing_L, ID_Processing_D);
        sendData(0, PRINT_TIME_HOUR_VP);
        sendData(0, PRINT_TIME_MIN_VP);
        sendData(0, PRINT_SURPLUS_TIME_HOUR_VP);
        sendData(0, PRINT_SURPLUS_TIME_MIN_VP);
        sdCardStop();
        print_state = 0;
        update_time_value = 0;
      }
      else if (recdat.data[0] == 0xF0) {
        if (start_print_flag)
          gotoPage(ID_PrintHeating_L, ID_PrintHeating_D);
        else if (!sdcard_pause_check)
          gotoPage(ID_PrintResume_L, ID_PrintResume_D);
        else {
          refreshTime();
          start_print_flag = false;
        }
      }
      break;

    case PausePrintKey: // Suspend printing
      if (recdat.data[0] == 0xF0) break;

      if (recdat.data[0] == 0xF1) {
        sendData(1, Wait_VP);
        gotoPage(ID_Processing_L, ID_Processing_D);
        // Reject to receive cmd
        change_page_number = ID_PrintResume_D;
        waitway = 1;
        card.pauseSDPrint();
        pause_action_flag = true;
        pause_flag = true;
        update_time_value = 0;
        sdcard_pause_check = false;
        print_state = 1;
      }
      break;

    case ResumePrintKey: // Restore printing
      if (recdat.data[0] == 1) { // Portal restoration printing
        if (TERN0(CHECKFILAMENT, runout.filament_ran_out)) gotoPage(ID_FilamentOut_L, ID_FilamentOut_D);
        //sendData(1, Wait_VP);
        //gotoPage(ID_Processing_L, ID_Processing_D);
        card.startOrResumeFilePrinting();
        update_time_value = 0;
        sdcard_pause_check = true;
        pause_action_flag = false;
        pause_flag = false;
        refreshTime();
        print_state = 2;
      }
      else if (recdat.data[0] == 2) {
        #if ENABLED(CHECKFILAMENT)
          if (runout.filament_ran_out) {
            gotoPage(ID_FilamentOut_L, ID_FilamentOut_D);
          }
          else {
            sendData(1, Wait_VP);
            gotoPage(ID_Processing_L, ID_Processing_D);
            card.startOrResumeFilePrinting();
            print_job_timer.start();

            update_time_value = 0;
            pause_action_flag = false;
            sdcard_pause_check = true;
            refreshTime();
            print_state = 2;
          }
        #endif
      }
      else if (recdat.data[0] == 3) { // Electricity disconnection is restored to print
        if (poweroff_continue) {
          #if ENABLED(CHECKFILAMENT)
            sendData(runout.filament_ran_out ? 0 : 1, CHANGE_FILAMENT_ICON_VP);
            gotoPage(ID_Change_L, ID_Change_D);
          #endif
        }
        else if (poweroff_continue == false) {
          if (TERN0(CHECKFILAMENT, runout.filament_ran_out)) { gotoPage(ID_FilamentOut_L, ID_FilamentOut_D); break; }

          char cmd[30];
          sprintf_P(cmd, M23_STR, cardRec.filename[FilenamesCount]);
          for (char *c = &cmd[4]; *c; c++) *c = tolower(*c);
          queue.enqueue_one_now(cmd);
          delay(20);
          queue.enqueue_now_P(M24_STR);

          // Clean screen
          for (uint8_t j = 0; j < 20; j++) sendData(0, PRINT_FILE_TEXT_VP + j);

          sendData(cardRec.display_filename[cardRec.recordcount], PRINT_FILE_TEXT_VP);
          delay(2);
          TERN_(BABYSTEPPING, sendData(0, AUTO_BED_LEVEL_ZOFFSET_VP));
          feedrate_percentage = 100;
          sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          zprobe_zoffset = last_zoffset;
          sendData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          poweroff_continue = true;
          gotoPage(ID_PrintHeating_L, ID_PrintHeating_D);
          sdcard_pause_check = true;
        }
      }
      else if (recdat.data[0] == 4) { // Card withdrawing to print
        if (!card.flag.mounted) {
          update_sd = true;
          sdCardUpdate();
          gotoPage(ID_MediaFail_L, ID_MediaFail_D);
        }
        else {
          if (TERN0(CHECKFILAMENT, runout.filament_ran_out)) gotoPage(ID_FilamentOut_L, ID_FilamentOut_D);
          sendData(1, Wait_VP);
          gotoPage(ID_Processing_L, ID_Processing_D);
          card.startOrResumeFilePrinting();
          update_time_value = 0;
          sdcard_pause_check = true;
          pause_action_flag = false;
          print_state = 2;
          for (uint16_t i = 0;i < cardRec.Filesum; i++)
            if (!strcmp(cardRec.filename[i], &recovery.info.sd_filename[1]))
              sendData(cardRec.display_filename[i], PRINT_FILE_TEXT_VP);

          refreshTime();
        }
      }
      break;

    case CoolScreenKey: // Temperature interface
      if (recdat.data[0] == 1) { // PLA Preheat
        thermalManager.setTargetHotend(PREHEAT_1_TEMP_HOTEND, 0);
        updateTempE0();
        thermalManager.setTargetBed(PREHEAT_1_TEMP_BED);
        updateTempBed();
      }
      else if (recdat.data[0] == 2) { // ABS Preheat
        thermalManager.setTargetHotend(PREHEAT_2_TEMP_HOTEND, 0);
        updateTempE0();
        thermalManager.setTargetBed(PREHEAT_2_TEMP_BED);
        updateTempBed();
      }
      else if (recdat.data[0] == 3) { // Back to Home page
        gotoPage(ID_Home_L, ID_Home_D);
      }
      else if (recdat.data[0] == 0xF1) { // Cool down
        #if FAN_COUNT > 0
          FANS_LOOP(i) thermalManager.set_fan_speed(i, 255);
        #endif
        updateFan0();
        thermalManager.setTargetHotend(0, 0);
        updateTempE0(); delay(1);

        #if HAS_HEATED_BED
          thermalManager.setTargetBed(0);
          updateTempBed(); delay(1);
        #endif

        gotoPage(ID_TempChange_L, ID_TempChange_D);
      }
      else if (recdat.data[0] == 0xF0) {
        gotoPage(ID_TempChange_L, ID_TempChange_D);
      }
      break;

    #if HAS_HOTEND
      case Heater0TempEnterKey:
        thermalManager.setTargetHotend(recdat.data[0], 0);
        updateTempE0();
        break;
    #endif

    #if HAS_HEATED_BED
      case HotBedTempEnterKey:
        thermalManager.setTargetBed(recdat.data[0]);
        updateTempBed();
        break;
    #endif

    case Heater0LoadEnterKey:
      filament_load_0 = float(recdat.data[0]) / 10.0f;
      break;

    case AxisPageSelectKey: // Mobile shaft interface
      switch (recdat.data[0]) {
        case 1:
          AxisUnitMode = 1;
          axis_unit = 10.0;
          gotoPage(ID_Move10_L, ID_Move10_D);
          break;
        case 2:
          AxisUnitMode = 2;
          axis_unit = 1.0;
          gotoPage(ID_Move1_L, ID_Move1_D);
          break;
        case 3:
          AxisUnitMode = 3;
          axis_unit = 0.1;
          gotoPage(ID_Move01_L, ID_Move01_D);
          break;
        case 4:
          waitway = 4;
          queue.enqueue_now(F("G28"));
          update_time_value = 0;
          sendData(1, Wait_VP);
          gotoPage(ID_AutoHome_L, ID_AutoHome_D);
          sendData(0, MOTOR_FREE_ICON_VP);
          break;
        case 5: // Unlock motor
          queue.enqueue_now(F("M84"));
          sendData(1, MOTOR_FREE_ICON_VP);
          break;
      }
      break;

    case SettingScreenKey: // Set interface
      switch (recdat.data[0]) {
        #if HAS_HOTEND
          case 2: // Go to Reload Filament
            filament_load_0 = 10.0f;
            sendData(filament_load_0 * 10.0f, HEAD0_FILAMENT_LOAD_DATA_VP);
            updateTempE0();
            delay(2);
            gotoPage(ID_Load_L, ID_Load_D);
            break;
        #endif

        case 3: // Go to Move Axis
          AxisUnitMode = 1;
          TERN_(HAS_X_AXIS, sendData(current_position.x * 10.0f, AXIS_X_COORD_VP));
          TERN_(HAS_Y_AXIS, sendData(current_position.y * 10.0f, AXIS_Y_COORD_VP));
          TERN_(HAS_Z_AXIS, sendData(current_position.z * 10.0f, AXIS_Z_COORD_VP));
          gotoPage(ID_Move10_L, ID_Move10_D);
          break;

        case 4: // Go to Advanced Settings
          TERN_(LIN_ADVANCE, sendData(planner.extruder_advance_K[0] * 100, Advance_K_VP));
          gotoPage(ID_AdvWarn_L, ID_AdvWarn_D);
          break;

        case 5: // Information
          sendPrinterInfo();
          sendData(MARLINVERSION, MARLIN_VERSION_TEXT_VP);
          gotoPage(ID_Info_L, ID_Info_D);
          break;

        case 7: // Return
        gotoPage(ID_Home_L, ID_Home_D);
          break;
      }
      break;

    case SettingBackKey: // Adjust interface Return button
      if (recdat.data[0] == 1) {
        update_time_value = RTS_UPDATE_VALUE;
        //settings.save();
        gotoPage(ID_Home_L, ID_Home_D);
      }
      else if (recdat.data[0] == 2) {
        if (!planner.has_blocks_queued()) {
          gotoPage(ID_Level5_L, ID_Level5_D);
        }
      }
      break;

    case BedLevelFunKey: // Leveling interface
      if (recdat.data[0] == 1) { // Save the button
        settings.save();
        waitway = 6;
        queue.enqueue_now(F("G28Z\nG1 F200 Z0.0"));
      }
      else if (recdat.data[0] == 4) {
        queue.enqueue_now(F("G34\nG1 F200 Z0.0"));
      }
      else if (recdat.data[0] == 5) {
        #if ENABLED(FIX_MOUNTED_PROBE)
          waitway = 3;
          sendData(1, AUTO_BED_LEVEL_ICON_VP);
          queue.enqueue_now(F("G29"));
        #endif
      }

      if (recdat.data[0] == 11) settings.save(); // Adjust interface saving

      sendData(0, MOTOR_FREE_ICON_VP);
      break;

    #if HAS_X_AXIS
      case XaxismoveKey: {
        waitway = 4;
        current_position.x = float(recdat.data[0] >= 32768 ? recdat.data[0] - 65536 : recdat.data[0]) / 10.0f;
        LIMIT(current_position.x, X_MIN_POS, X_MAX_POS);
        RTS_line_to_current(X_AXIS);
        sendData(current_position.x * 10.0f, AXIS_X_COORD_VP);
        sendData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      } break;
    #endif

    #if HAS_Y_AXIS
      case YaxismoveKey: {
        waitway = 4;
        current_position.y = float(recdat.data[0]) / 10.0f;
        LIMIT(current_position.y, Y_MIN_POS, Y_MAX_POS);
        RTS_line_to_current(Y_AXIS);
        sendData(current_position.y * 10.0f, AXIS_Y_COORD_VP);
        sendData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      } break;
    #endif

    #if HAS_Z_AXIS
      case ZaxismoveKey: {
        waitway = 4;
        current_position.z = float(recdat.data[0]) / 10.0f;
        LIMIT(current_position.z, Z_MIN_POS, Z_MAX_POS);
        RTS_line_to_current(Z_AXIS);
        sendData(current_position.z * 10.0f, AXIS_Z_COORD_VP);
        sendData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      } break;
    #endif

    case FilamentLoadKey: // Change
      switch (recdat.data[0]) {
        case 1:
          if (planner.has_blocks_queued()) break;
          if (TERN0(CHECKFILAMENT, runout.filament_ran_out)) gotoPage(ID_NoFilament_L, ID_NoFilament_D);
          current_position.e -= filament_load_0;

          if (thermalManager.degHotend(0) < change_filament_temp_0 - 5) {
            sendData(int16_t(change_filament_temp_0), CHANGE_FILAMENT0_TEMP_VP);
            gotoPage(ID_LoadCold_L, ID_LoadCold_D);
          }
          else {
            RTS_line_to_current(E_AXIS);
            sendData(filament_load_0 * 10.0f, HEAD0_FILAMENT_LOAD_DATA_VP);
          }
          break;

        case 2:
          if (planner.has_blocks_queued()) break;
          if (TERN0(CHECKFILAMENT, runout.filament_ran_out)) gotoPage(ID_NoFilament_L, ID_NoFilament_D);
          current_position.e += filament_load_0;

          if (thermalManager.degHotend(0) < change_filament_temp_0 - 5) {
            sendData(int16_t(change_filament_temp_0), CHANGE_FILAMENT0_TEMP_VP);
            gotoPage(ID_LoadCold_L, ID_LoadCold_D);
          }
          else {
            RTS_line_to_current(E_AXIS);
            sendData(filament_load_0 * 10.0f, HEAD0_FILAMENT_LOAD_DATA_VP);
          }
          break;

        case 3:
          updateTempE0();
          sendData(filament_load_0 * 10.0f, HEAD0_FILAMENT_LOAD_DATA_VP);
          gotoPage(ID_LoadCancel_L, ID_LoadCancel_D);
          break;

        case 4:
          if (planner.has_blocks_queued()) break;
          thermalManager.setTargetHotend(0, 0);
          updateTempE0();
          gotoPage(ID_Load_L, ID_Load_D);
          filament_load_0 = 10.0f;
          sendData(filament_load_0 * 10.0f, HEAD0_FILAMENT_LOAD_DATA_VP);
          break;

        case 5:
          if (planner.has_blocks_queued()) break;
          thermalManager.setTargetHotend(change_filament_temp_0, 0);
          updateTempE0();
          gotoPage(ID_LoadHeating_L, ID_LoadHeating_D);
          heatway = 1;
          break;

        case 6:
          if (planner.has_blocks_queued()) break;
          filament_load_0 = 10.0f;
          sendData(filament_load_0 * 10.0f, HEAD0_FILAMENT_LOAD_DATA_VP);
          gotoPage(ID_Load_L, ID_Load_D);
          break;

        case 8:
          if (planner.has_blocks_queued()) break;
          thermalManager.setTargetHotend(change_filament_temp_0, 0);
          updateTempE0();
          gotoPage(ID_LoadHeating_L, ID_LoadHeating_D);
          heatway = 1;
          break;
      }
      break;

    case FilamentCheckKey:
      switch (recdat.data[0]) {
        case 1:
          TERN_(CHECKFILAMENT, gotoPage(runout.filament_ran_out ? ID_NoFilament_L : ID_Load_L, runout.filament_ran_out ? ID_NoFilament_D : ID_Load_D)); // TODO: add page enum
          break;
        case 2:
          gotoPage(ID_Settings_L, ID_Settings_D);
          filament_load_0 = 10.0f;
          break;
      }
      break;

    case PowerContinuePrintKey: // Power off
      switch (recdat.data[0]) {
        case 1:
          if (!poweroff_continue) break;
          power_off_type_yes = true;
          update_time_value = 0;
          gotoPage(ID_PrintHeating_L, ID_PrintHeating_D);
          queue.enqueue_now(F("M1000"));

          poweroff_continue = true;
          sdcard_pause_check = true;
          zprobe_zoffset = probe.offset.z;
          sendData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          print_state = 2;
          break;

        case 2:
          update_time_value = RTS_UPDATE_VALUE;
          gotoPage(ID_Home_L, ID_Home_D);
          poweroff_continue = false;
          sdcard_pause_check = true;
          queue.clear();
          quickstop_stepper();
          print_job_timer.abort();
          thermalManager.disable_all_heaters();
          print_job_timer.reset();

          #if ALL(HAS_MEDIA, POWER_LOSS_RECOVERY)
            if (card.flag.mounted) {
              card.removeJobRecoveryFile();
              recovery.info.valid_head = 0;
              recovery.info.valid_foot = 0;
              recovery.close();
            }
          #endif

          wait_for_heatup = wait_for_user = false;
          print_state = 0;
          break;
      }
      break;

    case SelectFileKey:
      if (!sdDetected()) break;
      if (recdat.data[0] > cardRec.Filesum) break;

      cardRec.recordcount = recdat.data[0] - 1;
      for (uint8_t j = 0; j < 10; j++) {
        sendData(0, SELECT_FILE_TEXT_VP + j);
        sendData(0, PRINT_FILE_TEXT_VP + j);
      }
      sendData(cardRec.display_filename[cardRec.recordcount], SELECT_FILE_TEXT_VP);
      sendData(cardRec.display_filename[cardRec.recordcount], PRINT_FILE_TEXT_VP);
      delay(2);

      for (uint16_t j = 1; j <= cardRec.Filesum; j++)
        sendData(uint32_t(0x738E), FilenameNature + j * 16);

      sendData(uint32_t(0x041F), FilenameNature + recdat.data[0] * 16);
      sendData(1, FILE1_SELECT_ICON_VP + (recdat.data[0] - 1));
      break;

    case PrintFileKey: // List of print files
      switch (recdat.data[0]) {
        case 1: {
          if (!sdDetected()) break;
          if (cardRec.recordcount < 0) break;
          char cmd[30];
          char *c;
          sprintf_P(cmd, M23_STR, cardRec.filename[cardRec.recordcount]);
          for (c = &cmd[4]; *c; c++) *c = tolower(*c);

          ZERO(cmdbuf);
          strncpy(cmdbuf, cmd, 20);
          FilenamesCount = cardRec.recordcount;

          #if ENABLED(CHECKFILAMENT)
            if (runout.filament_ran_out) {
              gotoPage(ID_FilamentOut_L, ID_FilamentOut_D);
              sdcard_pause_check = false;
              break;
            }
          #endif

          queue.enqueue_one_now(cmd); delay(20);
          queue.enqueue_now_P(M24_STR);

          // Clean screen
          for (uint8_t j = 0; j < 20; j++) sendData(0, PRINT_FILE_TEXT_VP + j);

          sendData(cardRec.display_filename[cardRec.recordcount], PRINT_FILE_TEXT_VP);
          delay(2);

          TERN_(BABYSTEPPING, sendData(0, AUTO_BED_LEVEL_ZOFFSET_VP));

          feedrate_percentage = 100;
          sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          #if HAS_BED_PROBE
            zprobe_zoffset = last_zoffset;
            sendData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          #endif

          poweroff_continue = true;
          gotoPage(ID_PrintHeating_L, ID_PrintHeating_D);
          change_page_number = ID_PrintStatus_D;
          update_time_value = 0;
          start_print_flag = true;
          print_state = 2;
        } break;

        case 2: gotoPage(ID_Page2_L, ID_Page2_D); break;
        case 3: gotoPage(ID_Page1_L, ID_Page1_D); break;
        case 4: gotoPage(ID_Page3_L, ID_Page3_D); break;
        case 5: gotoPage(ID_Page2_L, ID_Page2_D); break;
        case 6: gotoPage(ID_Page4_L, ID_Page4_D); break;
        case 7: gotoPage(ID_Page3_L, ID_Page3_D); break;
        case 8: gotoPage(ID_Home_L, ID_Home_D); break;
      }
      break;

    case StoreMemoryKey: // Initialization
      if (recdat.data[0] == 0xF1) {
        settings.init_eeprom();
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          bool zig = true;
          int8_t inStart, inStop, inInc, showcount;
          showcount = 0;
          for (int8_t y = 0; y < GRID_MAX_POINTS_Y; y++) {
            // Away from origin
            if (zig) {
              inStart = 0;
              inStop = GRID_MAX_POINTS_X;
              inInc = 1;
            }
            else {
              // Towards origin
              inStart = GRID_MAX_POINTS_X - 1;
              inStop = -1;
              inInc = -1;
            }
            FLIP(zig);
            for (int8_t x = inStart; x != inStop; x += inInc) {
              sendData(bedlevel.z_values[x][y] * 100, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
              showcount++;
            }
          }
        #endif
        sendData(Beep, SoundAddr);
        zprobe_zoffset = 0;
        last_zoffset = 0;
        sendData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        gotoPage(ID_Home_L, ID_Home_D);
      }
      else if (recdat.data[0] == 0xF0) {
        gotoPage(ID_Settings_L, ID_Settings_D);
      }
      break;

    case AdvancedKey: // Advanced setting interface
      switch (recdat.data[0]) {
        case 1: { // PID
          #if ENABLED(PIDTEMP)
            const float hot_p = thermalManager.temp_hotend[0].pid.p() * 100.0f,
                        hot_i = (thermalManager.temp_hotend[0].pid.i() / 8.0f * 10000.0f) + 0.00001f,
                        hot_d = thermalManager.temp_hotend[0].pid.d() * 8.0f;
            sendData(hot_p, Nozzle_P_VP);
            sendData(hot_i, Nozzle_I_VP);
            sendData(hot_d, Nozzle_D_VP);
          #endif

          #if ENABLED(PIDTEMPBED)
            const float bed_p = thermalManager.temp_bed.pid.p() * 100.0f,
                        bed_i = (thermalManager.temp_bed.pid.i() / 8.0f * 10000.0f) + 0.0001f,
                        bed_d = thermalManager.temp_bed.pid.d() * 0.8f;

            sendData(bed_p, Hot_Bed_P_VP);
            sendData(bed_i, Hot_Bed_I_VP);
            sendData(bed_d, Hot_Bed_D_VP);
          #endif
          gotoPage(ID_TempPID_L, ID_TempPID_D);
        } break;

        case 2: // Speed
          TERN_(HAS_X_AXIS, sendData(planner.settings.max_feedrate_mm_s[X_AXIS], Vmax_X_VP));
          TERN_(HAS_Y_AXIS, sendData(planner.settings.max_feedrate_mm_s[Y_AXIS], Vmax_Y_VP));
          TERN_(HAS_Z_AXIS, sendData(planner.settings.max_feedrate_mm_s[Z_AXIS], Vmax_Z_VP));
          TERN_(HAS_HOTEND, sendData(planner.settings.max_feedrate_mm_s[E_AXIS], Vmax_E_VP));
          gotoPage(ID_Velocity_L, ID_Velocity_D);
          break;

        case 3: // Acceleration
          sendData(planner.settings.acceleration, Accel_VP);
          sendData(planner.settings.travel_acceleration, A_Travel_VP);

          TERN_(HAS_X_AXIS, sendData(planner.settings.max_acceleration_mm_per_s2[X_AXIS], Amax_X_VP));
          TERN_(HAS_Y_AXIS, sendData(planner.settings.max_acceleration_mm_per_s2[Y_AXIS], Amax_Y_VP));
          TERN_(HAS_Z_AXIS, sendData(planner.settings.max_acceleration_mm_per_s2[Z_AXIS], Amax_Z_VP));
          #if HAS_HOTEND
            sendData(planner.settings.retract_acceleration, A_Retract_VP);
            sendData(planner.settings.max_acceleration_mm_per_s2[E_AXIS], Amax_E_VP);
          #endif
          gotoPage(ID_Accel_L, ID_Accel_D);
          break;

        #if ENABLED(CLASSIC_JERK)
          case 4: // Jerk
            TERN_(HAS_X_AXIS, sendData(planner.max_jerk.x * 10.0f, Jerk_X_VP));
            TERN_(HAS_Y_AXIS, sendData(planner.max_jerk.y * 10.0f, Jerk_Y_VP));
            TERN_(HAS_Z_AXIS, sendData(planner.max_jerk.z * 10.0f, Jerk_Z_VP));
            TERN_(HAS_HOTEND, sendData(planner.max_jerk.e * 10.0f, Jerk_E_VP));
            gotoPage(ID_Jerk_L, ID_Jerk_D);
            break;
        #endif

        case 5: // Steps
          TERN_(HAS_X_AXIS, sendData(planner.settings.axis_steps_per_mm[X_AXIS] * 10.0f, Steps_X_VP));
          TERN_(HAS_Y_AXIS, sendData(planner.settings.axis_steps_per_mm[Y_AXIS] * 10.0f, Steps_Y_VP));
          TERN_(HAS_Z_AXIS, sendData(planner.settings.axis_steps_per_mm[Z_AXIS] * 10.0f, Steps_Z_VP));
          TERN_(HAS_HOTEND, sendData(planner.settings.axis_steps_per_mm[E_AXIS] * 10.0f, Steps_E_VP));
          gotoPage(ID_Steps_L, ID_Steps_D);
          break;

        case 6: gotoPage(ID_Settings_L, ID_Settings_D); break;  // Return

        #if ENABLED(LIN_ADVANCE)
          case 7: // Confirm
            sendData(planner.extruder_advance_K[0] * 100, Advance_K_VP);
            gotoPage(ID_Advanced_L, ID_Advanced_D);
            break;
        #endif

        case 8: gotoPage(ID_SettingsTMC_L, ID_SettingsTMC_D); break;  // TMC

        #if ENABLED(EEPROM_SETTINGS)
          case 9: settings.save(); break;   // Save Settings
        #endif
      }
      break;

    #if ENABLED(PIDTEMP)
      case Nozzle_P: SET_HOTEND_PID(Kp, 0, float(recdat.data[0]) / 100.0f); thermalManager.updatePID(); break;
      case Nozzle_I: SET_HOTEND_PID(Ki, 0, float(recdat.data[0]) * 8.0f / 10000.0f); thermalManager.updatePID(); break;
      case Nozzle_D: SET_HOTEND_PID(Kd, 0, float(recdat.data[0]) / 8.0f); thermalManager.updatePID(); break;
    #endif

    #if ENABLED(PIDTEMPBED)
      case Hot_Bed_P: thermalManager.temp_bed.pid.Kp = float(recdat.data[0]) / 100.0f; break;
      case Hot_Bed_I: thermalManager.temp_bed.pid.Ki = float(recdat.data[0]) * 8.0f / 10000.0f; break;
      case Hot_Bed_D: thermalManager.temp_bed.pid.Kd = float(recdat.data[0]) / 0.8f; break;
    #endif

    #if HAS_X_AXIS
      case Vmax_X: planner.settings.max_feedrate_mm_s[X_AXIS] = recdat.data[0]; break;
      case Amax_X: planner.settings.max_acceleration_mm_per_s2[X_AXIS] = recdat.data[0]; break;
      case Steps_X: planner.settings.axis_steps_per_mm[X_AXIS] = float(recdat.data[0]) / 10.0f; break;
      #if ENABLED(CLASSIC_JERK)
        case Jerk_X: planner.max_jerk.x = float(recdat.data[0]) / 10.0f; break;
      #endif
    #endif
    #if HAS_Y_AXIS
      case Vmax_Y: planner.settings.max_feedrate_mm_s[Y_AXIS] = recdat.data[0]; break;
      case Amax_Y: planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = recdat.data[0]; break;
      case Steps_Y: planner.settings.axis_steps_per_mm[Y_AXIS] = float(recdat.data[0]) / 10.0f; break;
      #if ENABLED(CLASSIC_JERK)
        case Jerk_Y: planner.max_jerk.y = float(recdat.data[0]) / 10.0f; break;
      #endif
    #endif
    #if HAS_Z_AXIS
      case Vmax_Z: planner.settings.max_feedrate_mm_s[Z_AXIS] = recdat.data[0]; break;
      case Amax_Z: planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = recdat.data[0]; break;
      case Steps_Z: planner.settings.axis_steps_per_mm[Z_AXIS] = float(recdat.data[0]) / 10.0f; break;
      #if ENABLED(CLASSIC_JERK)
        case Jerk_Z: planner.max_jerk.z = float(recdat.data[0]) / 10.0f; break;
      #endif
    #endif
    #if HAS_HOTEND
      case Vmax_E: planner.settings.max_feedrate_mm_s[E_AXIS] = recdat.data[0]; break;
      case Amax_E: planner.settings.max_acceleration_mm_per_s2[E_AXIS] = recdat.data[0]; break;
      case Steps_E: planner.settings.axis_steps_per_mm[E_AXIS] = float(recdat.data[0]) / 10.0f; break;
      #if ENABLED(CLASSIC_JERK)
        case Jerk_E: planner.max_jerk.e = float(recdat.data[0]) / 10.0f; break;
      #endif
      case A_Retract: planner.settings.retract_acceleration = recdat.data[0]; break;
      #if ENABLED(LIN_ADVANCE)
        case Advance_K: planner.extruder_advance_K[0] = float(recdat.data[0]) / 100.0f; break;
      #endif
    #endif
    case Accel: planner.settings.acceleration = recdat.data[0]; break;
    case A_Travel: planner.settings.travel_acceleration = recdat.data[0]; break;

    case AdvancedBackKey: gotoPage(ID_Advanced_L, ID_Advanced_D); break;

    #if HAS_FILAMENT_SENSOR
      case FilamentChange: // Automatic material
        switch (recdat.data[0]) {
          case 1: if (FILAMENT_IS_OUT()) break;
          case 2:
            updateTempE0();
            wait_for_heatup = wait_for_user = false;
            break;
          case 3: pause_menu_response = PAUSE_RESPONSE_EXTRUDE_MORE; break;
          case 4: pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT; break;
        }
        break;
    #endif

    #if HAS_BED_PROBE
      case ZoffsetUnitKey: // Zoffset unit
        switch (recdat.data[0]) {
          case 1: gotoPage(ID_PrintAdjust5_L, ID_PrintAdjust5_D); break;
          case 2: gotoPage(ID_PrintAdjust1_L, ID_PrintAdjust1_D); break;
          case 3: gotoPage(ID_Level5_L, ID_Level5_D); break;
          case 4: gotoPage(ID_Level1_L, ID_Level1_D); break;
        }
        break;

      case ZOffsetKey:
          last_zoffset = zprobe_zoffset;
          zprobe_zoffset = float(recdat.data[0] >= 32767 ? recdat.data[0] - 65537 : recdat.data[0]) / 100.0f + 0.0001f;
          if (WITHIN(zprobe_zoffset, PROBE_OFFSET_ZMIN, PROBE_OFFSET_ZMAX))
            babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
          sendData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        break;
    #endif

    #if HAS_TRINAMIC_CONFIG

      case TMCDriver:
        switch (recdat.data[0]) {
          case 1:  // Current
            #if AXIS_IS_TMC(X)
              sendData(stepperX.getMilliamps(), Current_X_VP);
            #endif
            #if AXIS_IS_TMC(Y)
              sendData(stepperY.getMilliamps(), Current_Y_VP);
            #endif
            #if AXIS_IS_TMC(Z)
              sendData(stepperZ.getMilliamps(), Current_Z_VP);
            #endif
            #if AXIS_IS_TMC(E0)
              sendData(stepperE0.getMilliamps(), Current_E_VP);
            #endif
            gotoPage(ID_DriverA_L, ID_DriverA_D);
            break;

          case 2:  // Threshold
            TERN_(X_HAS_STEALTHCHOP,  sendData(stepperX.get_pwm_thrs(), Threshold_X_VP));
            TERN_(Y_HAS_STEALTHCHOP,  sendData(stepperY.get_pwm_thrs(), Threshold_Y_VP));
            TERN_(Z_HAS_STEALTHCHOP,  sendData(stepperZ.get_pwm_thrs(), Threshold_Z_VP));
            TERN_(E0_HAS_STEALTHCHOP, sendData(stepperE0.get_pwm_thrs(), Threshold_E_VP));
            gotoPage(ID_DriverTrsh_L, ID_DriverTrsh_D);
            break;

          #if ENABLED(SENSORLESS_HOMING)
            case 3:  // Sensorless
              TERN_(X_SENSORLESS, sendData(stepperX.homing_threshold(), Sensorless_X_VP));
              TERN_(Y_SENSORLESS, sendData(stepperY.homing_threshold(), Sensorless_Y_VP));
              gotoPage(ID_DriverSens_L, ID_DriverSens_D);
              break;
          #endif

          case 4: gotoPage(ID_Advanced_L, ID_Advanced_D); break;
          case 5: gotoPage(ID_SettingsTMC_L, ID_SettingsTMC_D); break;
        }
        break;

      #if AXIS_IS_TMC(X)
        case Current_X:    sprintf_P(cmd, PSTR("M906 X%i"), recdat.data[0]); queue.inject(cmd); break;
      #endif
      #if X_HAS_STEALTHCHOP
        case Threshold_X:  sprintf_P(cmd, PSTR("M913 X%i"), recdat.data[0]); queue.inject(cmd); break;
      #endif
      #if X_SENSORLESS
        case Sensorless_X: sprintf_P(cmd, PSTR("M914 X%i"), recdat.data[0]); queue.inject(cmd); break;
      #endif

      #if AXIS_IS_TMC(Y)
        case Current_Y:    sprintf_P(cmd, PSTR("M906 Y%i"), recdat.data[0]); queue.inject(cmd); break;
      #endif
      #if Y_HAS_STEALTHCHOP
        case Threshold_Y:  sprintf_P(cmd, PSTR("M913 Y%i"), recdat.data[0]); queue.inject(cmd); break;
      #endif
      #if Y_SENSORLESS
        case Sensorless_Y: sprintf_P(cmd, PSTR("M914 Y%i"), recdat.data[0]); queue.inject(cmd); break;
      #endif

      #if AXIS_IS_TMC(Z)
        case Current_Z:    sprintf_P(cmd, PSTR("M906 Z%i"), recdat.data[0]); queue.inject(cmd); break;
      #endif
      #if Z_HAS_STEALTHCHOP
        case Threshold_Z:  sprintf_P(cmd, PSTR("M913 Z%i"), recdat.data[0]); queue.inject(cmd); break;
      #endif

      #if AXIS_IS_TMC(E0)
        case Current_E:   sprintf_P(cmd, PSTR("M906 E%i"), recdat.data[0]); queue.inject(cmd); break;
      #endif
      #if E0_HAS_STEALTHCHOP
        case Threshold_E: sprintf_P(cmd, PSTR("M913 E%i"), recdat.data[0]); queue.inject(cmd); break;
      #endif

    #endif // HAS_TRINAMIC_CONFIG

    case ChangePageKey:
      switch (change_page_number) {
        case ID_Resume_D: case ID_Settings_L: break; // This makes no sense & change_page_number is never set to those values

        case ID_PrintStatus_D:
          refreshTime();
          start_print_flag = false;
          break;

        case ID_PrintResume_D: gotoPage(ID_PrintResume_L, ID_PrintResume_D); break;

        default:
          if (card.isPrinting()) {
            for (uint16_t i = 0;i < cardRec.Filesum; i++)
              if (!strcmp(cardRec.filename[i], &recovery.info.sd_filename[1]))
                sendData(cardRec.display_filename[i], PRINT_FILE_TEXT_VP);
            refreshTime();
          }
          else
            gotoPage(change_page_number);
          break;
      }

      for (uint8_t i = 0; i < MAX_NUM_FILES; i++)
        for (uint8_t j = 0; j < 20; j++)
          sendData(0, FILE1_TEXT_VP + i * 20 + j);

      for (uint16_t i = 0; i < cardRec.Filesum; i++) {
        for (uint8_t j = 0; j < 20; j++) sendData(0, cardRec.addr[i] + j);
        sendData(uint32_t(0x738E), FilenameNature + (i + 1) * 16);
      }

      for (uint8_t j = 0; j < 20; j++) {
        sendData(0, PRINT_FILE_TEXT_VP + j);  // Clean screen
        sendData(0, SELECT_FILE_TEXT_VP + j); // Clean filename
      }

      // Clean filename Icon
      for (uint8_t j = 0; j < 20; j++)
        sendData(10, FILE1_SELECT_ICON_VP + j);

      sendData(cardRec.display_filename[cardRec.recordcount], PRINT_FILE_TEXT_VP);

      // Represents to update file list
      if (update_sd && lcd_sd_status && IS_SD_INSERTED()) {
        for (uint16_t i = 0; i < cardRec.Filesum; i++) {
          delay(3);
          sendData(cardRec.display_filename[i], cardRec.addr[i]);
          sendData(uint32_t(0x738E), FilenameNature + (i + 1) * 16);
          sendData(0, FILE1_SELECT_ICON_VP + i);
        }
      }

      sendPrinterInfo();

      updateFan0();

      job_percent = ui.get_progress_percent();
      if (job_percent <= 100) sendData(uint8_t(job_percent), PRINT_PROCESS_ICON_VP);

      sendData(uint8_t(card.percentDone()), PRINT_PROCESS_VP);

      TERN_(HAS_BED_PROBE, sendData(probe.offset.z * 100.0f, AUTO_BED_LEVEL_ZOFFSET_VP));

      sendData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      updateTempE0();
      updateTempBed();

      break;

    default: break;
  }
  ZERO(&recdat);
  recdat.head[0] = FHONE;
  recdat.head[1] = FHTWO;
}

void RTS::sendPrinterInfo() {
  sendData(MACVERSION, PRINTER_MACHINE_TEXT_VP);
  sendData(SOFTVERSION, PRINTER_VERSION_TEXT_VP);
  sendData(CORP_WEBSITE_E, PRINTER_WEBSITE_TEXT_VP);
  char printarea[20] = { '\0' };
  sprintf_P(printarea,
    PSTR(TERN_(HAS_X_AXIS, "%d x") TERN_(HAS_Y_AXIS, " %d x") TERN_(HAS_Z_AXIS, " %d"))
    , X_BED_SIZE, Y_BED_SIZE, Z_MAX_POS
  );
  sendData(printarea, PRINTER_PRINTSIZE_TEXT_VP);
}

void RTS::updateTempE0() {
  #if HAS_HOTEND
    sendData(thermalManager.degHotend(0), HEAD0_CURRENT_TEMP_VP);
    sendData(thermalManager.degTargetHotend(0), HEAD0_SET_TEMP_VP);
  #endif
}

void RTS::updateTempBed() {
  #if HAS_HEATED_BED
    sendData(thermalManager.degBed(), BED_CURRENT_TEMP_VP);
    sendData(thermalManager.degTargetBed(), BED_SET_TEMP_VP);
  #endif
}

void RTS::updateFan0() {
  TERN_(HAS_FAN, sendData(thermalManager.fan_speed[0], FAN_SPEED_VP));
}

void RTS::onIdle() {
  const millis_t ms = millis();
  if (PENDING(ms, next_rts_update_ms)) return;

  // Print the file before the power is off.
  if (!power_off_type_yes) {
    if (!poweroff_continue || (lcd_sd_status && poweroff_continue)) {
      sendData(ExchangePageBase, ExchangepageAddr);
      if (startprogress < 0) startprogress = 0;
      if (startprogress < 56) sendData(startprogress, START1_PROCESS_ICON_VP);
      delay(80);
      if (++startprogress > 56) {
        sendData(StartSoundSet, SoundAddr);
        power_off_type_yes = true;
      }
    }

    if (lcd_sd_status && poweroff_continue && power_off_type_yes) {
      for (uint16_t i = 0; i < cardRec.Filesum; i++) {
        if (!strcmp(cardRec.filename[i], &recovery.info.sd_filename[1])) {
          sendData(cardRec.display_filename[i], PRINT_FILE_TEXT_VP);
          gotoPage(ID_Resume_L, ID_Resume_D);
          break;
        }
      }
    }

    if (!poweroff_continue && power_off_type_yes) {
      update_time_value = RTS_UPDATE_VALUE;
      change_page_number = dark_mode ? ID_Home_D : ID_Home_L;
      gotoPage(change_page_number);
    }
    return;
  }

  next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + update_time_value;

  // TODO: optimize the following
  if (print_job_timer.duration() != 0) {
    duration_t elapsed = print_job_timer.duration();
    static uint8_t last_cardpercentValue = 100;
    sendData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
    sendData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);

    if (card.isPrinting() && (last_cardpercentValue != card.percentDone())) {
      if (card.percentDone() > 0) {
        job_percent = ui.get_progress_percent();
        if (job_percent <= 100) sendData(uint8_t(job_percent), PRINT_PROCESS_ICON_VP);
        // Estimate remaining time every 20 seconds
        static millis_t next_remain_time_update = 0;
        if (ELAPSED(ms, next_remain_time_update)) {
          if (thermalManager.degHotend(0) >= thermalManager.degTargetHotend(0) - 5) {
            remain_time = ui.get_remaining_time();
            next_remain_time_update += 20 * 1000UL;
            sendData(remain_time / 3600, PRINT_SURPLUS_TIME_HOUR_VP);
            sendData((remain_time % 3600) / 60, PRINT_SURPLUS_TIME_MIN_VP);
          }
        }
      }
      else {
        sendData(0, PRINT_PROCESS_ICON_VP);
        sendData(0, PRINT_SURPLUS_TIME_HOUR_VP);
        sendData(0, PRINT_SURPLUS_TIME_MIN_VP);
      }
      sendData(uint8_t(card.percentDone()), PRINT_PROCESS_VP);
      last_cardpercentValue = card.percentDone();
    }
  }

  if (pause_action_flag && !sdcard_pause_check && printingIsPaused() && !planner.has_blocks_queued()) {
    pause_action_flag = false;
    queue.enqueue_now(F("G0 F3000 X0 Y0\nM18 S0"));
  }

  TERN_(HAS_Z_AXIS, sendData(current_position.z * 10.0f, AXIS_Z_COORD_VP));

  #if HAS_HOTEND
    if (last_target_temperature[0] != thermalManager.degTargetHotend(0))
      last_target_temperature[0] = thermalManager.degTargetHotend(0);
    updateTempE0();
  #endif

  #if HAS_HEATED_BED
    if (last_target_temperature_bed != thermalManager.degTargetBed())
      last_target_temperature_bed = thermalManager.degTargetBed();
    updateTempBed();
  #endif

  #if HAS_HOTEND
    if (heatway == 1 && !thermalManager.isHeatingHotend(0)) {
      heatway = 0;
      gotoPage(ID_Load_L, ID_Load_D);
      sendData(filament_load_0 * 10.0f, HEAD0_FILAMENT_LOAD_DATA_VP);
    }
  #endif

  #if HAS_FILAMENT_SENSOR
    TERN_(CHECKFILAMENT, sendData(!runout.filament_ran_out, CHANGE_FILAMENT_ICON_VP));
  #endif
}

// Looping at the loop function
void RTS_Update() {
  // Check the status of card
  rts.sdCardUpdate();

  sd_printing = IS_SD_PRINTING();
  card_insert_st = IS_SD_INSERTED();

  if (!card_insert_st && sd_printing) {
    rts.gotoPage(ID_MediaFail_L, ID_MediaFail_D);
    rts.sendData(0, CHANGE_SDCARD_ICON_VP);
    // Passenger printing so that the nozzle can return to zero
    card.pauseSDPrint();
    print_job_timer.pause();
    pause_action_flag = true;
    sdcard_pause_check = false;
  }
  // Update the card withdrawal and card tips icon
  if (last_card_insert_st != card_insert_st) {
    // The current page is displayed as a card removal page, but the card has been inserted, and the card icon is updated.
    rts.sendData(int16_t(card_insert_st), CHANGE_SDCARD_ICON_VP);
    last_card_insert_st = card_insert_st;
  }

  #if ENABLED(CHECKFILAMENT)
    // Check filament status during printing
    // The runout class handles the runout threshold
    if (card.isPrinting() && poweroff_continue && runout.filament_ran_out) {
      rts.sendData(Beep, SoundAddr);
      if (TERN0(HAS_HOTEND, thermalManager.still_heating(0)) || TERN0(HAS_HEATED_BED, !thermalManager.isCoolingBed())) {
        rts.gotoPage(ID_FilamentOut_L, ID_FilamentOut_D);
      }
      else {
        rts.sendData(1, Wait_VP);
        rts.gotoPage(ID_Processing_L, ID_Processing_D);
        waitway = 5;
        TERN_(POWER_LOSS_RECOVERY, if (recovery.enabled) recovery.save(true, false));
      }
      card.pauseSDPrint();
      print_job_timer.pause();
      pause_action_flag = true;
      update_time_value = 0;
      sdcard_pause_check = false;
      rts.print_state = 1;
    }
  #endif // CHECKFILAMENT

  rts.onIdle();

  // Wait to receive massage and response
  while (rts.receiveData() > 0) rts.handleData();
}

void RTS_PauseMoveAxisPage() {
  if (waitway == 1) {
    rts.gotoPage(ID_PrintResume_L, ID_PrintResume_D);
    waitway = 0;
  }
  else if (waitway == 5) {
    rts.gotoPage(ID_FilamentOut_L, ID_FilamentOut_D);
    waitway = 0;
  }
}

void RTS_AutoBedLevelPage() {
  if (waitway == 3) {
    rts.gotoPage(ID_Level5_L, ID_Level5_D);
    waitway = 0;
  }
}

void RTS_MoveAxisHoming() {
  if (waitway == 4) {
    rts.gotoPage((SovolPage)(ID_Move10_L + AxisUnitMode - 1), (SovolPage)(ID_Move10_D + AxisUnitMode - 1));
    waitway = 0;
  }
  else if (waitway == 6) {
    rts.gotoPage(ID_Level5_L, ID_Level5_D);
    waitway = 0;
  }
  else if (waitway == 7) {
    // Click Print finish
    rts.gotoPage(ID_Home_L, ID_Home_D);
    waitway = 0;
  }

  TERN_(HAS_X_AXIS, rts.sendData(current_position.x * 10.0f, AXIS_X_COORD_VP));
  TERN_(HAS_Y_AXIS, rts.sendData(current_position.y * 10.0f, AXIS_Y_COORD_VP));
  TERN_(HAS_Z_AXIS, rts.sendData(current_position.z * 10.0f, AXIS_Z_COORD_VP));
}

#endif // SOVOL_SV06_RTS
