/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/* ****************************************
 * lcd/extui/ia_creality/ia_creality_rts.cpp
 * ****************************************
 * Creality DWIN Resistive Touch Screen
 * 10SPro, Max, CRX, and others
 * Based original Creality release
 * Written by Insanity Automation, sponsored by Tiny Machines 3D
 *
 * ***************************************/

#include "../../../inc/MarlinConfigPre.h"

#if DGUS_LCD_UI_IA_CREALITY

#include "ia_creality_rts.h"
#include "FileNavigator.h"
#include "../ui_api.h"

using namespace ExtUI;

#include <string.h> // for memset

// Singleton instance
RTS rts;

// Static data
DB RTS::recdat, RTS::snddat;
uint8_t RTS::databuf[DATA_BUF_SIZE];
rx_datagram_state_t RTS::rx_datagram_state = DGUS_IDLE;
uint8_t RTS::rx_datagram_len = 0;

// Local data
uint16_t fileIndex          = 0;
uint8_t recordcount         = 0;
uint8_t waitway_lock        = 0;
uint8_t startprogress       = 0;

char waitway                = 0;
int16_t recnum              = 0;
float changeMaterialBuf[2]  = { 0 };
char nozzleTempStatus[3]    = { 0 };
char printerStatusKey[2]    = { 0 };

uint8_t axisPageNum         = 0; // 0 for 10mm, 1 for 1mm, 2 for 0.1mm
bool show_status            = true;
bool tpShowStatus           = false; // true for only opening time and percentage, false for closing time and percentage.
bool autoHomeKey            = false;
uint8_t autoHomeIconNum;
int16_t userConfValidation  = 0;
uint8_t lastPauseMsgState   = 0;

creality_dwin_settings_t dwin_settings;

bool no_reentry         = false;
uint8_t reentryCount    = 0;
uint16_t idleThrottling = 0;

bool pause_resume_selected  = false;

#if HAS_PID_HEATING
  uint16_t pid_hotendAutoTemp = 150;
  uint16_t pid_bedAutoTemp    = 70;
#endif

#ifndef IA_CREALITY_BOOT_DELAY
  #define IA_CREALITY_BOOT_DELAY 500
#endif

void RTS::onStartup() {
  DWIN_SERIAL.begin(115200);
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  ZERO(databuf);

  delay_ms(IA_CREALITY_BOOT_DELAY); // Delay to allow screen startup
  setTouchScreenConfiguration();
  sendData(StartSoundSet, SoundAddr);
  delay_ms(400); // Delay to allow screen to configure

  onStatusChanged(MACHINE_NAME " Ready");

  sendData(100, FeedrateDisplay);

  /***************turn off motor*****************/
  sendData(11, FilenameIcon);

  /***************transmit temperature to screen*****************/
  sendData(0, NozzlePreheat);
  sendData(0, BedPreheat);
  sendData(getActualTemp_celsius(H0), NozzleTemp);
  sendData(TERN0(HAS_MULTI_HOTEND, getActualTemp_celsius(H1)), e2Temp);
  sendData(getActualTemp_celsius(BED), Bedtemp);
  /***************transmit Fan speed to screen*****************/
  sendData(getActualFan_percent((fan_t)getActiveTool()), FanKeyIcon);

  /***************transmit Printer information to screen*****************/
  for (int16_t j = 0; j < 20; j++) // clean filename
    sendData(0, MacVersion + j);
  char sizebuf[20] = {0};
  sprintf(sizebuf, "%d X %d X %d", Y_BED_SIZE, X_BED_SIZE, Z_MAX_POS);
  sendData(MACHINE_NAME, MacVersion);
  sendData(SHORT_BUILD_VERSION, SoftVersion);
  sendData(sizebuf, PrinterSize);
  sendData(WEBSITE_URL, CorpWebsite);

  /**************************some info init*******************************/
  sendData(0, PrintscheduleIcon);
  sendData(0, PrintscheduleIcon + 1);

  /************************clean screen*******************************/
  for (int16_t i = 0; i < MaxFileNumber; i++)
    for (int16_t j = 0; j < 10; j++) sendData(0, SDFILE_ADDR + i * 10 + j);

  for (int16_t j = 0; j < 10; j++) {
    sendData(0, Printfilename + j); // clean screen.
    sendData(0, Choosefilename + j); // clean filename
  }
  for (int16_t j = 0; j < 8; j++) sendData(0, FilenameCount + j);
  for (int16_t j = 1; j <= MaxFileNumber; j++) {
    sendData(10, FilenameIcon + j);
    sendData(10, FilenameIcon1 + j);
  }
}

void RTS::onIdle() {
  while (rts.receiveData() > 0 && (rts.recdat.data[0] != 0 || rts.recdat.addr != 0))
    rts.handleData();

  if (no_reentry && reentryCount < 120) { reentryCount++; return; }
  reentryCount = 0;

  if (idleThrottling++ < 750) return;

  // Always send temperature data
  rts.sendData(getActualTemp_celsius(getActiveTool()), NozzleTemp);
  rts.sendData(getActualTemp_celsius(BED), Bedtemp);
  rts.sendData(getTargetTemp_celsius(getActiveTool()), NozzlePreheat);
  rts.sendData(getTargetTemp_celsius(BED), BedPreheat);
  rts.sendData(TERN0(HAS_MULTI_HOTEND, getActualTemp_celsius(H1)), e2Temp);
  rts.sendData(TERN0(HAS_MULTI_HOTEND, getTargetTemp_celsius(H1)), e2Preheat);
  TERN_(HAS_MULTI_HOTEND, rts.sendData(uint8_t(getActiveTool() + 1), ActiveToolVP));

  if (awaitingUserConfirm() && (lastPauseMsgState != ExtUI::pauseModeStatus || userConfValidation > 99)) {
    if (ExtUI::pauseModeStatus < PAUSE_MESSAGE_COUNT)
      ui.pause_show_message(ExtUI::pauseModeStatus);
    else
      ExtUI::onUserConfirmRequired(F("Confirm Continue"));
    userConfValidation = 0;
  }
  else if (pause_resume_selected && !awaitingUserConfirm()) {
    rts.sendData(ExchangePageBase + 53, ExchangepageAddr);
    pause_resume_selected = false;
    userConfValidation = 0;
  }
  else if (awaitingUserConfirm()) {
    if (pause_resume_selected) {
      pause_resume_selected = false;
      userConfValidation = 100;
    }
    else
      userConfValidation++;
  }

  no_reentry = true;
  idleThrottling = 0;
  if (waitway && !commandsInQueue())
    waitway_lock++;
  else
    waitway_lock = 0;

  if (waitway_lock > 100) {
    waitway_lock = 0;
    waitway = 0; // clear waitway if nothing is going on
  }

  switch (waitway) {
    case 1:
      if (isPositionKnown()) {
        show_status = true;
        rts.sendData(ExchangePageBase + 54, ExchangepageAddr);
        waitway = 0;
      }
      break;

    case 2:
      if (isPositionKnown() && !commandsInQueue()) waitway = 0;
      break;

    case 3:
      //if (isPositionKnown() && (getActualTemp_celsius(BED) >= (getTargetTemp_celsius(BED)-1))) {
      rts.sendData(ExchangePageBase + 64, ExchangepageAddr);
      waitway = 7;
      //return;
      //}
      break;

    case 4:
      if (autoHomeKey && isPositionKnown() && !commandsInQueue()) { // Manual Move Home Done
        //rts.sendData(ExchangePageBase + 71 + axisPageNum, ExchangepageAddr);
        autoHomeKey = false;
        waitway = 0;
      }
      break;
    case 5:
      if (isPositionKnown() && !commandsInQueue()) {
        show_status = true;
        waitway = 0;
        rts.sendData(ExchangePageBase + 78, ExchangepageAddr); // exchange to 78 page
      }
      break;
    case 6:
      if (!commandsInQueue()) {
        setAxisPosition_mm(BED_TRAMMING_HEIGHT, axis_t(Z));
        waitway = 0;
      }
      break;
    case 7:
      if (!commandsInQueue()) waitway = 0;
      break;
  }

  TERN_(HAS_MESH, rts.sendData(getLevelingActive() ? 3 : 2, AutoLevelIcon));
  TERN_(HAS_FILAMENT_SENSOR, rts.sendData(getFilamentRunoutEnabled() ? 3 : 2, RunoutToggle));
  TERN_(CASE_LIGHT_ENABLE, rts.sendData(getCaseLightState() ? 3 : 2, LedToggle));
  TERN_(POWER_LOSS_RECOVERY, rts.sendData(getPowerLossRecoveryEnabled() ? 3 : 2, PowerLossToggle));

  if (startprogress == 0) {
    startprogress += 25;
    delay_ms(3000);     // Delay to show bootscreen
  }
  else if (startprogress < 250) {
    if (isMediaMounted()) // Re init media as it happens too early on STM32 boards often
      onMediaMounted();
    else
      injectCommands(F("M22\nM21"));
    startprogress = 254;
    show_status = true;
    tpShowStatus = false;
    rts.sendData(ExchangePageBase + 45, ExchangepageAddr);
    no_reentry = false;
    return;
  }
  if (startprogress <= 100)
    rts.sendData(startprogress, StartIcon);
  else
    rts.sendData(startprogress - 100, StartIcon + 1);

  //rts.sendData((startprogress++) % 5, ExchFlmntIcon);

  if (isPrinting()) {
    rts.sendData(getActualFan_percent((fan_t)getActiveTool()), FanKeyIcon);
    rts.sendData(getProgress_seconds_elapsed() / 3600, Timehour);
    rts.sendData((getProgress_seconds_elapsed() % 3600) / 60, Timemin);
    if (getProgress_percent() > 0) {
      const uint16_t perc = getProgress_percent() + 1;
      if (perc <= 50) {
        rts.sendData(uint16_t(perc) * 2, PrintscheduleIcon);
        rts.sendData(0, PrintscheduleIcon + 1);
      }
      else {
        rts.sendData(100, PrintscheduleIcon);
        rts.sendData(uint16_t(perc) * 2 - 100, PrintscheduleIcon + 1);
      }
    }
    else {
      rts.sendData(0, PrintscheduleIcon);
      rts.sendData(0, PrintscheduleIcon + 1);
    }
    rts.sendData(uint16_t(getProgress_percent()), Percentage);
  }
  else { // Not printing settings
    rts.sendData(map(constrain(dwin_settings.display_volume, 0, 255), 0, 255, 0, 100), VolumeDisplay);
    rts.sendData(dwin_settings.screen_brightness, DisplayBrightness);
    rts.sendData(dwin_settings.standby_brightness, DisplayStandbyBrightness);
    rts.sendData(dwin_settings.standby_time_seconds, DisplayStandbySeconds);
    if (dwin_settings.display_standby)
      rts.sendData(3, DisplayStandbyEnableIndicator);
    else
      rts.sendData(2, DisplayStandbyEnableIndicator);

    rts.sendData(getAxisSteps_per_mm(X)  * 10, StepMM_X);
    rts.sendData(getAxisSteps_per_mm(Y)  * 10, StepMM_Y);
    rts.sendData(getAxisSteps_per_mm(Z)  * 10, StepMM_Z);
    rts.sendData(getAxisSteps_per_mm(E0) * 10, StepMM_E);

    rts.sendData(getAxisMaxAcceleration_mm_s2(X) / 100, Accel_X);
    rts.sendData(getAxisMaxAcceleration_mm_s2(Y) / 100, Accel_Y);
    rts.sendData(getAxisMaxAcceleration_mm_s2(Z) /  10, Accel_Z);
    rts.sendData(getAxisMaxAcceleration_mm_s2(E0),      Accel_E);

    rts.sendData(getAxisMaxFeedrate_mm_s(X),  Feed_X);
    rts.sendData(getAxisMaxFeedrate_mm_s(Y),  Feed_Y);
    rts.sendData(getAxisMaxFeedrate_mm_s(Z),  Feed_Z);
    rts.sendData(getAxisMaxFeedrate_mm_s(E0), Feed_E);

    rts.sendData(getAxisMaxJerk_mm_s(X) * 100, Jerk_X);
    rts.sendData(getAxisMaxJerk_mm_s(Y) * 100, Jerk_Y);
    rts.sendData(getAxisMaxJerk_mm_s(Z) * 100, Jerk_Z);
    rts.sendData(getAxisMaxJerk_mm_s(E0) * 100, Jerk_E);

    #if HAS_HOTEND_OFFSET
      rts.sendData(getNozzleOffset_mm(X, E1) * 10, T2Offset_X);
      rts.sendData(getNozzleOffset_mm(Y, E1) * 10, T2Offset_Y);
      rts.sendData(getNozzleOffset_mm(Z, E1) * 10, T2Offset_Z);
      rts.sendData(getAxisSteps_per_mm(E1)   * 10, T2StepMM_E);
    #endif

    #if HAS_BED_PROBE
      rts.sendData(getProbeOffset_mm(X) * 100, ProbeOffset_X);
      rts.sendData(getProbeOffset_mm(Y) * 100, ProbeOffset_Y);
    #endif

    #if HAS_PID_HEATING
      rts.sendData(pid_hotendAutoTemp, HotendPID_AutoTmp);
      rts.sendData(pid_bedAutoTemp, BedPID_AutoTmp);
      rts.sendData(getPID_Kp(E0) * 10, HotendPID_P);
      rts.sendData(getPID_Ki(E0) * 10, HotendPID_I);
      rts.sendData(getPID_Kd(E0) * 10, HotendPID_D);
      #if ENABLED(PIDTEMPBED)
        rts.sendData(getBedPID_Kp() * 10, BedPID_P);
        rts.sendData(getBedPID_Ki() * 10, BedPID_I);
        rts.sendData(getBedPID_Kd() * 10, BedPID_D);
      #endif
    #endif
  }

  rts.sendData(getZOffset_mm() * 100, ProbeOffset_Z);
  rts.sendData(uint16_t(getFlow_percent(E0)), Flowrate);

  if (nozzleTempStatus[0] || nozzleTempStatus[2]) { // statuse of loadfilament and unloadfinement when temperature is less than
    uint16_t IconTemp = getActualTemp_celsius(getActiveTool()) * 100 / getTargetTemp_celsius(getActiveTool());
    NOMORE(IconTemp, 100U);
    rts.sendData(IconTemp, HeatPercentIcon);
    if (getActualTemp_celsius(getActiveTool()) > EXTRUDE_MINTEMP && nozzleTempStatus[0] != 0) {
      nozzleTempStatus[0] = 0;
      rts.sendData(10 * changeMaterialBuf[0], FilamentUnit1);
      rts.sendData(10 * changeMaterialBuf[1], FilamentUnit2);
      rts.sendData(ExchangePageBase + 65, ExchangepageAddr);
    }
    else if (getActualTemp_celsius(getActiveTool()) >= getTargetTemp_celsius(getActiveTool()) && nozzleTempStatus[2]) {
      nozzleTempStatus[2] = 0;
      tpShowStatus = true;
      rts.sendData(4, ExchFlmntIcon);
      rts.sendData(ExchangePageBase + 83, ExchangepageAddr);
    }
    else if (nozzleTempStatus[2]) {
      //rts.sendData((startprogress++) % 5, ExchFlmntIcon);
    }
  }

  if (autoHomeKey) {
    rts.sendData(autoHomeIconNum, AutoZeroIcon);
    if (++autoHomeIconNum > 9) autoHomeIconNum = 0;
  }

  if (isMediaMounted()) {
    const uint16_t currPage = fileIndex == 0 ? 1 : CEIL(float(fileIndex) / float(DISPLAY_FILES)) + 1,
                   maxPageAdd = filenavigator.folderdepth ? 1 : 0,
                   maxPages = CEIL(float(filenavigator.maxFiles() + maxPageAdd) / float(DISPLAY_FILES) );

    rts.sendData(currPage, FilesCurentPage);
    rts.sendData(maxPages, FilesMaxPage);
  }
  else {
    rts.sendData(0, FilesCurentPage);
    rts.sendData(0, FilesMaxPage);
  }

  if (rts.recdat.addr != DisplayZaxis && rts.recdat.addr != DisplayYaxis && rts.recdat.addr != DisplayZaxis) {
    rts.sendData(10 * getAxisPosition_mm(axis_t(X)), DisplayXaxis);
    rts.sendData(10 * getAxisPosition_mm(axis_t(Y)), DisplayYaxis);
    rts.sendData(10 * getAxisPosition_mm(axis_t(Z)), DisplayZaxis);
  }
  no_reentry = false;
}

RTS::RTS() {
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  ZERO(databuf);
}

int16_t RTS::receiveData() {
  uint8_t receivedbyte;
  while (DWIN_SERIAL.available())
    switch (rx_datagram_state) {

      case DGUS_IDLE: // Waiting for the first header byte
        receivedbyte = DWIN_SERIAL.read();
        if (FHONE == receivedbyte) rx_datagram_state = DGUS_HEADER1_SEEN;
        break;

      case DGUS_HEADER1_SEEN: // Waiting for the second header byte
        receivedbyte = DWIN_SERIAL.read();
        rx_datagram_state = (FHTWO == receivedbyte) ? DGUS_HEADER2_SEEN : DGUS_IDLE;
        break;

      case DGUS_HEADER2_SEEN: // Waiting for the length byte
        rx_datagram_len = DWIN_SERIAL.read();
        //DEBUGLCDCOMM_ECHOPGM(" (", rx_datagram_len, ") ");

        // Telegram min len is 3 (command and one word of payload)
        rx_datagram_state = WITHIN(rx_datagram_len, 3, DGUS_RX_BUFFER_SIZE) ? DGUS_WAIT_TELEGRAM : DGUS_IDLE;
        break;

      case DGUS_WAIT_TELEGRAM: // wait for complete datagram to arrive.
        if (DWIN_SERIAL.available() < rx_datagram_len) return -1;

        uint8_t command = DWIN_SERIAL.read();

        //DEBUGLCDCOMM_ECHOPGM("# ", command);

        uint8_t readlen = rx_datagram_len - 1; // command is part of len.
        uint8_t tmp[rx_datagram_len - 1];
        uint8_t *ptmp = tmp;
        while (readlen--) {
          receivedbyte = DWIN_SERIAL.read();
          //DEBUGLCDCOMM_ECHOPGM(" ", receivedbyte);
          *ptmp++ = receivedbyte;
        }
        //DEBUGLCDCOMM_ECHOPGM(" # ");
        // mostly we'll get this: 5A A5 03 82 4F 4B -- ACK on 0x82, so discard it.
        if (command == VarAddr_W && 'O' == tmp[0] && 'K' == tmp[1]) {
          rx_datagram_state = DGUS_IDLE;
          break;
        }

        /* AutoUpload, (and answer to) Command 0x83 :
         *      tmp[0  1  2  3  4 ... ]
         * Example 5A A5 06 83 20 01 01 78 01 ……
         *          / /  |  |   \ /   |  \     \
         *        Header |  |    |    |   \_____\_ DATA (Words!)
         *     DatagramLen  /  VPAdr  |
         *           Command          DataLen (in Words)
         */
        if (command == VarAddr_R) {
          const uint16_t vp = tmp[0] << 8 | tmp[1];

          const uint8_t dlen = tmp[2] << 1; // Convert to Bytes. (Display works with words)
          recdat.addr = vp;
          recdat.len = tmp[2];
          for (uint16_t i = 0; i < dlen; i += 2) {
            recdat.data[i / 2] = tmp[3 + i];
            recdat.data[i / 2] = (recdat.data[i / 2] << 8 ) | tmp[4 + i];
          }

          rx_datagram_state = DGUS_IDLE;
          return 2;
        }

        // discard anything else
        rx_datagram_state = DGUS_IDLE;
    }
  return -1;
}

void RTS::sendData() {
  if (snddat.head[0] == FHONE && snddat.head[1] == FHTWO && snddat.len >= 3) {
    databuf[0] = snddat.head[0];
    databuf[1] = snddat.head[1];
    databuf[2] = snddat.len;
    databuf[3] = snddat.command;
    if (snddat.command == 0x80) { // to write data to the register
      databuf[4] = snddat.addr;
      for (int16_t i = 0; i < (snddat.len - 2); i++) databuf[5 + i] = snddat.data[i];
    }
    else if (snddat.len == 3 && (snddat.command == 0x81)) { // to read data from the register
      databuf[4] = snddat.addr;
      databuf[5] = snddat.bytelen;
    }
    else if (snddat.command == 0x82) { // to write data to the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      for (int16_t i = 0; i < (snddat.len - 3); i += 2) {
        databuf[6 + i] = snddat.data[i / 2] >> 8;
        databuf[7 + i] = snddat.data[i / 2] & 0xFF;
      }
    }
    else if (snddat.len == 4 && (snddat.command == 0x83)) { // to read data from the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      databuf[6] = snddat.bytelen;
    }
    for (int16_t i = 0; i < (snddat.len + 3); i++) {
      DWIN_SERIAL.write(databuf[i]);
      delay_us(1);
    }

    memset(&snddat, 0, sizeof(snddat));
    ZERO(databuf);
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }
}

void RTS::sendData(const String &s, uint32_t addr, uint8_t cmd/*=VarAddr_W*/) {
  if (s.length() < 1) return;
  sendData(s.c_str(), addr, cmd);
}

void RTS::sendData(const char *str, uint32_t addr, uint8_t cmd/*=VarAddr_W*/) {
  int16_t len = strlen(str);
  constexpr int16_t maxlen = DATA_BUF_SIZE - 6;
  if (len > 0) {
    if (len > maxlen) len = maxlen;
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3 + len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for (int16_t i = 0; i < len; i++) databuf[6 + i] = str[i];

    for (int16_t i = 0; i < (len + 6); i++) {
      DWIN_SERIAL.write(databuf[i]);
      delay_us(1);
    }
    ZERO(databuf);
  }
}

void RTS::sendData(const char c, const uint32_t addr, const uint8_t cmd/*=VarAddr_W*/) {
  snddat.command = cmd;
  snddat.addr    = addr;
  snddat.data[0] = uint32_t(uint16_t(c) << 8);
  snddat.len     = 5;
  sendData();
}

void RTS::sendData(const_float_t f, const uint32_t addr, const uint8_t cmd/*=VarAddr_W*/) {
  int16_t n = f;
  if (cmd == VarAddr_W) {
    snddat.data[0] = n;
    snddat.len = 5;
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

void RTS::sendData(const int n, const uint32_t addr, const uint8_t cmd/*=VarAddr_W*/) {
  if (cmd == VarAddr_W) {
    if ((unsigned int)n > 0xFFFF) {
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

void RTS::sendData(const unsigned long n, uint32_t addr, uint8_t cmd/*=VarAddr_W*/) {
  if (cmd == VarAddr_W) {
    if (n > 0xFFFF) {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      //snddat.data[0] = n >> 24;
      //snddat.data[1] = n >> 16;
      //snddat.data[2] = n >> 8;
      //snddat.data[3] = n;
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

void RTS::handleData() {
  int16_t Checkkey = -1;
  if (waitway > 0) { // for waiting
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  for (int16_t i = 0; Addrbuf[i] != 0; i++) {
    if (recdat.addr == Addrbuf[i]) {
      if (Addrbuf[i] == NzBdSet || Addrbuf[i] == NozzlePreheat || Addrbuf[i] == BedPreheat || Addrbuf[i] == Flowrate)
        Checkkey = ManualSetTemp;
      else if (WITHIN(Addrbuf[i], Stopprint, Resumeprint))
        Checkkey = PrintChoice;
      else if (WITHIN(Addrbuf[i], AutoZero, DisplayZaxis))
        Checkkey = XYZEaxis;
      else if (WITHIN(Addrbuf[i], FilamentUnit1, FilamentUnit2))
        Checkkey = Filament;
      else
        Checkkey = i;
      break;
    }
  }

  switch (recdat.addr) {
    case Flowrate:
    #if ENABLED(EDITABLE_STEPS_PER_UNIT)
      case StepMM_X ... StepMM_E:
    #endif
    case ProbeOffset_X ... ProbeOffset_Y:
    case HotendPID_AutoTmp ... BedPID_AutoTmp:
    case HotendPID_P ... HotendPID_D:
    case BedPID_P ... BedPID_D:
    #if ENABLED(DUAL_X_CARRIAGE)
      case T2Offset_X ... T2Offset_Z:
      #if ENABLED(EDITABLE_STEPS_PER_UNIT)
        case T2StepMM_E:
      #endif
    #endif
    case Accel_X ... Accel_E:
    case Feed_X ... Feed_E:
    case Jerk_X ... Jerk_E:
    case RunoutToggle:
    case PowerLossToggle:
    case FanKeyIcon:
    case LedToggle:
    case e2Preheat:                Checkkey = ManualSetTemp; break;
    case ProbeOffset_Z:            Checkkey = Zoffset_Value; break;
    case VolumeDisplay:            Checkkey = VolumeDisplay; break;
    case DisplayBrightness:        Checkkey = DisplayBrightness; break;
    case DisplayStandbyBrightness: Checkkey = DisplayStandbyBrightness; break;
    case DisplayStandbySeconds:    Checkkey = DisplayStandbySeconds; break;
    default:
      if (WITHIN(recdat.addr, AutolevelVal, AutolevelVal + 2 * (5 * 5 - 1))) // Assuming 5x5 mesh or smaller
        Checkkey = AutolevelVal;
      else if (WITHIN(recdat.addr, SDFILE_ADDR, SDFILE_ADDR + 10 * (FileNum + 1)))
        Checkkey = Filename;
      break;
  }

  if (Checkkey < 0) {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }

  #if ENABLED(LCD_BED_TRAMMING)
    constexpr float lfrb[4] = BED_TRAMMING_INSET_LFRB;
  #endif

  switch (Checkkey) {

    case Printfile: {
      if (recdat.data[0] == 1) { // card
        show_status = false;
        filenavigator.getFiles(0);
        fileIndex = 0;
        recordcount = 0;
        sendData(ExchangePageBase + 46, ExchangepageAddr);
      }
      else if (recdat.data[0] == 2) { // return after printing result.
        show_status = true;
        tpShowStatus = false;
        stopPrint();
        injectCommands(F("M84"));
        sendData(11, FilenameIcon);
        sendData(0, PrintscheduleIcon);
        sendData(0, PrintscheduleIcon + 1);
        sendData(0, Percentage);
        delay_ms(2);
        sendData(0, Timehour);
        sendData(0, Timemin);

        sendData(ExchangePageBase + 45, ExchangepageAddr); // exchange to 45 page
      }
      else if (recdat.data[0] == 3) { // Temperature control
        show_status = true;
        tpShowStatus = false;
        if (getTargetFan_percent((fan_t)getActiveTool()) == 0)
          sendData(ExchangePageBase + 58, ExchangepageAddr); // exchange to 58 page, the fans off
        else
          sendData(ExchangePageBase + 57, ExchangepageAddr); // exchange to 57 page, the fans on
      }
      else if (recdat.data[0] == 4) { // Settings
        show_status = false;
      }
    } break;

    case Adjust: {
      if (recdat.data[0] == 1) {
        show_status = false;
      }
      else if (recdat.data[0] == 2) {
        show_status = true;
        if (printerStatusKey[1] == 3) // during heating
          sendData(ExchangePageBase + 53, ExchangepageAddr);
        else if (printerStatusKey[1] == 4)
          sendData(ExchangePageBase + 54, ExchangepageAddr);
        else
          sendData(ExchangePageBase + 53, ExchangepageAddr);
      }
      else if (recdat.data[0] == 3)
        setTargetFan_percent(getTargetFan_percent((fan_t)getActiveTool()) != 0 ? 100 : 0, FAN0);

    } break;

    case Feedrate: {
      setFeedrate_percent(recdat.data[0]);
    } break;

    case PrintChoice: {
      if (recdat.addr == Stopprint) {
        if (recdat.data[0] == 240) { // no
          sendData(ExchangePageBase + 53, ExchangepageAddr);
        }
        else {
          sendData(ExchangePageBase + 45, ExchangepageAddr);
          sendData(0, Timehour);
          sendData(0, Timemin);
          stopPrint();
        }
      }
      else if (recdat.addr == Pauseprint) {
        if (recdat.data[0] != 0xF1)
          break;

        sendData(ExchangePageBase + 54, ExchangepageAddr);
        pausePrint();
      }
      else if (recdat.addr == Resumeprint && recdat.data[0] == 1) {
        resumePrint();

        printerStatusKey[1] = 0;
        show_status = true;

        sendData(ExchangePageBase + 53, ExchangepageAddr);
      }
      if (recdat.addr == Resumeprint && recdat.data[0] == 2) { // warming
        resumePrint();
        nozzleTempStatus[2] = 1;
        printerStatusKey[1] = 0;
        show_status = true;
        sendData(ExchangePageBase + 82, ExchangepageAddr);
      }
    } break;

    #if HAS_BED_PROBE

      case Zoffset: {
        float tmp_zprobe_offset;
        if (recdat.data[0] >= 32768)
          tmp_zprobe_offset = (float(recdat.data[0]) - 65536) / 100;
        else
          tmp_zprobe_offset = float(recdat.data[0]) / 100;
        if (WITHIN(tmp_zprobe_offset, PROBE_OFFSET_ZMIN, PROBE_OFFSET_ZMAX)) {
          int16_t tmpSteps = mmToWholeSteps(getZOffset_mm() - tmp_zprobe_offset, axis_t(Z));
          if (tmpSteps == 0) tmpSteps = getZOffset_mm() < tmp_zprobe_offset ? 1 : -1;
          smartAdjustAxis_steps(-tmpSteps, axis_t(Z), false);
          char zOffs[20], tmp1[11];
          sprintf_P(zOffs, PSTR("Z Offset : %s"), dtostrf(getZOffset_mm(), 1, 3, tmp1));
          onStatusChanged(zOffs);
        }
        else {
          onStatusChanged(F("Requested Offset Beyond Limits"));
        }

        sendData(getZOffset_mm() * 100, ProbeOffset_Z);
      } break;

    #endif // HAS_BED_PROBE

    case TempControl: {
      if (recdat.data[0] == 0) {
        show_status = true;
        tpShowStatus = false;
      }
      else if (recdat.data[0] == 1) {
        if (getTargetFan_percent((fan_t)getActiveTool()) == 0)
          sendData(ExchangePageBase + 60, ExchangepageAddr); // exchange to 60 page, the fans off
        else
          sendData(ExchangePageBase + 59, ExchangepageAddr); // exchange to 59 page, the fans on
      }
      else if (recdat.data[0] == 2) {
        show_status = true;
      }
      else if (recdat.data[0] == 3) {
        if (getTargetFan_percent((fan_t)getActiveTool()) == 0) { // turn on the fan
          setTargetFan_percent(100, FAN0);
          sendData(ExchangePageBase + 57, ExchangepageAddr); // exchange to 57 page, the fans on
        }
        else { // turn off the fan
          setTargetFan_percent(0, FAN0);
          sendData(ExchangePageBase + 58, ExchangepageAddr); // exchange to 58 page, the fans on
        }
      }
      else if (recdat.data[0] == 5) { // PLA mode
        setTargetTemp_celsius(PREHEAT_1_TEMP_HOTEND, getActiveTool());
        setTargetTemp_celsius(PREHEAT_1_TEMP_BED, BED);
        sendData(PREHEAT_1_TEMP_HOTEND, NozzlePreheat);
        sendData(PREHEAT_1_TEMP_BED, BedPreheat);
      }
      else if (recdat.data[0] == 6) { // ABS mode
        setTargetTemp_celsius(PREHEAT_2_TEMP_HOTEND, getActiveTool());
        setTargetTemp_celsius(PREHEAT_2_TEMP_BED, BED);
        sendData(PREHEAT_2_TEMP_HOTEND, NozzlePreheat);
        sendData(PREHEAT_2_TEMP_BED, BedPreheat);
      }
      else if (recdat.data[0] == 0xF1) {
        //show_status = true;
        #if FAN_COUNT > 0
          for (uint8_t i = 0; i < FAN_COUNT; i++) setTargetFan_percent(0, (fan_t)i);
        #endif
        setTargetTemp_celsius(0.0, H0);
        TERN_(HAS_MULTI_HOTEND, setTargetTemp_celsius(0.0, H1));
        setTargetTemp_celsius(0.0, BED);
        sendData(0, NozzlePreheat); delay_ms(1);
        sendData(0, BedPreheat);    delay_ms(1);
        sendData(ExchangePageBase + 57, ExchangepageAddr);
        printerStatusKey[1] = 2;
      }
    } break;

    case ManualSetTemp: {
      if (recdat.addr == NzBdSet) {
        if (recdat.data[0] == 0) {
          if (getTargetFan_percent((fan_t)getActiveTool()) == 0)
            sendData(ExchangePageBase + 58, ExchangepageAddr); // exchange to 58 page, the fans off
          else
            sendData(ExchangePageBase + 57, ExchangepageAddr); // exchange to 57 page, the fans on
        }
        else if (recdat.data[0] == 1) {
          setTargetTemp_celsius(0.0, getActiveTool());
          sendData(0, NozzlePreheat);
        }
        else if (recdat.data[0] == 2) {
          setTargetTemp_celsius(0.0, BED);
          sendData(0, BedPreheat);
        }
      }
      else if (recdat.addr == NozzlePreheat) {
        setTargetTemp_celsius(float(recdat.data[0]), H0);
      }
      #if HAS_MULTI_HOTEND
        else if (recdat.addr == e2Preheat) {
          setTargetTemp_celsius(float(recdat.data[0]), H1);
        }
      #endif
      else if (recdat.addr == BedPreheat) {
        setTargetTemp_celsius(float(recdat.data[0]), BED);
      }
      else if (recdat.addr == Flowrate) {
        setFlow_percent(int16_t(recdat.data[0]), getActiveTool());
      }

      #if HAS_PID_HEATING
        else if (recdat.addr == HotendPID_AutoTmp) {
          pid_hotendAutoTemp = uint16_t(recdat.data[0]);
        }
        else if (recdat.addr == BedPID_AutoTmp) {
          pid_bedAutoTemp = uint16_t(recdat.data[0]);
        }
      #endif

      else if (recdat.addr == Accel_X) {
        setAxisMaxAcceleration_mm_s2(uint16_t(recdat.data[0]) * 100, X);
      }
      else if (recdat.addr == Accel_Y) {
        setAxisMaxAcceleration_mm_s2(uint16_t(recdat.data[0]) * 100, Y);
      }
      else if (recdat.addr == Accel_Z) {
        setAxisMaxAcceleration_mm_s2(uint16_t(recdat.data[0]) * 10, Z);
      }
      else if (recdat.addr == Accel_E) {
        setAxisMaxAcceleration_mm_s2(uint16_t(recdat.data[0]), E0);
        setAxisMaxAcceleration_mm_s2(uint16_t(recdat.data[0]), E1);
      }

      else if (recdat.addr == Feed_X) {
        setAxisMaxFeedrate_mm_s(uint16_t(recdat.data[0]), X);
      }
      else if (recdat.addr == Feed_Y) {
        setAxisMaxFeedrate_mm_s(uint16_t(recdat.data[0]), Y);
      }
      else if (recdat.addr == Feed_Z) {
        setAxisMaxFeedrate_mm_s(uint16_t(recdat.data[0]), Z);
      }
      else if (recdat.addr == Feed_E) {
        setAxisMaxFeedrate_mm_s(uint16_t(recdat.data[0]), E0);
        setAxisMaxFeedrate_mm_s(uint16_t(recdat.data[0]), E1);
      }
      else if (recdat.addr == FanKeyIcon) {
        setTargetFan_percent(uint16_t(recdat.data[0]), (fan_t)getActiveTool());
      }
      else {
        float tmp_float_handling = float(recdat.data[0]);
        if (tmp_float_handling >= 32768) tmp_float_handling -= 65536;
        tmp_float_handling /= 100;

        if (false) {}

        #if ENABLED(EDITABLE_STEPS_PER_UNIT)
          else if (recdat.addr == StepMM_X) {
            setAxisSteps_per_mm(tmp_float_handling * 10, X);
          }
          else if (recdat.addr == StepMM_Y) {
            setAxisSteps_per_mm(tmp_float_handling * 10, Y);
          }
          else if (recdat.addr == StepMM_Z) {
            setAxisSteps_per_mm(tmp_float_handling * 10, Z);
          }
          else if (recdat.addr == StepMM_E) {
            setAxisSteps_per_mm(tmp_float_handling * 10, E0);
            #if DISABLED(DUAL_X_CARRIAGE)
              setAxisSteps_per_mm(tmp_float_handling * 10, E1);
            #endif
          }
        #endif // EDITABLE_STEPS_PER_UNIT

        #if ENABLED(DUAL_X_CARRIAGE)
          #if ENABLED(EDITABLE_STEPS_PER_UNIT)
            else if (recdat.addr == T2StepMM_E) {
              setAxisSteps_per_mm(tmp_float_handling * 10, E1);
            }
          #endif
          else if (recdat.addr == T2Offset_X) {
            setNozzleOffset_mm(tmp_float_handling * 10, X, E1);
          }
          else if (recdat.addr == T2Offset_Y) {
            setNozzleOffset_mm(tmp_float_handling * 10, Y, E1);
          }
          else if (recdat.addr == T2Offset_Z) {
            setNozzleOffset_mm(tmp_float_handling * 10, Z, E1);
          }
        #endif
        #if HAS_BED_PROBE
          else if (recdat.addr == ProbeOffset_X) {
            setProbeOffset_mm(tmp_float_handling, X);
          }
          else if (recdat.addr == ProbeOffset_Y) {
            setProbeOffset_mm(tmp_float_handling, Y);
          }
          else if (recdat.addr == ProbeOffset_Z) {
            setProbeOffset_mm(tmp_float_handling, Z);
          }
        #endif

        #if ENABLED(CLASSIC_JERK)
          else if (recdat.addr == Jerk_X) {
            setAxisMaxJerk_mm_s(tmp_float_handling, X);
          }
          else if (recdat.addr == Jerk_Y) {
            setAxisMaxJerk_mm_s(tmp_float_handling, Y);
          }
          else if (recdat.addr == Jerk_Z) {
            setAxisMaxJerk_mm_s(tmp_float_handling, Z);
          }
          else if (recdat.addr == Jerk_E) {
            setAxisMaxJerk_mm_s(tmp_float_handling, E0);
            setAxisMaxJerk_mm_s(tmp_float_handling, E1);
          }
        #endif

        #if HAS_FILAMENT_SENSOR
          else if (recdat.addr == RunoutToggle) {
            setFilamentRunoutEnabled(!getFilamentRunoutEnabled());
          }
        #endif

        #if ENABLED(POWER_LOSS_RECOVERY)
          else if (recdat.addr == PowerLossToggle) {
            setPowerLossRecoveryEnabled(!getPowerLossRecoveryEnabled());
          }
        #endif

        #if ENABLED(CASE_LIGHT_ENABLE)
          else if (recdat.addr == LedToggle) {
            setCaseLightState(!getCaseLightState());
          }
        #endif

        #if HAS_PID_HEATING
          else if (recdat.addr == HotendPID_P) {
            setPID(tmp_float_handling * 10, getPID_Ki(getActiveTool()), getPID_Kd(getActiveTool()), getActiveTool());
          }
          else if (recdat.addr == HotendPID_I) {
            setPID(getPID_Kp(getActiveTool()), tmp_float_handling * 10, getPID_Kd(getActiveTool()), getActiveTool());
          }
          else if (recdat.addr == HotendPID_D) {
            setPID(getPID_Kp(getActiveTool()), getPID_Ki(getActiveTool()), tmp_float_handling * 10, getActiveTool());
          }
          #if ENABLED(PIDTEMPBED)
            else if (recdat.addr == BedPID_P) {
              setBedPID(tmp_float_handling * 10, getBedPID_Ki(), getBedPID_Kd());
            }
            else if (recdat.addr == BedPID_I) {
              setBedPID(getBedPID_Kp(), tmp_float_handling * 10, getBedPID_Kd());
            }
            else if (recdat.addr == BedPID_D) {
              setBedPID(getBedPID_Kp(), getBedPID_Ki(), tmp_float_handling * 10);
            }
          #endif
        #endif // HAS_PID_HEATING
      }
    } break;

    case Setting: {
      if (recdat.data[0] == 0) { // return to main page
        show_status = true;
        tpShowStatus = false;
      }
      else if (recdat.data[0] == 1) { // Bed Autoleveling
        #if HAS_MESH
          sendData(getLevelingActive() ? 3 : 2, AutoLevelIcon);

          if (ExtUI::getLevelingIsValid()) {
            uint8_t abl_probe_index = 0;
            for (uint8_t outer = 0; outer < GRID_MAX_POINTS_Y; outer++)
              for (uint8_t inner = 0; inner < GRID_MAX_POINTS_X; inner++) {
                const bool zig = outer & 1;
                const xy_uint8_t point = { uint8_t(zig ? (GRID_MAX_POINTS_X - 1) - inner : inner), outer };
                sendData(ExtUI::getMeshPoint(point) * 1000, AutolevelVal + abl_probe_index * 2);
                ++abl_probe_index;
              }
          }
        #endif
        sendData(10, FilenameIcon); // Motor Icon
        injectCommands(isPositionKnown() ? F("G1F1000Z0.0") : F("G28\nG1F1000Z0.0"));
        waitway = 2;
        sendData(ExchangePageBase + 64, ExchangepageAddr);
      }
      else if (recdat.data[0] == 2) { // Exchange filament
        show_status = true;
        tpShowStatus = false;
        ZERO(changeMaterialBuf);
        changeMaterialBuf[1] = changeMaterialBuf[0] = 10;
        sendData(10 * changeMaterialBuf[0], FilamentUnit1); // It's changeMaterialBuf for show, instead of current_position.e in them.
        sendData(10 * changeMaterialBuf[1], FilamentUnit2);
        sendData(getActualTemp_celsius(H0), NozzleTemp);
        sendData(getTargetTemp_celsius(H0), NozzlePreheat);
        sendData(TERN0(HAS_MULTI_HOTEND, getActualTemp_celsius(H1)), e2Temp);
        sendData(TERN0(HAS_MULTI_HOTEND, getTargetTemp_celsius(H1)), e2Preheat);
        delay_ms(2);
        sendData(ExchangePageBase + 65, ExchangepageAddr);
      }
      else if (recdat.data[0] == 3) { // Move
        axisPageNum = 0;
        sendData(ExchangePageBase + 71, ExchangepageAddr);
      }
      else if (recdat.data[0] == 4) { // Language
        // Just loads language screen, now used for tools
      }
      else if (recdat.data[0] == 5) { // Printer Information
        sendData(WEBSITE_URL, CorpWebsite);
      }
      else if (recdat.data[0] == 6) { // Diabalestepper
        injectCommands(F("M84"));
        sendData(11, FilenameIcon);
      }
    } break;

    case ReturnBack: {
      if (recdat.data[0] == 1) { // return to the tool page
        show_status = false;
        sendData(ExchangePageBase + 63, ExchangepageAddr);
      }
      else if (recdat.data[0] == 2) // return to the Level mode page
        sendData(ExchangePageBase + 64, ExchangepageAddr);
    } break;

    case Bedlevel: {
      switch (recdat.data[0]) {
        case 1:   // Z-axis to home
          // Disallow Z homing if X or Y are unknown
          injectCommands(isAxisPositionKnown(axis_t(X)) && isAxisPositionKnown(axis_t(Y)) ? F("G28Z\nG1F1500Z0.0") : F("G28\nG1F1500Z0.0"));
          sendData(getZOffset_mm() * 100, ProbeOffset_Z);
          break;

        #if HAS_BED_PROBE
          case 2:   // Z-axis to Up
            if (WITHIN((getZOffset_mm() + 0.1), PROBE_OFFSET_ZMIN, PROBE_OFFSET_ZMAX)) {
              smartAdjustAxis_steps(getAxisSteps_per_mm(Z) / 10, axis_t(Z), false);
              //setZOffset_mm(getZOffset_mm() + 0.1);
              sendData(getZOffset_mm() * 100, ProbeOffset_Z);
              onStatusChanged(MString<20>(GET_TEXT_F(MSG_UBL_Z_OFFSET), p_float_t(getZOffset_mm(), 3)));
            }
            break;

          case 3:   // Z-axis to Down
            if (WITHIN((getZOffset_mm() - 0.1), PROBE_OFFSET_ZMIN, PROBE_OFFSET_ZMAX)) {
              smartAdjustAxis_steps(-getAxisSteps_per_mm(Z) / 10, axis_t(Z), false);
              //babystepAxis_steps(int16_t(-getAxisSteps_per_mm(Z)) / 10, axis_t(Z));
              //setZOffset_mm(getZOffset_mm() - 0.1);
              sendData(getZOffset_mm() * 100, ProbeOffset_Z);
              onStatusChanged(MString<20>(GET_TEXT_F(MSG_UBL_Z_OFFSET), p_float_t(getZOffset_mm(), 3)));
            }
            break;
        #endif // HAS_BED_PROBE

        case 4:   // Assistant Level
          TERN_(HAS_MESH, setLevelingActive(false));
          injectCommands(isPositionKnown() ? F("G1 F1000 Z0.0") : F("G28\nG1 F1000 Z0.0"));
          waitway = 2;
          sendData(ExchangePageBase + 84, ExchangepageAddr);
          break;

        case 5: { // AutoLevel "Measuring" Button
          #if ENABLED(MESH_BED_LEVELING)
            sendData(ExchangePageBase + 93, ExchangepageAddr);
          #else
            waitway = 3; // Only for prohibiting to receive message
            sendData(3, AutolevelIcon);
            uint8_t abl_probe_index = 0;
            while (abl_probe_index < 25) {
              sendData(0, AutolevelVal + abl_probe_index * 2);
              ++abl_probe_index;
            }
            sendData(ExchangePageBase + 64, ExchangepageAddr);
            injectCommands(F(MEASURING_GCODE));
          #endif
        } break;

        #if ENABLED(LCD_BED_TRAMMING)
          case 6:   // Bed Tramming,  Centre 1
            if (BED_TRAMMING_Z_HOP) setAxisPosition_mm(current_position.z + (BED_TRAMMING_Z_HOP), axis_t(Z));
            setAxisPosition_mm(X_CENTER, axis_t(X));
            setAxisPosition_mm(Y_CENTER, axis_t(Y));
            waitway = 6;
            break;

          case 7:   // Bed Tramming, Front Left 2
            if (BED_TRAMMING_Z_HOP) setAxisPosition_mm(current_position.z + (BED_TRAMMING_Z_HOP), axis_t(Z));
            setAxisPosition_mm(X_MIN_BED + lfrb[0], axis_t(X));
            setAxisPosition_mm(Y_MIN_BED + lfrb[1], axis_t(Y));
            waitway = 6;
            break;

          case 8:   // Bed Tramming, Front Right 3
            if (BED_TRAMMING_Z_HOP) setAxisPosition_mm(current_position.z + (BED_TRAMMING_Z_HOP), axis_t(Z));
            setAxisPosition_mm(X_MAX_BED - lfrb[2], axis_t(X));
            setAxisPosition_mm(Y_MIN_BED + lfrb[1], axis_t(Y));
            waitway = 6;
            break;

          case 9:   // Bed Tramming, Back Right 4
            if (BED_TRAMMING_Z_HOP) setAxisPosition_mm(current_position.z + (BED_TRAMMING_Z_HOP), axis_t(Z));
            setAxisPosition_mm(X_MAX_BED - lfrb[2], axis_t(X));
            setAxisPosition_mm(Y_MAX_BED - lfrb[3], axis_t(Y));
            waitway = 6;
            break;

          case 10:   // Bed Tramming, Back Left 5
            if (BED_TRAMMING_Z_HOP) setAxisPosition_mm(current_position.z + (BED_TRAMMING_Z_HOP), axis_t(Z));
            setAxisPosition_mm(X_MIN_BED + lfrb[0], axis_t(X));
            setAxisPosition_mm(Y_MAX_BED - lfrb[3], axis_t(Y));
            waitway = 6;
            break;

        #endif // LCD_BED_TRAMMING

        case 11: { // Autolevel switch
          #if HAS_MESH
            const bool gla = !getLevelingActive();
            setLevelingActive(gla);
            sendData(gla ? 3 : 2, AutoLevelIcon);
          #endif
          #if HAS_BED_PROBE
            sendData(getZOffset_mm() * 100, ProbeOffset_Z);
          #endif
        } break;

        #if ENABLED(G26_MESH_VALIDATION)
          case 12:
            injectCommands(F("G26R255"));
            onStatusChanged(F("Beginning G26.. Heating"));
            break;
        #endif
        #if ENABLED(MESH_BED_LEVELING)
          case 13:
            injectCommands(F("G29S1"));
            onStatusChanged(F("Begin Manual Mesh"));
            break;
          case 14:
            injectCommands(F("G29S2"));
            onStatusChanged(F("Moving to Next Mesh Point"));
            break;
        #endif
        case 15:
          injectCommands(F("M211S0\nG91\nG1Z-0.025\nG90\nM211S1"));
          onStatusChanged(F("Moved down 0.025"));
          break;
        case 16:
          injectCommands(F("M211S0\nG91\nG1Z0.025\nG90\nM211S1"));
          onStatusChanged(F("Moved up 0.025"));
          break;
        case 17:
          dwin_settings.display_volume = 0;
          dwin_settings.display_sound = false;
          setTouchScreenConfiguration();
          break;
        case 18:
          dwin_settings.display_volume = 255;
          dwin_settings.display_sound = true;
          setTouchScreenConfiguration();
          break;
        case 19:
          dwin_settings.screen_brightness = 10;
          setTouchScreenConfiguration();
          break;
        case 20:
          dwin_settings.screen_brightness = 100;
          setTouchScreenConfiguration();
          break;
        case 21:
          FLIP(dwin_settings.display_standby);
          setTouchScreenConfiguration();
          break;
        case 22:
          dwin_settings.screen_rotation = 10 - dwin_settings.screen_rotation == 10 ? 0 : 10;
          setTouchScreenConfiguration();
          break;
        case 23:  // Set IDEX Autopark
          injectCommands(F("M605S1\nG28X\nG1X0"));
          break;
        case 24:  // Set IDEX Duplication
          injectCommands(F("M605S1\nT0\nG28\nM605S2\nG28X\nG1X0"));
          break;
        case 25:  // Set IDEX Mirrored Duplication
          injectCommands(F("M605S1\nT0\nG28\nM605S2\nG28X\nG1X0\nM605S3"));
          break;
        case 26:  // Set IDEX Full Control
          injectCommands(F("M605S0\nG28X"));
          break;
        case 27:  // Change Tool
          setActiveTool(getActiveTool() == E0 ? E1 : E0, !isAxisPositionKnown(X));
          break;

        default: break;
      }

      sendData(10, FilenameIcon);
    } break;

    case XYZEaxis: {
      axis_t axis = X;
      float min = 0.0f, max = 0.0f;
      waitway = 4;
      if (recdat.addr == DisplayXaxis) {
        axis = X; min = X_MIN_POS; max = X_MAX_POS;
      }
      else if (recdat.addr == DisplayYaxis) {
        axis = Y; min = Y_MIN_POS; max = Y_MAX_POS;
      }
      else if (recdat.addr == DisplayZaxis) {
        axis = Z; min = Z_MIN_POS; max = Z_MAX_POS;
      }
      else if (recdat.addr == AutoZero) {
        if (recdat.data[0] == 3) { // autohome
          waitway = 4;
          injectCommands(F("G28\nG1 F1000 Z10"));
          show_status = autoHomeKey = true;
          autoHomeIconNum = 0;
          sendData(10, FilenameIcon);
        }
        else {
          axisPageNum = recdat.data[0];
          waitway = 0;
        }
      } break;

      float targetPos = float(recdat.data[0]) / 10;
      LIMIT(targetPos, min, max);
      setAxisPosition_mm(targetPos, axis);
      waitway = 0;
      sendData(10, FilenameIcon);
    } break;

    case Filament: {
      uint16_t IconTemp;
      if (recdat.addr == Exchfilament) {
        if (getActualTemp_celsius(getActiveTool()) < EXTRUDE_MINTEMP && recdat.data[0] < 5) {
          sendData(int16_t(EXTRUDE_MINTEMP), 0x1020);
          delay_ms(5);
          sendData(ExchangePageBase + 66, ExchangepageAddr);
          break;
        }

        switch (recdat.data[0]) {
          case 1:   // Unload filament1
            setAxisPosition_mm(getAxisPosition_mm(E0) - changeMaterialBuf[0], E0);
            break;

          case 2:   // Load filament1
            setAxisPosition_mm(getAxisPosition_mm(E0) + changeMaterialBuf[0], E0);
            break;

          case 3:   // Unload filament2
            setAxisPosition_mm(getAxisPosition_mm(E1) - changeMaterialBuf[1], E1);
            break;

          case 4:   // Load filament2
            setAxisPosition_mm(getAxisPosition_mm(E1) + changeMaterialBuf[1], E1);
            break;

          case 5:   // sure to heat
            nozzleTempStatus[0] = 1;

            setTargetTemp_celsius((PREHEAT_1_TEMP_HOTEND + 10), getActiveTool());
            IconTemp = getActualTemp_celsius(getActiveTool()) * 100 / getTargetTemp_celsius(getActiveTool());
            NOMORE(IconTemp, 100U);
            sendData(IconTemp, HeatPercentIcon);

            sendData(getActualTemp_celsius(H0), NozzleTemp);
            sendData(getTargetTemp_celsius(H0), NozzlePreheat);
            sendData(TERN0(HAS_MULTI_HOTEND, getActualTemp_celsius(H1)), e2Temp);
            sendData(TERN0(HAS_MULTI_HOTEND, getTargetTemp_celsius(H1)), e2Preheat);
            delay_ms(5);
            sendData(ExchangePageBase + 68, ExchangepageAddr);
            break;

          case 6:   // cancel to heat
            sendData(ExchangePageBase + 65, ExchangepageAddr);
            break;

          case 0xF1:   // Sure to cancel heating
            nozzleTempStatus[0] = 0;
            delay_ms(1);
            sendData(ExchangePageBase + 65, ExchangepageAddr);
            break;

          case 0xF0: // not to cancel heating
            break;
        }
        sendData(10 * changeMaterialBuf[0], FilamentUnit1); // It's changeMaterialBuf for show, instead of current_position.e in them.
        sendData(10 * changeMaterialBuf[1], FilamentUnit2);
      }
      else if (recdat.addr == FilamentUnit1) {
        changeMaterialBuf[0] = float(recdat.data[0]) / 10;
      }
      else if (recdat.addr == FilamentUnit2) {
        changeMaterialBuf[1] = float(recdat.data[0]) / 10;
      }
    } break;

    case LanguageChoice: {

      //if (recdat.data[0] == 1) settings.save(); else injectCommands(F("M300"));

      // may at some point use language change screens to save eeprom explicitly
      switch (recdat.data[0]) {
        case 0:
          injectCommands(F("M500"));
          break;

        case 1:
          sendData(ExchangePageBase + 94, ExchangepageAddr);
          break;

        #if ENABLED(PIDTEMP)
          case 2:
            onStatusChanged(F("Hotend PID Started"));
            startPIDTune(static_cast<celsius_t>(pid_hotendAutoTemp), getActiveTool());
            break;
        #endif

        case 3:
          injectCommands(F("M502\nM500"));
          break;

        case 4:
          injectCommands(F("M999\nM280P0S160"));
          break;

        #if ENABLED(PIDTEMPBED)
          case 5:
            onStatusChanged(F("Bed PID Started"));
            startBedPIDTune(static_cast<celsius_t>(pid_bedAutoTemp));
            break;
        #endif

        case 6:
          injectCommands(F("M500"));
          break;

        default: break;
      }
    } break;

    case No_Filament: {
      if (recdat.data[0] == 1) { // Filament is out, resume / resume selected on screen
        if (ExtUI::pauseModeStatus != PAUSE_MESSAGE_PURGE && ExtUI::pauseModeStatus != PAUSE_MESSAGE_OPTION) {
          // setPauseMenuResponse(PAUSE_RESPONSE_RESUME_PRINT);
          setUserConfirmed();
          // printerStatusKey[1] = 3;
          // pause_resume_selected = true;
        }
        else {
          #if HAS_FILAMENT_SENSOR
            bool runouton = false;
            if (getFilamentRunoutState()) {
              #if NUM_RUNOUT_SENSORS > 1
                if ((runouton = getFilamentRunoutEnabled(getActiveTool())))
                  ExtUI::setFilamentRunoutEnabled(false, getActiveTool());
              #else
                if ((runouton = getFilamentRunoutEnabled()))
                  ExtUI::setFilamentRunoutEnabled(false);
              #endif
            }
          #else
            constexpr bool runouton = false;
          #endif
          if (!runouton) {
            setPauseMenuResponse(PAUSE_RESPONSE_RESUME_PRINT);
            setUserConfirmed();
            printerStatusKey[1] = 3;
            pause_resume_selected = true;
          }
        }
      }
      else if (recdat.data[0] == 0) { // Filamet is out, Cancel Selected
        if (ExtUI::pauseModeStatus == PAUSE_MESSAGE_PURGE || ExtUI::pauseModeStatus == PAUSE_MESSAGE_OPTION) {
          setPauseMenuResponse(PAUSE_RESPONSE_EXTRUDE_MORE);
          setUserConfirmed();
        }
      }
    } break;

    #if ENABLED(POWER_LOSS_RECOVERY)
      case PwrOffNoF:
        if (recdat.data[0] == 1) // Yes: continue to print the 3Dmode during power-off.
          injectCommands(F("M1000"));
        else if (recdat.data[0] == 2) // No
          injectCommands(F("M1000C"));
        break;
    #endif

    case Volume: {
      dwin_settings.display_volume = constrain(recdat.data[0], 0, 255);
      if (dwin_settings.display_volume == 0) {
        sendData(0, VolumeIcon);
        sendData(9, SoundIcon);
      }
      else {
        sendData((dwin_settings.display_volume + 1) / 32 - 1, VolumeIcon);
        sendData(8, SoundIcon);
      }
      sendData(dwin_settings.display_volume << 8, SoundAddr + 1);
    } break;

    case Filename: {
      if (isMediaMounted() && recdat.addr == FilenameChs) {

        recordcount = recdat.data[0] - 1;
        if (filenavigator.currentindex == 0 && filenavigator.folderdepth > 0 && (fileIndex + recordcount) == 0) {
          filenavigator.upDIR();
          filenavigator.getFiles(0);
          fileIndex = 0;
          return;
        }

        if (filenavigator.currentindex == 0 && filenavigator.folderdepth > 0)
          recordcount = recordcount - 1; // account for return dir link in file index

        for (int16_t j = 1; j <= 4; j++) { // Clear filename BG Color and Frame
          sendData(0xFFFFUL, FilenameNature + j * 16); // white
          sendData(10, FilenameIcon1 + j);            // clean
        }
        for (int16_t j = 0; j < 10; j++) // clear current filename
          sendData(0, Choosefilename + j);

        if (filenavigator.getIndexisDir(fileIndex + recordcount)) {
          filenavigator.changeDIR((char *)filenavigator.getIndexName(fileIndex + recordcount));
          filenavigator.getFiles(0);
          fileIndex = 0;
          return;
        }
        else {
          sendData(filenavigator.getIndexName(fileIndex + recordcount), Choosefilename);
          sendData(0x87F0UL, FilenameNature + recdat.data[0] * 16); // Change BG of selected line to Light Green
          sendData(6, FilenameIcon1 + recdat.data[0]);             // show frame
        }
      }
      else if (recdat.addr == FilenamePlay) {
        if (recdat.data[0] == 1 && isMediaMounted()) { // for sure
          printFile(filenavigator.getIndexName(fileIndex + recordcount));

          for (int16_t j = 0; j < 10; j++) // clean screen.
            sendData(0, Printfilename + j);

          sendData(filenavigator.getIndexName(fileIndex + recordcount), Printfilename);

          delay_ms(4);

          sendData(ExchangePageBase + 53, ExchangepageAddr);

          tpShowStatus        = show_status = true;
          printerStatusKey[0] = 1;
          printerStatusKey[1] = 3;
          fileIndex           = 0;
          recordcount         = 0;
        }
        else if (recdat.data[0] == 2) { // Page Down
          if ((fileIndex + DISPLAY_FILES) < (filenavigator.maxFiles() + (filenavigator.folderdepth != 0))) {
            fileIndex = fileIndex + DISPLAY_FILES;
            // if (filenavigator.folderdepth!=0 && fileIndex!=0) //Shift to acknowledge Return DIR button on first page
            //  filenavigator.getFiles(fileIndex-1);
            // else
            filenavigator.getFiles(fileIndex);
            // filenavigator.getFiles(filenavigator.currentindex+1);
          }
        }
        else if (recdat.data[0] == 3) { // Page Up
          if (fileIndex >= DISPLAY_FILES) {
            fileIndex = fileIndex - DISPLAY_FILES;
            // if (filenavigator.folderdepth!=0 && fileIndex!=0) //Shift to acknowledge Return DIR button on first page
            // filenavigator.getFiles(filenavigator.currentindex-DISPLAY_FILES);
            // else
            filenavigator.getFiles(fileIndex);
          }
        }
        else if (recdat.data[0] == 4) { // Page Up
          injectCommands(F("M22\nM21"));
        }
        else if (recdat.data[0] == 0) { // return to main page
          show_status = true;
          tpShowStatus = false;
        }
      }
    } break;

    case VolumeDisplay: {
      if (recdat.data[0] == 0) {
        dwin_settings.display_volume = 0;
        dwin_settings.display_sound = false;
      }
      else if (recdat.data[0] > 100) {
        dwin_settings.display_volume = 255;
        dwin_settings.display_sound = true;
      }
      else {
        dwin_settings.display_volume = (uint8_t)map(constrain(recdat.data[0], 0, 100), 0, 100, 0, 255);
        dwin_settings.display_sound = true;
      }
      setTouchScreenConfiguration();
    } break;

    case DisplayBrightness: {
      if (recdat.data[0] < 10)
        dwin_settings.screen_brightness = 10;
      else if (recdat.data[0] > 100)
        dwin_settings.screen_brightness = 100;
      else
        dwin_settings.screen_brightness = (uint8_t)recdat.data[0];
      setTouchScreenConfiguration();
    } break;

    case DisplayStandbyBrightness: {
      if (recdat.data[0] < 10)
        dwin_settings.standby_brightness = 10;
      else if (recdat.data[0] > 100)
        dwin_settings.standby_brightness = 100;
      else
        dwin_settings.standby_brightness = (uint8_t)recdat.data[0];
      setTouchScreenConfiguration();
    } break;

    case DisplayStandbySeconds: {
      if (recdat.data[0] < 5)
        dwin_settings.standby_time_seconds = 5;
      else if (recdat.data[0] > 100)
        dwin_settings.standby_time_seconds = 100;
      else
        dwin_settings.standby_time_seconds = (uint8_t)recdat.data[0];
      setTouchScreenConfiguration();
    } break;

    case AutolevelVal: {
      uint8_t meshPoint = (recdat.addr - AutolevelVal) / 2,
              yPnt = meshPoint / (GRID_MAX_POINTS_X),
              xPnt = meshPoint - yPnt * (GRID_MAX_POINTS_X);
      if (yPnt % 2 != 0) xPnt = (GRID_MAX_POINTS_X) - 1 - xPnt; // zag row

      float meshVal = float(recdat.data[0] - (recdat.data[0] >= 32768 ? 65536 : 0)) / 1000;

      #ifndef Z_PROBE_LOW_POINT
        #define Z_PROBE_LOW_POINT -2
      #endif
      LIMIT(meshVal, Z_PROBE_LOW_POINT, Z_CLEARANCE_BETWEEN_PROBES);
      xy_uint8_t point = { xPnt, yPnt };
      setMeshPoint(point, meshVal);
      sendData(meshVal * 1000, recdat.addr);
    } break;

    default: break;
  }

  memset(&recdat, 0, sizeof(recdat));
  recdat.head[0] = FHONE;
  recdat.head[1] = FHTWO;
}

void RTS::writeVariable(const uint16_t adr, const void * const values, uint8_t valueslen, const bool isstr/*=false*/, const char fillChar/*=' '*/) {
  const char* myvalues = static_cast<const char*>(values);
  bool strend = !myvalues;
  DWIN_SERIAL.write(FHONE);
  DWIN_SERIAL.write(FHTWO);
  DWIN_SERIAL.write(valueslen + 3);
  DWIN_SERIAL.write(0x82);
  DWIN_SERIAL.write(adr >> 8);
  DWIN_SERIAL.write(adr & 0xFF);
  while (valueslen--) {
    char x;
    if (!strend) x = *myvalues++;
    if ((isstr && !x) || strend) {
      strend = true;
      x = fillChar;
    }
    DWIN_SERIAL.write(x);
  }
}

void RTS::setTouchScreenConfiguration() {
  // Main configuration (System_Config)
  LIMIT(dwin_settings.screen_brightness, 10, 100); // Prevent a possible all-dark screen
  LIMIT(dwin_settings.standby_time_seconds, 10, 655); // Prevent a possible all-dark screen for standby, yet also don't go higher than the DWIN limitation

  uint8_t cfg_bits = (0x0
    |                                   _BV(7)       // 7: Enable Control ... TERN0(DWINOS_4, _BV(7))
    |                                   _BV(5)       // 5: load 22 touch file
    |                                   _BV(4)       // 4: auto-upload should always be enabled
    | (dwin_settings.display_sound         ? _BV(3) : 0)  // 3: audio
    | (dwin_settings.display_standby       ? _BV(2) : 0)  // 2: backlight on standby
    | (dwin_settings.screen_rotation == 10 ? _BV(1) : 0)  // 1 & 0: Inversion
    #if LCD_SCREEN_ROTATE == 90
      |                                 _BV(0)       // Portrait Mode or 800x480 display has 0 point rotated 90deg from 480x272 display
    #elif LCD_SCREEN_ROTATE
      #error "Only 90° rotation is supported for the selected LCD."
    #endif
  );

  const uint8_t config_set[] = { 0x5A, 0x00, TERN(DWINOS_4, 0x00, 0xFF), cfg_bits };
  writeVariable(0x80 /*System_Config*/, config_set, sizeof(config_set));

  // Standby brightness (LED_Config)
  uint16_t dwinStandbyTimeSeconds = 100 * dwin_settings.standby_time_seconds; /* milliseconds, but divided by 10 (not 5 like the docs say) */
  const uint8_t brightness_set[] = {
    dwin_settings.screen_brightness /*% active*/,
    dwin_settings.standby_brightness /*% standby*/,
    static_cast<uint8_t>(dwinStandbyTimeSeconds >> 8),
    static_cast<uint8_t>(dwinStandbyTimeSeconds)
  };
  writeVariable(0x82 /*LED_Config*/, brightness_set, sizeof(brightness_set));

  if (!dwin_settings.display_sound) {
    sendData(0, VolumeIcon);
    sendData(9, SoundIcon);
  }
  else {
    sendData((dwin_settings.display_volume + 1) / 32 - 1, VolumeIcon);
    sendData(8, SoundIcon);
  }
  sendData(dwin_settings.display_volume, VolumeIcon - 2);
  sendData(dwin_settings.display_volume << 8, SoundAddr + 1);
  sendData(map(constrain(dwin_settings.display_volume, 0, 255), 0, 255, 0, 100), VolumeDisplay);
  sendData(dwin_settings.screen_brightness, DisplayBrightness);
  sendData(dwin_settings.standby_brightness, DisplayStandbyBrightness);
  sendData(dwin_settings.standby_time_seconds, DisplayStandbySeconds);
  sendData(dwin_settings.display_standby ? 3 : 2, DisplayStandbyEnableIndicator);
}

#endif // DGUS_LCD_UI_IA_CREALITY
