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

#include "draw_ui.h"
#include "pic_manager.h"
#include "draw_ready_print.h"
#include "mks_hardware.h"
#include "SPIFlashStorage.h"
#include "../../../libs/W25Qxx.h"
#include "../../../sd/cardreader.h"
#include "../../../MarlinCore.h"

#include <string.h>

extern uint16_t DeviceCode;

#if HAS_MEDIA
  extern char *createFilename(char * const buffer, const dir_t &p);
#endif

static FSTR_P const assets[] = {
  // Homing screen
  F("bmp_zeroAll.bin"),
  F("bmp_zero.bin"),
  F("bmp_zeroX.bin"),
  F("bmp_zeroY.bin"),
  F("bmp_zeroZ.bin"),
  F("bmp_manual_off.bin"),

  // Tool screen
  F("bmp_preHeat.bin"),
  F("bmp_extruct.bin"),
  F("bmp_mov.bin"),
  F("bmp_leveling.bin"),
  F("bmp_filamentchange.bin"),
  F("bmp_more.bin"),

  // Fan screen
  F("bmp_Add.bin"),
  F("bmp_Dec.bin"),
  F("bmp_speed255.bin"),
  F("bmp_speed127.bin"),
  F("bmp_speed0.bin"),

  F("bmp_bed.bin"),
  F("bmp_step1_degree.bin"),
  F("bmp_step5_degree.bin"),
  F("bmp_step10_degree.bin"),

  // Extrusion screen
  F("bmp_in.bin"),
  F("bmp_out.bin"),
  F("bmp_extru1.bin"),
  #if HAS_MULTI_EXTRUDER
    F("bmp_extru2.bin"),
  #endif
  F("bmp_speed_high.bin"),
  F("bmp_speed_slow.bin"),
  F("bmp_speed_normal.bin"),
  F("bmp_step1_mm.bin"),
  F("bmp_step5_mm.bin"),
  F("bmp_step10_mm.bin"),

  // Select file screen
  F("bmp_pageUp.bin"),
  F("bmp_pageDown.bin"),
  F("bmp_back.bin"), // TODO: why two back buttons? Why not just one? (return / back)
  F("bmp_dir.bin"),
  F("bmp_file.bin"),

  // Move motor screen
  // TODO: 6 equal icons, just in diffenct rotation... it may be optimized too
  F("bmp_xAdd.bin"),
  F("bmp_xDec.bin"),
  F("bmp_yAdd.bin"),
  F("bmp_yDec.bin"),
  F("bmp_zAdd.bin"),
  F("bmp_zDec.bin"),
  F("bmp_step_move0_1.bin"),
  F("bmp_step_move1.bin"),
  F("bmp_step_move10.bin"),

  // Operation screen
  F("bmp_auto_off.bin"),
  F("bmp_speed.bin"),
  F("bmp_fan.bin"),
  F("bmp_temp.bin"),
  F("bmp_extrude_opr.bin"),
  F("bmp_move_opr.bin"),

  // Change speed screen
  F("bmp_step1_percent.bin"),
  F("bmp_step5_percent.bin"),
  F("bmp_step10_percent.bin"),
  F("bmp_extruct_sel.bin"),
  F("bmp_mov_changespeed.bin"),
  F("bmp_mov_sel.bin"),
  F("bmp_speed_extruct.bin"),

  // Printing screen
  F("bmp_pause.bin"),
  F("bmp_resume.bin"),
  F("bmp_stop.bin"),
  F("bmp_ext1_state.bin"),
  #if HAS_MULTI_EXTRUDER
    F("bmp_ext2_state.bin"),
  #endif
  F("bmp_bed_state.bin"),
  F("bmp_fan_state.bin"),
  F("bmp_time_state.bin"),
  F("bmp_zpos_state.bin"),
  F("bmp_operate.bin"),

  // Manual Level screen (only if auto level is disabled)
  #if DISABLED(AUTO_BED_LEVELING_BILINEAR)
    F("bmp_leveling1.bin"),
    F("bmp_leveling2.bin"),
    F("bmp_leveling3.bin"),
    F("bmp_leveling4.bin"),
    F("bmp_leveling5.bin"),
  #endif

  // Language Select screen
  #if HAS_LANG_SELECT_SCREEN
    F("bmp_language.bin"),
    F("bmp_simplified_cn.bin"),
    F("bmp_simplified_cn_sel.bin"),
    F("bmp_traditional_cn.bin"),
    F("bmp_traditional_cn_sel.bin"),
    F("bmp_english.bin"),
    F("bmp_english_sel.bin"),
    F("bmp_russian.bin"),
    F("bmp_russian_sel.bin"),
    F("bmp_spanish.bin"),
    F("bmp_spanish_sel.bin"),
    F("bmp_french.bin"),
    F("bmp_french_sel.bin"),
    F("bmp_italy.bin"),
    F("bmp_italy_sel.bin"),
  #endif // HAS_LANG_SELECT_SCREEN

  // G-code preview
  #if HAS_GCODE_DEFAULT_VIEW_IN_FLASH
    F("bmp_preview.bin"),
  #endif

  #if HAS_LOGO_IN_FLASH
    F("bmp_logo.bin"),
  #endif

  // Settings screen
  F("bmp_about.bin"),
  F("bmp_eeprom_settings.bin"),
  F("bmp_machine_para.bin"),
  F("bmp_function1.bin"),

  // Start screen
  F("bmp_printing.bin"),
  F("bmp_set.bin"),
  F("bmp_tool.bin"),

  // Base icons
  F("bmp_arrow.bin"),
  F("bmp_back70x40.bin"),
  F("bmp_value_blank.bin"),
  F("bmp_blank_sel.bin"),
  F("bmp_disable.bin"),
  F("bmp_enable.bin"),
  F("bmp_return.bin"),

  #if ENABLED(MKS_WIFI_MODULE)
    // Wifi screen
    F("bmp_wifi.bin"),
    F("bmp_cloud.bin"),
  #endif

  #if HAS_MULTI_VOLUME
    F("bmp_usb_disk.bin"),
    //F("bmp_usb_disk_sel.bin"),
    F("bmp_sd.bin"),
    //F("bmp_sd_sel.bin"),
  #endif

  // Babystep screen
  F("bmp_baby_move0_01.bin"),
  F("bmp_baby_move0_05.bin"),
  F("bmp_baby_move0_1.bin"),

  // More screen
  F("bmp_custom1.bin"),
  F("bmp_custom2.bin"),
  F("bmp_custom3.bin"),
  F("bmp_custom4.bin"),
  F("bmp_custom5.bin"),
  F("bmp_custom6.bin"),
  F("bmp_custom7.bin")
};

#if HAS_SPI_FLASH_FONT
  static FSTR_P const fonts[] = { F("FontUNIGBK.bin") };
#endif

uint8_t currentFlashPage = 0;

uint32_t lv_get_pic_addr(uint8_t *Pname) {
  uint8_t Pic_cnt;
  uint8_t i, j;
  PIC_MSG PIC;
  uint32_t tmp_cnt = 0;
  uint32_t addr = 0;

  currentFlashPage = 0;

  #if ENABLED(MARLIN_DEV_MODE)
    SERIAL_ECHOLNPGM("Getting picture SPI Flash Address: ", (const char*)Pname);
  #endif

  W25QXX.init(SPI_QUARTER_SPEED);

  W25QXX.SPI_FLASH_BufferRead(&Pic_cnt, PIC_COUNTER_ADDR, 1);
  if (Pic_cnt == 0xFF) Pic_cnt = 0;
  for (i = 0; i < Pic_cnt; i++) {
    j = 0;
    do {
      W25QXX.SPI_FLASH_BufferRead(&PIC.name[j], PIC_NAME_ADDR + tmp_cnt, 1);
      tmp_cnt++;
    } while (PIC.name[j++] != '\0');

    if ((strcasecmp((char*)Pname, (char*)PIC.name)) == 0) {
      if (DeviceCode == 0x9488 || DeviceCode == 0x5761)
        addr = PIC_DATA_ADDR_TFT35 + i * PER_PIC_MAX_SPACE_TFT35;
      else
        addr = PIC_DATA_ADDR_TFT32 + i * PER_PIC_MAX_SPACE_TFT32;
      return addr;
    }
  }
  return addr;
}

const char *assetsPath = "assets";
const char *bakPath = "_assets";

void spiFlashErase_PIC() {
  volatile uint32_t pic_sectorcnt = 0;
  W25QXX.init(SPI_QUARTER_SPEED);
  // erase 0x001000 -64K
  for (pic_sectorcnt = 0; pic_sectorcnt < (64 - 4) / 4; pic_sectorcnt++) {
    hal.watchdog_refresh();
    W25QXX.SPI_FLASH_SectorErase(PICINFOADDR + pic_sectorcnt * 4 * 1024);
  }
  // erase 64K -- 6M
  for (pic_sectorcnt = 0; pic_sectorcnt < (PIC_SIZE_xM * 1024 / 64 - 1); pic_sectorcnt++) {
    hal.watchdog_refresh();
    W25QXX.SPI_FLASH_BlockErase((pic_sectorcnt + 1) * 64 * 1024);
  }
}

#if HAS_SPI_FLASH_FONT
  void spiFlashErase_FONT() {
    volatile uint32_t Font_sectorcnt = 0;
    W25QXX.init(SPI_QUARTER_SPEED);
    for (Font_sectorcnt = 0; Font_sectorcnt < 32 - 1; Font_sectorcnt++) {
      hal.watchdog_refresh();
      W25QXX.SPI_FLASH_BlockErase(FONTINFOADDR + Font_sectorcnt * 64 * 1024);
    }
  }
#endif

uint32_t LogoWrite_Addroffset = 0;

uint8_t picLogoWrite(uint8_t *LogoName, uint8_t *Logo_Wbuff, uint32_t LogoWriteSize) {
  if (LogoWriteSize <= 0) return 0;

  W25QXX.SPI_FLASH_BufferWrite(Logo_Wbuff, PIC_LOGO_ADDR + LogoWrite_Addroffset, LogoWriteSize);

  for (uint32_t i = 0; i < LogoWriteSize; i++) {
    uint8_t temp1;
    W25QXX.SPI_FLASH_BufferRead(&temp1, PIC_LOGO_ADDR + LogoWrite_Addroffset + i, 1);
    if (*(Logo_Wbuff + i) != temp1) return 0;
  }
  LogoWrite_Addroffset += LogoWriteSize;
  const uint32_t logo_maxsize = DeviceCode == 0x9488 || DeviceCode == 0x5761 ? LOGO_MAX_SIZE_TFT35 : LOGO_MAX_SIZE_TFT32;
  if (LogoWrite_Addroffset >= logo_maxsize) LogoWrite_Addroffset = 0;
  return 1;
}

uint32_t TitleLogoWrite_Addroffset = 0;
uint8_t picTitleLogoWrite(uint8_t *TitleLogoName, uint8_t *TitleLogo_Wbuff, uint32_t TitleLogoWriteSize) {
  if (TitleLogoWriteSize <= 0)
    return 0;
  if ((DeviceCode == 0x9488) || (DeviceCode == 0x5761))
    W25QXX.SPI_FLASH_BufferWrite(TitleLogo_Wbuff, PIC_ICON_LOGO_ADDR_TFT35 + TitleLogoWrite_Addroffset, TitleLogoWriteSize);
  else
    W25QXX.SPI_FLASH_BufferWrite(TitleLogo_Wbuff, PIC_ICON_LOGO_ADDR_TFT32 + TitleLogoWrite_Addroffset, TitleLogoWriteSize);
  TitleLogoWrite_Addroffset += TitleLogoWriteSize;
  if (TitleLogoWrite_Addroffset >= TITLELOGO_MAX_SIZE)
    TitleLogoWrite_Addroffset = 0;
  return 1;
}

uint32_t default_view_addroffset_r = 0;
void default_view_Write(uint8_t *default_view__Rbuff, uint32_t default_view_Writesize) {
  W25QXX.SPI_FLASH_BufferWrite(default_view__Rbuff, DEFAULT_VIEW_ADDR_TFT35 + default_view_addroffset_r, default_view_Writesize);
  default_view_addroffset_r += default_view_Writesize;
  if (default_view_addroffset_r >= DEFAULT_VIEW_MAX_SIZE)
    default_view_addroffset_r = 0;
}

uint32_t picInfoWrite(uint8_t *P_name, uint32_t P_size) {
  uint8_t pic_counter = 0;
  uint32_t picSaveAddr;
  uint32_t Pic_SizeSaveAddr;
  uint32_t picNameSaveAddr;
  uint8_t pNameTemp;
  uint32_t i, j;
  uint32_t name_len = 0;
  uint32_t saveNameLen = 0;
  union union32 size_tmp;

  W25QXX.SPI_FLASH_BufferRead(&pic_counter, PIC_COUNTER_ADDR, 1);

  if (pic_counter == 0xFF)
    pic_counter = 0;

  if ((DeviceCode == 0x9488) || (DeviceCode == 0x5761))
    picSaveAddr = PIC_DATA_ADDR_TFT35 + pic_counter * PER_PIC_MAX_SPACE_TFT35;
  else
    picSaveAddr = PIC_DATA_ADDR_TFT32 + pic_counter * PER_PIC_MAX_SPACE_TFT32;

  for (j = 0; j < pic_counter; j++) {
    do {
      W25QXX.SPI_FLASH_BufferRead(&pNameTemp, PIC_NAME_ADDR + saveNameLen, 1);
      saveNameLen++;
    } while (pNameTemp != '\0');
  }
  i = 0;
  while ((*(P_name + i) != '\0')) {
    i++;
    name_len++;
  }

  picNameSaveAddr = PIC_NAME_ADDR + saveNameLen;
  W25QXX.SPI_FLASH_BufferWrite(P_name, picNameSaveAddr, name_len + 1);
  Pic_SizeSaveAddr = PIC_SIZE_ADDR + 4 * pic_counter;
  size_tmp.dwords = P_size;
  W25QXX.SPI_FLASH_BufferWrite(size_tmp.bytes, Pic_SizeSaveAddr, 4);

  pic_counter++;
  W25QXX.SPI_FLASH_SectorErase(PIC_COUNTER_ADDR);
  W25QXX.SPI_FLASH_BufferWrite(&pic_counter, PIC_COUNTER_ADDR, 1);

  return picSaveAddr;
}

#if HAS_MEDIA

  static void dosName2LongName(const char dosName[11], char *longName) {
    uint8_t j = 0;
    for (uint8_t i = 0; i < 11; ++i) {
      if (i == 8) longName[j++] = '.';
      if (dosName[i] == '\0' || dosName[i] == ' ') continue;
      longName[j++] = dosName[i];
    }
    longName[j] = '\0';
  }

  static int8_t arrayFindStr(FSTR_P const arr[], uint8_t arraySize, const char *str) {
    for (uint8_t a = 0; a < arraySize; a++) {
      if (strcasecmp(FTOP(arr[a]), str) == 0)
        return a;
    }
    return -1;
  }

  #if ENABLED(MARLIN_DEV_MODE)
    static uint32_t totalSizes = 0, totalCompressed = 0;
  #endif

  #define ASSET_TYPE_ICON       0
  #define ASSET_TYPE_LOGO       1
  #define ASSET_TYPE_TITLE_LOGO 2
  #define ASSET_TYPE_G_PREVIEW  3
  #define ASSET_TYPE_FONT       4
  static void loadAsset(MediaFile &dir, dir_t& entry, FSTR_P const fn, int8_t assetType) {
    MediaFile file;
    char dosFilename[FILENAME_LENGTH];
    createFilename(dosFilename, entry);
    if (!file.open(&dir, dosFilename, O_READ)) {
      #if ENABLED(MARLIN_DEV_MODE)
        SERIAL_ECHOLNPGM("Error opening Asset: ", fn);
      #endif
      return;
    }

    hal.watchdog_refresh();
    disp_assets_update_progress(fn);

    W25QXX.init(SPI_QUARTER_SPEED);

    uint16_t pbr;
    uint32_t pfileSize;
    uint32_t totalSizeLoaded = 0;
    uint32_t Pic_Write_Addr;
    pfileSize = file.fileSize();
    totalSizeLoaded += pfileSize;
    if (assetType == ASSET_TYPE_LOGO) {
      do {
        hal.watchdog_refresh();
        pbr = file.read(public_buf, BMP_WRITE_BUF_LEN);
        picLogoWrite((uint8_t*)fn, public_buf, pbr);
      } while (pbr >= BMP_WRITE_BUF_LEN);
    }
    else if (assetType == ASSET_TYPE_TITLE_LOGO) {
      do {
        hal.watchdog_refresh();
        pbr = file.read(public_buf, BMP_WRITE_BUF_LEN);
        picTitleLogoWrite((uint8_t*)fn, public_buf, pbr);
      } while (pbr >= BMP_WRITE_BUF_LEN);
    }
    else if (assetType == ASSET_TYPE_G_PREVIEW) {
      do {
        hal.watchdog_refresh();
        pbr = file.read(public_buf, BMP_WRITE_BUF_LEN);
        default_view_Write(public_buf, pbr);
      } while (pbr >= BMP_WRITE_BUF_LEN);
    }
    else if (assetType == ASSET_TYPE_ICON) {
      Pic_Write_Addr = picInfoWrite((uint8_t*)fn, pfileSize);
      SPIFlash.beginWrite(Pic_Write_Addr);
      #if HAS_SPI_FLASH_COMPRESSION
        do {
          hal.watchdog_refresh();
          pbr = file.read(public_buf, SPI_FLASH_PageSize);
          TERN_(MARLIN_DEV_MODE, totalSizes += pbr);
          SPIFlash.writeData(public_buf, SPI_FLASH_PageSize);
        } while (pbr >= SPI_FLASH_PageSize);
      #else
        do {
          pbr = file.read(public_buf, BMP_WRITE_BUF_LEN);
          W25QXX.SPI_FLASH_BufferWrite(public_buf, Pic_Write_Addr, pbr);
          Pic_Write_Addr += pbr;
        } while (pbr >= BMP_WRITE_BUF_LEN);
      #endif
      #if ENABLED(MARLIN_DEV_MODE)
        SERIAL_ECHOLNPGM("Space used: ", fn, " - ", (SPIFlash.getCurrentPage() + 1) * SPI_FLASH_PageSize / 1024, "KB");
        totalCompressed += (SPIFlash.getCurrentPage() + 1) * SPI_FLASH_PageSize;
      #endif
      SPIFlash.endWrite();
    }
    else if (assetType == ASSET_TYPE_FONT) {
      Pic_Write_Addr = UNIGBK_FLASH_ADDR;
      do {
        hal.watchdog_refresh();
        pbr = file.read(public_buf, BMP_WRITE_BUF_LEN);
        W25QXX.SPI_FLASH_BufferWrite(public_buf, Pic_Write_Addr, pbr);
        Pic_Write_Addr += pbr;
      } while (pbr >= BMP_WRITE_BUF_LEN);
    }

    file.close();

    #if ENABLED(MARLIN_DEV_MODE)
      SERIAL_ECHOLNPGM("Asset added: ", fn);
    #endif
  }

  void UpdateAssets() {
    if (!card.isMounted()) return;
    MediaFile dir, root = card.getroot();
    if (dir.open(&root, assetsPath, O_RDONLY)) {

      disp_assets_update();
      disp_assets_update_progress(F("Erasing pics..."));
      hal.watchdog_refresh();
      spiFlashErase_PIC();
      #if HAS_SPI_FLASH_FONT
        disp_assets_update_progress(F("Erasing fonts..."));
        hal.watchdog_refresh();
        spiFlashErase_FONT();
      #endif

      disp_assets_update_progress(F("Reading files..."));
      dir_t d;
      while (dir.readDir(&d, card.longFilename) > 0) {
        // If we don't get a long name, but gets a short one, try it
        if (card.longFilename[0] == 0 && d.name[0] != 0)
          dosName2LongName((const char*)d.name, card.longFilename);
        if (card.longFilename[0] == 0) continue;
        if (card.longFilename[0] == '.') continue;

        int8_t a = arrayFindStr(assets, COUNT(assets), card.longFilename);
        if (a >= 0 && a < (int8_t)COUNT(assets)) {
          uint8_t assetType = ASSET_TYPE_ICON;
          if (strstr_P(FTOP(assets[a]), PSTR("_logo")))
            assetType = ASSET_TYPE_LOGO;
          else if (strstr_P(FTOP(assets[a]), PSTR("_titlelogo")))
            assetType = ASSET_TYPE_TITLE_LOGO;
          else if (strstr_P(FTOP(assets[a]), PSTR("_preview")))
            assetType = ASSET_TYPE_G_PREVIEW;

          loadAsset(dir, d, assets[a], assetType);

          continue;
        }

        #if HAS_SPI_FLASH_FONT
          a = arrayFindStr(fonts, COUNT(fonts), card.longFilename);
          if (a >= 0 && a < (int8_t)COUNT(fonts))
            loadAsset(dir, d, fonts[a], ASSET_TYPE_FONT);
        #endif
      }
      dir.rename(&root, bakPath);
    }
    dir.close();

    #if ENABLED(MARLIN_DEV_MODE)
      uint8_t pic_counter = 0;
      W25QXX.SPI_FLASH_BufferRead(&pic_counter, PIC_COUNTER_ADDR, 1);
      SERIAL_ECHOLNPGM("Total assets loaded: ", pic_counter);
      SERIAL_ECHOLNPGM("Total Uncompressed: ", totalSizes, ", Compressed: ", totalCompressed);
    #endif
  }

  #if HAS_SPI_FLASH_FONT
    void spi_flash_read_test() { W25QXX.SPI_FLASH_BufferRead(public_buf, UNIGBK_FLASH_ADDR, BMP_WRITE_BUF_LEN); }
  #endif

#endif // HAS_MEDIA

void picRead(uint8_t *Pname, uint8_t *P_Rbuff) {
  uint8_t i, j;
  uint8_t Pic_cnt;
  uint32_t tmp_cnt = 0;
  PIC_MSG PIC;

  W25QXX.SPI_FLASH_BufferRead(&Pic_cnt, PIC_COUNTER_ADDR, 1);
  if (Pic_cnt == 0xFF)
    Pic_cnt = 0;

  for (i = 0; i < Pic_cnt; i++) {
    j = 0;
    do {
      W25QXX.SPI_FLASH_BufferRead(&PIC.name[j], PIC_NAME_ADDR + tmp_cnt, 1);
      tmp_cnt++;
    } while (PIC.name[j++] != '\0');
    // pic size
    W25QXX.SPI_FLASH_BufferRead(PIC.size.bytes, PIC_SIZE_ADDR + i * 4, 4);

    if ((strcmp((char*)Pname, (char*)PIC.name)) == 0) {
      W25QXX.SPI_FLASH_BufferRead((uint8_t *)P_Rbuff, PIC_DATA_ADDR_TFT35 + i * PER_PIC_MAX_SPACE_TFT35, PIC.size.dwords);
      break;
    }
  }
}

void lv_pic_test(uint8_t *P_Rbuff, uint32_t addr, uint32_t size) {
  #if HAS_SPI_FLASH_COMPRESSION
    if (currentFlashPage == 0)
      SPIFlash.beginRead(addr);
    SPIFlash.readData(P_Rbuff, size);
    currentFlashPage++;
  #else
    W25QXX.init(SPI_QUARTER_SPEED);
    W25QXX.SPI_FLASH_BufferRead((uint8_t *)P_Rbuff, addr, size);
  #endif
}

#if HAS_SPI_FLASH_FONT
  void get_spi_flash_data(const char *rec_buf, int addr, int size) {
    W25QXX.init(SPI_QUARTER_SPEED);
    W25QXX.SPI_FLASH_BufferRead((uint8_t *)rec_buf, UNIGBK_FLASH_ADDR + addr, size);
  }
#endif

uint32_t logo_addroffset = 0;
void picLogoRead(uint8_t *LogoName, uint8_t *Logo_Rbuff, uint32_t LogoReadsize) {
  W25QXX.init(SPI_QUARTER_SPEED);
  W25QXX.SPI_FLASH_BufferRead(Logo_Rbuff, PIC_LOGO_ADDR + logo_addroffset, LogoReadsize);
  logo_addroffset += LogoReadsize;
  if (logo_addroffset >= LOGO_MAX_SIZE_TFT35)
    logo_addroffset = 0;
}

uint32_t default_view_addroffset = 0;
void default_view_Read(uint8_t *default_view_Rbuff, uint32_t default_view_Readsize) {
  W25QXX.init(SPI_QUARTER_SPEED);
  W25QXX.SPI_FLASH_BufferRead(default_view_Rbuff, DEFAULT_VIEW_ADDR_TFT35 + default_view_addroffset, default_view_Readsize);
  default_view_addroffset += default_view_Readsize;
  if (default_view_addroffset >= DEFAULT_VIEW_MAX_SIZE)
    default_view_addroffset = 0;
}

#if HAS_BAK_VIEW_IN_FLASH
  uint32_t flash_view_addroffset = 0;
  void flash_view_Read(uint8_t *flash_view_Rbuff, uint32_t flash_view_Readsize) {
    W25QXX.init(SPI_QUARTER_SPEED);
    W25QXX.SPI_FLASH_BufferRead(flash_view_Rbuff, BAK_VIEW_ADDR_TFT35 + flash_view_addroffset, flash_view_Readsize);
    flash_view_addroffset += flash_view_Readsize;
    if (flash_view_addroffset >= FLASH_VIEW_MAX_SIZE)
      flash_view_addroffset = 0;
  }
#endif

#endif // HAS_TFT_LVGL_UI
