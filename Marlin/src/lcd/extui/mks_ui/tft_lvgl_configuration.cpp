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
#include "draw_ready_print.h"

#include "pic_manager.h"
#include "mks_hardware.h"
#include "draw_ui.h"
#include "SPIFlashStorage.h"
#include <lvgl.h>

#include "../../../MarlinCore.h"
#include "../../marlinui.h"

#include "../../../inc/MarlinConfig.h"

#include HAL_PATH(../../.., tft/xpt2046.h)
XPT2046 touch;

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#if HAS_SERVOS
  #include "../../../module/servo.h"
#endif

#if ANY(PROBE_TARE, HAS_Z_SERVO_PROBE)
  #include "../../../module/probe.h"
#endif

#include "../../tft_io/touch_calibration.h"
#if ENABLED(TOUCH_SCREEN_CALIBRATION)
  #include "draw_touch_calibration.h"
#endif

#if ENABLED(MKS_WIFI_MODULE)
  #include "wifi_module.h"
#endif

#include <SPI.h>

#ifndef TFT_WIDTH
  #define TFT_WIDTH  480
#endif
#ifndef TFT_HEIGHT
  #define TFT_HEIGHT 320
#endif

#if HAS_SPI_FLASH_FONT
  void init_gb2312_font();
#endif

static lv_disp_buf_t disp_buf;
lv_group_t*  g;
#if HAS_MEDIA
  void UpdateAssets();
#endif
uint16_t DeviceCode = 0x9488;
extern uint8_t sel_id;

uint8_t bmp_public_buf[14 * 1024];
uint8_t public_buf[513];

extern bool flash_preview_begin, default_preview_flg, gcode_preview_over;

void SysTick_Callback() {
  lv_tick_inc(1);
  print_time_count();
  #if ENABLED(MKS_WIFI_MODULE)
    if (tips_disp.timer == TIPS_TIMER_START)
      tips_disp.timer_count++;
  #endif
  if (uiCfg.filament_loading_time_flg) {
    uiCfg.filament_loading_time_cnt++;
    uiCfg.filament_rate = uint32_t(100.0f * uiCfg.filament_loading_time_cnt / SEC_TO_MS(uiCfg.filament_loading_time) + 0.5f);
    if (uiCfg.filament_loading_time_cnt >= SEC_TO_MS(uiCfg.filament_loading_time)) {
      uiCfg.filament_loading_time_cnt  = 0;
      uiCfg.filament_loading_time_flg  = false;
      uiCfg.filament_loading_completed = true;
    }
  }
  if (uiCfg.filament_unloading_time_flg) {
    uiCfg.filament_unloading_time_cnt++;
    uiCfg.filament_rate = uint32_t(100.0f * uiCfg.filament_unloading_time_cnt / SEC_TO_MS(uiCfg.filament_unloading_time) + 0.5f);
    if (uiCfg.filament_unloading_time_cnt >= SEC_TO_MS(uiCfg.filament_unloading_time)) {
      uiCfg.filament_unloading_time_cnt  = 0;
      uiCfg.filament_unloading_time_flg  = false;
      uiCfg.filament_unloading_completed = true;
      uiCfg.filament_rate = 100;
    }
  }
}

void tft_lvgl_init() {

  W25QXX.init(SPI_QUARTER_SPEED);

  gCfgItems_init();
  ui_cfg_init();
  disp_language_init();

  hal.watchdog_refresh();     // LVGL init takes time

  // Init TFT first!
  SPI_TFT.spiInit(SPI_FULL_SPEED);
  SPI_TFT.lcdInit();

  hal.watchdog_refresh();     // LVGL init takes time

  #if HAS_USB_FLASH_DRIVE
    uint16_t usb_flash_loop = 1000;
    #if HAS_MULTI_VOLUME && !HAS_SD_HOST_DRIVE
      if (IS_SD_INSERTED())
        card.changeMedia(&card.media_driver_sdcard);
      else
        card.changeMedia(&card.media_driver_usbFlash);
    #endif
    do {
      card.media_driver_usbFlash.idle();
      hal.watchdog_refresh();
      delay(2);
    } while (!card.media_driver_usbFlash.isInserted() && usb_flash_loop--);
    card.mount();
  #elif HAS_LOGO_IN_FLASH
    delay(1000);
    hal.watchdog_refresh();
    delay(1000);
  #endif

  hal.watchdog_refresh();     // LVGL init takes time

  #if HAS_MEDIA
    UpdateAssets();
    hal.watchdog_refresh();   // LVGL init takes time
    TERN_(MKS_TEST, mks_test_get());
  #endif

  touch.init();

  lv_init();

  lv_disp_buf_init(&disp_buf, bmp_public_buf, nullptr, LV_HOR_RES_MAX * 14); // Initialize the display buffer

  lv_disp_drv_t disp_drv;     // Descriptor of a display driver
  lv_disp_drv_init(&disp_drv);    // Basic initialization
  disp_drv.flush_cb = my_disp_flush; // Set your driver function
  disp_drv.buffer = &disp_buf;    // Assign the buffer to the display
  lv_disp_drv_register(&disp_drv);  // Finally register the driver

  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);     // Descriptor of a input device driver
  indev_drv.type = LV_INDEV_TYPE_POINTER; // Touch pad is a pointer-like device
  indev_drv.read_cb = my_touchpad_read;  // Set your driver function
  lv_indev_drv_register(&indev_drv);   // Finally register the driver

  #if HAS_ROTARY_ENCODER
    g = lv_group_create();
    lv_indev_drv_t enc_drv;
    lv_indev_drv_init(&enc_drv);
    enc_drv.type = LV_INDEV_TYPE_ENCODER;
    enc_drv.read_cb = my_mousewheel_read;
    lv_indev_t * enc_indev = lv_indev_drv_register(&enc_drv);
    lv_indev_set_group(enc_indev, g);
  #endif

  lv_fs_drv_t spi_flash_drv;
  lv_fs_drv_init(&spi_flash_drv);
  spi_flash_drv.letter = 'F';
  spi_flash_drv.open_cb = spi_flash_open_cb;
  spi_flash_drv.close_cb = spi_flash_close_cb;
  spi_flash_drv.read_cb = spi_flash_read_cb;
  spi_flash_drv.seek_cb = spi_flash_seek_cb;
  spi_flash_drv.tell_cb = spi_flash_tell_cb;
  lv_fs_drv_register(&spi_flash_drv);

  lv_fs_drv_t sd_drv;
  lv_fs_drv_init(&sd_drv);
  sd_drv.letter = 'S';
  sd_drv.open_cb = sd_open_cb;
  sd_drv.close_cb = sd_close_cb;
  sd_drv.read_cb = sd_read_cb;
  sd_drv.seek_cb = sd_seek_cb;
  sd_drv.tell_cb = sd_tell_cb;
  lv_fs_drv_register(&sd_drv);

  systick_attach_callback(SysTick_Callback);

  TERN_(HAS_SPI_FLASH_FONT, init_gb2312_font());

  tft_style_init();
  filament_pin_setup();

  #if ENABLED(MKS_WIFI_MODULE)
    mks_esp_wifi_init();
    mks_wifi_firmware_update();
  #endif
  TERN_(HAS_SERVOS, servo_init());
  TERN_(HAS_Z_SERVO_PROBE, probe.servo_probe_init());
  bool ready = true;
  #if ENABLED(POWER_LOSS_RECOVERY)
    recovery.load();
    if (recovery.valid()) {
      ready = false;
      if (gCfgItems.from_flash_pic)
        flash_preview_begin = true;
      else
        default_preview_flg = true;

      uiCfg.print_state = REPRINTING;

      #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
        strlcpy(public_buf_m, recovery.info.sd_filename, sizeof(public_buf_m));
        card.printLongPath(public_buf_m);
        strlcpy(list_file.long_name[sel_id], card.longFilename, sizeof(list_file.long_name[0]));
      #else
        strlcpy(list_file.long_name[sel_id], recovery.info.sd_filename, sizeof(list_file.long_name[0]));
      #endif
      lv_draw_printing();
    }
  #endif

  if (ready) lv_draw_ready_print();

  #if ALL(MKS_TEST, HAS_MEDIA)
    if (mks_test_flag == 0x1E) mks_gpio_test();
  #endif
}

static lv_disp_drv_t* disp_drv_p;

#if ENABLED(USE_SPI_DMA_TC)
  bool lcd_dma_trans_lock = false;
#endif

void dmc_tc_handler(struct __DMA_HandleTypeDef * hdma) {
  #if ENABLED(USE_SPI_DMA_TC)
    lv_disp_flush_ready(disp_drv_p);
    lcd_dma_trans_lock = false;
    TFT_SPI::abort();
  #endif
}

void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p) {
  uint16_t width = area->x2 - area->x1 + 1,
          height = area->y2 - area->y1 + 1;

  disp_drv_p = disp;

  SPI_TFT.setWindow((uint16_t)area->x1, (uint16_t)area->y1, width, height);

  #if ENABLED(USE_SPI_DMA_TC)
    lcd_dma_trans_lock = true;
    SPI_TFT.tftio.writeSequenceIT((uint16_t*)color_p, width * height);
    TFT_SPI::DMAtx.XferCpltCallback = dmc_tc_handler;
  #else
    SPI_TFT.tftio.writeSequence((uint16_t*)color_p, width * height);
    lv_disp_flush_ready(disp_drv_p); // Indicate you are ready with the flushing
  #endif

  W25QXX.init(SPI_QUARTER_SPEED);
}

#if ENABLED(USE_SPI_DMA_TC)
  bool get_lcd_dma_lock() { return lcd_dma_trans_lock; }
#endif

void lv_fill_rect(lv_coord_t x1, lv_coord_t y1, lv_coord_t x2, lv_coord_t y2, lv_color_t bk_color) {
  uint16_t width, height;
  width = x2 - x1 + 1;
  height = y2 - y1 + 1;
  SPI_TFT.setWindow((uint16_t)x1, (uint16_t)y1, width, height);
  SPI_TFT.tftio.writeMultiple(bk_color.full, width * height);
  W25QXX.init(SPI_QUARTER_SPEED);
}

uint16_t getTickDiff(const uint16_t curTick, const uint16_t lastTick) {
  return (TICK_CYCLE) * (lastTick <= curTick ? (curTick - lastTick) : (0xFFFFFFFF - lastTick + curTick));
}

static bool get_point(int16_t * const x, int16_t * const y) {
  if (!touch.getRawPoint(x, y)) return false;

  #if ENABLED(TOUCH_SCREEN_CALIBRATION)
    const calibrationState state = touch_calibration.get_calibration_state();
    if (WITHIN(state, CALIBRATION_TOP_LEFT, CALIBRATION_BOTTOM_LEFT)) {
      if (touch_calibration.handleTouch(*x, *y)) lv_update_touch_calibration_screen();
      return false;
    }
  #endif

  *x = int16_t((int32_t(*x) * _TOUCH_CALIBRATION_X) >> 16) + _TOUCH_OFFSET_X;
  *y = int16_t((int32_t(*y) * _TOUCH_CALIBRATION_Y) >> 16) + _TOUCH_OFFSET_Y;

  return true;
}

bool my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data) {
  static xy_int_t last { 0, 0 };
  data->state = get_point(&last.x, &last.y) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
  data->point.x = (TFT_ROTATION == TFT_ROTATE_180) ? TFT_WIDTH - last.x : last.x;
  data->point.y = (TFT_ROTATION == TFT_ROTATE_180) ? TFT_HEIGHT - last.y : last.y;
  return false; // Return `false` since no data is buffering or left to read
}

int16_t enc_diff = 0;
lv_indev_state_t indev_enc_state = LV_INDEV_STATE_REL; // ENC button is pressed or released

bool my_mousewheel_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data) {
  UNUSED(indev_drv);

  data->state = indev_enc_state;
  data->enc_diff = enc_diff;
  enc_diff = 0;

  return false;       // No more data to read so return false
}

extern uint8_t currentFlashPage;

// spi_flash
uint32_t pic_read_base_addr = 0, pic_read_addr_offset = 0;
lv_fs_res_t spi_flash_open_cb (lv_fs_drv_t * drv, void * file_p, const char * path, lv_fs_mode_t mode) {
  static char last_path_name[30];
  if (strcasecmp(last_path_name, path) != 0) {
    pic_read_base_addr = lv_get_pic_addr((uint8_t *)path);
    strcpy(last_path_name, path);
  }
  else {
    W25QXX.init(SPI_QUARTER_SPEED);
    currentFlashPage = 0;
  }
  pic_read_addr_offset = pic_read_base_addr;
  return LV_FS_RES_OK;
}

lv_fs_res_t spi_flash_close_cb (lv_fs_drv_t * drv, void * file_p) {
  lv_fs_res_t res = LV_FS_RES_OK;
  /* Add your code here */
  pic_read_addr_offset = pic_read_base_addr;
  return res;
}

lv_fs_res_t spi_flash_read_cb (lv_fs_drv_t * drv, void * file_p, void * buf, uint32_t btr, uint32_t * br) {
  lv_pic_test((uint8_t *)buf, pic_read_addr_offset, btr);
  *br = btr;
  return LV_FS_RES_OK;
}

lv_fs_res_t spi_flash_seek_cb(lv_fs_drv_t * drv, void * file_p, uint32_t pos) {
  #if HAS_SPI_FLASH_COMPRESSION
    if (pos == 4) {
      uint8_t bmp_header[4];
      SPIFlash.beginRead(pic_read_base_addr);
      SPIFlash.readData(bmp_header, 4);
      currentFlashPage = 1;
    }
    pic_read_addr_offset = pic_read_base_addr;
  #else
    pic_read_addr_offset = pic_read_base_addr + pos;
  #endif
  return LV_FS_RES_OK;
}

lv_fs_res_t spi_flash_tell_cb(lv_fs_drv_t * drv, void * file_p, uint32_t * pos_p) {
  *pos_p = pic_read_addr_offset - pic_read_base_addr;
  return LV_FS_RES_OK;
}

// sd
char *cur_namefff;
uint32_t sd_read_base_addr = 0, sd_read_addr_offset = 0, small_image_size = 409;
lv_fs_res_t sd_open_cb (lv_fs_drv_t * drv, void * file_p, const char * path, lv_fs_mode_t mode) {
  char name_buf[100];
  *name_buf = '/';
  strcpy(name_buf + 1, path);
  char *temp = strstr(name_buf, ".bin");
  if (temp) strcpy(temp, ".GCO");
  sd_read_base_addr = lv_open_gcode_file((char *)name_buf);
  sd_read_addr_offset = sd_read_base_addr;
  if (sd_read_addr_offset == UINT32_MAX) return LV_FS_RES_NOT_EX;
  // find small image size
  card.read(public_buf, 512);
  public_buf[511] = '\0';
  const char* eol = strpbrk((const char*)public_buf, "\n\r");
  small_image_size = (uintptr_t)eol - (uintptr_t)((uint32_t *)(&public_buf[0])) + 1;
  return LV_FS_RES_OK;
}

lv_fs_res_t sd_close_cb (lv_fs_drv_t * drv, void * file_p) {
  /* Add your code here */
  lv_close_gcode_file();
  return LV_FS_RES_OK;
}

lv_fs_res_t sd_read_cb (lv_fs_drv_t * drv, void * file_p, void * buf, uint32_t btr, uint32_t * br) {
  if (btr == 200) {
    lv_gcode_file_read((uint8_t *)buf);
    //pic_read_addr_offset += 208;
    *br = 200;
  }
  else if (btr == 4) {
    uint8_t header_pic[4] = { 0x04, 0x90, 0x81, 0x0C };
    memcpy(buf, header_pic, 4);
    *br = 4;
  }
  return LV_FS_RES_OK;
}

lv_fs_res_t sd_seek_cb(lv_fs_drv_t * drv, void * file_p, uint32_t pos) {
  sd_read_addr_offset = sd_read_base_addr + (pos - 4) / 200 * small_image_size;
  lv_gcode_file_seek(sd_read_addr_offset);
  return LV_FS_RES_OK;
}

lv_fs_res_t sd_tell_cb(lv_fs_drv_t * drv, void * file_p, uint32_t * pos_p) {
  if (sd_read_addr_offset) *pos_p = 0;
  else *pos_p = (sd_read_addr_offset - sd_read_base_addr) / small_image_size * 200 + 4;
  return LV_FS_RES_OK;
}

void lv_update_encoder() {

  #if ANY_BUTTON(EN1, EN2)
    constexpr uint8_t epps = ENCODER_PULSES_PER_STEP;   // We can fill in
    static uint8_t pulse_count;
    pulse_count += ui.get_encoder_delta();
    const int8_t fullSteps = pulse_count / epps;
    pulse_count -= fullSteps * epps;
    enc_diff += fullSteps;
  #endif

  #if ANY_BUTTON(ENC, BACK, UP, DOWN, LEFT, RIGHT)
    static millis_t last_encoder_ms;
    const millis_t now = millis(), diffTime = getTickDiff(now, last_encoder_ms);
    if (diffTime <= 50) return;
  #endif

  #if BUTTON_EXISTS(ENC)
    static uint8_t old_button_enc = LV_INDEV_STATE_REL;
    const uint8_t enc_c = BUTTON_PRESSED(ENC) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    if (enc_c != old_button_enc) {
      indev_enc_state = enc_c ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
      old_button_enc = enc_c;
    }
  #endif

  #if BUTTON_EXISTS(BACK)
    if (BUTTON_PRESSED(BACK)) {}
  #endif
  #if BUTTON_EXISTS(UP)
    if (BUTTON_PRESSED(UP)) {}
  #endif
  #if BUTTON_EXISTS(DOWN)
    if (BUTTON_PRESSED(DOWN)) {}
  #endif
  #if BUTTON_EXISTS(LEFT)
    if (BUTTON_PRESSED(LEFT)) {}
  #endif
  #if BUTTON_EXISTS(RIGHT)
    if (BUTTON_PRESSED(RIGHT)) {}
  #endif

}

#ifdef __PLAT_NATIVE_SIM__
  #include <lv_misc/lv_log.h>
  typedef void (*lv_log_print_g_cb_t)(lv_log_level_t level, const char *, uint32_t, const char *);
  extern "C" void lv_log_register_print_cb(lv_log_print_g_cb_t print_cb) {}
#endif

#endif // HAS_TFT_LVGL_UI
