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
 * Conditionals-2-LCD.h
 * Conditionals that need to be set before Configuration_adv.h or pins.h
 */

//========================================================
// Get requirements for the benefit of IntelliSense, etc.
//
#include "Conditionals-1-axes.h"
//========================================================

// Support for SD Card and other file storage
#if ENABLED(SDSUPPORT)
  #define HAS_MEDIA 1
#endif
#if ENABLED(MULTI_VOLUME)
  #define HAS_MULTI_VOLUME 1
#endif
#if ENABLED(USB_FLASH_DRIVE_SUPPORT)
  #define HAS_USB_FLASH_DRIVE 1
#endif

//
// Serial Port Info
//
#ifdef SERIAL_PORT_2
  #define HAS_MULTI_SERIAL 1
  #ifdef SERIAL_PORT_3
    #define NUM_SERIAL 3
  #else
    #define NUM_SERIAL 2
  #endif
#elif defined(SERIAL_PORT)
  #define NUM_SERIAL 1
#else
  #define NUM_SERIAL 0
  #undef BAUD_RATE_GCODE
#endif
#if SERIAL_PORT == -1 || SERIAL_PORT_2 == -1 || SERIAL_PORT_3 == -1
  #define HAS_USB_SERIAL 1
#endif
#ifdef RS485_SERIAL_PORT
  #define HAS_RS485_SERIAL 1
#endif
#if SERIAL_PORT_2 == -2
  #define HAS_ETHERNET 1
#endif

/**
 * Conditionals for the configured LCD / Controller
 */

// MKS_LCD12864A/B is a variant of MKS_MINI_12864
#if ANY(MKS_LCD12864A, MKS_LCD12864B)
  #define MKS_MINI_12864
#endif

// MKS_MINI_12864_V3 , BTT_MINI_12864 and BEEZ_MINI_12864 are nearly identical to FYSETC_MINI_12864_2_1
#if ANY(MKS_MINI_12864_V3, BTT_MINI_12864, BEEZ_MINI_12864)
  #define FYSETC_MINI_12864_2_1
#endif

// Old settings are now conditional on DGUS_LCD_UI
#if DGUS_UI_IS(ORIGIN)
  #define DGUS_LCD_UI_ORIGIN 1
#elif DGUS_UI_IS(FYSETC)
  #define DGUS_LCD_UI_FYSETC 1
#elif DGUS_UI_IS(HIPRECY)
  #define DGUS_LCD_UI_HIPRECY 1
#elif DGUS_UI_IS(MKS)
  #define DGUS_LCD_UI_MKS 1
#elif DGUS_UI_IS(RELOADED)
  #define DGUS_LCD_UI_RELOADED 1
#elif DGUS_UI_IS(IA_CREALITY)
  #define DGUS_LCD_UI_IA_CREALITY 1
#elif DGUS_UI_IS(E3S1PRO)
  #define DGUS_LCD_UI_E3S1PRO 1
#endif

/**
 * General Flags that may be set below by specific LCDs
 *
 *  DOGLCD                  : Run a Graphical LCD through U8GLib (with MarlinUI)
 *  IS_ULTIPANEL            : Define LCD_PINS_D5/6/7 for direct-connected "Ultipanel" LCDs
 *  HAS_WIRED_LCD           : Ultra LCD, not necessarily Ultipanel.
 *  IS_RRD_SC               : Common RRD Smart Controller digital interface pins
 *  IS_RRD_FG_SC            : Common RRD Full Graphical Smart Controller digital interface pins
 *  IS_U8GLIB_ST7920        : Most common DOGM display SPI interface, supporting a "lightweight" display mode.
 *  U8GLIB_SH1106           : SH1106 OLED with I2C interface via U8GLib
 *  IS_U8GLIB_SSD1306       : SSD1306 OLED with I2C interface via U8GLib (U8GLIB_SSD1306)
 *  U8GLIB_SSD1309          : SSD1309 OLED with I2C interface via U8GLib (HAS_U8GLIB_I2C_OLED, HAS_WIRED_LCD, DOGLCD)
 *  IS_U8GLIB_ST7565_64128N : ST7565 128x64 LCD with SPI interface via U8GLib
 *  IS_U8GLIB_LM6059_AF     : LM6059 with Hardware SPI via U8GLib
 */
#if ANY(MKS_MINI_12864, ENDER2_STOCKDISPLAY)

  #define MINIPANEL

#elif ENABLED(YHCB2004)

  #define IS_ULTIPANEL 1

#elif ENABLED(CARTESIO_UI)

  #define DOGLCD
  #define IS_ULTIPANEL 1

#elif ANY(DWIN_MARLINUI_PORTRAIT, DWIN_MARLINUI_LANDSCAPE)

  #define IS_DWIN_MARLINUI 1
  #define IS_ULTIPANEL 1

#elif ENABLED(ZONESTAR_LCD)

  #define HAS_ADC_BUTTONS 1
  #define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0
  #define ADC_KEY_NUM 8
  #define IS_ULTIPANEL 1

  // This helps to implement HAS_ADC_BUTTONS menus
  #define REVERSE_MENU_DIRECTION
  #define STD_ENCODER_PULSES_PER_STEP 1
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
  #define ENCODER_FEEDRATE_DEADZONE 2

#elif ENABLED(ZONESTAR_12864LCD)
  #define DOGLCD
  #define IS_RRD_SC 1
  #define IS_U8GLIB_ST7920 1

#elif ENABLED(ZONESTAR_12864OLED)
  #define IS_RRD_SC 1
  #define U8GLIB_SH1106_SPI

#elif ENABLED(ZONESTAR_12864OLED_SSD1306)
  #define IS_RRD_SC 1
  #define U8GLIB_SSD1306_SPI

#elif ENABLED(RADDS_DISPLAY)
  #define IS_ULTIPANEL 1
  #define STD_ENCODER_PULSES_PER_STEP 2

#elif ANY(miniVIKI, VIKI2, WYH_L12864, ELB_FULL_GRAPHIC_CONTROLLER, AZSMZ_12864, EMOTION_TECH_LCD)

  #define DOGLCD
  #define IS_DOGM_12864 1
  #define IS_ULTIPANEL 1

  #if ENABLED(miniVIKI)
    #define IS_U8GLIB_ST7565_64128N 1
  #elif ENABLED(VIKI2)
    #define IS_U8GLIB_ST7565_64128N 1
  #elif ENABLED(WYH_L12864)
    #define IS_U8GLIB_ST7565_64128N 1
    #define ST7565_XOFFSET 0x04
  #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
    #define IS_U8GLIB_LM6059_AF 1
  #elif ENABLED(AZSMZ_12864)
    #define IS_U8GLIB_ST7565_64128N 1
  #elif ENABLED(EMOTION_TECH_LCD)
    #define IS_U8GLIB_ST7565_64128N 1
    #define ST7565_VOLTAGE_DIVIDER_VALUE 0x07
  #endif

#elif ENABLED(OLED_PANEL_TINYBOY2)

  #define IS_U8GLIB_SSD1306
  #define IS_ULTIPANEL 1

#elif ENABLED(RA_CONTROL_PANEL)

  #define LCD_I2C_TYPE_PCA8574
  #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
  #define IS_ULTIPANEL 1

#elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

  #define DOGLCD
  #define IS_U8GLIB_ST7920 1
  #define IS_ULTIPANEL 1
  #define ENCODER_PULSES_PER_STEP 2

#elif ENABLED(MKS_12864OLED)

  #define IS_RRD_SC 1
  #define U8GLIB_SH1106

#elif ENABLED(MKS_12864OLED_SSD1306)

  #define IS_RRD_SC 1
  #define IS_U8GLIB_SSD1306

#elif ENABLED(SAV_3DGLCD)

  #if ENABLED(U8GLIB_SSD1306)
    #define IS_U8GLIB_SSD1306 // Allow for U8GLIB_SSD1306 + SAV_3DGLCD
  #endif
  #define IS_NEWPANEL 1

#elif ENABLED(FYSETC_242_OLED_12864)

  #define IS_RRD_SC 1
  #define U8GLIB_SH1106

  #ifndef NEOPIXEL_BRIGHTNESS
    #define NEOPIXEL_BRIGHTNESS 127
  #endif

#elif ANY(FYSETC_MINI_12864_X_X, FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0, FYSETC_MINI_12864_2_1, FYSETC_GENERIC_12864_1_1)

  #define FYSETC_MINI_12864
  #define DOGLCD
  #define IS_ULTIPANEL 1
  #define LED_COLORS_REDUCE_GREEN

  // Require LED backlighting enabled
  #if ENABLED(FYSETC_MINI_12864_2_1)
    #ifndef NEOPIXEL_BRIGHTNESS
      #define NEOPIXEL_BRIGHTNESS 127
    #endif
    //#define NEOPIXEL_STARTUP_TEST
  #endif

#elif ENABLED(ULTI_CONTROLLER)

  #define IS_ULTIPANEL 1
  #define U8GLIB_SSD1309
  #define LCD_RESET_PIN LCD_PINS_D6 //  This controller need a reset pin
  #define STD_ENCODER_PULSES_PER_STEP 4
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
  #ifndef PCA9632
    #define PCA9632
  #endif

#elif ENABLED(MAKEBOARD_MINI_2_LINE_DISPLAY_1602)

  #define IS_RRD_SC 1
  #define LCD_WIDTH 16
  #define LCD_HEIGHT 2

#elif ANY(TFTGLCD_PANEL_SPI, TFTGLCD_PANEL_I2C)

  #define IS_TFTGLCD_PANEL 1
  #define IS_ULTIPANEL 1                    // Note that IS_ULTIPANEL leads to HAS_WIRED_LCD

  #if HAS_MEDIA && DISABLED(LCD_PROGRESS_BAR)
    #define LCD_PROGRESS_BAR
  #endif
  #if ENABLED(TFTGLCD_PANEL_I2C)
    #define LCD_I2C_ADDRESS           0x33  // Must be 0x33 for STM32 main boards and equal to panel's I2C slave address
  #endif
  #define LCD_USE_I2C_BUZZER                // Enable buzzer on LCD, used for both I2C and SPI buses (LiquidTWI2 not required)
  #define STD_ENCODER_PULSES_PER_STEP 2
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
  #define LCD_WIDTH                   20    // 20 or 24 chars in line
  #define LCD_HEIGHT                  10    // Character lines
  #define LCD_CONTRAST_MIN            127
  #define LCD_CONTRAST_MAX            255
  #define LCD_CONTRAST_DEFAULT        250
  #define CONVERT_TO_EXT_ASCII        // Use extended 128-255 symbols from ASCII table.
                                      // At this time present conversion only for cyrillic - bg, ru and uk languages.
                                      // First 7 ASCII symbols in panel font must be replaced with Marlin's special symbols.

#elif ENABLED(CR10_STOCKDISPLAY)

  #define IS_RRD_FG_SC 1
  #define NO_LCD_SDCARD
  #define LCD_ST7920_DELAY_1           125
  #define LCD_ST7920_DELAY_2           125
  #define LCD_ST7920_DELAY_3           125

#elif ANY(ANET_FULL_GRAPHICS_LCD, CTC_A10S_A13)

  #define IS_RRD_FG_SC 1
  #define LCD_ST7920_DELAY_1           150
  #define LCD_ST7920_DELAY_2           150
  #define LCD_ST7920_DELAY_3           150

#elif ANY(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER, BQ_LCD_SMART_CONTROLLER, K3D_FULL_GRAPHIC_SMART_CONTROLLER)

  #define IS_RRD_FG_SC 1

#elif ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)

  #define IS_RRD_SC 1   // RepRapDiscount LCD or Graphical LCD with rotary click encoder

#elif ENABLED(K3D_242_OLED_CONTROLLER)

  #define IS_RRD_SC 1
  #define U8GLIB_SSD1309

#endif

#if ANY(FYSETC_MINI_12864, MKS_MINI_12864)
  #define U8G_SPI_USE_MODE_3 1
#endif

// ST7920-based graphical displays
#if ANY(IS_RRD_FG_SC, LCD_FOR_MELZI, SILVER_GATE_GLCD_CONTROLLER)
  #define DOGLCD
  #define IS_U8GLIB_ST7920 1
  #define IS_RRD_SC 1
#endif

// ST7565 / 64128N graphical displays
#if ANY(MAKRPANEL, MINIPANEL)
  #define IS_ULTIPANEL 1
  #define DOGLCD
  #if ENABLED(MAKRPANEL)
    #define IS_U8GLIB_ST7565_64128N 1
  #endif
#endif

#if ENABLED(IS_U8GLIB_SSD1306)
  #define U8GLIB_SSD1306
#endif

#if ENABLED(OVERLORD_OLED)
  #define IS_ULTIPANEL 1
  #define U8GLIB_SH1106
  /**
   * PCA9632 for buzzer and LEDs via i2c
   * No auto-inc, red and green leds switched, buzzer
   */
  #define PCA9632
  #define PCA9632_NO_AUTO_INC
  #define PCA9632_GRN         0x00
  #define PCA9632_RED         0x02
  #define PCA9632_BUZZER
  #define PCA9632_BUZZER_DATA { 0x09, 0x02 }

  #define STD_ENCODER_PULSES_PER_STEP     1 // Overlord uses buttons
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
#endif

// 128x64 I2C OLED LCDs (SSD1306 / SSD1309 / SH1106)
// ...and 128x64 SPI OLED LCDs (SSD1306 / SH1106)
#if ANY(U8GLIB_SSD1306, U8GLIB_SSD1309, U8GLIB_SH1106)
  #define HAS_U8GLIB_I2C_OLED 1
#endif
#if ANY(HAS_U8GLIB_I2C_OLED, U8GLIB_SSD1306_SPI, U8GLIB_SH1106_SPI)
  #define HAS_WIRED_LCD 1
  #define DOGLCD
#endif

/**
 * SPI Ultipanels
 */

// Basic Ultipanel-like displays
#if ANY(ULTIMAKERCONTROLLER, IS_RRD_SC, G3D_PANEL, RIGIDBOT_PANEL, PANEL_ONE, U8GLIB_SH1106)
  #define IS_ULTIPANEL 1
#endif

// Einstart OLED has Cardinal nav via pins defined in pins_EINSTART-S.h
#if ENABLED(U8GLIB_SH1106_EINSTART)
  #define DOGLCD
  #define IS_ULTIPANEL 1
#endif

// TFT Legacy options masquerade as TFT_GENERIC
#if ANY(FSMC_GRAPHICAL_TFT, SPI_GRAPHICAL_TFT, TFT_320x240, TFT_480x320, TFT_320x240_SPI, TFT_480x320_SPI, TFT_LVGL_UI_FSMC, TFT_LVGL_UI_SPI)
  #define IS_LEGACY_TFT 1
  #define TFT_GENERIC
  #if ANY(FSMC_GRAPHICAL_TFT, TFT_320x240, TFT_480x320, TFT_LVGL_UI_FSMC)
    #define TFT_INTERFACE_FSMC
  #elif ANY(SPI_GRAPHICAL_TFT, TFT_320x240_SPI, TFT_480x320_SPI, TFT_LVGL_UI_SPI)
    #define TFT_INTERFACE_SPI
  #endif
  #if ANY(FSMC_GRAPHICAL_TFT, SPI_GRAPHICAL_TFT)
    #define TFT_CLASSIC_UI
  #elif ANY(TFT_320x240, TFT_480x320, TFT_320x240_SPI, TFT_480x320_SPI)
    #define TFT_COLOR_UI
  #elif ANY(TFT_LVGL_UI_FSMC, TFT_LVGL_UI_SPI)
    #define TFT_LVGL_UI
  #endif
#endif

// FSMC/SPI TFT Panels (LVGL) with encoder click wheel
#if ENABLED(TFT_LVGL_UI)
  #define HAS_TFT_LVGL_UI 1
  #define SERIAL_RUNTIME_HOOK 1
  #define STD_ENCODER_PULSES_PER_STEP 4
#endif

// FSMC/SPI TFT Panels
#if ENABLED(TFT_CLASSIC_UI)
  #define TFT_SCALED_DOGLCD 1
#endif

#if TFT_SCALED_DOGLCD
  #define DOGLCD
  #define IS_ULTIPANEL 1
  #define DELAYED_BACKLIGHT_INIT
#elif HAS_TFT_LVGL_UI
  #define DELAYED_BACKLIGHT_INIT
#endif

// Color UI
#if ENABLED(TFT_COLOR_UI)
  #define HAS_GRAPHICAL_TFT 1
  #define IS_ULTIPANEL 1
#endif

/**
 * I2C Panels
 */

#if ANY(IS_RRD_SC, IS_DOGM_12864, OLED_PANEL_TINYBOY2, LCD_I2C_PANELOLU2)

  #define STD_ENCODER_PULSES_PER_STEP 4
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1

  #if ENABLED(LCD_I2C_PANELOLU2)  // PANELOLU2 LCD with status LEDs, separate encoder and click inputs
    #define LCD_I2C_TYPE_MCP23017 // I2C Character-based 12864 display
    #define LCD_I2C_ADDRESS 0x20  // I2C Address of the port expander
    #define LCD_USE_I2C_BUZZER    // Enable buzzer on LCD (optional)
    #define IS_ULTIPANEL 1
  #endif

#elif ANY(LCD_SAINSMART_I2C_1602, LCD_SAINSMART_I2C_2004)

  #define LCD_I2C_TYPE_PCF8575    // I2C Character-based 12864 display
  #define LCD_I2C_ADDRESS 0x27    // I2C Address of the port expander
  #define IS_ULTIPANEL 1

  #if ENABLED(LCD_SAINSMART_I2C_2004)
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 4
  #endif

#elif ENABLED(LCD_I2C_VIKI)

  /**
   * Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
   *
   * This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
   * Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
   * Note: The pause/stop/resume LCD button pin should be connected to the Arduino
   *       BTN_ENC pin (or set BTN_ENC to -1 if not used)
   */
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER   // Enable buzzer on LCD (requires LiquidTWI2 v1.2.3 or later)
  #define IS_ULTIPANEL 1

  #define ENCODER_FEEDRATE_DEADZONE 4

  #define STD_ENCODER_PULSES_PER_STEP 1
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 2

#elif ENABLED(G3D_PANEL)

  #define STD_ENCODER_PULSES_PER_STEP 2
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1

#endif

#if ANY(LCD_I2C_TYPE_MCP23017, LCD_I2C_TYPE_MCP23008) && DISABLED(NO_LCD_DETECT)
  #define DETECT_I2C_LCD_DEVICE 1
#endif

/**
 * Ender-3 V2 DWIN with Encoder
 */
#if ANY(DWIN_CREALITY_LCD, DWIN_LCD_PROUI)
  #define HAS_DWIN_E3V2_BASIC 1
#endif
#if ANY(HAS_DWIN_E3V2_BASIC, DWIN_CREALITY_LCD_JYERSUI, SOVOL_SV06_RTS)
  #define HAS_DWIN_E3V2 1
  #define STD_ENCODER_PULSES_PER_STEP 4
#endif

// Encoder behavior
#ifndef STD_ENCODER_PULSES_PER_STEP
  #if ENABLED(TOUCH_SCREEN)
    #define STD_ENCODER_PULSES_PER_STEP 2
  #else
    #define STD_ENCODER_PULSES_PER_STEP 5
  #endif
#endif
#ifndef STD_ENCODER_STEPS_PER_MENU_ITEM
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
#endif
#ifndef ENCODER_PULSES_PER_STEP
  #define ENCODER_PULSES_PER_STEP STD_ENCODER_PULSES_PER_STEP
#endif
#ifndef ENCODER_STEPS_PER_MENU_ITEM
  #define ENCODER_STEPS_PER_MENU_ITEM STD_ENCODER_STEPS_PER_MENU_ITEM
#endif
#ifndef ENCODER_FEEDRATE_DEADZONE
  #define ENCODER_FEEDRATE_DEADZONE 6
#endif

/**
 * Shift register panels
 * ---------------------
 * 2 wire Non-latching LCD SR from:
 * https://github.com/fmalpartida/New-LiquidCrystal/wiki/schematics#user-content-ShiftRegister_connection
 */
#if ENABLED(FF_INTERFACEBOARD)
  #define SR_LCD_3W_NL    // Non latching 3 wire shift register
  #define IS_ULTIPANEL 1
#elif ENABLED(SAV_3DLCD)
  #define SR_LCD_2W_NL    // Non latching 2 wire shift register
  #define IS_ULTIPANEL 1
#elif ENABLED(ULTIPANEL)
  #define IS_ULTIPANEL 1
#endif

#if ANY(IS_ULTIPANEL, ULTRA_LCD)
  #define HAS_WIRED_LCD 1
#endif

#if ANY(IS_ULTIPANEL, REPRAPWORLD_KEYPAD)
  #define IS_NEWPANEL 1
#endif

#if ANY(ZONESTAR_LCD, REPRAPWORLD_KEYPAD)
  #define IS_RRW_KEYPAD 1
  #ifndef REPRAPWORLD_KEYPAD_MOVE_STEP
    #define REPRAPWORLD_KEYPAD_MOVE_STEP 1.0
  #endif
#endif

// Aliases for LCD features
#if !DGUS_UI_IS(NONE) || ENABLED(ANYCUBIC_LCD_VYPER)
  #define HAS_DGUS_LCD 1
  #if DGUS_UI_IS(ORIGIN, FYSETC, HIPRECY, MKS)
    #define HAS_DGUS_LCD_CLASSIC 1
  #endif
#endif

// Extensible UI serial touch screens. (See src/lcd/extui)
#if ANY(HAS_DGUS_LCD, MALYAN_LCD, ANYCUBIC_LCD_I3MEGA, ANYCUBIC_LCD_CHIRON, NEXTION_TFT, TOUCH_UI_FTDI_EVE, DWIN_LCD_PROUI)
  #define IS_EXTUI 1 // Just for sanity check.
  #define EXTENSIBLE_UI
#endif

// E3V2 extras
#if ANY(HAS_DWIN_E3V2, IS_DWIN_MARLINUI, SOVOL_SV06_RTS)
  #define SERIAL_CATCHALL 0
  #define HAS_LCD_BRIGHTNESS 1
  #define LCD_BRIGHTNESS_MAX 250
#endif

#if ENABLED(DWIN_LCD_PROUI)
  #define DO_LIST_BIN_FILES 1
  #define LCD_BRIGHTNESS_DEFAULT 127
  #define STATUS_DO_CLEAR_EMPTY
#endif

// Serial Controllers require LCD_SERIAL_PORT
#if ANY(IS_DWIN_MARLINUI, HAS_DWIN_E3V2, HAS_DGUS_LCD, MALYAN_LCD, ANYCUBIC_LCD_I3MEGA, ANYCUBIC_LCD_CHIRON, NEXTION_TFT, SOVOL_SV06_RTS)
  #define LCD_IS_SERIAL_HOST 1
#endif

#if HAS_WIRED_LCD
  #if ENABLED(DOGLCD)
    #define HAS_MARLINUI_U8GLIB 1
  #elif IS_TFTGLCD_PANEL
    // Neither DOGM nor HD44780. Fully customized interface.
  #elif IS_DWIN_MARLINUI
    // Since HAS_MARLINUI_U8GLIB refers to U8G displays
    // the DWIN display can define its own flags
  #elif !HAS_GRAPHICAL_TFT
    #define HAS_MARLINUI_HD44780 1
  #endif
#endif

#if ANY(HAS_WIRED_LCD, EXTENSIBLE_UI, HAS_DWIN_E3V2)
  /**
   * HAS_DISPLAY indicates the display uses these MarlinUI methods...
   *  - update
   *  - abort_print
   *  - pause_print
   *  - resume_print
   *  - poweroff        (for PSU_CONTROL and HAS_MARLINUI_MENU)
   *
   *  ...and implements these MarlinUI methods:
   *  - init_lcd
   *  - clear_lcd
   *  - clear_for_drawing
   *  - zoffset_overlay (if BABYSTEP_GFX_OVERLAY or MESH_EDIT_GFX_OVERLAY are supported)
   *  - draw_kill_screen
   *  - kill_screen
   *  - draw_status_message
   *    (calling advance_status_scroll, status_and_len for a scrolling status message)
   */
  #define HAS_DISPLAY 1
  #define HAS_UI_UPDATE 1
  #define HAS_STATUS_MESSAGE 1
#endif

#if HAS_WIRED_LCD && !HAS_GRAPHICAL_TFT && !IS_DWIN_MARLINUI
  #define HAS_LCDPRINT 1
#endif

#if HAS_DISPLAY || HAS_LCDPRINT
  #define HAS_UTF8_UTILS 1
#endif

#if IS_ULTIPANEL && DISABLED(NO_LCD_MENUS)
  #define HAS_MARLINUI_MENU 1
#endif

#if ANY(HAS_MARLINUI_MENU, EXTENSIBLE_UI, HAS_DWIN_E3V2)
  #define HAS_MANUAL_MOVE_MENU 1
#endif

#if HAS_MARLINUI_U8GLIB
  #ifndef LCD_PIXEL_WIDTH
    #define LCD_PIXEL_WIDTH 128
  #endif
  #ifndef LCD_PIXEL_HEIGHT
    #define LCD_PIXEL_HEIGHT 64
  #endif
#endif

/**
 * Unused LCD options
 */
#if NONE(HAS_MARLINUI_HD44780, IS_DWIN_MARLINUI, IS_TFTGLCD_PANEL) && !MB(SIMULATED)
  #undef DISPLAY_CHARSET_HD44780
#endif
#if !HAS_MARLINUI_HD44780
  #undef LCD_INFO_SCREEN_STYLE
#endif
#if NONE(HAS_MARLINUI_U8GLIB, HAS_TFT_LVGL_UI, TFT_COLOR_UI, DGUS_LCD_UI_E3S1PRO)
  #undef LCD_LANGUAGE
#endif
#if DISABLED(MPC_AUTOTUNE)
  #undef MPC_AUTOTUNE_MENU
#endif

// Slim menu optimizations
#if ENABLED(SLIM_LCD_MENUS)
  #define BOOT_MARLIN_LOGO_SMALL
#endif

// Flow and feedrate editing
#if HAS_EXTRUDERS && ANY(HAS_MARLINUI_MENU, DWIN_CREALITY_LCD, DWIN_LCD_PROUI, MALYAN_LCD, TOUCH_SCREEN)
  #define HAS_FLOW_EDIT 1
#endif
#if ANY(HAS_MARLINUI_MENU, ULTIPANEL_FEEDMULTIPLY, DWIN_CREALITY_LCD, DWIN_LCD_PROUI, MALYAN_LCD, TOUCH_SCREEN)
  #define HAS_FEEDRATE_EDIT 1
#endif

/**
 * TFT Displays
 *
 * Configure parameters for TFT displays:
 *  - TFT_DEFAULT_ORIENTATION
 *  - TFT_DRIVER
 *  - TFT_WIDTH
 *  - TFT_HEIGHT
 *  - TFT_INTERFACE_(SPI|FSMC)
 *  - TFT_COLOR
 *  - GRAPHICAL_TFT_UPSCALE
 */
#if ANY(MKS_TS35_V2_0, BTT_TFT35_SPI_V1_0)                                    // ST7796
  #define TFT_DEFAULT_DRIVER ST7796
  #define TFT_DEFAULT_ORIENTATION TFT_EXCHANGE_XY
  #define TFT_RES_480x320
  #define TFT_INTERFACE_SPI
  #define NO_LCD_SDCARD
#elif ANY(LERDGE_TFT35, ANET_ET5_TFT35)                                       // ST7796
  #define TFT_DEFAULT_ORIENTATION TFT_EXCHANGE_XY
  #define TFT_RES_480x320
  #define TFT_INTERFACE_FSMC
#elif ANY(ANET_ET4_TFT28, MKS_ROBIN_TFT24, MKS_ROBIN_TFT28, MKS_ROBIN_TFT32)  // ST7789
  #define TFT_DEFAULT_ORIENTATION (TFT_EXCHANGE_XY | TFT_INVERT_Y)
  #define TFT_RES_320x240
  #define TFT_INTERFACE_FSMC
#elif ANY(MKS_ROBIN_TFT35, TFT_TRONXY_X5SA, ANYCUBIC_TFT35)                   // ILI9488
  #define TFT_DRIVER ILI9488
  #define TFT_DEFAULT_ORIENTATION (TFT_EXCHANGE_XY | TFT_INVERT_X | TFT_INVERT_Y)
  #define TFT_RES_480x320
  #define TFT_INTERFACE_FSMC
#elif ENABLED(MKS_ROBIN_TFT43)
  #define TFT_DRIVER SSD1963
  #define TFT_DEFAULT_ORIENTATION 0
  #define TFT_RES_480x272
  #define TFT_INTERFACE_FSMC
#elif ANY(MKS_ROBIN_TFT_V1_1R, LONGER_LK_TFT28)                               // ILI9328 or R61505
  #define TFT_DEFAULT_ORIENTATION (TFT_EXCHANGE_XY | TFT_INVERT_X | TFT_INVERT_Y)
  #define TFT_RES_320x240
  #define TFT_INTERFACE_FSMC
#elif ENABLED(BIQU_BX_TFT70)                                                  // RGB
  #define TFT_DEFAULT_ORIENTATION TFT_EXCHANGE_XY
  #define TFT_RES_1024x600
  #define TFT_INTERFACE_LTDC
  #if ENABLED(TOUCH_SCREEN)
    #define TFT_TOUCH_DEVICE_GT911
  #endif
#elif ENABLED(TFT_GENERIC)
  #define TFT_DEFAULT_ORIENTATION (TFT_EXCHANGE_XY | TFT_INVERT_X | TFT_INVERT_Y)
  #if NONE(TFT_RES_320x240, TFT_RES_480x272, TFT_RES_480x320, TFT_RES_1024x600)
    #define TFT_RES_320x240
  #endif
  #if NONE(TFT_INTERFACE_FSMC, TFT_INTERFACE_SPI)
    #define TFT_INTERFACE_SPI
  #endif
#endif

// FSMC/SPI TFT Panels using standard HAL/tft/tft_(fsmc|spi|ltdc).h
#if ENABLED(TFT_INTERFACE_FSMC)
  #define HAS_FSMC_TFT 1
  #if TFT_SCALED_DOGLCD
    #define HAS_FSMC_GRAPHICAL_TFT 1
  #elif HAS_TFT_LVGL_UI
    #define HAS_TFT_LVGL_UI_FSMC 1
  #endif
#elif ENABLED(TFT_INTERFACE_SPI)
  #define HAS_SPI_TFT 1
  #if TFT_SCALED_DOGLCD
    #define HAS_SPI_GRAPHICAL_TFT 1
  #elif HAS_TFT_LVGL_UI
    #define HAS_TFT_LVGL_UI_SPI 1
  #endif
#elif ENABLED(TFT_INTERFACE_LTDC)
  #define HAS_LTDC_TFT 1
  #if TFT_SCALED_DOGLCD
    #define HAS_LTDC_GRAPHICAL_TFT 1
  #elif HAS_TFT_LVGL_UI
    #define HAS_TFT_LVGL_UI_LTDC 1
  #endif
#endif

#if ANY(HAS_SPI_TFT, HAS_FSMC_TFT, HAS_LTDC_TFT)
  #include "../lcd/tft_io/tft_orientation.h" // for TFT_COLOR_UI_PORTRAIT
#endif

#if ENABLED(TFT_RES_320x240)
  #if ENABLED(TFT_COLOR_UI_PORTRAIT)
    #define TFT_WIDTH  240
    #define TFT_HEIGHT 320
  #else
    #define TFT_WIDTH  320
    #define TFT_HEIGHT 240
  #endif
  #define GRAPHICAL_TFT_UPSCALE 2
#elif ENABLED(TFT_RES_480x272)
  #define TFT_WIDTH  480
  #define TFT_HEIGHT 272
  #define GRAPHICAL_TFT_UPSCALE 2
#elif ENABLED(TFT_RES_480x320)
  #if ENABLED(TFT_COLOR_UI_PORTRAIT)
    #define TFT_WIDTH  320
    #define TFT_HEIGHT 480
  #else
    #define TFT_WIDTH  480
    #define TFT_HEIGHT 320
  #endif
  #define GRAPHICAL_TFT_UPSCALE 3
#elif ENABLED(TFT_RES_1024x600)
  #define TFT_WIDTH  1024
  #define TFT_HEIGHT 600
  #if ENABLED(TOUCH_SCREEN)
    #define GRAPHICAL_TFT_UPSCALE 6
    #define TFT_PIXEL_OFFSET_X 120
  #else
    #define GRAPHICAL_TFT_UPSCALE 8
    #define TFT_PIXEL_OFFSET_X 0
  #endif
#endif

#if ENABLED(TFT_COLOR_UI)
  #if (TFT_WIDTH == 320 && TFT_HEIGHT == 240) || (TFT_WIDTH == 240 && TFT_HEIGHT == 320)
    #if ENABLED(TFT_INTERFACE_SPI)
      #define TFT_320x240_SPI
    #elif ENABLED(TFT_INTERFACE_FSMC)
      #define TFT_320x240
    #endif
  #elif TFT_HEIGHT == 320 || (TFT_HEIGHT == 480 && ENABLED(TFT_COLOR_UI_PORTRAIT))
    #if ENABLED(TFT_INTERFACE_SPI)
      #define TFT_480x320_SPI
    #elif ENABLED(TFT_INTERFACE_FSMC)
      #define TFT_480x320
    #endif
  #elif TFT_HEIGHT == 272
    #if ENABLED(TFT_INTERFACE_SPI)
      #define TFT_480x272_SPI
    #elif ENABLED(TFT_INTERFACE_FSMC)
      #define TFT_480x272
    #endif
  #elif TFT_HEIGHT == 600
    #if ENABLED(TFT_INTERFACE_LTDC)
      #define TFT_1024x600_LTDC
    #else
      #define TFT_1024x600_SIM  // "Simulation" - for testing purposes only
    #endif
  #endif
#endif

#if ANY(TFT_320x240, TFT_320x240_SPI)
  #define HAS_UI_320x240 1
#elif ANY(TFT_480x320, TFT_480x320_SPI)
  #define HAS_UI_480x320 1
#elif ANY(TFT_480x272, TFT_480x272_SPI)
  #define HAS_UI_480x272 1
#elif ANY(TFT_1024x600_LTDC, TFT_1024x600_SIM)
  #define HAS_UI_1024x600 1
#endif

// Number of text lines the screen can display (may depend on font used)
// Touch screens leave space for extra buttons at the bottom
#if ANY(HAS_UI_320x240, HAS_UI_480x272)
  #if ENABLED(TFT_COLOR_UI_PORTRAIT)
    #define LCD_HEIGHT TERN(TOUCH_SCREEN, 8, 9)
  #else
    #define LCD_HEIGHT TERN(TOUCH_SCREEN, 6, 7)
  #endif
#elif HAS_UI_480x320
  #if ENABLED(TFT_COLOR_UI_PORTRAIT)
    #define LCD_HEIGHT TERN(TOUCH_SCREEN, 9, 10)
  #else
    #define LCD_HEIGHT TERN(TOUCH_SCREEN, 6, 7)
  #endif
#elif HAS_UI_1024x600
  #define LCD_HEIGHT TERN(TOUCH_SCREEN, 12, 13)
#endif

// This emulated DOGM has 'touch/xpt2046', not 'tft/xpt2046'
#if ENABLED(TOUCH_SCREEN)
  #if NONE(TFT_TOUCH_DEVICE_GT911, TFT_TOUCH_DEVICE_XPT2046)
    #define TFT_TOUCH_DEVICE_XPT2046          // ADS7843/XPT2046 ADC Touchscreen such as ILI9341 2.8
  #endif
  #if !HAS_GRAPHICAL_TFT
    #undef TOUCH_SCREEN
    #if ENABLED(TFT_CLASSIC_UI)
      #define HAS_TOUCH_BUTTONS 1
      #if ENABLED(TFT_TOUCH_DEVICE_GT911)
        #define HAS_CAP_TOUCH_BUTTONS 1
      #else
        #define HAS_RES_TOUCH_BUTTONS 1
      #endif
    #endif
  #endif
#endif

#if  (HAS_X_AXIS && X_HOME_DIR) || (HAS_Y_AXIS && Y_HOME_DIR) || (HAS_Z_AXIS && Z_HOME_DIR) \
  || (HAS_I_AXIS && I_HOME_DIR) || (HAS_J_AXIS && J_HOME_DIR) || (HAS_K_AXIS && K_HOME_DIR) \
  || (HAS_U_AXIS && U_HOME_DIR) || (HAS_V_AXIS && V_HOME_DIR) || (HAS_W_AXIS && W_HOME_DIR)
  #define HAS_ENDSTOPS 1
  #define COORDINATE_OKAY(N,L,H) WITHIN(N,L,H)
#else
  #define COORDINATE_OKAY(N,L,H) true
#endif

/**
 * LED Backlight INDEX END
 */
#if defined(NEOPIXEL_BKGD_INDEX_FIRST) && !defined(NEOPIXEL_BKGD_INDEX_LAST)
  #define NEOPIXEL_BKGD_INDEX_LAST NEOPIXEL_BKGD_INDEX_FIRST
#endif

#if LED_POWEROFF_TIMEOUT > 0
  #define HAS_LED_POWEROFF_TIMEOUT 1
#endif
