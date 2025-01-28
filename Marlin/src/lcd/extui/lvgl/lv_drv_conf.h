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

/**
 * @file lv_drv_conf.h
 * Configuration file for v7.9.1
 */
#pragma once

#include "lv_conf.h"
#include "../../../inc/MarlinConfigPre.h"

/*********************
 * DELAY INTERFACE
 *********************/
#define LV_DRV_DELAY_INCLUDE  <stdint.h>            // Dummy include by default
#define LV_DRV_DELAY_US(us)  /*delay_us(us)*/       // Delay the given number of microseconds
#define LV_DRV_DELAY_MS(ms)  /*delay_ms(ms)*/       // Delay the given number of milliseconds

/*********************
 * DISPLAY INTERFACE
 *********************/

/*------------
 *  Common
 *------------*/
#define LV_DRV_DISP_INCLUDE         <stdint.h>           // Dummy include by default
#define LV_DRV_DISP_CMD_DATA(val)  /*pin_x_set(val)*/    // Set the command/data pin to 'val'
#define LV_DRV_DISP_RST(val)       /*pin_x_set(val)*/    // Set the reset pin to 'val'

/*---------
 *  SPI
 *---------*/
#define LV_DRV_DISP_SPI_CS(val)          /*spi_cs_set(val)*/     // Set the SPI's Chip select to 'val'
#define LV_DRV_DISP_SPI_WR_BYTE(data)    /*spi_wr(data)*/        // Write a byte the SPI bus
#define LV_DRV_DISP_SPI_WR_ARRAY(adr, n) /*spi_wr_mem(adr, n)*/  // Write 'n' bytes to SPI bus from 'adr'

/*------------------
 *  Parallel port
 *-----------------*/
#define LV_DRV_DISP_PAR_CS(val)          /*par_cs_set(val)*/   // Set the Parallel port's Chip select to 'val'
#define LV_DRV_DISP_PAR_SLOW             /*par_slow()*/        // Set low speed on the parallel port
#define LV_DRV_DISP_PAR_FAST             /*par_fast()*/        // Set high speed on the parallel port
#define LV_DRV_DISP_PAR_WR_WORD(data)    /*par_wr(data)*/      // Write a word to the parallel port
#define LV_DRV_DISP_PAR_WR_ARRAY(adr, n) /*par_wr_mem(adr,n)*/ // Write 'n' bytes to Parallel ports from 'adr'

/***************************
 * INPUT DEVICE INTERFACE
 ***************************/

/*----------
 *  Common
 *----------*/
#define LV_DRV_INDEV_INCLUDE     <stdint.h>             // Dummy include by default
#define LV_DRV_INDEV_RST(val)    /*pin_x_set(val)*/     // Set the reset pin to 'val'
#define LV_DRV_INDEV_IRQ_READ    0 /*pn_x_read()*/      // Read the IRQ pin

/*---------
 *  SPI
 *---------*/
#define LV_DRV_INDEV_SPI_CS(val)            /*spi_cs_set(val)*/     // Set the SPI's Chip select to 'val'
#define LV_DRV_INDEV_SPI_XCHG_BYTE(data)    0 /*spi_xchg(val)*/     // Write 'val' to SPI and give the read value

/*---------
 *  I2C
 *---------*/
#define LV_DRV_INDEV_I2C_START              /*i2c_start()*/       // Make an I2C start
#define LV_DRV_INDEV_I2C_STOP               /*i2c_stop()*/        // Make an I2C stop
#define LV_DRV_INDEV_I2C_RESTART            /*i2c_restart()*/     // Make an I2C restart
#define LV_DRV_INDEV_I2C_WR(data)           /*i2c_wr(data)*/      // Write a byte to the I1C bus
#define LV_DRV_INDEV_I2C_READ(last_read)    0 /*i2c_rd()*/        // Read a byte from the I2C bud

/*********************
 *  DISPLAY DRIVERS
 *********************/

/*-------------------
 *  Monitor of PC
 *-------------------*/
#if ENABLED(SDL_CONTROLLER)
  #define USE_MONITOR         1
#endif

#if USE_MONITOR
  #define MONITOR_HOR_RES     TFT_WIDTH
  #define MONITOR_VER_RES     TFT_HEIGHT

  // Scale window by this factor (useful when simulating small screens)
  #define MONITOR_ZOOM        1

/* Used to test true double buffering with only address changing.
 * Set LV_VDB_SIZE = (LV_HOR_RES * LV_VER_RES) and  LV_VDB_DOUBLE = 1 and LV_COLOR_DEPTH = 32" */
  #define MONITOR_DOUBLE_BUFFERED 0

  #define MONITOR_SDL_INCLUDE_PATH   <SDL2/SDL.h>

  // Open two windows to test multi display support
  #define MONITOR_DUAL            0

  // Display Api
  #include <SDL2/SDL.h>
  #include <display/monitor.h>
  #define DISPLAY_INIT monitor_init
  #define DISPLAY_FLUSH monitor_flush
#endif

/*-----------------------------------
 *  Native Windows (including mouse)
 *----------------------------------*/
#ifndef USE_WINDOWS
  #define USE_WINDOWS       0
#endif

#if USE_WINDOWS
  #define WINDOW_HOR_RES      480
  #define WINDOW_VER_RES      320
#endif

/*----------------------------------------
 *  GTK drivers (monitor, mouse, keyboard
 *---------------------------------------*/
#ifndef USE_GTK
  #define USE_GTK       0
#endif

/*----------------
 *    SSD1963
 *--------------*/
#ifndef USE_SSD1963
  #define USE_SSD1963         0
#endif

#if USE_SSD1963
  #define SSD1963_HOR_RES     LV_HOR_RES
  #define SSD1963_VER_RES     LV_VER_RES
  #define SSD1963_HT          531
  #define SSD1963_HPS         43
  #define SSD1963_LPS         8
  #define SSD1963_HPW         10
  #define SSD1963_VT          288
  #define SSD1963_VPS         12
  #define SSD1963_FPS         4
  #define SSD1963_VPW         10
  #define SSD1963_HS_NEG      0   // Negative hsync
  #define SSD1963_VS_NEG      0   // Negative vsync
  #define SSD1963_ORI         0   // 0, 90, 180, 270
  #define SSD1963_COLOR_DEPTH 16
#endif

/*----------------
 *    R61581
 *--------------*/
#ifndef USE_R61581
  #define USE_R61581          0
#endif

#if USE_R61581
  #define R61581_HOR_RES      LV_HOR_RES
  #define R61581_VER_RES      LV_VER_RES
  #define R61581_HSPL         0       // HSYNC signal polarity
  #define R61581_HSL          10      // HSYNC length (Not Implemented)
  #define R61581_HFP          10      // Horitontal Front poarch (Not Implemented)
  #define R61581_HBP          10      // Horitontal Back poarch (Not Implemented
  #define R61581_VSPL         0       // VSYNC signal polarity
  #define R61581_VSL          10      // VSYNC length (Not Implemented)
  #define R61581_VFP          8       // Vertical Front poarch
  #define R61581_VBP          8       // Vertical Back poarch
  #define R61581_DPL          0       // DCLK signal polarity
  #define R61581_EPL          1       // ENABLE signal polarity
  #define R61581_ORI          0       // 0, 180
  #define R61581_LV_COLOR_DEPTH 16    // Fix 16 bit
#endif

/*------------------------------
 *  ST7565 (Monochrome, low res.)
 *-----------------------------*/
#ifndef USE_ST7565
  #define USE_ST7565          0
#endif

#if USE_ST7565
  // No settings
#endif

/*------------------------------
 *  GC9A01 (color, low res.)
 *-----------------------------*/
#ifndef USE_GC9A01
  #define USE_GC9A01          0
#endif

#if USE_GC9A01
  // No settings
#endif

/*------------------------------------------
 *  UC1610 (4 gray 160*[104|128])
 *  (EA DOGXL160 160x104 tested)
 *-----------------------------------------*/
#ifndef USE_UC1610
  #define USE_UC1610          0
#endif

#if USE_UC1610
  #define UC1610_HOR_RES         LV_HOR_RES
  #define UC1610_VER_RES         LV_VER_RES
  #define UC1610_INIT_CONTRAST   33   // init contrast, values in [%]
  #define UC1610_INIT_HARD_RST   0    // 1 : hardware reset at init, 0 : software reset
  #define UC1610_TOP_VIEW        0    // 0 : Bottom View, 1 : Top View
#endif

/*-------------------------------------------------
 *  SHARP memory in pixel monochrome display series
 *      LS012B7DD01 (184x38  pixels.)
 *      LS013B7DH03 (128x128 pixels.)
 *      LS013B7DH05 (144x168 pixels.)
 *      LS027B7DH01 (400x240 pixels.) (tested)
 *      LS032B7DD02 (336x536 pixels.)
 *      LS044Q7DH01 (320x240 pixels.)
 *------------------------------------------------*/
#ifndef USE_SHARP_MIP
  #define USE_SHARP_MIP       0
#endif

#if USE_SHARP_MIP
  #define SHARP_MIP_HOR_RES             LV_HOR_RES
  #define SHARP_MIP_VER_RES             LV_VER_RES
  #define SHARP_MIP_SOFT_COM_INVERSION  0
  #define SHARP_MIP_REV_BYTE(b)         /*((uint8_t) __REV(__RBIT(b)))*/  // Architecture / compiler dependent byte bits order reverse
#endif

/*-------------------------------------------------
 *  ILI9341 240X320 TFT LCD
 *------------------------------------------------*/
#ifndef USE_ILI9341
  #define USE_ILI9341       0
#endif

#if USE_ILI9341
  #define ILI9341_HOR_RES       LV_HOR_RES
  #define ILI9341_VER_RES       LV_VER_RES
  #define ILI9341_GAMMA         1
  #define ILI9341_TEARING       0
#endif

/*-----------------------------------------
 *  Linux frame buffer device (/dev/fbx)
 *-----------------------------------------*/
#ifndef USE_FBDEV
  #define USE_FBDEV           0
#endif

#if USE_FBDEV
  #define FBDEV_PATH          "/dev/fb0"
#endif

/*-----------------------------------------
 *  FreeBSD frame buffer device (/dev/fbx)
 *.........................................*/
#ifndef USE_BSD_FBDEV
  #define USE_BSD_FBDEV		0
#endif

#if USE_BSD_FBDEV
 #define FBDEV_PATH		"/dev/fb0"
#endif

/*-----------------------------------------
 *  DRM/KMS device (/dev/dri/cardX)
 *-----------------------------------------*/
#ifndef USE_DRM
  #define USE_DRM           0
#endif

#if USE_DRM
  #define DRM_CARD          "/dev/dri/card0"
  #define DRM_CONNECTOR_ID  -1	// -1 for the first connected one
#endif

/*********************
 *  INPUT DEVICES
 *********************/

/*--------------
 *    XPT2046
 *--------------*/
#ifndef USE_XPT2046
  #define USE_XPT2046         0
#endif

#if USE_XPT2046
  #define XPT2046_HOR_RES     480
  #define XPT2046_VER_RES     320
  #define XPT2046_X_MIN       200
  #define XPT2046_Y_MIN       200
  #define XPT2046_X_MAX       3800
  #define XPT2046_Y_MAX       3800
  #define XPT2046_AVG         4
  #define XPT2046_X_INV       0
  #define XPT2046_Y_INV       0
  #define XPT2046_XY_SWAP     0
#endif

/*-----------------
 *    FT5406EE8
 *-----------------*/
#ifndef USE_FT5406EE8
  #define USE_FT5406EE8       0
#endif

#if USE_FT5406EE8
 #define FT5406EE8_I2C_ADR   0x38                  // 7 bit address
#endif

/*---------------
 *  AD TOUCH
 *--------------*/
#ifndef USE_AD_TOUCH
  #define USE_AD_TOUCH        0
#endif

#if USE_AD_TOUCH
  // No settings
#endif

/*---------------------------------------
 * Mouse or touchpad on PC (using SDL)
 *-------------------------------------*/

#if ENABLED(SDL_MOUSE_INPUT)
  #define USE_MOUSE           1
  // Pointer Input Api
  #include <indev/mouse.h>
  #define POINTER_INIT mouse_init
  #define POINTER_READ mouse_read
#endif

#if USE_MOUSE
  // No settings
#endif

/*-------------------------------------------
 * Mousewheel as encoder on PC (using SDL)
 *------------------------------------------*/
#if ENABLED(SDL_MOUSEWHEEL_ENCODER)
  #define USE_MOUSEWHEEL      1
  // Rotary Input Api
  #include <indev/mousewheel.h>
  #define ROTARY_INIT mousewheel_init
  #define ROTARY_READ mousewheel_read
#endif

#if USE_MOUSEWHEEL
  // No settings
#endif

/*-------------------------------------------------
 * Touchscreen as libinput interface (for Linux based systems)
 *------------------------------------------------*/
#ifndef USE_LIBINPUT
  #define USE_LIBINPUT           0
#endif

#if USE_LIBINPUT
  #define LIBINPUT_NAME   "/dev/input/event0"        // You can use the "evtest" Linux tool to get the list of devices and test them
#endif

/*-------------------------------------------------
 * Mouse or touchpad as evdev interface (for Linux based systems)
 *------------------------------------------------*/
#ifndef USE_EVDEV
  #define USE_EVDEV           0
#endif

#ifndef USE_BSD_EVDEV
  #define USE_BSD_EVDEV       0
#endif

#if USE_EVDEV || USE_BSD_EVDEV
  #define EVDEV_NAME   "/dev/input/event0"        // You can use the "evtest" Linux tool to get the list of devices and test them
  #define EVDEV_SWAP_AXES         0               // Swap the x and y axes of the touchscreen

  #define EVDEV_CALIBRATE         0               // Scale and offset the touchscreen coordinates by using maximum and minimum values for each axis

  #if EVDEV_CALIBRATE
    #define EVDEV_HOR_MIN         0               // to invert axis swap EVDEV_XXX_MIN by EVDEV_XXX_MAX
    #define EVDEV_HOR_MAX      4096               // "evtest" Linux tool can help to get the correct calibraion values>
    #define EVDEV_VER_MIN         0
    #define EVDEV_VER_MAX      4096
  #endif
#endif

/*-------------------------------
 *   Keyboard of a PC (using SDL)
 *------------------------------*/
#if ENABLED(SDL_KEYBOARD_INPUT)
  #define USE_KEYBOARD        1
  #include <indev/keyboard.h>
  #define KEYBOARD_INIT keyboard_init
  #define KEYBOARD_READ keyboard_read
#endif

#if USE_KEYBOARD
  // No settings
#endif
