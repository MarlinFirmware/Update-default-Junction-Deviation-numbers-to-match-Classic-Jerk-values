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
#pragma once

/**
 * BRICOLEMON LITE Board. Based on atsamd51 (AGCM4), bootloader and credits by ADAFRUIT.
 * This board its a 3.3V LOGIC Board, following the ADAFRUIT example, all of the board is open source.
 * Schematic: Refer to the Bricolemon
 * 3DSTEP: https://github.com/bricogeek/bricolemon/blob/master/Documentacion/Bricolemon%20Lite/LC_BG_002_PCB_V1I4.step
 * PinDemux: https://github.com/bricogeek/bricolemon/blob/master/Documentacion/Bricolemon/PinDEMUX.xlsx
 *
 * NOTE: We need the Serial port on the -1 to make it work!!. Remember to change it on configuration.h #define SERIAL_PORT -1
 */

#if NOT_TARGET(ARDUINO_GRAND_CENTRAL_M4)
  #error "Oops! Select 'Adafruit Grand Central M4' in 'Tools > Board.'"
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "BRICOLEMON LITE V1.0" // , Lemoncrest & BricoGeek collaboration.
#endif

/**
 * EEPROM EMULATION: Works with some bugs already, but the board needs an I2C EEPROM memory soldered on.
 */
//#define FLASH_EEPROM_EMULATION
#define I2C_EEPROM                                // EEPROM on I2C-0
#define MARLIN_EEPROM_SIZE              0x10000U  // 64K (CAT24C512)

// This is another option to emulate an EEPROM, but it's more efficient to not lose the data in the first place.
//#define SDCARD_EEPROM_EMULATION

//
// BLTOUCH PIN: This pin is the signal pin for the BLTOUCH sensor.
//
#define SERVO0_PIN                            33  // BLTouch PWM

//
// Limit Switches
//
#define X_STOP_PIN                            10
#define Y_STOP_PIN                            11
#define Z_STOP_PIN                            12

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     12
#endif

//
// Steppers
//
#define X_STEP_PIN                             3
#define X_DIR_PIN                             22
#define X_ENABLE_PIN                          26

#define Y_STEP_PIN                             4
#define Y_DIR_PIN                             23
#define Y_ENABLE_PIN                          27

#define Z_STEP_PIN                             5
#define Z_DIR_PIN                             24
#define Z_ENABLE_PIN                          28

#define E0_STEP_PIN                            2
#define E0_DIR_PIN                            25
#define E0_ENABLE_PIN                         29

// Filament runout. You may choose to use this pin for some other purpose. It's a normal GPIO that can be configured as I/O.
// For example, a switch to detect any kind of behavior, Power supply pin .... etc.
#define FIL_RUNOUT_PIN                        32

//
// Temperature Sensors
//
#define TEMP_0_PIN                             1
#define TEMP_BED_PIN                           3

//
// Heaters / Fans
//
#define HEATER_0_PIN                           6
#define HEATER_BED_PIN                         7
#define FAN0_PIN                               8
#define FAN1_PIN                               9

//
// LCD / Controller
//

/**
 * Bricolemon Expansion connectors
 *
 *               ------                    ------
 *        VCC   | 1  2 | GND        KILL  | 1  2 | GND
 *        LCD7  | 3  4 | LCDD6      RESET | 3  4 | SD_DETECT
 *        LCD5  | 5  6   LCDD4      MOSI  | 5  6   BTN_EN1
 *        LCDRS | 7  8 | LCDDE      SS    | 7  8 | BTN_EN2
 *  BTN_ENCODER | 9 10 | BEEPER     SCLK  | 9 10 | MISO
 *               ------                    ------
 *                EXP1                      EXP2
 *
 *- This extra connector is for a serial display like the MKS TFT.
 *               MKS TFT
 *               ------
 *        RX0   | .  . | NC
 *        TX0   | .  .   NC
 *        GND   | .  .   GND
 *        5B    | .  . | 5V
 *               ------
 *
 *- Special mapping of EXP1 to work with Ender displays.
 *
 *               ------
 *       BEEPER | 1  2 | ENC
 *          EN1 | 3  4 | RESET
 *          EN2   5  6 | LCD_D4
 *       LCD_RS | 7  8 | LCD_EN
 *          GND | 9 10 | 5V
 *               ------
 *                EXP1
 *
 *- Digital pinout reference of the Bricolemon for advanced users/configurations.
 *
 *               ------                    ------
 *          D41 | 1  2 | D40          D50 | 1  2 | D52
 *          D35 | 3  4 | D34          D43 | 3  4 | D53
 *          D36   5  6 | D37          D42   5  6 | D51
 *          D38 | 7  8 | D39          D44 | 7  8 | RST
 *          GND | 9 10 | VCC          GND | 9 10 | D49
 *               ------                    ------
 *                EXP1                      EXP2
 *
 *- Pin table overview
 *    LCD---                   SPI---
 *      LCD4        36           MISO        50
 *      LCD5        37           SS          53
 *      LCD6        38           SCKL        52
 *      LCD7        39           MOSI        51
 *      LCDRS       34         BEEPER
 *      LCDDE       35           BEEP        41
 *    ENCODER---               SD-CARD---
 *      BTN_EN1     42
 *      BTN_EN2     43           SD_DETECT   44
 *      BTN_ENCODER 40         KILL_PIN      49
 */

#define EXP1_01_PIN                           41
#define EXP1_02_PIN                           40
#define EXP1_03_PIN                           35
#define EXP1_04_PIN                           34
#define EXP1_05_PIN                           36
#define EXP1_06_PIN                           37
#define EXP1_07_PIN                           38
#define EXP1_08_PIN                           39

#define EXP2_01_PIN                           50
#define EXP2_02_PIN                           52
#define EXP2_03_PIN                           43
#define EXP2_04_PIN                           53
#define EXP2_05_PIN                           42
#define EXP2_06_PIN                           51
#define EXP2_07_PIN                           44
#define EXP2_10_PIN                           49

#if ENABLED(CR10_STOCKDISPLAY)
  #define EXP3_01_PIN                EXP1_01_PIN
  #define EXP3_02_PIN                EXP1_02_PIN
  #define EXP3_03_PIN                EXP1_03_PIN
  #define EXP3_04_PIN                EXP1_04_PIN
  #define EXP3_05_PIN                EXP1_05_PIN
  #define EXP3_06_PIN                EXP1_06_PIN
  #define EXP3_07_PIN                EXP1_07_PIN
  #define EXP3_08_PIN                EXP1_08_PIN
#endif

/************************************/
/***** Configurations Section  ******/
/************************************/

/**
 * This section starts with the pins_RAMPS_144.h as example, after if you need any new
 * display, you could use normal duponts and connect it with with the scheme showed before.
 * Tested:
 *   - Ender-3 Old display (Character LCD)
 *   - Ender-3 New Serial DWING Display
 *   - Reprap Display
 *   - Ender-5 New Serial Display
 *   - Any Reprap character display like
 */

#if HAS_WIRED_LCD

  //
  // LCD Display output pins
  //
  #if HAS_DWIN_E3V2 || IS_DWIN_MARLINUI

    #if LCD_SERIAL_PORT != 1
      #error "LCD_SERIAL_PORT should be 1 for this display."
    #endif

    // DWIN Encoder
    #define BTN_ENC                  EXP1_02_PIN
    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_04_PIN

    #ifndef BEEPER_PIN
      #define BEEPER_PIN             EXP1_01_PIN
      #define NO_SPEAKER
    #endif

  #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

    // TO TEST
    //#define LCD_PINS_RS            EXP2_10_PIN  // CS chip select /SS chip slave select
    //#define LCD_PINS_EN            EXP2_06_PIN  // SID (MOSI)
    //#define LCD_PINS_D4            EXP2_02_PIN  // SCK (CLK) clock

  #elif ALL(IS_NEWPANEL, PANEL_ONE)

    // TO TEST
    //#define LCD_PINS_RS            EXP1_02_PIN
    //#define LCD_PINS_EN            EXP2_05_PIN
    //#define LCD_PINS_D4                     57  // Mega/Due:65 - AGCM4:57
    //#define LCD_PINS_D5                     58  // Mega/Due:66 - AGCM4:58
    //#define LCD_PINS_D6            EXP2_07_PIN
    //#define LCD_PINS_D7                     56  // Mega/Due:64 - AGCM4:56

  #else

    #if ENABLED(CR10_STOCKDISPLAY)

      // TO TEST
      #define LCD_PINS_RS            EXP3_04_PIN
      #define LCD_PINS_EN            EXP3_03_PIN
      #define LCD_PINS_D4            EXP3_05_PIN

      #if !IS_NEWPANEL
        // TO TEST
        //#define BEEPER_PIN         EXP3_05_PIN
      #endif

    #elif ENABLED(ZONESTAR_LCD)

      // TO TEST
      //#define LCD_PINS_RS                   56  // Mega/Due:64 - AGCM4:56
      //#define LCD_PINS_EN          EXP2_07_PIN
      //#define LCD_PINS_D4                   55  // Mega/Due:63 - AGCM4:55
      //#define LCD_PINS_D5          EXP1_02_PIN
      //#define LCD_PINS_D6          EXP2_05_PIN
      //#define LCD_PINS_D7                   57  // Mega/Due:65 - AGCM4:57

    #else

      #if ANY(MKS_12864OLED, MKS_12864OLED_SSD1306)
        // TO TEST
        //#define LCD_PINS_DC                 25  // Set as output on init
        //#define LCD_PINS_RS                 27  // Pull low for 1s to init
        // DOGM SPI LCD Support
        //#define DOGLCD_CS                   16
        //#define DOGLCD_MOSI                 17
        //#define DOGLCD_SCK                  23
        //#define DOGLCD_A0          LCD_PINS_DC

      #else
        // Definitions for any standard Display
        #define LCD_PINS_RS          EXP1_04_PIN
        #define LCD_PINS_EN          EXP1_03_PIN
        #define LCD_PINS_D4          EXP1_05_PIN
        #define LCD_PINS_D5          EXP1_06_PIN
        #define LCD_PINS_D6          EXP1_07_PIN
      #endif

      #define LCD_PINS_D7            EXP1_08_PIN

      #if !IS_NEWPANEL
        #define BEEPER_PIN           EXP1_01_PIN
      #endif

    #endif

    #if !IS_NEWPANEL
      // Buttons attached to a shift register
      // Not wired yet
      //#define SHIFT_CLK_PIN        EXP1_07_PIN
      //#define SHIFT_LD_PIN         EXP2_05_PIN
      //#define SHIFT_OUT_PIN        EXP1_02_PIN
      //#define SHIFT_EN_PIN                  17
    #endif

  #endif

  //
  // LCD Display input pins
  //
  #if IS_NEWPANEL

    #if IS_RRD_SC

      #define BEEPER_PIN             EXP1_01_PIN

      #if ENABLED(CR10_STOCKDISPLAY)
        // TO TEST
        #define BTN_EN1              EXP3_08_PIN
        #define BTN_EN2              EXP3_06_PIN

      #else
        // Definitions for any standard Display
        #define BTN_EN1              EXP2_05_PIN
        #define BTN_EN2              EXP2_03_PIN
        #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
          #define BTN_ENC_EN         LCD_PINS_D7  // Detect the presence of the encoder
        #endif
      #endif

      #define BTN_ENC                EXP1_02_PIN
      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN        EXP2_07_PIN
      #endif
      #define KILL_PIN               EXP2_10_PIN

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        //#define LCD_BACKLIGHT_PIN  EXP1_08_PIN  // TO TEST
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

      // TO TEST
      //#define BTN_EN1                       56  // Mega/Due:64 - AGCM4:56
      //#define BTN_EN2                       72  // Mega/Due:59 - AGCM4:72
      //#define BTN_ENC                       55
      //#define SD_DETECT_PIN        EXP2_05_PIN

    #elif ENABLED(LCD_I2C_PANELOLU2)

      // TO TEST
      //#define BTN_EN1                       47
      //#define BTN_EN2              EXP2_03_PIN
      //#define BTN_ENC                       32
      //#define LCD_SDSS_PIN           SD_SS_PIN
      //#define KILL_PIN             EXP1_01_PIN
      //#undef LCD_PINS_EN                        // not used, causes false pin conflict report

    #elif ENABLED(LCD_I2C_VIKI)

      // TO TEST
      //#define BTN_EN1              EXP1_02_PIN  // https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf explains 40/42.
      //#define BTN_EN2              EXP2_05_PIN
      //#define BTN_ENC                       -1

      //#define LCD_SDSS_PIN           SD_SS_PIN
      //#define SD_DETECT_PIN        EXP2_10_PIN

    #elif ANY(VIKI2, miniVIKI)

      // TO TEST
      //#define DOGLCD_CS                     45
      //#define DOGLCD_A0            EXP2_07_PIN
      //#define LCD_SCREEN_ROT_180

      //#define BEEPER_PIN                    33
      //#define STAT_LED_RED_PIN              32
      //#define STAT_LED_BLUE_PIN    EXP1_03_PIN

      //#define BTN_EN1                       22
      //#define BTN_EN2                        7
      //#define BTN_ENC              EXP1_08_PIN

      //#define SD_DETECT_PIN                 -1  // Pin 49 for display SD interface, 72 for easy adapter board
      //#define KILL_PIN                      31

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)

      // TO TEST
      //#define DOGLCD_CS                     29
      //#define DOGLCD_A0                     27

      //#define BEEPER_PIN                    23
      //#define LCD_BACKLIGHT_PIN             33

      //#define BTN_EN1              EXP1_03_PIN
      //#define BTN_EN2              EXP1_06_PIN
      //#define BTN_ENC                       31

      //#define LCD_SDSS_PIN           SD_SS_PIN
      //#define SD_DETECT_PIN        EXP2_10_PIN
      //#define KILL_PIN             EXP1_01_PIN

    #elif ANY(MKS_MINI_12864, FYSETC_MINI_12864)

      // TO TEST
      //#define BEEPER_PIN           EXP1_06_PIN
      //#define BTN_ENC              EXP1_03_PIN
      //#define SD_DETECT_PIN        EXP2_10_PIN

      //#ifndef KILL_PIN
      //  #define KILL_PIN           EXP1_01_PIN
      //#endif

      #if ENABLED(MKS_MINI_12864)

        // TO TEST
        //#define DOGLCD_A0                   27
        //#define DOGLCD_CS                   25

        // GLCD features
        // Uncomment screen orientation
        //#define LCD_SCREEN_ROT_90
        //#define LCD_SCREEN_ROT_180
        //#define LCD_SCREEN_ROT_270

        // not connected to a pin
        //#define LCD_BACKLIGHT_PIN           57  // backlight LED on A11/D? (Mega/Due:65 - AGCM4:57)

        //#define BTN_EN1                     31
        //#define BTN_EN2                     33

      #elif ENABLED(FYSETC_MINI_12864)

        // From https://wiki.fysetc.com/Mini12864_Panel/

        // TO TEST
        //#define DOGLCD_A0                   16
        //#define DOGLCD_CS                   17

        //#define BTN_EN1                     33
        //#define BTN_EN2                     31

        //#define FORCE_SOFT_SPI                  // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

        //#define LCD_RESET_PIN               23  // Must be high or open for LCD to operate normally.

        #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
          #ifndef RGB_LED_R_PIN
            // TO TEST
            //#define RGB_LED_R_PIN           25
          #endif
          #ifndef RGB_LED_G_PIN
            // TO TEST
            //#define RGB_LED_G_PIN           27
          #endif
          #ifndef RGB_LED_B_PIN
            // TO TEST
            //#define RGB_LED_B_PIN           29
          #endif
        #elif ENABLED(FYSETC_MINI_12864_2_1)
          // TO TEST
          //#define NEOPIXEL_PIN              25
        #endif

      #endif

    #elif ENABLED(MINIPANEL)

      // TO TEST
      //#define BEEPER_PIN           EXP2_05_PIN
      // not connected to a pin
      //#define LCD_BACKLIGHT_PIN             57  // backlight LED on A11/D? (Mega/Due:65 - AGCM4:57)

      //#define DOGLCD_A0            EXP2_07_PIN
      //#define DOGLCD_CS                     58  // Mega/Due:66 - AGCM4:58

      // GLCD features
      // Uncomment screen orientation
      //#define LCD_SCREEN_ROT_90
      //#define LCD_SCREEN_ROT_180
      //#define LCD_SCREEN_ROT_270

      //#define BTN_EN1              EXP1_02_PIN
      //#define BTN_EN2                       55  // Mega/Due:63 - AGCM4:55
      //#define BTN_ENC                       72  // Mega/Due:59 - AGCM4:72

      //#define SD_DETECT_PIN        EXP2_10_PIN
      //#define KILL_PIN                      56  // Mega/Due:64 - AGCM4:56

    #elif ENABLED(ZONESTAR_LCD)

      // TO TEST
      //#define ADC_KEYPAD_PIN                12

    #elif ENABLED(AZSMZ_12864)

      // TO TEST

    #else

      // Beeper on AUX-4
      //#define BEEPER_PIN                    33

      // Buttons are directly attached to AUX-2
      #if IS_RRW_KEYPAD
        // TO TEST
        //#define SHIFT_OUT_PIN      EXP1_02_PIN
        //#define SHIFT_CLK_PIN      EXP2_07_PIN
        //#define SHIFT_LD_PIN       EXP2_05_PIN
        //#define BTN_EN1                     56  // Mega/Due:64 - AGCM4:56
        //#define BTN_EN2                     72  // Mega/Due:59 - AGCM4:72
        //#define BTN_ENC                     55  // Mega/Due:63 - AGCM4:55
      #elif ENABLED(PANEL_ONE)
        // TO TEST
        //#define BTN_EN1                     72  // AUX2 PIN 3 (Mega/Due:59 - AGCM4:72)
        //#define BTN_EN2                     55  // AUX2 PIN 4 (Mega/Due:63 - AGCM4:55)
        //#define BTN_ENC            EXP2_10_PIN  // AUX3 PIN 7
      #else
        // TO TEST
        //#define BTN_EN1            EXP1_06_PIN
        //#define BTN_EN2            EXP1_03_PIN
        //#define BTN_ENC                     31
      #endif

      #if ENABLED(G3D_PANEL)
        // TO TEST
        //#define SD_DETECT_PIN      EXP2_10_PIN
        //#define KILL_PIN           EXP1_01_PIN
      #endif

    #endif
  #endif // IS_NEWPANEL

#endif // HAS_WIRED_LCD

//
// SD Support
//

/**
 * Bricolemon has an SD slot, but to change it to your LCD or Custom, set the option to one of the following:
 *
 *           LCD - Use the SD drive in the external LCD controller.
 *       ONBOARD - Use the SD drive on the control board.
 *  CUSTOM_CABLE - Use a custom cable to access the SD (as defined in a pins file).
 *
 * :[ 'LCD', 'ONBOARD', 'CUSTOM_CABLE' ]
 */

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define SD_SS_PIN                           83
  #define SD_DETECT_PIN                       95
#else
  #define SD_SS_PIN                  EXP2_04_PIN
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *  It seems to work perfectly fine on Software Serial, if an advanced user wants to test, you could use the SAMD51 Serial1 and Serial 2. Be careful with the Sercom configurations.
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define E0_HARDWARE_SERIAL Serial1

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                  0
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                  1
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                  2
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                 3
  #endif
  static_assert(X_SLAVE_ADDRESS == 0, "X_SLAVE_ADDRESS must be 0 for BOARD_BRICOLEMON_LITE_V1_0.");
  static_assert(Y_SLAVE_ADDRESS == 1, "Y_SLAVE_ADDRESS must be 1 for BOARD_BRICOLEMON_LITE_V1_0.");
  static_assert(Z_SLAVE_ADDRESS == 2, "Z_SLAVE_ADDRESS must be 2 for BOARD_BRICOLEMON_LITE_V1_0.");
  static_assert(E0_SLAVE_ADDRESS == 3, "E0_SLAVE_ADDRESS must be 3 for BOARD_BRICOLEMON_LITE_V1_0.");

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200 // 250000
  #endif

  //
  // Software serial
  //
  #define X_SERIAL_TX_PIN                      0
  #define X_SERIAL_RX_PIN                      1

  #define Y_SERIAL_TX_PIN        X_SERIAL_TX_PIN
  #define Y_SERIAL_RX_PIN        X_SERIAL_RX_PIN

  #define Z_SERIAL_TX_PIN        X_SERIAL_TX_PIN
  #define Z_SERIAL_RX_PIN        X_SERIAL_RX_PIN

  #define E0_SERIAL_TX_PIN       X_SERIAL_TX_PIN
  #define E0_SERIAL_RX_PIN       X_SERIAL_RX_PIN

#endif
