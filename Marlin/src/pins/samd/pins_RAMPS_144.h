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
 * AGCM4 with RAMPS v1.4.4 pin assignments
 */

#if NOT_TARGET(ARDUINO_GRAND_CENTRAL_M4)
  #error "Oops! Select 'Adafruit Grand Central M4' in 'Tools > Board.'"
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "AGCM4 RAMPS 1.4.4"
#endif

//
// Servos
//
#define SERVO0_PIN                            11
#define SERVO1_PIN                             6
#define SERVO2_PIN                             5
#define SERVO3_PIN                             4

//
// EEPROM
//
//#define QSPI_EEPROM                             // Use AGCM4 onboard QSPI EEPROM (Uses 4K of RAM)
#define I2C_EEPROM                                // EEPROM on I2C-0
#define MARLIN_EEPROM_SIZE                0x8000  // 32K (24lc256)

//
// Foam Cutter requirements
//

#if ENABLED(FOAMCUTTER_XYUV)
  #define MOSFET_C_PIN                        -1
  #if HAS_CUTTER && !defined(SPINDLE_LASER_ENA_PIN) && NUM_SERVOS < 2
    #define SPINDLE_LASER_PWM_PIN              8  // Hardware PWM
  #endif
  #define Z_MIN_PIN                           -1
  #define Z_MAX_PIN                           -1
  #define I_STOP_PIN                          18
  #define J_STOP_PIN                          19
#endif

//
// Limit Switches
//
#define X_MIN_PIN                              3
#define X_MAX_PIN                              2
#define Y_MIN_PIN                             14
#define Y_MAX_PIN                             15
#define Z_MIN_PIN                             18
#define Z_MAX_PIN                             19

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     18
#endif

//
// Steppers
//
#define X_STEP_PIN                            67  // Mega/Due:54 - AGCM4:67
#define X_DIR_PIN                             68  // Mega/Due:55 - AGCM4:68
#define X_ENABLE_PIN                          38
#ifndef X_CS_PIN
  #define X_CS_PIN                            47
#endif

#define Y_STEP_PIN                            73  // Mega/Due:60 - AGCM4:73
#define Y_DIR_PIN                             74  // Mega/Due:61 - AGCM4:74
#define Y_ENABLE_PIN                          69  // Mega/Due:56 - AGCM4:69
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            45
#endif

#define Z_STEP_PIN                            46
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          54  // Mega/Due:62 - AGCM4:54
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            32
#endif

#define E0_STEP_PIN                           26
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         24
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           43
#endif

#define E1_STEP_PIN                           36
#define E1_DIR_PIN                            34
#define E1_ENABLE_PIN                         30
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           22
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                            13
#if TEMP_SENSOR_BED
  #define TEMP_BED_PIN                        14
#else
  #define TEMP_1_PIN                          14
#endif
#define TEMP_CHAMBER_PIN                      15

//
// Heaters / Fans
//
#define HEATER_0_PIN                          10
#if TEMP_SENSOR_BED
  #define HEATER_BED_PIN                       8
#else
  #define HEATER_1_PIN                         8
#endif
#define FAN0_PIN                               9
#define FAN1_PIN                               7
#define FAN2_PIN                              12

//
// Misc. Functions
//
#define LED_PIN                               13

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                         5  // Analog Input on AUX2
#endif

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      70
#endif

#ifndef PS_ON_PIN
  #define PS_ON_PIN                           39
#endif

#if ENABLED(CASE_LIGHT_ENABLE) && !defined(CASE_LIGHT_PIN) && !defined(SPINDLE_LASER_ENA_PIN)
  #if NUM_SERVOS <= 1                             // Prefer the servo connector
    #define CASE_LIGHT_PIN                     6  // Hardware PWM
  #endif
#endif

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER && !defined(SPINDLE_LASER_ENA_PIN)
  #if !NUM_SERVOS                                 // Use servo connector if possible
    #define SPINDLE_LASER_PWM_PIN              6  // Hardware PWM
    #define SPINDLE_LASER_ENA_PIN              4  // Pullup or pulldown!
    #define SPINDLE_DIR_PIN                    5
  #else
    #error "No auto-assignable Spindle/Laser pins available."
  #endif
#endif

//
// TMC SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                        58  // Mega/Due:66 - AGCM4:58
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                        44
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                         56  // Mega/Due:64 - AGCM4:56
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1

  #ifndef X_SERIAL_TX_PIN
    #define X_SERIAL_TX_PIN                   47
  #endif
  #ifndef X2_SERIAL_TX_PIN
    #define X2_SERIAL_TX_PIN                  -1
  #endif

  #ifndef Y_SERIAL_TX_PIN
    #define Y_SERIAL_TX_PIN                   45
  #endif
  #ifndef Y2_SERIAL_TX_PIN
    #define Y2_SERIAL_TX_PIN                  -1
  #endif

  #ifndef Z_SERIAL_TX_PIN
    #define Z_SERIAL_TX_PIN                   32
  #endif
  #ifndef Z2_SERIAL_TX_PIN
    #define Z2_SERIAL_TX_PIN                  22
  #endif

  #ifndef E0_SERIAL_TX_PIN
    #define E0_SERIAL_TX_PIN                  43
  #endif
  #ifndef E1_SERIAL_TX_PIN
    #define E1_SERIAL_TX_PIN                  -1
  #endif
#endif

//
// SD Support
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define SDSS                                83
  #undef SD_DETECT_PIN
  #define SD_DETECT_PIN                       95
#else
  #define SDSS                                53
#endif

//
// Průša i3 MK2 Multiplexer Support
//
#if HAS_PRUSA_MMU1
  #ifndef E_MUX0_PIN
    #define E_MUX0_PIN                   AUX2_06  // Z_CS_PIN
  #endif
  #ifndef E_MUX1_PIN
    #define E_MUX1_PIN                   AUX2_08  // E0_CS_PIN
  #endif
  #ifndef E_MUX2_PIN
    #define E_MUX2_PIN                   AUX2_07  // E1_CS_PIN
  #endif
#endif

//
// AUX1   VCC  GND D2  D1
//          2   4   6   8
//          1   3   5   7
//        VCC  GND A3  A4
//
#define AUX1_05                               57  // (A3)
#define AUX1_06                                2
#define AUX1_07                               58  // (A4)
#define AUX1_08                                1

//
// AUX2    GND A9 D40 D42 A11
//          2   4   6   8  10
//          1   3   5   7   9
//         VCC A5 A10 D44 A12
//
#define AUX2_03                               59  // (A5)
#define AUX2_04                               63  // (A9)
#define AUX2_05                               64  // (A10)
#define AUX2_06                               40
#define AUX2_07                               44
#define AUX2_08                               42
#define AUX2_09                               66  // (A12)
#define AUX2_10                               65  // (A11)

//
// AUX3
//                   SCK  MISO
//         RST  GND  D52   D50  VCC
//          9    7    5    3    1
//         10    8    6    4    2
//         NC   5V   D53   D51  D49
//                        MOSI
//
#define AUX3_02                               49
#define AUX3_03                               50
#define AUX3_04                               51
#define AUX3_05                               52
#define AUX3_06                               53

//
// AUX4   VCC GND D32 D47 D45 D43 D41 D39 D37 D35 D33 D31 D29 D27 D25 D23 D17 D16
//
#define AUX4_03                               32
#define AUX4_04                               47
#define AUX4_05                               45
#define AUX4_06                               43
#define AUX4_07                               41
#define AUX4_08                               39
#define AUX4_09                               37
#define AUX4_10                               35
#define AUX4_11                               33
#define AUX4_12                               31
#define AUX4_13                               29
#define AUX4_14                               27
#define AUX4_15                               25
#define AUX4_16                               23
#define AUX4_17                               17
#define AUX4_18                               16

/**
 * LCD adapters come in different variants. The socket keys can be
 * on either side, and may be backwards on some boards / displays.
 */
#ifndef EXP1_08_PIN

  #define EXP1_03_PIN                    AUX4_17
  #define EXP1_04_PIN                    AUX4_18
  #define EXP1_05_PIN                    AUX4_16
  #define EXP1_06_PIN                    AUX4_15
  #define EXP1_07_PIN                    AUX4_14
  #define EXP1_08_PIN                    AUX4_13

  #define EXP2_01_PIN                    AUX3_03
  #define EXP2_02_PIN                    AUX3_05
  #define EXP2_04_PIN                    AUX3_06
  #define EXP2_06_PIN                    AUX3_04
  #define EXP2_07_PIN                    AUX3_02

  #if ENABLED(G3D_PANEL)
    /**                  Gadgets3D Smart Adapter
     *              ------                        ------
     *        4-11 | 1  2 | 4-12     (MISO) 3-03 | 1  2 | 3-05 (SCK)
     *        4-17 | 3  4 | 4-18            4-10 | 3  4 | 3-06
     *        4-16   5  6 | 4-15            4-09   5  6 | 3-04 (MOSI)
     *        4-14 | 7  8 | 4-13            3-02 | 7  8 | 4-07
     *  (GND) 4-02 | 9 10 | 4-01 (5V)         -- | 9 10 | --
     *              ------                        ------
     *               EXP1                          EXP2
     */
    #define EXP1_01_PIN                  AUX4_11
    #define EXP1_02_PIN                  AUX4_12

    #define EXP2_03_PIN                  AUX4_10
    #define EXP2_05_PIN                  AUX4_09
    #define EXP2_08_PIN                  AUX4_07

  #else

    /**                     Smart Adapter (c) RRD
     *             ------                           ------
     *       4-09 | 1  2 | 4-10        (MISO) 3-03 | 1  2 | 3-05 (SCK)
     *       4-17 | 3  4 | 4-18               4-12 | 3  4 | 3-06
     *       4-16   5  6 | 4-15               4-11   5  6 | 3-04 (MOSI)
     *       4-14 | 7  8 | 4-13               3-02 | 7  8 | 4-07
     * (GND) 3-07 | 9 10 | 3-01 (5V)    (GND) 3-07 | 9 10 | --
     *             ------                           ------
     *              EXP1                             EXP2
     */
    #define EXP1_01_PIN                  AUX4_09
    #define EXP1_02_PIN                  AUX4_10

    #if ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)
      #define EXP2_03_PIN                AUX4_11
      #define EXP2_05_PIN                AUX4_12
      #define EXP2_08_PIN                     -1  // RESET
    #else
      #define EXP2_03_PIN                AUX4_12
      #define EXP2_05_PIN                AUX4_11
      #define EXP2_08_PIN                AUX4_07
    #endif

  #endif

#endif

//
// LCD / Controller
//

#ifdef LCD_PINS_DEFINED

  // LCD pins already defined by including header

#elif HAS_WIRED_LCD

  //#define LCD_SCREEN_ROTATE                180  // 0, 90, 180, 270

  //
  // LCD Display output pins
  //
  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

    #define LCD_PINS_RS              EXP2_07_PIN  // CS chip select /SS chip slave select
    #define LCD_PINS_EN              EXP2_06_PIN  // SID (MOSI)
    #define LCD_PINS_D4              EXP2_02_PIN  // SCK (CLK) clock

  #elif ALL(IS_NEWPANEL, PANEL_ONE)

    #define LCD_PINS_RS                  AUX2_06
    #define LCD_PINS_EN                  AUX2_08
    #define LCD_PINS_D4                  AUX2_10
    #define LCD_PINS_D5                  AUX2_09
    #define LCD_PINS_D6                  AUX2_07
    #define LCD_PINS_D7                  AUX2_05

  #elif ENABLED(TFTGLCD_PANEL_SPI)

    #define TFTGLCD_CS               EXP2_05_PIN

  #else

    #if ENABLED(CR10_STOCKDISPLAY)

      #define LCD_PINS_RS            EXP1_07_PIN
      #define LCD_PINS_EN            EXP1_08_PIN
      #define LCD_PINS_D4            EXP1_06_PIN

      #if !IS_NEWPANEL
        #define BEEPER_PIN           EXP1_01_PIN
      #endif

    #elif ENABLED(ZONESTAR_LCD)

      CONTROLLER_WARNING("RAMPS_144", "ZONESTAR_LCD", " Plugs into AUX2 but GND and 5V must be swapped.")

      #define LCD_PINS_RS                AUX2_05
      #define LCD_PINS_EN                AUX2_07
      #define LCD_PINS_D4                AUX2_04
      #define LCD_PINS_D5                AUX2_06
      #define LCD_PINS_D6                AUX2_08
      #define LCD_PINS_D7                AUX2_10

    #elif ENABLED(AZSMZ_12864)

      // TODO

    #else

      #if ANY(MKS_12864OLED, MKS_12864OLED_SSD1306)
        #define LCD_PINS_DC          EXP1_06_PIN  // Set as output on init
        #define LCD_PINS_RS          EXP1_07_PIN  // Pull low for 1s to init
        // DOGM SPI LCD Support
        #define DOGLCD_A0            LCD_PINS_DC
        #define DOGLCD_CS            EXP1_04_PIN
        #define DOGLCD_MOSI          EXP1_03_PIN
        #define DOGLCD_SCK           EXP1_05_PIN
      #else
        #define LCD_PINS_RS          EXP1_04_PIN
        #define LCD_PINS_EN          EXP1_03_PIN
        #define LCD_PINS_D4          EXP1_05_PIN
        #define LCD_PINS_D5          EXP1_06_PIN
        #define LCD_PINS_D6          EXP1_07_PIN
      #endif

      #define LCD_PINS_D7            EXP1_08_PIN

      #if !IS_NEWPANEL
        #define BEEPER_PIN           EXP2_05_PIN
      #endif

    #endif

    #if !IS_NEWPANEL
      // Buttons attached to a shift register
      // Not wired yet
      //#define SHIFT_CLK_PIN                 38
      //#define SHIFT_LD_PIN             AUX2_08
      //#define SHIFT_OUT_PIN            AUX2_06
      //#define SHIFT_EN_PIN         EXP1_03_PIN
    #endif

  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

  //
  // LCD Display input pins
  //
  #if IS_NEWPANEL

    #if IS_RRD_SC

      #define BEEPER_PIN             EXP1_01_PIN

      #if ENABLED(CR10_STOCKDISPLAY)
        #define BTN_EN1              EXP1_03_PIN
        #define BTN_EN2              EXP1_05_PIN
      #else
        #define BTN_EN1              EXP2_03_PIN
        #define BTN_EN2              EXP2_05_PIN
      #endif

      #define BTN_ENC                EXP1_02_PIN
      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN        EXP2_07_PIN
      #endif
      #ifndef KILL_PIN
        #define KILL_PIN             EXP2_08_PIN
      #endif

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        #define LCD_BACKLIGHT_PIN        AUX4_08  // Probably a slightly different adapter from RRD SC
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

      #define BTN_EN1                    AUX2_05
      #define BTN_EN2                    AUX2_03
      #define BTN_ENC                    AUX2_04
      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN            AUX2_08
      #endif

    #elif ENABLED(LCD_I2C_PANELOLU2)

      #define BTN_EN1                    AUX4_04
      #define BTN_EN2                    AUX4_06
      #define BTN_ENC                    AUX4_03
      #define LCD_SDSS                      SDSS
      #define KILL_PIN                   AUX4_07
      #undef LCD_PINS_EN                          // not used, causes false pin conflict report

    #elif ENABLED(LCD_I2C_VIKI)

      #define BTN_EN1                    AUX2_06  // https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf explains AUX2-06 and AUX2-08.
      #define BTN_EN2                    AUX2_08
      #define BTN_ENC                         -1

      #define LCD_SDSS                      SDSS
      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN        EXP2_07_PIN
      #endif

    #elif ANY(VIKI2, miniVIKI)

      #define DOGLCD_CS                  AUX4_05
      #define DOGLCD_A0                  AUX2_07
      #define LCD_SCREEN_ROTATE              180  // 0, 90, 180, 270

      #define BEEPER_PIN             EXP2_05_PIN
      #define STAT_LED_RED_PIN           AUX4_03
      #define STAT_LED_BLUE_PIN      EXP1_02_PIN

      #define BTN_EN1                         22
      #define BTN_EN2                          7
      #define BTN_ENC                    AUX4_08

      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN                 -1  // Pin 49 for display SD interface, 72 for easy adapter board
      #endif
      #define KILL_PIN               EXP2_03_PIN

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)

      #define DOGLCD_CS              EXP1_08_PIN
      #define DOGLCD_A0              EXP1_07_PIN

      #define BEEPER_PIN             EXP1_05_PIN
      #define LCD_BACKLIGHT_PIN      EXP2_05_PIN

      #define BTN_EN1                EXP1_02_PIN
      #define BTN_EN2                EXP1_01_PIN
      #define BTN_ENC                EXP2_03_PIN

      #define LCD_SDSS                      SDSS
      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN        EXP2_07_PIN
      #endif
      #define KILL_PIN               EXP2_08_PIN

    #elif ANY(MKS_MINI_12864, FYSETC_MINI_12864)

      #define BEEPER_PIN             EXP1_01_PIN
      #define BTN_ENC                EXP1_02_PIN
      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN        EXP2_07_PIN
      #endif

      #ifndef KILL_PIN
        #define KILL_PIN             EXP2_08_PIN
      #endif

      #if ENABLED(MKS_MINI_12864)

        #define DOGLCD_A0            EXP1_07_PIN
        #define DOGLCD_CS            EXP1_06_PIN

        // not connected to a pin
        #define LCD_BACKLIGHT_PIN             -1  // 65 (MKS mini12864 can't adjust backlight by software!)

        #define BTN_EN1              EXP2_03_PIN
        #define BTN_EN2              EXP2_05_PIN

      #elif ENABLED(FYSETC_MINI_12864)

        // From https://wiki.fysetc.com/Mini12864_Panel/

        #define DOGLCD_A0            EXP1_04_PIN
        #define DOGLCD_CS            EXP1_03_PIN

        #define BTN_EN1              EXP2_05_PIN
        #define BTN_EN2              EXP2_03_PIN

        //#define FORCE_SOFT_SPI                  // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

        #define LCD_RESET_PIN        EXP1_05_PIN  // Must be high or open for LCD to operate normally.

        #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
          #ifndef RGB_LED_R_PIN
            #define RGB_LED_R_PIN    EXP1_06_PIN
          #endif
          #ifndef RGB_LED_G_PIN
            #define RGB_LED_G_PIN    EXP1_07_PIN
          #endif
          #ifndef RGB_LED_B_PIN
            #define RGB_LED_B_PIN    EXP1_08_PIN
          #endif
        #elif ENABLED(FYSETC_MINI_12864_2_1)
          #define NEOPIXEL_PIN       EXP1_06_PIN
        #endif

      #endif

    #elif ENABLED(MINIPANEL)

      #define BEEPER_PIN                 AUX2_08
      #define LCD_BACKLIGHT_PIN          AUX2_10

      #define DOGLCD_A0                  AUX2_07
      #define DOGLCD_CS                  AUX2_09

      #define BTN_EN1                    AUX2_06
      #define BTN_EN2                    AUX2_04
      #define BTN_ENC                    AUX2_03

      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN            AUX3_02
      #endif
      #define KILL_PIN                   AUX2_05

    #elif ENABLED(ZONESTAR_LCD)

      #define ADC_KEYPAD_PIN                  12

    #elif ENABLED(AZSMZ_12864)

      // TODO

    #elif ENABLED(G3D_PANEL)

      #define BEEPER_PIN             EXP1_01_PIN

      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN        EXP2_07_PIN
      #endif
      #define KILL_PIN               EXP2_08_PIN

      #define BTN_EN1                EXP2_05_PIN
      #define BTN_EN2                EXP2_03_PIN
      #define BTN_ENC                EXP1_02_PIN

    #elif IS_TFTGLCD_PANEL

      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN        EXP2_07_PIN
      #endif

    #else

      #define BEEPER_PIN             EXP2_05_PIN

      #if ENABLED(PANEL_ONE)                      // Buttons connect directly to AUX-2
        #define BTN_EN1                  AUX2_03
        #define BTN_EN2                  AUX2_04
        #define BTN_ENC                  AUX3_02
      #else
        #define BTN_EN1              EXP1_01_PIN
        #define BTN_EN2              EXP1_02_PIN
        #define BTN_ENC              EXP2_03_PIN
      #endif

    #endif
  #endif // IS_NEWPANEL

#endif // HAS_WIRED_LCD && !LCD_PINS_DEFINED

#if IS_RRW_KEYPAD && !HAS_ADC_BUTTONS
  #define SHIFT_OUT_PIN                  AUX2_06
  #define SHIFT_CLK_PIN                  AUX2_07
  #define SHIFT_LD_PIN                   AUX2_08
  #ifndef BTN_EN1
    #define BTN_EN1                      AUX2_05
  #endif
  #ifndef BTN_EN2
    #define BTN_EN2                      AUX2_03
  #endif
  #ifndef BTN_ENC
    #define BTN_ENC                      AUX2_04
  #endif
#endif

#if ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)

  CONTROLLER_WARNING("RAMPS_144", "LCD_FYSETC_TFT81050")

  /**
   * FYSETC TFT-81050 display pinout
   *
   *               Board                            Display
   *               ------                           ------
   *    (MISO) 50 | 1  2 | 52 (SCK)             5V |10  9 | GND
   *  (LCD_CS) 33 | 3  4 | 53 (SD_CS)        RESET | 8  7 | (SD_DET)
   *           31   5  6 | 51 (MOSI)        (MOSI)   6  5 | (LCD_CS)
   *  (SD_DET) 49 | 7  8 | RESET           (SD_CS) | 4  3 | (MOD_RESET)
   *          GND | 9 10 | --                (SCK) | 2  1 | (MISO)
   *               ------                           ------
   *                EXP2                             EXP1
   *
   * Needs custom cable:
   *
   *    Board   Adapter   Display
   *   ----------------------------------
   *   EXP2-1 <--diode--- EXP1-1    MISO
   *   EXP2-2 ----------- EXP1-2    SCK
   *   EXP2-4 ----------- EXP1-3    MOD_RST
   *   EXP2-4 ----------- EXP1-4    SD_CS
   *   EXP2-3 ----------- EXP1-5    LCD_CS
   *   EXP2-6 ----------- EXP1-6    MOSI
   *   EXP2-7 ----------- EXP1-7    SD DET
   *   EXP2-8 ----------- EXP1-8    RESET
   *   EXP2-1 ----------- EXP1-9    MISO->GND
   *   EXP1-10 ---------- EXP1-10   5V
   *
   *  NOTE: The MISO pin should not get a 5V signal.
   *        To fix, insert a 1N4148 diode in the MISO line.
   */

  #define BEEPER_PIN                 EXP1_01_PIN

  #ifndef SD_DETECT_PIN
    #define SD_DETECT_PIN            EXP2_07_PIN
  #endif

  #define CLCD_MOD_RESET             EXP2_05_PIN
  #define CLCD_SPI_CS                EXP2_03_PIN

#endif // TOUCH_UI_FTDI_EVE && LCD_FYSETC_TFT81050
