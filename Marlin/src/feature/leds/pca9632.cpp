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

/**
 * Driver for the Philips PCA9632 LED driver.
 * Written by Robert Mendon Feb 2017.
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(PCA9632)

#include "pca9632.h"
#include "leds.h"
#include <Wire.h>

#define PCA9632_MODE1_VALUE   0b00000001 //(ALLCALL)
#define PCA9632_MODE2_VALUE   0b00010101 //(DIMMING, INVERT, CHANGE ON STOP,TOTEM)
#define PCA9632_LEDOUT_VALUE  0b00101010

/* Register addresses */
#define PCA9632_MODE1       0x00
#define PCA9632_MODE2       0x01
#define PCA9632_PWM0        0x02
#define PCA9632_PWM1        0x03
#define PCA9632_PWM2        0x04
#define PCA9632_PWM3        0x05
#define PCA9632_GRPPWM      0x06
#define PCA9632_GRPFREQ     0x07
#define PCA9632_LEDOUT      0x08
#define PCA9632_SUBADR1     0x09
#define PCA9632_SUBADR2     0x0A
#define PCA9632_SUBADR3     0x0B
#define PCA9632_ALLCALLADDR 0x0C

#define PCA9632_NO_AUTOINC  0x00
#define PCA9632_AUTO_ALL    0x80
#define PCA9632_AUTO_IND    0xA0
#define PCA9632_AUTOGLO     0xC0
#define PCA9632_AUTOGI      0xE0

// Red=LED0   Green=LED1   Blue=LED2  White=LED3
#ifndef PCA9632_RED
  #define PCA9632_RED 0x00
#endif
#ifndef PCA9632_GRN
  #define PCA9632_GRN 0x02
#endif
#ifndef PCA9632_BLU
  #define PCA9632_BLU 0x04
#endif
#if HAS_WHITE_LED && !defined(PCA9632_WHT)
  #define PCA9632_WHT 0x06
#endif

// If any of the color indexes are greater than 0x04 they can't use auto increment
#if !defined(PCA9632_NO_AUTO_INC) && (PCA9632_RED > 0x04 || PCA9632_GRN > 0x04 || PCA9632_BLU > 0x04 || PCA9632_WHT > 0x04)
  #define PCA9632_NO_AUTO_INC
#endif

#define LED_OFF   0x00
#define LED_ON    0x01
#define LED_PWM   0x02

#define PCA9632_ADDRESS 0b01100000

byte PCA_init = 0;

static void PCA9632_WriteRegister(const byte addr, const byte regadd, const byte value) {
  Wire.beginTransmission(I2C_ADDRESS(addr));
  Wire.write(regadd);
  Wire.write(value);
  Wire.endTransmission();
}

static void PCA9632_WriteAllRegisters(const byte addr, const byte regadd, const byte vr, const byte vg, const byte vb
  OPTARG(PCA9632_RGBW, const byte vw)
) {
  #if DISABLED(PCA9632_NO_AUTO_INC)
    uint8_t data[4];
    data[0] = PCA9632_AUTO_IND | regadd;
    data[1 + (PCA9632_RED >> 1)] = vr;
    data[1 + (PCA9632_GRN >> 1)] = vg;
    data[1 + (PCA9632_BLU >> 1)] = vb;
    Wire.beginTransmission(I2C_ADDRESS(addr));
    Wire.write(data, sizeof(data));
    Wire.endTransmission();
  #else
    PCA9632_WriteRegister(addr, regadd + (PCA9632_RED >> 1), vr);
    PCA9632_WriteRegister(addr, regadd + (PCA9632_GRN >> 1), vg);
    PCA9632_WriteRegister(addr, regadd + (PCA9632_BLU >> 1), vb);
    #if ENABLED(PCA9632_RGBW)
      PCA9632_WriteRegister(addr, regadd + (PCA9632_WHT >> 1), vw);
    #endif
  #endif
}

#if 0
  static byte PCA9632_ReadRegister(const byte addr, const byte regadd) {
    Wire.beginTransmission(I2C_ADDRESS(addr));
    Wire.write(regadd);
    const byte value = Wire.read();
    Wire.endTransmission();
    return value;
  }
#endif

void PCA9632_set_led_color(const LEDColor &color) {
  Wire.begin();
  if (!PCA_init) {
    PCA_init = 1;
    PCA9632_WriteRegister(PCA9632_ADDRESS,PCA9632_MODE1, PCA9632_MODE1_VALUE);
    PCA9632_WriteRegister(PCA9632_ADDRESS,PCA9632_MODE2, PCA9632_MODE2_VALUE);
  }

  const byte LEDOUT = (color.r ? LED_PWM << PCA9632_RED : 0)
                    | (color.g ? LED_PWM << PCA9632_GRN : 0)
                    | (color.b ? LED_PWM << PCA9632_BLU : 0)
                    #if ENABLED(PCA9632_RGBW)
                      | (color.w ? LED_PWM << PCA9632_WHT : 0)
                    #endif
                    ;

  PCA9632_WriteAllRegisters(PCA9632_ADDRESS,PCA9632_PWM0, color.r, color.g, color.b
    OPTARG(PCA9632_RGBW, color.w)
  );
  PCA9632_WriteRegister(PCA9632_ADDRESS,PCA9632_LEDOUT, LEDOUT);
}

#if ENABLED(PCA9632_BUZZER)

  void PCA9632_buzz(const long, const uint16_t/*=0*/) {
    uint8_t data[] = PCA9632_BUZZER_DATA;
    Wire.beginTransmission(I2C_ADDRESS(PCA9632_ADDRESS));
    Wire.write(data, sizeof(data));
    Wire.endTransmission();
  }

#endif // PCA9632_BUZZER

#endif // PCA9632
