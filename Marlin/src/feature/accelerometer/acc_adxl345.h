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

// ADXL345 registers and defines
#define ADXL345_DEV_ID_REG        0x00

#define ADXL345_OFFSET_X_REG      0x1E // int8_t, 15.6mg/LSB (0x7F=2G)
#define ADXL345_OFFSET_Y_REG      0x1F // int8_t, 15.6mg/LSB (0x7F=2G)
#define ADXL345_OFFSET_Z_REG      0x20 // int8_t, 15.6mg/LSB (0x7F=2G)

#define ADXL345_RATE_REG          0x2C // Sample rate register 0.1 - 3200Hz
#define ADXL345_RATE_0_1HZ        0x0
#define ADXL345_RATE_0_2HZ        0x1
#define ADXL345_RATE_0_39HZ       0x2
#define ADXL345_RATE_0_78HZ       0x3
#define ADXL345_RATE_1_56HZ       0x4
#define ADXL345_RATE_3_13HZ       0x5
#define ADXL345_RATE_6_25HZ       0x6
#define ADXL345_RATE_12_5HZ       0x7
#define ADXL345_RATE_25HZ         0x8
#define ADXL345_RATE_50HZ         0x9
#define ADXL345_RATE_100HZ        0xA
#define ADXL345_RATE_200HZ        0xB
#define ADXL345_RATE_400HZ        0xC
#define ADXL345_RATE_800HZ        0xD
#define ADXL345_RATE_1600HZ       0xE
#define ADXL345_RATE_3200HZ       0xF

#define ADXL345_POWER_CTL_REG     0x2D // Power control register
#define ADXL345_POWER_CTL_STANDBY 0x00
#define ADXL345_POWER_CTL_MEASURE 0x08

#define ADXL345_DATA_FORMAT_REG   0x31 // Data format register
#define ADXL345_DATA_SELFTEST_BIT 0x80
#define ADXL345_DATA_3WIRE_BIT    0x40
#define ADXL345_DATA_FULLRES_BIT  0x08
#define ADXL345_DATA_MSB_BIT      0x04
#define ADXL345_DATA_RANGE_MASK   3
#define ADXL345_DATA_RANGE_2G     0
#define ADXL345_DATA_RANGE_4G     1
#define ADXL345_DATA_RANGE_8G     2
#define ADXL345_DATA_RANGE_16G    3

#define ADXL345_DATA_X0_REG       0x32 // Measurement data registers
#define ADXL345_DATA_X1_REG       0x33
#define ADXL345_DATA_Y0_REG       0x34
#define ADXL345_DATA_Y1_REG       0x35
#define ADXL345_DATA_Z0_REG       0x36
#define ADXL345_DATA_Z1_REG       0x37

#define ADXL345_FIFO_CTL_REG                0x38 // FIFO control register
#define ADXL345_FIFO_CTL_MODE_BYPASS        0x00
#define ADXL345_FIFO_CTL_MODE_ENABLED       0x40
#define ADXL345_FIFO_CTL_MODE_STREAM        0x80
#define ADXL345_FIFO_CTL_MODE_TRIGGER       0xC0
#define ADXL345_FIFO_CTL_SAMPLE_COUNT_MASK  0x1F

#define ADXL345_FIFO_STATUS_REG             0x39 // FIFO status register
#define ADXL345_FIFO_STATUS_ENTRIES_MASK    0x3F

typedef int16_t ADXL345_measurement_t;

struct ADXL345_measurementXYZ_t {
    ADXL345_measurement_t x;
    ADXL345_measurement_t y;
    ADXL345_measurement_t z;
};

class ADXL345 {
  public:
    void begin();
    void end();
    void readMeasurementXYZ(ADXL345_measurementXYZ_t * acceleration);
    void readMeasurementX(ADXL345_measurement_t * acceleration);
    void readMeasurementY(ADXL345_measurement_t * acceleration);
    void readMeasurementZ(ADXL345_measurement_t * acceleration);
    void setDataRate(uint8_t dataRate); // 0->0.1Hz ... 15->3200Hz
    void writeRegister(uint8_t registerAddress, uint8_t data);
    void readRegister(uint8_t registerAddress, int numBytes, uint8_t * buffer);
  private:
    void select(const bool select);
};

extern ADXL345 adxl345;