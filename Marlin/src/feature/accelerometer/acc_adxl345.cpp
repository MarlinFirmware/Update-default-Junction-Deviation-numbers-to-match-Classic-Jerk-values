/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/***************************************************************
 *
 * ADXL345 3-AXIS ACCELEROMETER ON SPI BUS
 * 4-WIRE SPI COMMUNICATION
 * Define: SD_SS_PIN, SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN
 *
 ****************************************************************/

#include "../../inc/MarlinConfig.h"

#if ENABLED(ACCELEROMETER_ADXL345)

#include "acc_ADXL345.h"

#include "../../MarlinCore.h"
#include "../../HAL/shared/Delay.h"

#include "SPI.h"
extern SPIClass SPI;

ADXL345 adxl345;

void ADXL345::begin() {
  OUT_WRITE(SD_SS_PIN, HIGH); // set ADXL345 Chip Select to OUTPUT mode, HIGH before configuring SPI
  spiBegin();                 // sets Chip Select to HIGH (again)
  spiInit(SPI_HALF_SPEED);    // calls SPI.begin(), sets speed to 4MHz, max is 5MHz for ADXL345
  SPI.setDataMode(SPI_MODE3); // ADXL345 uses SPI_MODE3 to communicate (CPOL=1, CPHA = 1)

  // set range to 2g (4wire SPI, right justify data, 10-bit resolution)
  writeRegister(ADXL345_DATA_FORMAT_REG, ADXL345_DATA_RANGE_2G);

  // enable measurement mode, use streaming mode
  writeRegister(ADXL345_POWER_CTL_REG, ADXL345_POWER_CTL_MEASURE | ADXL345_FIFO_CTL_MODE_STREAM);

  // set to 100Hz sampling rate
  writeRegister(ADXL345_RATE_REG, ADXL345_RATE_100HZ);
}

void ADXL345::end() { // put device in standby mode
  writeRegister(ADXL345_POWER_CTL_REG, ADXL345_POWER_CTL_STANDBY);
}

void ADXL345::select(const bool select) {
  WRITE(SD_SS_PIN, !select); // ADXL345 is selected on LOW
}

void ADXL345::writeRegister(const uint8_t registerAddress, const uint8_t data) {
  WRITE(SD_SS_PIN, LOW);    // set Chip Select to LOW to start the write
  spiSend(registerAddress); // send the register address
  spiSend(data);            // send the data
  WRITE(SD_SS_PIN, HIGH);   // set Chip Select to HIGH to complete the write
}

void ADXL345::readRegister(const uint8_t registerAddress, const int numBytes, uint8_t * const buffer) {
  uint8_t address = registerAddress | 0x80; // set read bit
  if (numBytes > 1) address |= 0x40;        // and multi-byte read, if needed

  WRITE(SD_SS_PIN, LOW);      // set Chip Select to LOW to start the read
  spiSend(address);           // send the register address
  for (int i = 0; i < numBytes; i++)
    buffer[i] = spiRec();     // read the data
  WRITE(SD_SS_PIN, HIGH);     // set Chip Select to HIGH to complete the read
  delayMicroseconds(5);       // allow 5us for the FIFO/registers to update (see datasheet)
}

// get an acceleration measurement for the X, Y and Z axis
void ADXL345::readMeasurementXYZ(ADXL345_measurementXYZ_t * const acceleration) {
  readRegister(ADXL345_DATA_X0_REG, 6, (uint8_t*)acceleration);
}

// get an acceleration measurement for the X axis
void ADXL345::readMeasurementX(ADXL345_measurement_t * const acceleration) {
  readRegister(ADXL345_DATA_X0_REG, 2, (uint8_t*)acceleration);
}

// get an acceleration measurement for the Y axis
void ADXL345::readMeasurementY(ADXL345_measurement_t * const acceleration) {
  readRegister(ADXL345_DATA_Y0_REG, 2, (uint8_t*)acceleration);
}
// get an acceleration measurement for the Z axis
void ADXL345::readMeasurementZ(ADXL345_measurement_t * const acceleration) {
  readRegister(ADXL345_DATA_Z0_REG, 2, (uint8_t*)acceleration);
}

void ADXL345::setDataRate(const uint8_t dataRate) { // 0->0.1Hz ... 15->3200Hz
  writeRegister(ADXL345_DATA_FORMAT_REG, dataRate & 0xF);
}

#endif // ACCELEROMETER_ADXL345
