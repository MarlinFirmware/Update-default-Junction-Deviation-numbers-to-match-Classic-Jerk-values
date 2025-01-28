/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include "../../platforms.h"

#ifdef HAL_STM32

#include "../../../inc/MarlinConfig.h"

#if ENABLED(TFT_TOUCH_DEVICE_GT911)

#include "gt911.h"
#include "pinconfig.h"

SW_IIC::SW_IIC(uint16_t sda, uint16_t scl) {
  scl_pin = scl;
  sda_pin = sda;
}

// Software I2C hardware io init
void SW_IIC::init() {
  OUT_WRITE(scl_pin, HIGH);
  OUT_WRITE(sda_pin, HIGH);
}

// Software I2C start signal
void SW_IIC::start() {
  write_sda(HIGH); // SDA = 1
  write_scl(HIGH); // SCL = 1
  iic_delay(2);
  write_sda(LOW); // SDA = 0
  iic_delay(1);
  write_scl(LOW); // SCL = 0  // keep SCL low, avoid false stop caused by level jump caused by SDA switching IN/OUT
}

// Software I2C stop signal
void SW_IIC::stop() {
  write_scl(LOW); // SCL = 0
  iic_delay(2);
  write_sda(LOW); // SDA = 0
  iic_delay(2);
  write_scl(HIGH); // SCL = 1
  iic_delay(2);
  write_sda(HIGH); // SDA = 1
}

// Software I2C sends ACK or NACK signal
void SW_IIC::send_ack(bool ack) {
  write_sda(ack ? LOW : HIGH); // SDA = !ack
  iic_delay(2);
  write_scl(HIGH); // SCL = 1
  iic_delay(2);
  write_scl(LOW); // SCL = 0
}

// Software I2C read ACK or NACK signal
bool SW_IIC::read_ack() {
  bool error = 0;
  set_sda_in();

  iic_delay(2);

  write_scl(HIGH); // SCL = 1
  error = read_sda();

  iic_delay(2);

  write_scl(LOW);  // SCL = 0

  set_sda_out();
  return error;
}

void SW_IIC::send_byte(uint8_t txd) {
  for (uint_fast8_t i = 0; i < 8; ++i) {
    write_sda(txd & 0x80); // write data bit
    txd <<= 1;
    iic_delay(1);
    write_scl(HIGH); // SCL = 1
    iic_delay(2);
    write_scl(LOW); // SCL = 0
    iic_delay(1);
  }

  read_ack();  // wait ack
}

uint8_t SW_IIC::read_byte(bool ack) {
  uint8_t data = 0;

  set_sda_in();
  for (uint_fast8_t i = 0; i < 8; ++i) {
    write_scl(HIGH); // SCL = 1
    iic_delay(1);
    data <<= 1;
    if (read_sda()) data++;
    write_scl(LOW); // SCL = 0
    iic_delay(2);
  }
  set_sda_out();

  send_ack(ack);

  return data;
}

GT911_REG_MAP GT911::reg;
SW_IIC GT911::sw_iic = SW_IIC(GT911_SW_I2C_SDA_PIN, GT911_SW_I2C_SCL_PIN);

void GT911::write_reg(uint16_t reg, uint8_t reg_len, uint8_t* w_data, uint8_t w_len) {
  sw_iic.start();
  sw_iic.send_byte(gt911_slave_address);  // Set IIC Slave address
  for (uint_fast8_t i = 0; i < reg_len; ++i) {  // Set reg address
    uint8_t r = (reg >> (8 * (reg_len - 1 - i))) & 0xFF;
    sw_iic.send_byte(r);
  }

  for (uint_fast8_t i = 0; i < w_len; ++i) {  // Write data to reg
    sw_iic.send_byte(w_data[i]);
  }
  sw_iic.stop();
}

void GT911::read_reg(uint16_t reg, uint8_t reg_len, uint8_t* r_data, uint8_t r_len) {
  sw_iic.start();
  sw_iic.send_byte(gt911_slave_address);  // Set IIC Slave address
  for (uint_fast8_t i = 0; i < reg_len; ++i) {  // Set reg address
    uint8_t r = (reg >> (8 * (reg_len - 1 - i))) & 0xFF;
    sw_iic.send_byte(r);
  }

  sw_iic.start();
  sw_iic.send_byte(gt911_slave_address + 1);  // Set read mode

  for (uint_fast8_t i = 0; i < r_len; ++i)
    r_data[i] = sw_iic.read_byte(1);  // Read data from reg

  sw_iic.stop();
}

void GT911::init() {
  OUT_WRITE(GT911_RST_PIN, LOW);
  OUT_WRITE(GT911_INT_PIN, LOW);
  delay(11);
  WRITE(GT911_INT_PIN, HIGH);
  delayMicroseconds(110);
  WRITE(GT911_RST_PIN, HIGH);
  delay(6);
  WRITE(GT911_INT_PIN, LOW);
  delay(55);
  SET_INPUT(GT911_INT_PIN);

  sw_iic.init();

  uint8_t clear_reg = 0x00;
  write_reg(0x814E, 2, &clear_reg, 1); // Reset to 0 for start
}

bool GT911::getFirstTouchPoint(int16_t *x, int16_t *y) {
  read_reg(0x814E, 2, &reg.REG.status, 1);

  if (reg.REG.status >= 0x80 && reg.REG.status <= 0x85) {
    read_reg(0x8150, 2, reg.map + 2, 38);
    uint8_t clear_reg = 0x00;
    write_reg(0x814E, 2, &clear_reg, 1); // Reset to 0 for start
    // First touch point
    *x = ((reg.REG.point[0].xh & 0x0F) << 8) | reg.REG.point[0].xl;
    *y = ((reg.REG.point[0].yh & 0x0F) << 8) | reg.REG.point[0].yl;
    return true;
  }
  return false;
}

bool GT911::getRawPoint(int16_t * const x, int16_t * const y) {
  static bool touched = false;
  static int16_t read_x = 0, read_y = 0;
  static millis_t next_time = 0;

  if (ELAPSED(millis(), next_time)) {
    touched = getFirstTouchPoint(&read_x, &read_y);
    next_time = millis() + 20;
  }

  *x = read_x;
  *y = read_y;
  return touched;
}

#endif // TFT_TOUCH_DEVICE_GT911
#endif // HAL_STM32
