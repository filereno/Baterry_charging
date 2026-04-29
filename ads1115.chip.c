#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define I2C_ADDRESS 0x48
#define REG_CONVERT 0x00
#define REG_CONFIG  0x01

typedef struct {
  pin_t pin_a0;
  i2c_dev_t i2c_dev;
  i2c_config_t i2c_config;
  uint8_t reg_pointer;
  uint8_t write_phase;
  uint8_t read_byte_num;
  uint16_t config_reg;
} chip_state_t;

void chip_init();
bool on_i2c_connect(void *user_data, uint32_t address, bool connect);
uint8_t on_i2c_read(void *user_data);
bool on_i2c_write(void *user_data, uint8_t data);

void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip, 0, sizeof(chip_state_t));

  chip->pin_a0 = pin_init("A0", ANALOG);

  chip->i2c_config.address = I2C_ADDRESS;
  chip->i2c_config.scl = pin_init("SCL", INPUT);
  chip->i2c_config.sda = pin_init("SDA", INPUT);
  chip->i2c_config.connect = on_i2c_connect;
  chip->i2c_config.read = on_i2c_read;
  chip->i2c_config.write = on_i2c_write;
  chip->i2c_config.user_data = chip;
  chip->i2c_dev = i2c_init(&(chip->i2c_config));

  chip->config_reg = 0x8583;
  chip->reg_pointer = REG_CONVERT;
  chip->write_phase = 0;
  chip->read_byte_num = 0;
}

bool on_i2c_connect(void *user_data, uint32_t address, bool connect) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (connect) {
    chip->write_phase = 0;
    chip->read_byte_num = 0;
  }
  return address == I2C_ADDRESS;
}

uint8_t on_i2c_read(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  uint16_t val = 0;

  if (chip->reg_pointer == REG_CONVERT) {
    float voltage = pin_adc_read(chip->pin_a0);
    // ADS1115: FSR = 4.096V, 16-bit signed, 1 LSB = 4.096/32768
    // For 0-3.3V input with FSR=4.096V:
    int16_t raw = (int16_t)(voltage / 4.096f * 32767.0f);
    val = (uint16_t)raw;
  } else if (chip->reg_pointer == REG_CONFIG) {
    // OS bit set = conversion complete
    val = chip->config_reg | 0x8000;
  }

  uint8_t result;
  if (chip->read_byte_num == 0) {
    result = (val >> 8) & 0xFF;
  } else {
    result = val & 0xFF;
  }
  chip->read_byte_num++;
  return result;
}

bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  if (chip->write_phase == 0) {
    chip->reg_pointer = data;
    chip->read_byte_num = 0;
  } else if (chip->write_phase == 1 && chip->reg_pointer == REG_CONFIG) {
    chip->config_reg = (chip->config_reg & 0x00FF) | ((uint16_t)data << 8);
  } else if (chip->write_phase == 2 && chip->reg_pointer == REG_CONFIG) {
    chip->config_reg = (chip->config_reg & 0xFF00) | data;
  }
  chip->write_phase++;
  return true;
}