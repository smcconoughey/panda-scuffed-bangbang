#include "MCP9802A0.hpp"

#define DEV_ADDR 0x48
#define REG_TEMP_DATA 0b00
#define REG_CONFIG 0b01
#define REG_HYST 0b10
#define REG_LIMIT 0b11

void MCP9802A0::initialize() {
  uint8_t cmd = 0b01100000;
  Wire2.beginTransmission(DEV_ADDR);
  Wire2.write(REG_CONFIG);
  Wire2.write(cmd);
  Wire2.endTransmission();
}

float MCP9802A0::getTemp() {
  uint8_t buffer_temperature[2];
  Wire2.beginTransmission(DEV_ADDR);
  Wire2.write(REG_TEMP_DATA);
  Wire2.requestFrom(DEV_ADDR, 2);
  buffer_temperature[0] = Wire2.read();
  buffer_temperature[1] = Wire2.read();
  uint8_t error = Wire2.endTransmission();

  if (error != 0) {
    return -1;
  }
  
  uint16_t temperature = (buffer_temperature[0] << 8) | (buffer_temperature[1]);
  return (float) (temperature >> 4) / 16.f;
}