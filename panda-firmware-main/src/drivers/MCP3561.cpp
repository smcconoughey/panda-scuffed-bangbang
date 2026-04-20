#include "MCP3561.hpp"
#include <SPI.h> // Assuming this is needed for SPISettings and SPI object


MCP3561::MCP3561(uint8_t chip_select, SPIClass& spi_bus, SPISettings settings, float vref_ = 1.25)
  : chip_select_pin(chip_select)
  , spi(spi_bus)
  , spi_setting(settings)
  , vref(vref_)         // ← bind the reference here
{
  pinMode(chip_select_pin, OUTPUT);
  digitalWrite(chip_select_pin, HIGH);
}

// =========== Basic SPI Communication functions ==========

void MCP3561::writeRegister(uint8_t reg_addr, uint8_t data) {

  uint8_t command = (DEVICE_ADDRESS << 6) | (reg_addr << 2) | WRITE_COMMAND_MASK;

  spi.beginTransaction(spi_setting);
  digitalWrite(chip_select_pin, LOW);
  spi.transfer(command);
  spi.transfer(data);
  digitalWrite(chip_select_pin, HIGH);
  spi.endTransaction();

}

uint8_t MCP3561::readRegister(uint8_t reg_addr) {

  uint8_t command = (DEVICE_ADDRESS << 6) | (reg_addr << 2) | SREAD_COMMAND_MASK;

  spi.beginTransaction(spi_setting);
  digitalWrite(chip_select_pin, LOW);
  spi.transfer(command);
  uint8_t data = spi.transfer(0);
  digitalWrite(chip_select_pin, HIGH);
  spi.endTransaction();

  return data;

}

void MCP3561::writeFastCommand(uint8_t fastCmd) {

  spi.beginTransaction(spi_setting);
  digitalWrite(chip_select_pin, LOW);
  spi.transfer(fastCmd);
  digitalWrite(chip_select_pin, HIGH);
  spi.endTransaction();

}

void MCP3561::setSettings(SPISettings settings) {spi_setting = settings;}

// ==========

// ========== Configuration Functions ==========


/**
 * @brief Configure which channels are routed to VIN+ and VIN–.
 *
 * @param vinp  Which channel to select for VIN+ (e.g. MuxChannel::CH0).
 * @param vinm  Which channel to select for VIN– (e.g. MuxChannel::AGND).
 * 
 * The ADC references @param vinp to @param vinm.
 */
void MCP3561::setMuxInputs(MuxSettings vinp, MuxSettings vinm) {

  uint8_t data = (static_cast<uint8_t>(vinp) << 4) | (static_cast<uint8_t>(vinm));
  writeRegister(MUX_ADDR, data);

}

void MCP3561::setVREF(float vref_) {vref = vref_;}

/**
 * @brief Configures the ADC's bias-source current.
 * 
 * Enables one of the four settings:
 *  - I_0 = No bias current (Default)
 *  - I_0_9UA = 0.9uA
 *  - I_3_7UA = 3.7uA
 *  - I_15UA = 15uA
 * 
 */
void MCP3561::setBiasCurrent(BiasCurrentSettings setting) {

  uint8_t data = readRegister(CONFIG0_ADDR);
  uint8_t mask = 0b11110011;

  data &= mask;
  data |= (static_cast<uint8_t>(setting) << 2);

  writeRegister(CONFIG0_ADDR, data);

}

/**
 * @brief Sets the gain of the ADC
 * 
 * Settings:
 *  - GAIN_1 = 1x (Default)
 *  - GAIN_2 = 2x
 *  - GAIN_4 = 4x
 *  - GAIN_8 = 8x
 *  - GAIN_16 = 16x
 * 
 */
void MCP3561::setGain(GainSettings gain) {

  uint8_t data = readRegister(CONFIG2_ADDR);
  uint8_t mask = 0b11000111;

  data &= mask;
  data |= (static_cast<uint8_t>(gain) << 3);

  writeRegister(CONFIG2_ADDR, data);

}




void MCP3561::trigger(void) {
  writeFastCommand(FAST_CMD_START_RESTART);
}

void MCP3561::initialize(void) {

  // First write to CONFIG0 register
  uint8_t command_byte = CONFIG0_WRITE;
  // Configure the ADC to read use its own internal clock and output it to the MCLK pin
  uint8_t data_byte = CONFIG0_CLK_SEL_INT;
  // Reconfigure the ADC to be in conversion mode rather than shutdown mode
  data_byte |= CONFIG0_ADC_MODE_CONV;
  writeRegister(CONFIG0_ADDR, data_byte);
  // digitalWrite(chip_select_pin, LOW);
  // spi.transfer(command_byte);
  // spi.transfer(data_byte);
  // digitalWrite(chip_select_pin, HIGH);
  // Next write to CONFIG1 register. 
  command_byte = CONFIG1_WRITE;
  // This gives a oversampling ratio of 32 with sampling at MCLK
  data_byte = CONFIG1_OSR_256;
  writeRegister(CONFIG1_ADDR, data_byte);
  
  // digitalWrite(chip_select_pin, LOW);
  // spi.transfer(command_byte);
  // spi.transfer(data_byte);
  // digitalWrite(chip_select_pin, HIGH);
  // Next, write to CONFIG2 register. No need to change anything, defaults OK.
  command_byte = CONFIG2_WRITE;
  data_byte = CONFIG2_DEFAULT;
  writeRegister(CONFIG2_ADDR, data_byte);

  // Next, write to CONFIG3 register.
  command_byte = CONFIG3_WRITE;
  // Change ADC mode to one-shot conversion
  // data_byte = CONFIG3_CONV_MODE_ONESHOT;
  data_byte = CONFIG3_DEFAULT;
  writeRegister(CONFIG3_ADDR, data_byte);

  // digitalWrite(chip_select_pin, LOW);
  // spi.transfer(command_byte);
  // spi.transfer(data_byte);
  // digitalWrite(chip_select_pin, HIGH);

  // Next, write to the IRQ register to enable a conversion to trigger an interrupt.
  // command_byte = IRQ_WRITE;
  // data_byte = IRQ_MODE_HIGH;
  // writeRegister(IRQ_ADDR, data_byte);
  // digitalWrite(chip_select_pin, LOW);
  // spi.transfer(command_byte);
  // spi.transfer(data_byte);
  // digitalWrite(chip_select_pin, HIGH);

  // Writing to the MUX register. Will likely make this a separate function
  // command_byte = MUX_WRITE;
  // data_byte = 0b00011000; // References CH0 to AGND
  // digitalWrite(chip_select_pin, LOW);
  // spi.transfer(command_byte);
  // spi.transfer(data_byte);
  // digitalWrite(chip_select_pin, HIGH);
  // spi.endTransaction();

  // writeRegister(0x06, 0b00011000);
}

bool MCP3561::getInterrupt(void) {
  uint8_t irqReg = readRegister(0x5);
  // return !(irqReg & (1 << 6));
  return ((irqReg & 0x40) == 0);
}



 float MCP3561::getOutput() {
  // 1) pull the data off the bus
  spi.beginTransaction(spi_setting);
  digitalWrite(chip_select_pin, LOW);
  spi.transfer(SREAD_DATA_COMMAND);
  // uint8_t status = spi.transfer(0);
  // uint8_t sgn = spi.transfer(0);
  uint8_t b0 = spi.transfer(0);  // MSB
  uint8_t b1 = spi.transfer(0);
  uint8_t b2 = spi.transfer(0);  // LSB
  digitalWrite(chip_select_pin, HIGH);
  spi.endTransaction();

  uint32_t raw24 = (uint32_t(b0) << 16)
                 | (uint32_t(b1) <<  8)
                 |  uint32_t(b2);
  
  raw24 &= 0x00FFFFFF;  // ensure only bits 0–23 remain

  // int32_t value = (raw24 & 0x800000)
  //               ? int32_t(raw24 | 0xFF000000)
  //               : int32_t(raw24);

  // if (sgn == 0b0) {value *= 1;}
  // else if (sgn == 0b1) {value *= -1;}
  
  // Serial.print("Raw bytes: 0x");
  // if (b0 < 0x10) Serial.print('0');
  // Serial.print(b0, HEX);
  // Serial.print(" 0x");
  // if (b1 < 0x10) Serial.print('0');
  // Serial.print(b1, HEX);
  // Serial.print(" 0x");
  // if (b2 < 0x10) Serial.print('0');
  // Serial.print(b2, HEX);
  // Serial.println();
  // Serial.print("Raw value: ");
  // Serial.println(raw24);

  // Serial.print("Sign value: ");
  // Serial.println(sgn, HEX);

  // return raw24;
  return vref * ((raw24 & 0x800000) ? int32_t(raw24 | 0xFF000000) : int32_t(raw24)) / 8388608.0f;
  // return (float(raw24) / 8388608.0f);
}

void MCP3561::readAllRegisters(void) {
  spi.beginTransaction(spi_setting);
  digitalWrite(chip_select_pin, LOW);
  spi.transfer(IREAD_COMMAND);
  // First, read the ADCDATA register
  adc_data[2] = spi.transfer(0);
  adc_data[1] = spi.transfer(0);
  adc_data[0] = spi.transfer(0);
  adc_sample = (adc_data[2] << 16) | (adc_data[1] << 8) | (adc_data[0]);

  config0_data = spi.transfer(0);
  config1_data = spi.transfer(0);
  config2_data = spi.transfer(0);
  config3_data = spi.transfer(0);
  irq_data = spi.transfer(0);
  mux_data = spi.transfer(0);
  byte temp0 = spi.transfer(0);
  byte temp1 = spi.transfer(0);
  byte temp2 = spi.transfer(0);
  scan_data = (temp2 << 16) | (temp1 << 8) | temp0;
  temp0 = spi.transfer(0);
  temp1 = spi.transfer(0);
  temp2 = spi.transfer(0);
  timer_data = (temp2 << 16) | (temp1 << 8) | temp0;
  temp0 = spi.transfer(0);
  temp1 = spi.transfer(0);
  temp2 = spi.transfer(0);
  offsetcal_data =  (temp2 << 16) | (temp1 << 8) | temp0;
  temp0 = spi.transfer(0);
  temp1 = spi.transfer(0);
  temp2 = spi.transfer(0);
  gaincal_data = (temp2 << 16) | (temp1 << 8) | temp0;
  temp0 = spi.transfer(0);
  temp1 = spi.transfer(0);
  temp2 = spi.transfer(0);
  reserved_1_data = (temp2 << 16) | (temp1 << 8) | temp0;
  reserved_2_data = spi.transfer(0);
  lock_data = spi.transfer(0);
  temp0 = spi.transfer(0);
  temp1 = spi.transfer(0);
  reserved_3_data = (temp1 << 8) | temp0;
  temp0 = spi.transfer(0);
  temp1 = spi.transfer(0);
  crccfg_data = (temp1 << 8) | temp0;
  digitalWrite(chip_select_pin, HIGH);
  spi.endTransaction();

}

// PARTIALLY COMPLETE. DOES NOT VERIFY ALL REGISTERS.
void MCP3561::verifyRegisters(void) {
  if(config0_data == 0b00000011) config0_ok = true;
  if(config1_data == 0b00001100) config1_ok = true;
  if(config2_data == 0b10001011) config2_ok = true;
  if(config3_data == 0b11000000) config3_ok = true;
  if(irq_data == 0b00110111) irq_ok = true;

  bool all_ok = config0_ok && config1_ok && config2_ok && config3_ok && irq_ok;

  if(all_ok == true) {
    Serial.println("ALL REGISTERS OK.");
  }
  else
  {
    Serial.println("SOME REGISTER NOT OK. REGISTER TABLE:");
    Serial.print("CONFIG0: ");
    Serial.println(config0_ok ? "OK" : "BAD");
    Serial.print("CONFIG1: ");
    Serial.println(config1_ok ? "OK" : "BAD");
    Serial.print("CONFIG2: ");
    Serial.println(config2_ok ? "OK" : "BAD");
    Serial.print("CONFIG3: ");
    Serial.println(config3_ok ? "OK" : "BAD");
    Serial.print("IRQ: ");
    Serial.println(irq_ok ? "OK" : "BAD");
  }
  
}

void MCP3561::printRegisters(void) {
  Serial.print("ADCDATA Register: ");
  Serial.print(adc_data[2], BIN);
  Serial.print(adc_data[1], BIN);
  Serial.println(adc_data[0], BIN);
  Serial.print("CONFIG0: ");
  Serial.println(config0_data, BIN);
  Serial.print("CONFIG1: ");
  Serial.println(config1_data, BIN);
  Serial.print("CONFIG2: ");
  Serial.println(config2_data, BIN);
  Serial.print("CONFIG3: ");
  Serial.println(config3_data, BIN);
  Serial.print("IRQ: ");
  Serial.println(irq_data, BIN);
  Serial.print("MUX: ");
  Serial.println(mux_data, BIN);
  Serial.print("SCAN: ");
  Serial.println(scan_data, BIN);
  Serial.print("TIMER: ");
  Serial.println(scan_data, BIN);
  Serial.print("OFFSETCAL: ");
  Serial.println(scan_data, BIN);
  Serial.print("GAINCAL: ");
  Serial.println(scan_data, BIN);
  Serial.print("RESERVED1: ");
  Serial.println(scan_data, HEX);
  Serial.print("RESERVED2: ");
  Serial.println(scan_data, HEX);
  Serial.print("LOCK: ");
  Serial.println(scan_data, BIN);
  Serial.print("RESERVED3: ");
  Serial.println(scan_data, HEX);
  Serial.print("CRCCFG: ");
  Serial.println(scan_data, BIN);
  Serial.println("-----------------------------");
}
