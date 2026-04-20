
/*
 * NOTES - 
 * The MCP3561 uses SPI mode 0,0.
 * This should be turned into a class so that it's simpler to use.
 */

 // Driver header file for the MCP3561 ADC

 // Header file guard
#pragma once

#include <SPI.h>

#define DEVICE_ADDRESS 0b01
#define DEVICE_ADDRESS_MASK (DEVICE_ADDRESS << 6)
#define COMMAND_ADDR_POS 2

// USEFUL MASKS FOR ADC COMMUNICATION
#define DATA_READY_MASK 0b00000100 // Tells us whether data is ready from an SPI transaction
#define ADDRESS_MASK 0b00111000
#define WRITE_COMMAND_MASK 0b00000010
#define WRITE_COMMAND WRITE_COMMAND_MASK | DEVICE_ADDRESS_MASK
#define IREAD_COMMAND_MASK 0b00000011 // Incremental read command
#define IREAD_COMMAND IREAD_COMMAND_MASK | DEVICE_ADDRESS_MASK
#define SREAD_COMMAND_MASK 0b1 // Static read command
#define SREAD_DATA_COMMAND SREAD_COMMAND_MASK | DEVICE_ADDRESS_MASK

#define CONFIG0_ADDR 0x01
#define CONFIG0_WRITE (CONFIG0_ADDR << COMMAND_ADDR_POS) | WRITE_COMMAND
#define CONFIG0_CLK_SEL_MASK 0b00110000
#define CONFIG0_CLK_SEL_POS 4
#define CONFIG0_CLK_SEL_INT 0b11 << CONFIG0_CLK_SEL_POS
#define CONFIG0_CLK_SEL_EXT 0b00 << CONFIG0_CLK_SEL_POS
#define CONFIG0_ADC_MODE_POS 0
#define CONFIG0_ADC_MODE_CONV 0b11 << CONFIG0_ADC_MODE_POS

#define CONFIG1_ADDR 0x02
#define CONFIG1_WRITE (CONFIG1_ADDR << COMMAND_ADDR_POS) | WRITE_COMMAND
#define CONFIG1_OSR_POS 2
#define CONFIG1_OSR_32 0b0000 << CONFIG1_OSR_POS
#define CONFIG1_OSR_256 0b0011 << CONFIG1_OSR_POS

#define CONFIG2_ADDR 0x03
#define CONFIG2_WRITE (CONFIG2_ADDR << COMMAND_ADDR_POS) | WRITE_COMMAND
#define CONFIG2_DEFAULT 0b10001111;

#define CONFIG3_ADDR 0x04
#define CONFIG3_DEFAULT 0b10000000 // (In order of register settings: one-shot, 8-bit sign + 24-bit data, everything else default)
#define CONFIG3_WRITE (CONFIG3_ADDR << COMMAND_ADDR_POS) | WRITE_COMMAND
// #define CONFIG3_CONV_MODE_POS 6
// #define CONFIG3_CONV_MODE_CONTINUOUS 0b11 << CONFIG3_CONV_MODE_POS
// #define CONFIG3_CONV_MODE_ONESHOT (0b10 << CONFIG3_CONV_MODE_POS)

#define IRQ_ADDR 0x05
#define IRQ_WRITE (IRQ_ADDR << COMMAND_ADDR_POS) | WRITE_COMMAND
#define IRQ_MODE_POS 2
#define IRQ_MODE_HIGH 0b01110111

#define MUX_ADDR 0x06
#define MUX_WRITE (MUX_ADDR << COMMAND_ADDR_POS) | WRITE_COMMAND


#define EXTERNAL_SYNC_PIN 5
#define MAX_SYNCHRONIZATION_POINTS 5000

// USEFUL FAST COMMANDS AND OTHER COMMANDS
#define FAST_CMD_START_RESTART DEVICE_ADDRESS_MASK |0b101000
// Resets the device registers to their default  values
#define DEVICE_RESET_COMMAND DEVICE_ADDRESS_BYTE | 0b111000

enum class MuxSettings : uint8_t {
  CH0 = 0b0000,
  CH1 = 0b0001,
  AGND = 0b1000,
  AVDD = 0b1001,
  REFIN_P = 0b1011,
  REFIN_N = 0b1100
};

enum class GainSettings : uint8_t {
  GAIN_1 = 0b001,
  GAIN_2 = 0b010,
  GAIN_4 = 0b011,
  GAIN_8 = 0b100,
  GAIN_16 = 0b101
};

enum class BiasCurrentSettings : uint8_t {
  I_0 = 0b00,
  I_0_9UA = 0b01,
  I_3_7UA = 0b10,
  I_15UA = 0b11
};

enum class OversamplingSettings : uint8_t {

};

class MCP3561 {
  private:
    uint8_t chip_select_pin;
    uint8_t miso_pin;
    uint8_t sck_pin;
    uint8_t irq_pin;

    // Static class variables used in interrupts
    uint32_t data_counter;
    uint32_t data_points_to_sample;
    uint8_t adc_data[3];
    uint32_t adc_sample;
    uint32_t synchronization_data[MAX_SYNCHRONIZATION_POINTS];
    uint32_t synchronization_counter;

    // Class variables
    SPIClass& spi;
    byte config0_data;
    byte config1_data;
    byte config2_data;
    byte config3_data;
    byte irq_data;
    byte mux_data;
    uint32_t scan_data; // A 24-bit register
    uint32_t timer_data; // A 24-bit register
    uint32_t offsetcal_data; // A 24-bit register
    uint32_t gaincal_data; // A 24-bit register
    uint32_t reserved_1_data;
    byte reserved_2_data;
    byte lock_data;
    uint16_t reserved_3_data;
    uint16_t crccfg_data;
    
    bool config0_ok = false; 
    bool config1_ok = false;
    bool config2_ok = false;
    bool config3_ok = false;
    bool irq_ok = false;
    bool mux_ok = false;
    bool scan_ok = false;
    bool timer_ok = false;
    bool offsetcal_ok = false;
    bool gaincal_ok = false;
    bool reserved1_ok = false;
    bool reserved2_ok = false;
    bool lock_ok = false;
    bool reserved3_ok = false;
    bool crccfg_ok = false;

    float vref;

    void writeRegister(uint8_t reg_addr, uint8_t data);
    void writeFastCommand(uint8_t fastCmd);
    uint8_t readRegister(uint8_t reg_addr);

  SPISettings spi_setting;

  public:
    MCP3561(uint8_t chip_select, SPIClass& spi_bus, SPISettings settings, float vref = 1.25);
    // Class methods


    void readAllRegisters(void);
    void verifyRegisters(void);
    void printRegisters(void);
    void trigger(void);
    float getOutput(void);
    bool getInterrupt(void);
    void initialize(void);
    void setVREF(float vref_ = 1.25f);

    void setSettings(SPISettings settings);

    void setBiasCurrent(BiasCurrentSettings setting);
    void setMuxInputs(MuxSettings vinp, MuxSettings vinm);
    void setGain(GainSettings gain);
    void setOversamplingRate();

    
};

// uint32_t MCP3561::data_counter;
// uint32_t MCP3561::data_points_to_sample = 1;
// byte MCP3561::adc_data[3];
// uint32_t MCP3561::adc_sample = 0;
// uint32_t MCP3561::synchronization_data[MAX_SYNCHRONIZATION_POINTS];
// uint32_t MCP3561::synchronization_counter = 0;

