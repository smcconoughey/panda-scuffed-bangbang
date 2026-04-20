#pragma once
#include <SPI.h>
#include <Arduino.h>

/**
 * Configuration file
 * Includes DC channel and Bang-Bang implementation, time values, and conversion constants
 */

 // =================== SPI & ADC configurations ================= //

static const SPISettings SPISettingsDefault(20000000, MSBFIRST, SPI_MODE0);
static constexpr unsigned int T_MUX_SETTLE_US = 500;
static constexpr unsigned int T_CONV_US = 1000;

// =================== Serial configurations ================= //

static constexpr bool DEBUG_F_ADC = false;
static constexpr bool DEBUG_BB = false;
static constexpr bool DEBUG_PACKET = false;

static constexpr char S_IDENTIFIER = 's';
static constexpr char LCTC_IDENTIFIER = 't';

static constexpr unsigned int SERIAL_BAUD_RATE = 460800; // Originally 115200
static constexpr unsigned int SERIAL_TIMEOUT = 2000;
// static constexpr unsigned int SERIAL_WRITE_DELAY = 100; // Microseconds to wait between writes to prevent overwhelming the serial bus

static constexpr unsigned int PACKET_IDLE_MS = 100;
static constexpr unsigned int PULSE_DURATION = 500;
static constexpr size_t RX_BUF_SIZE = 256;

static constexpr uint8_t NUM_MAX_COMMANDS = 32;
static constexpr uint8_t DATA_DECIMALS = 6; // Number of decimal places in telemetry data

static constexpr uint8_t NUM_DC_CHANNELS = 12;
static constexpr uint8_t NUM_PT_CHANNELS = 16;
static constexpr uint8_t NUM_LC_CHANNELS = 6;
static constexpr uint8_t NUM_TC_CHANNELS = 6;

static constexpr uint8_t PACKET_SIZE = NUM_DC_CHANNELS + NUM_PT_CHANNELS + NUM_LC_CHANNELS + NUM_TC_CHANNELS;
static constexpr size_t TX_BUF_SIZE = 2048;

// Conversion constants
static constexpr float tcConstant = 2217.294;
static constexpr float tcOffset = 160;
static constexpr float sConstant = 0.5;

static constexpr float tcOffsets[NUM_TC_CHANNELS] = {
    -0.07429,
    -0.06889,
    -0.07387,
    -0.0786,
    -0.06998,
    -0.07754
};