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
static constexpr char PT_IDENTIFIER = 'p'; // PT CSV id (forwarded from V2)

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

// =================== Bang-Bang configuration =================== //
// Bang-bang runs on V1 (DC channels functional here) using PT readings
// forwarded from V2 over the secondary RS-485 crossover.

// Sentinel values for unset hardware channels.
static constexpr uint8_t BB_DC_CH_UNSET = 0;     // DC channels are 1-indexed
static constexpr uint8_t BB_PT_CH_UNSET = 0xFF;  // PT channels are 0-indexed

// PT indices into the V2-forwarded ptData[] array (0-indexed).
static constexpr uint8_t BB_LOX_PT_CH  = 0;
static constexpr uint8_t BB_FUEL_PT_CH = 1;

// DC press-solenoid channels (1-indexed; maps to SequenceHandler::channelArr[ch-1]).
static constexpr uint8_t BB_LOX_DC_CH  = 4;
static constexpr uint8_t BB_FUEL_DC_CH = 7;

// DC vent-solenoid channels (1-indexed). Set to BB_DC_CH_UNSET to disable
// auto-vent / abort for that side; those commands will be rejected with
// EVT:...:AV_NO_HW until a real channel is configured here.
// TODO(user): set these to the actual vent DC channels on your board.
static constexpr uint8_t BB_LOX_VENT_DC_CH  = BB_DC_CH_UNSET;
static constexpr uint8_t BB_FUEL_VENT_DC_CH = BB_DC_CH_UNSET;

// Venturi PT indices for mass-flow calculation (0-indexed into v2PtData[]).
// BB_PT_CH_UNSET disables mass-flow correction for that side regardless of
// the `mdot_enabled` config flag — no fake numbers will drive the loop.
// TODO(user): set these once the additional 4 venturi PTs are installed on V2.
static constexpr uint8_t BB_LOX_VENTURI_UP_PT  = BB_PT_CH_UNSET;
static constexpr uint8_t BB_LOX_VENTURI_DN_PT  = BB_PT_CH_UNSET;
static constexpr uint8_t BB_FUEL_VENTURI_UP_PT = BB_PT_CH_UNSET;
static constexpr uint8_t BB_FUEL_VENTURI_DN_PT = BB_PT_CH_UNSET;

// Sanity bounds — BB disables if PT reading falls outside this range.
static constexpr float BB_PRESSURE_MIN_PSI = -50.0f;
static constexpr float BB_PRESSURE_MAX_PSI = 4000.0f;

// Mass-flow correction update cadence (ms between setpoint nudges).
static constexpr uint32_t BB_MDOT_UPDATE_MS = 500;

// Venturi CdA (discharge coefficient × effective throat area) in m².
// Single value covers both LOX and Fuel venturis — update here if they
// are ever sized differently. With this form of the mass-flow equation
// upstream/throat areas are not used separately.
//
//   m_dot [kg/s] = CdA · sqrt( 2 · ρ · ΔP )
//   ΔP [Pa]      = (P_up − P_dn) [psi] · PSI_TO_PA
static constexpr float BB_VENTURI_CDA_M2 = 3.22e-5f;
static constexpr float PSI_TO_PA         = 6894.757f;

// EEPROM layout for BB config persistence. Magic bumped when BBConfig
// struct layout changed; old EEPROM contents ignored on mismatch.
static constexpr uint16_t BB_EEPROM_MAGIC = 0xBB44;
static constexpr int      BB_EEPROM_ADDR  = 0;

static constexpr uint8_t NUM_ACTUATORS = NUM_DC_CHANNELS;