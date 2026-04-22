#pragma once

// RTU V2 — Teensy 4.1 pin assignments (PT-only firmware).
// Source: Microcontroller.SchDoc + ADC/RS-485 sheets.
// Only the pins used by the current firmware are kept here. The full board
// pinout (mux B/C, ADC2, INA230s, arming, etc.) lives in docs/pinout.md —
// restore defines from there if re-introducing those sensor paths.

// ---------------------------------------------------------------------------
// SPI bus 0 — ADC1 (MCP3561RT, U23)
// Hardware SPI (LPSPI4): MOSI=11, MISO=12, SCK=13
// ---------------------------------------------------------------------------
#define PIN_ADC1_CS     10
#define PIN_ADC1_IRQ    6   // active-low, open-drain; attach interrupt INPUT_PULLUP
#define PIN_SPI0_MOSI   11
#define PIN_SPI0_MISO   12
#define PIN_SPI0_SCK    13

// ---------------------------------------------------------------------------
// UART links
//
// PRIMARY (GC ↔ V2): RS-485 over the ISL83491 transceiver (U32, bus 1).
//   /RE tied to GND, DE tied to +3V3 → point-to-point, always driving.
//   No GPIO DE control; firmware passes 0xFF to CommsHandler.
//
// SECONDARY (V1 ↔ V2 crossover): direct TTL UART, no transceiver.
//   Bus-2 RS-485 transceiver has been bypassed — Teensy TX/RX are wired
//   straight to V1's Serial5 TX/RX with a common GND.
// ---------------------------------------------------------------------------
#define PIN_RS485_1_RX  28   // Serial7 RX
#define PIN_RS485_1_TX  29   // Serial7 TX
#define PIN_XOVER_RX    25   // Serial6 RX ← V1 pin 20 (TX)
#define PIN_XOVER_TX    24   // Serial6 TX → V1 pin 21 (RX)
#define PIN_RS485_1_DE  0xFF // DE tied to +3V3; no GPIO control

// ---------------------------------------------------------------------------
// Analog mux A — CD74HCT4067 (U21), feeds ADC1 CH0 (PTs only).
// EN hardwired to GND (always active). Write S0..S3, wait for settle, convert.
// ---------------------------------------------------------------------------
#define PIN_MUX_A_S0    5
#define PIN_MUX_A_S1    4
#define PIN_MUX_A_S2    3
#define PIN_MUX_A_S3    2
