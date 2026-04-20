#pragma once
#include <Arduino.h>
#include "hardware-configs/BoardConfig.hpp"

// Firmware-side bang-bang pressure controller (ported from PandaV2).
//
// Two instances (LOX, Fuel) run autonomously on V1 once enabled. Each reads a
// single PT channel from the ptData[] array fed by the V2 crossover, drives a
// hardwired DC solenoid on V1, and enforces a minimum dwell time between valve
// transitions to prevent chatter.
//
// Safety invariants:
//   - Always starts DISABLED on power-up, even if EEPROM config is present.
//   - Disarm forces disable + valve closed (call disableNow()).
//   - PT reading outside sanity bounds → auto-disable + valve closed.
//   - Cannot enable unless the caller has verified arm state.
//
// Config is persisted to EEPROM (magic + CRC). Call bbLoadEeprom() /
// bbSaveEeprom() from main.cpp.

struct BBConfig {
    float    setpoint_psi  = 200.0f;
    float    deadband_psi  = 10.0f;
    uint32_t wait_ms       = 500;
};

struct BBEepromBlock {
    uint16_t magic;
    BBConfig lox;
    BBConfig fuel;
    uint8_t  crc;
};

class BBController {
public:
    // ptSrc   — pointer into ptData[]; dereferenced every update()
    // dcCh    — 1-indexed DC channel (maps to SequenceHandler::channelArr)
    // busId   — 'L' or 'F', used only for telemetry labels
    BBController(const float* ptSrc, uint8_t dcCh, char busId);

    void configure(float setpoint, float deadband, uint32_t waitMs);

    // Enable: only call after verifying armed. Returns false if already enabled.
    bool enable();

    // Disable and force valve closed via the provided setChannel callback.
    void disableNow(bool (*setChannel)(uint8_t, bool));

    // Call every loop iteration. armed must reflect current arm state.
    void update(bool armed, bool (*setChannel)(uint8_t, bool));

    bool            isEnabled()    const { return _enabled; }
    bool            isValveOpen()  const { return _valveOpen; }
    float           lastPressure() const { return _lastPressure; }
    const BBConfig& config()       const { return _cfg; }
    char            busId()        const { return _busId; }

private:
    const float* _ptSrc;
    uint8_t      _dcCh;
    char         _busId;
    BBConfig     _cfg;
    bool         _enabled   = false;
    bool         _valveOpen = false;
    float        _lastPressure = 0.0f;
    elapsedMillis _switchTimer;

    void _closeValve(bool (*setChannel)(uint8_t, bool));
};

void bbLoadEeprom(BBController& lox, BBController& fuel);
void bbSaveEeprom(const BBController& lox, const BBController& fuel);
