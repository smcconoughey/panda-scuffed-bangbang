#pragma once

#include "Scanner.hpp"
#include "hardware-configs/pins.hpp"

class SScanner : public Scanner {

    private:

    MCP3561 adc;
    MuxSettings adcCh = MuxSettings::CH0;

    State state = IDLE;
    // uint8_t bank = 0;
    uint8_t channel = 0;
    elapsedMicros timer;

    SPIClass& spiBus;
    SPISettings settings;

    uint8_t irqPin, csPin;

    public:

    float sData[NUM_DC_CHANNELS] = {0};

    SScanner(uint8_t csPin_, uint8_t irqPin_, SPIClass& spiBus_, SPISettings settings_) : csPin(csPin_), irqPin(irqPin_), spiBus(spiBus_), settings(settings_), adc(csPin_, spiBus_, settings_, 3.3) {}
    

    void getSOutput(float* out);
    void setup() override;
    void update() override;


};
