#pragma once
#include "Scanner.hpp"
#include "hardware-configs/pins.hpp"
#include "drivers/MCP9802A0.hpp"
// #if 0

class FScanner : public Scanner {

    private:

    struct ChannelSettings {
        MuxSettings muxChannel;
        const uint8_t* muxPins;
        const uint8_t numChannels;
        float* out;
    };

    // MCP3561 adc(const uint8_t pinIRQ = ptADCPins.irq, const uint8_t pinMCLK = -1, const uint pinCS = ptADCPins.cs, SPIClass *theSPI = &SPI2, const uint8_t pinMOSI = ptADCPins.mosi, const uint8_t pinMISO = ptADCPins.miso, const uint8_t pinCLK = ptADCPins.sck);
    // MCP3561 adc(1, 2, 3, &SPI2, 2, 3);
    State state = IDLE;
    uint8_t channel = 0;
    uint8_t index = 0;
    uint8_t irqPin, csPin;
    elapsedMicros timer;
    
    MCP3561 adc;
    SPIClass& spiBus;
    SPISettings settings;


    uint8_t numSettings = 2;

    MCP9802A0 tSensor;
    float boardTemp;

    float ptOutput[NUM_PT_CHANNELS] = {0}, lctcOutput[NUM_TC_CHANNELS + NUM_LC_CHANNELS] = {0};

    ChannelSettings ptSettings = {
        MuxSettings::CH1,
        ptMux,
        NUM_PT_CHANNELS,
        ptOutput
    };

    ChannelSettings lctcSettings = {
        MuxSettings::CH0,
        lctcMux,
        NUM_LC_CHANNELS + NUM_TC_CHANNELS,
        lctcOutput
    };
    ChannelSettings settingsArr[2] = {lctcSettings, ptSettings};

    public:

    FScanner(uint8_t csPin_, uint8_t irqPin_, SPIClass& spiBus_, SPISettings settings_) : csPin(csPin_), irqPin(irqPin_), spiBus(spiBus_), settings(settings_), adc(csPin_, spiBus_, settings_, 1.25) {}
    void setup() override;
    void update() override;

    void getPTOutput(float* out);
    void getLCTCOutput(float* out);

};
// #endif