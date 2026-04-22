#pragma once
#include <Arduino.h>
#include "hardware-configs/BoardConfig.hpp"

class TelemetryHandler {
    private:

    Stream& stream;

    char rxBuf[RX_BUF_SIZE] = {0};

    size_t rxBufSize;
    size_t index = 0;
    size_t lastPacketLen = 0;
    bool lastPacketHadDelimiter = false;

    bool packetReady = false; // Packet ready flag

    elapsedMillis timer = 0; // Polling timer

    public:

    TelemetryHandler(Stream &stream_, size_t rxBufSize_ = 128) : stream(stream_), rxBufSize(rxBufSize_) {}

    // Receiver functions

    void poll();
    bool isPacketReady();
    char* takePacket();
    size_t getLastPacketLen() const { return lastPacketLen; }
    bool didLastPacketEndWithDelimiter() const { return lastPacketHadDelimiter; }
    void reset();

    // quality of life functions
    bool toCSVRow(const float* data, char identifier, size_t n, char* out, size_t outSize, uint8_t decimals);

};