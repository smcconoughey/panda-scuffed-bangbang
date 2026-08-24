#include "TelemetryHandler.hpp"

void TelemetryHandler::poll() {

    while (stream.available() > 0) {

        char c = stream.read();
        timer = 0;

        if (c == '\r' || c == '\n') {
            if (index > 0) {
                rxBuf[index] = '\0';
                lastPacketLen = index;
                lastPacketHadDelimiter = true;
                packetReady = true;
            }
            index = 0;
            break;
        }

        if (index < sizeof(rxBuf) - 1) {
            rxBuf[index++] = c;
        }

        else {
            rxBuf[sizeof(rxBuf) - 1] = '\0';
            lastPacketLen = sizeof(rxBuf) - 1;
            lastPacketHadDelimiter = false;
            packetReady = true;
            index = 0;
            break;
        }

       
    }
    if (!packetReady && index > 0 && timer > PACKET_IDLE_MS) {
        rxBuf[index] = '\0';
        lastPacketLen = index;
        lastPacketHadDelimiter = false;
        packetReady = true;
        index = 0;
    }

}

char* TelemetryHandler::takePacket() {
    packetReady = false;
    return rxBuf;
}

bool TelemetryHandler::isPacketReady() {
    return packetReady;
}

bool TelemetryHandler::toCSVRow(const float* data, char identifier, size_t n, char* out, size_t outSize, uint8_t decimals) {
    if (!data || !out || outSize == 0) return false;
    out[0] = '\0';
    size_t used = 0;

    for (size_t i = 0; i < n ; ++i) {
        char digits[32];
        const int written = snprintf(digits, sizeof(digits), "%c%.*f",
                                     identifier, decimals, data[i]);
        if (written < 0 || static_cast<size_t>(written) >= sizeof(digits)) {
            out[used] = '\0';
            return false;
        }
        const size_t len = static_cast<size_t>(written);

        // need = number chars + 1 delimiter (',' or '\n') + 1 final '\0' reserve
        size_t need = len + 1 + 1;
        if (used + need > outSize) {
            out[used] = '\0';
            return false;
        }

        memcpy(out + used, digits, len);                // append number
        used += len;

        out[used++] = (i + 1 < n) ? ',' : '\n';         // delimiter
    }

    out[used] = '\0';                                  // null-terminate
    return true;
    
}
