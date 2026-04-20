#include "TelemetryHandler.hpp"

void TelemetryHandler::poll() {

    while (stream.available() > 0) {

        char c = stream.read();
        timer = 0;

        if (c == '\r' || c == '\n') {
            if (index > 0) {
                rxBuf[index] = '\0';
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
            packetReady = true;
            index = 0;
            break;
        }

       
    }
    if (!packetReady && index > 0 && timer > PACKET_IDLE_MS) {
        rxBuf[index] = '\0';
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
    size_t used = 0; 

    for (size_t i = 0; i < n ; ++i) {
        char digits[32];
        digits[0] = identifier;                                // scratch for one number
        dtostrf(data[i], 0, decimals, digits + 1);         // <-- write INTO digits
        size_t len = strlen(digits);

        // need = number chars + 1 delimiter (',' or '\n') + 1 final '\0' reserve
        size_t need = len + 1 + 1;
        if (used + need > outSize) return false;        // prevent overflow

        memcpy(out + used, digits, len);                // append number
        used += len;

        out[used++] = (i + 1 < n) ? ',' : '\n';         // delimiter
    }

    out[used] = '\0';                                  // null-terminate
    return true;
    
}