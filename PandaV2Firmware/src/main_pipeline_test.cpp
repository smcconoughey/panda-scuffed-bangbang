#include <Arduino.h>

static constexpr uint32_t BAUD = 460800;
static constexpr uint32_t SEND_PERIOD_MS = 50;   // 20 Hz
static constexpr uint32_t STATS_PERIOD_MS = 1000;

static elapsedMillis sendTimer;
static elapsedMillis statsTimer;
static uint32_t packetCounter = 0;
static uint32_t bytesSent = 0;

void setup() {
  Serial.begin(BAUD);  // USB debug
  Serial6.begin(BAUD); // V2 pin 24(TX) -> V1 pin 21(RX)

  Serial.println("V2 pipeline test sender ready");
}

void loop() {
  if (sendTimer >= SEND_PERIOD_MS) {
    sendTimer = 0;

    char line[96];
    const int n = snprintf(line, sizeof(line), "P2,n=%lu,ms=%lu,sig=A55A\n",
                           static_cast<unsigned long>(packetCounter++),
                           static_cast<unsigned long>(millis()));
    if (n > 0) {
      Serial6.print(line);
      bytesSent += static_cast<uint32_t>(n);
    }
  }

  if (statsTimer >= STATS_PERIOD_MS) {
    statsTimer = 0;
    Serial.print("V2_TX packets=");
    Serial.print(packetCounter);
    Serial.print(" bytes=");
    Serial.println(bytesSent);
  }
}
