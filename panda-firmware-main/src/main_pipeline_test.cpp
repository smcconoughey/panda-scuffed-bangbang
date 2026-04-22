#include <Arduino.h>

static constexpr uint32_t BAUD = 460800;
static constexpr uint32_t STATS_PERIOD_MS = 1000;

static elapsedMillis statsTimer;
static uint32_t bytesFromV2 = 0;
static uint32_t newlineCount = 0;

void setup() {
  Serial.begin(BAUD);  // USB debug
  Serial2.begin(BAUD); // V1 -> GC
  Serial5.begin(BAUD); // V2 -> V1 crossover

  Serial.println("V1 pipeline test bridge ready");
}

void loop() {
  while (Serial5.available() > 0) {
    const int raw = Serial5.read();
    if (raw < 0) {
      break;
    }

    const uint8_t b = static_cast<uint8_t>(raw);
    Serial2.write(b); // byte-for-byte forward to GC

    bytesFromV2++;
    if (b == '\n' || b == '\r') {
      newlineCount++;
    }
  }

  if (statsTimer >= STATS_PERIOD_MS) {
    statsTimer = 0;
    Serial.print("V1_BRIDGE bytes=");
    Serial.print(bytesFromV2);
    Serial.print(" newlines=");
    Serial.print(newlineCount);
    Serial.print(" s5_avail=");
    Serial.println(Serial5.available());
  }
}
