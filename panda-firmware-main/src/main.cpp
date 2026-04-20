#include <FlexSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include "hardware-configs/pins.hpp"

// #include "scanners/Scanner.hpp"
#include "scanners/FScanner.hpp" // Fluids DAQ
#include "scanners/SScanner.hpp" // Solenoid current DAQ

#include "telemetry/TelemetryHandler.hpp"
#include "dc-controllers/SequenceHandler.hpp"
#include "board-functions/ArmingController.hpp"
#include "board-functions/BangBang.hpp"

#include "drivers/MCP9802A0.hpp" // Temperature sensors

/*
TODO:
- Replace while(Serial.available()) with non-blocking reads
- Clean up SPI and ADC initialization
- Implement Bang-Bang controllers and DCChannel object *NOT BEING IMPLEMENTED 10/7/2025*
- Add channel state to data packet
- Implement telemetry on/off control
- Rewrite SequenceHandler to use dcChannels array
- Overhaul telemetry
- Parallelize ADC readings

*/

/**
 * SERIAL PROTOCOL (Baud rate: 460800)
 *
 * Command Format (Serial2 primary ← GC):
 * 'a' - Arm
 * 'r' - Disarm (mirrors V2; also clears BB state)
 * 's<Channel Identifier in Hex><State>.<Five digit delay in milliseconds>' - Solenoid Sequence
 * 'S<Channel Identifier in Hex><State>' - Solenoid command
 * 'f' - Fire loaded sequence
 * 'B<side><setpoint>,<deadband>,<wait_ms>' - Configure bang-bang (side = L or F)
 * 'b<side><0|1>' - Enable/disable bang-bang (requires armed)
 *
 * Telemetry Format:
 * 'p<PT voltage>' x16 - Pressure Transducer Packet (forwarded from V2)
 * 's<Voltage across shunt resistor>' x12 - Solenoid Current Packet
 * 'l/t<Conditioned load cell/thermocouple voltage>' - LC+TC Packet
 *
 * Serial5 is a dedicated crossover from Panda V2 carrying PT CSV rows only.
 */

TelemetryHandler th(Serial2, 1024);              // primary ← GC
TelemetryHandler thXover(Serial5, 1024);         // secondary ← V2 PT crossover
SequenceHandler sh;
ArmingController ac(PIN_ARM, PIN_DISARM);

SScanner sScanner(sADCPins.cs, sADCPins.irq, SPI, SPISettingsDefault);
FScanner fScanner(ptADCPins.cs, ptADCPins.irq, SPI1, SPISettingsDefault);

// ── V2-forwarded PT data (filled by secondary-bus parser) ─────────────────
static float v2PtData[NUM_PT_CHANNELS] = {0};

// Mirror of master-arm state maintained alongside the existing 'a'/'r'
// handlers. V1 ArmingController is left untouched (per design) — this flag
// is what BB gates on.
static bool gArmed = false;

// ── Bang-bang controllers ────────────────────────────────────────────────
BBController bbLox (&v2PtData[BB_LOX_PT_CH],  BB_LOX_DC_CH,  'L');
BBController bbFuel(&v2PtData[BB_FUEL_PT_CH], BB_FUEL_DC_CH, 'F');

// ── BB-owned DC channel table ────────────────────────────────────────────
// Tracks which DC channels are explicitly claimed by the bang-bang
// controller in firmware. While a channel is owned by BB, manual 'S'
// commands on that channel are rejected so GC/operator toggles cannot
// race the BB loop. The only path to drive an owned channel is through
// bbSetChannel() invoked by BBController::update().
static bool gBbOwned[NUM_DC_CHANNELS] = {false};

static bool bbClaimChannel(uint8_t ch1) {
    if (ch1 < 1 || ch1 > NUM_DC_CHANNELS) return false;
    gBbOwned[ch1 - 1] = true;
    return true;
}

static void bbReleaseChannel(uint8_t ch1) {
    if (ch1 < 1 || ch1 > NUM_DC_CHANNELS) return;
    gBbOwned[ch1 - 1] = false;
}

static bool isBbOwned(uint8_t ch1) {
    if (ch1 < 1 || ch1 > NUM_DC_CHANNELS) return false;
    return gBbOwned[ch1 - 1];
}

// Explicit "bang-bang → DC" write helper. Only BBController::update()
// reaches the DC pins through this function.
static bool bbSetChannel(uint8_t ch1, bool state) {
    if (ch1 < 1 || ch1 > NUM_DC_CHANNELS) return false;
    sh.channelArr[ch1 - 1].setState(state);
    return true;
}

// ── Parse PT CSV from V2 into v2PtData[] ────────────────────────────────
// Expected format: "p<f0>,p<f1>,...,p<fN>\n". Identifier repeats per value
// (matches V2 CommsHandler::toCSVRow output).
static void parseV2PtPacket(char* packet) {
    if (!packet || packet[0] == '\0') return;

    uint8_t idx = 0;
    char* tok = strtok(packet, ",");
    while (tok && idx < NUM_PT_CHANNELS) {
        const char* num = (tok[0] == PT_IDENTIFIER) ? tok + 1 : tok;
        v2PtData[idx++] = atof(num);
        tok = strtok(nullptr, ",");
    }
}

void setup() {
  // Primary RS-485 (GC)
  Serial2.begin(SERIAL_BAUD_RATE);
  Serial2.setTimeout(100);
  static uint8_t rxBuf[RX_BUF_SIZE];
  Serial2.addMemoryForRead(rxBuf, RX_BUF_SIZE);
  static uint8_t txBuf[TX_BUF_SIZE];
  Serial2.addMemoryForWrite(txBuf, TX_BUF_SIZE);

  // Secondary RS-485 (crossover from V2: Serial6 ↔ V1 Serial5)
  Serial5.begin(SERIAL_BAUD_RATE);
  Serial5.setTimeout(100);
  static uint8_t rxBufXover[RX_BUF_SIZE];
  Serial5.addMemoryForRead(rxBufXover, RX_BUF_SIZE);
  static uint8_t txBufXover[TX_BUF_SIZE];
  Serial5.addMemoryForWrite(txBufXover, TX_BUF_SIZE);

  Serial.begin(SERIAL_BAUD_RATE); // Debugging via serial monitor

  SPI.begin();
  SPI.setClockDivider(4);

  SPI1.begin();
  SPI1.setClockDivider(4);

  Wire2.begin();

  // Call scanner setup()'s here
  sh.setup();
  // ac.setup();
  sScanner.setup();
  fScanner.setup();

  pinMode(PIN_DISARM, OUTPUT);
  pinMode(PIN_ARM, OUTPUT);

  digitalWrite(PIN_DISARM, HIGH);

  // Load persisted BB config (setpoints / deadbands / wait times)
  bbLoadEeprom(bbLox, bbFuel);

  Serial2.println("Panda Initialized!");

}

void loop() {
  // // put your main code here, to run repeatedly:

  char idChar;

  // ── Secondary bus: drain PT forwards from V2 ────────────────────────
  thXover.poll();
  if (thXover.isPacketReady()) {
    char* xPacket = thXover.takePacket();
    if (xPacket[0] == PT_IDENTIFIER) {
      parseV2PtPacket(xPacket);
    }
    size_t n = strnlen(xPacket, RX_BUF_SIZE);
    memset(xPacket, 0, n);
  }

  // ── Primary bus: commands from GC ───────────────────────────────────
  th.poll();
  if (th.isPacketReady()) {
    char* rxPacket = th.takePacket();
    Serial.println(rxPacket);
    idChar = rxPacket[0];
    // Feed rxPacket into command handler
    if (idChar == 's') {
      Serial.print("Sequence packet: ");
      Serial.println(rxPacket);
      sh.setCommand(rxPacket);
    }

    else if (idChar == 'S') {

      char channelChar = rxPacket[1];
      char stateChar = rxPacket[2];

      unsigned channel, state;

      if (channelChar >= '0' && channelChar <= '9') channel = channelChar - '0';
      else channel = 10 + (toupper(channelChar) - 'A');     // A-F

      state = stateChar - '0';

      // digitalWrite(dcChannels[channel - 1], state);
      if (channel >= 1 && channel <= NUM_DC_CHANNELS) {
        if (isBbOwned(channel)) {
          Serial2.print("CMD_ERROR: chan ");
          Serial2.print(channel);
          Serial2.println(" owned by BB");
        } else {
          sh.channelArr[channel - 1].setState(state);
          Serial2.print("Solenoid Command: ");
          Serial2.print(channel);
          Serial2.print(" | ");
          Serial2.println(state);
        }
      }

    }

    else if (idChar == 'a') {
      // Arm
      // ac.setState(ArmingController::ARM);

      digitalWrite(PIN_DISARM, LOW);
      digitalWrite(PIN_ARM, HIGH);
      gArmed = true;
      Serial2.println("Arming!");

    }

    else if (idChar == 'r') {
      // Disarm
      // ac.setState(ArmingController::DISARM);
      digitalWrite(PIN_ARM, LOW);
      digitalWrite(PIN_DISARM, HIGH);
      gArmed = false;
      // Force BB down with valves closed before anything else runs
      bbLox.disableNow(bbSetChannel);
      bbFuel.disableNow(bbSetChannel);
      bbReleaseChannel(BB_LOX_DC_CH);
      bbReleaseChannel(BB_FUEL_DC_CH);
      Serial.println("Disarming!");
    sh.cancelExecution();
    sh.setAllChannelsOff();
    Serial2.println("SEQ_ABORT: Outputs de-energized");
    if (sh.hasSequence()) {
      Serial2.print("SEQ_READY:");
      Serial2.println(sh.getLastCommand());
    }
    }

    else if (idChar == 'f') {
    Serial.println(rxPacket);
    if (!sh.hasSequence()) {
      Serial2.println("SEQ_ERROR: No sequence loaded");
    } else {
      Serial2.print("SEQ_EXEC_START:count=");
      Serial2.print(sh.getNumCommands());
      const char* lastCmd = sh.getLastCommand();
      if (lastCmd && lastCmd[0] != '\0') {
        Serial2.print(",raw=");
        Serial2.println(lastCmd);
      } else {
        Serial2.println();
      }
      sh.execute(true);
      Serial2.println("Firing sequence!");
    }
    }

    // ── Bang-bang configuration ('B' = configure, 'b' = enable/disable) ──
    else if (idChar == 'B') {
      // 'B<side><setpoint>,<deadband>,<wait_ms>'
      if (strlen(rxPacket) < 4) {
        Serial2.println("BB_ERROR:short");
      } else {
        char side = rxPacket[1];
        float sp, db;
        unsigned long wt;
        if ((side != 'L' && side != 'F') ||
            sscanf(rxPacket + 2, "%f,%f,%lu", &sp, &db, &wt) != 3 ||
            sp < 0.0f || db <= 0.0f || wt > 60000UL) {
          Serial2.println("BB_ERROR:parse");
        } else {
          BBController& ctrl = (side == 'L') ? bbLox : bbFuel;
          ctrl.configure(sp, db, (uint32_t)wt);
          bbSaveEeprom(bbLox, bbFuel);
          Serial2.print("BB_CFG:");
          Serial2.print(side);
          Serial2.print(':'); Serial2.print(sp, 1);
          Serial2.print(':'); Serial2.print(db, 1);
          Serial2.print(':'); Serial2.println(wt);
        }
      }
    }

    else if (idChar == 'b') {
      // 'b<side><0|1>'
      if (strlen(rxPacket) < 3) {
        Serial2.println("BB_ERROR:short");
      } else {
        char side = rxPacket[1];
        char stCh = rxPacket[2];
        if ((side != 'L' && side != 'F') || (stCh != '0' && stCh != '1')) {
          Serial2.println("BB_ERROR:bad_arg");
        } else {
          BBController& ctrl = (side == 'L') ? bbLox : bbFuel;
          uint8_t dcCh = (side == 'L') ? BB_LOX_DC_CH : BB_FUEL_DC_CH;
          if (stCh == '1') {
            if (!gArmed) {
              Serial2.println("BB_ERROR:not_armed");
            } else {
              bbClaimChannel(dcCh);
              ctrl.enable();
              Serial2.print("BB:"); Serial2.print(side); Serial2.println(":ON");
            }
          } else {
            ctrl.disableNow(bbSetChannel);
            bbReleaseChannel(dcCh);
            Serial2.print("BB:"); Serial2.print(side); Serial2.println(":OFF");
          }
        }
      }
    }

    // Resetting all incoming buffers
    size_t n = strnlen(rxPacket, 256);   // your rxBuf capacity
    memset(rxPacket, 0, n);
    // memset(rxPacket, 0, sizeof(rxPacket));
    // packetReady = false;
  }

  // ac.update();
  sh.update();

  // ── Bang-bang step ───────────────────────────────────────────────────
  bbLox.update(gArmed,  bbSetChannel);
  bbFuel.update(gArmed, bbSetChannel);


  // ========== DAQ ==========
  sScanner.update();
  fScanner.update();

  float sData[NUM_DC_CHANNELS] = {0}, lctcData[NUM_LC_CHANNELS + NUM_TC_CHANNELS];
  sScanner.getSOutput(sData);
  fScanner.getLCTCOutput(lctcData);

  char sPacket[512], lctcPacket[512], ptPacket[512];

  th.toCSVRow(sData, S_IDENTIFIER, NUM_DC_CHANNELS, sPacket, 512, 5);
  th.toCSVRow(lctcData, LCTC_IDENTIFIER, NUM_TC_CHANNELS + NUM_LC_CHANNELS, lctcPacket, 512, 5);
  // Re-transmit V2 PTs to GC
  th.toCSVRow(v2PtData, PT_IDENTIFIER, NUM_PT_CHANNELS, ptPacket, 512, 5);

  // uint32_t current = millis();
  Serial2.print(lctcPacket);
  // Serial.print(lctcPacket);
  Serial2.print(sPacket);
  Serial2.print(ptPacket);

  // BB status line (operator visibility into firmware state)
  Serial2.print("BB:L:");
  Serial2.print(bbLox.isEnabled() ? 1 : 0);
  Serial2.print(':');
  Serial2.print(bbLox.isValveOpen() ? 1 : 0);
  Serial2.print(':');
  Serial2.println(bbLox.lastPressure(), 1);

  Serial2.print("BB:F:");
  Serial2.print(bbFuel.isEnabled() ? 1 : 0);
  Serial2.print(':');
  Serial2.print(bbFuel.isValveOpen() ? 1 : 0);
  Serial2.print(':');
  Serial2.println(bbFuel.lastPressure(), 1);

  // uint32_t tTime = millis() - current;
  // Serial.print("Transmission time: ");
  // Serial.println(tTime);
}


