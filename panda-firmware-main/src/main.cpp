#include <EEPROM.h>
#include <FlexSerial.h>
#include <SPI.h>
#include <Wire.h>

#include "hardware-configs/pins.hpp"

// #include "scanners/Scanner.hpp"
#include "scanners/FScanner.hpp" // Fluids DAQ
#include "scanners/SScanner.hpp" // Solenoid current DAQ

#include "board-functions/ArmingController.hpp"
#include "board-functions/BangBang.hpp"
#include "dc-controllers/SequenceHandler.hpp"
#include "telemetry/TelemetryHandler.hpp"

#include "drivers/MCP9802A0.hpp" // Temperature sensors

/**
 * SERIAL PROTOCOL (Baud rate: 460800)
 *
 * See docs/GC_USERS_GUIDE.md for the canonical command + telemetry reference.
 * Summary:
 *
 * Commands (Serial2 ← GC):
 *   'a'                                 Arm
 *   'r'                                 Disarm (also forceSafe()s BB)
 *   'S<chHex><state>'                   Direct solenoid command
 *   's<chHex><state>.<ms 5-digit>'      Sequence step append
 *   'f'                                 Fire loaded sequence
 *   'B<side><sp>,<db>,<wait>,<maxOpen>' Configure BB core (L or F)
 *   'V<side><trig>,<autoOn01>'          Configure BB auto-vent
 *   'M<side><mdot>,<spMin>,<spMax>,<gain>,<rho>,<on01>'  Configure BB massflow
 *   'b<side><0|1>'                      Arm/disarm bang-bang control
 *   'v<side><0|1>'                      Manual vent open(1) / close(0)
 *   'x<side>'                           Latched abort (cleared only by 'r')
 *
 * Telemetry (Serial2 → GC):
 *   'p<f0>,p<f1>,...'   — 16 PT voltages, forwarded from V2
 *   's<f0>,s<f1>,...'   — 12 solenoid current voltages (V1 local)
 *   't<f0>,t<f1>,...'   — 12 LC+TC voltages (V1 local)
 *   'BB:<L|F>:<state>:<press>:<vent>:<pressure>'   1 Hz summary heartbeat
 *   'EVT:<ms>:<cat>:<L|F>:<detail>'                audit event on every BB
 * transition
 *
 * Serial5 is a direct TTL crossover from Panda V2 carrying PT CSV rows only.
 * The RS-485 transceiver on this bus is bypassed: V1 pin 20 → V2 pin 25,
 * V2 pin 24 → V1 pin 21, common GND. No DE control. Short-run point-to-point.
 */

TelemetryHandler th(Serial2, 1024);      // primary ← GC
TelemetryHandler thXover(Serial5, 1024); // secondary ← V2 PT crossover
SequenceHandler sh;
ArmingController ac(PIN_ARM, PIN_DISARM);

SScanner sScanner(sADCPins.cs, sADCPins.irq, SPI, SPISettingsDefault);
FScanner fScanner(ptADCPins.cs, ptADCPins.irq, SPI1, SPISettingsDefault);

// ── V2-forwarded PT data (filled by secondary-bus parser) ─────────────────
static float v2PtData[NUM_PT_CHANNELS] = {0};

// Mirror of master-arm state (what BB gates on).
static bool gArmed = false;

// ── Bang-bang controllers ────────────────────────────────────────────────
BBController bbLox(v2PtData, BB_LOX_PT_CH, BB_LOX_DC_CH, BB_LOX_VENT_DC_CH,
                   BB_LOX_VENTURI_UP_PT, BB_LOX_VENTURI_DN_PT, 'L');
BBController bbFuel(v2PtData, BB_FUEL_PT_CH, BB_FUEL_DC_CH, BB_FUEL_VENT_DC_CH,
                    BB_FUEL_VENTURI_UP_PT, BB_FUEL_VENTURI_DN_PT, 'F');

// ── BB → DC channel setter ───────────────────────────────────────────────
static bool bbSetChannel(uint8_t ch1, bool state) {
  if (ch1 < 1 || ch1 > NUM_DC_CHANNELS)
    return false;
  sh.channelArr[ch1 - 1].setState(state);
  return true;
}

// ── BB → audit-event emitter ─────────────────────────────────────────────
// Format: EVT:<ms>:<cat>:<side>:<detail>\n
static void bbEmit(const char *cat, char side, const char *detail) {
  Serial2.print("EVT:");
  Serial2.print(millis());
  Serial2.print(':');
  Serial2.print(cat);
  Serial2.print(':');
  Serial2.print(side);
  Serial2.print(':');
  Serial2.println(detail ? detail : "");
}

// ── Channel-ownership check (manual S commands blocked on BB channels) ───
static bool isBbOwned(uint8_t ch1) {
  return bbLox.ownsChannel(ch1) || bbFuel.ownsChannel(ch1);
}

// ── Parse PT CSV from V2 into v2PtData[] ────────────────────────────────
static void parseV2PtPacket(char *packet) {
  if (!packet || packet[0] == '\0')
    return;

  uint8_t idx = 0;
  char *tok = strtok(packet, ",");
  while (tok && idx < NUM_PT_CHANNELS) {
    const char *num = (tok[0] == PT_IDENTIFIER) ? tok + 1 : tok;
    v2PtData[idx++] = atof(num);
    tok = strtok(nullptr, ",");
  }
}

// ── BB command dispatch ──────────────────────────────────────────────────
// Each handler returns true if the packet was consumed as a BB command so
// main can skip further parsing.
static BBController *pickSide(char c) {
  if (c == 'L')
    return &bbLox;
  if (c == 'F')
    return &bbFuel;
  return nullptr;
}

static void handleB(const char *pkt) { // core config
  if (strlen(pkt) < 4) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  float sp, db;
  unsigned long wt, maxOpen;
  if (sscanf(pkt + 2, "%f,%f,%lu,%lu", &sp, &db, &wt, &maxOpen) != 4 ||
      sp < 0.0f || db <= 0.0f || wt > 60000UL || maxOpen > 60000UL) {
    Serial2.println("BB_ERROR:parse");
    return;
  }
  ctrl->configureCore(sp, db, (uint32_t)wt, (uint32_t)maxOpen);
  bbSaveEeprom(bbLox, bbFuel);
}

static void handleV(const char *pkt) { // auto-vent config
  if (strlen(pkt) < 4) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  float trig;
  int autoOn;
  if (sscanf(pkt + 2, "%f,%d", &trig, &autoOn) != 2 ||
      (autoOn != 0 && autoOn != 1)) {
    Serial2.println("BB_ERROR:parse");
    return;
  }
  ctrl->configureVent(trig, autoOn != 0);
  bbSaveEeprom(bbLox, bbFuel);
}

static void handleM(const char *pkt) { // massflow config
  if (strlen(pkt) < 4) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  float mdot, spMin, spMax, gain, rho;
  int on;
  if (sscanf(pkt + 2, "%f,%f,%f,%f,%f,%d", &mdot, &spMin, &spMax, &gain, &rho,
             &on) != 6 ||
      (on != 0 && on != 1) || spMin > spMax || rho < 0.0f) {
    Serial2.println("BB_ERROR:parse");
    return;
  }
  ctrl->configureMdot(mdot, spMin, spMax, gain, rho, on != 0);
  bbSaveEeprom(bbLox, bbFuel);
}

static void handleLowerB(const char *pkt) { // enable/disable sustain
  if (strlen(pkt) < 3) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  char stCh = pkt[2];
  if (stCh == '1') {
    if (!gArmed) {
      Serial2.println("BB_ERROR:not_armed");
      return;
    }
    ctrl->enableSustain();
  } else if (stCh == '0') {
    ctrl->disableSustain();
  } else {
    Serial2.println("BB_ERROR:bad_arg");
  }
}

static void handleLowerV(const char *pkt) { // manual vent
  if (strlen(pkt) < 3) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  char stCh = pkt[2];
  if (stCh == '1') {
    if (!gArmed) {
      Serial2.println("BB_ERROR:not_armed");
      return;
    }
    ctrl->manualVent();
  } else if (stCh == '0') {
    ctrl->manualVentClose(false);
  } else {
    Serial2.println("BB_ERROR:bad_arg");
  }
}

static void handleLowerX(const char *pkt) { // latched abort
  if (strlen(pkt) < 2) {
    Serial2.println("BB_ERROR:short");
    return;
  }
  BBController *ctrl = pickSide(pkt[1]);
  if (!ctrl) {
    Serial2.println("BB_ERROR:bad_side");
    return;
  }
  ctrl->latchAbort();
}

// ── 1 Hz BB summary heartbeat ────────────────────────────────────────────
static const char *stateStr(BBState s) {
  switch (s) {
  case BBState::DISABLED:
    return "OFF";
  case BBState::SUSTAIN:
    return "SUS";
  case BBState::AUTO_VENT:
    return "AV";
  case BBState::ABORT:
    return "ABT";
  }
  return "??";
}

static void printBbHeartbeat(const BBController &c) {
  Serial2.print("BB:");
  Serial2.print(c.busId());
  Serial2.print(':');
  Serial2.print(stateStr(c.state()));
  Serial2.print(':');
  Serial2.print(c.isPressOpen() ? 1 : 0);
  Serial2.print(':');
  Serial2.print(c.isVentOpen() ? 1 : 0);
  Serial2.print(':');
  Serial2.println(c.lastPressure(), 1);
}

// Compact debug helper for validating UART framing bytes without flooding logs.
static void printPacketHexBrief(const char *label, const char *packet,
                                size_t maxLen) {
  if (!packet) {
    return;
  }

  const size_t n = strnlen(packet, maxLen);
  Serial.print("HEX ");
  Serial.print(label);
  Serial.print(" len=");
  Serial.print(n);
  Serial.print(" head=");

  const size_t headCount = (n < 8) ? n : 8;
  for (size_t i = 0; i < headCount; i++) {
    if (i)
      Serial.print(' ');
    if ((uint8_t)packet[i] < 0x10)
      Serial.print('0');
    Serial.print((uint8_t)packet[i], HEX);
  }

  Serial.print(" tail=");
  const size_t tailCount = (n < 3) ? n : 3;
  for (size_t i = n - tailCount; i < n; i++) {
    if (i != n - tailCount)
      Serial.print(' ');
    if ((uint8_t)packet[i] < 0x10)
      Serial.print('0');
    Serial.print((uint8_t)packet[i], HEX);
  }
  Serial.println();
}

void setup() {
  // Primary RS-485 (GC) — Serial2 = LPUART4, pins 7(RX)/8(TX).
  // TX differential pair (Y/Z) is swapped on the V1 PCB, same as V2.
  // TXINV corrects polarity so the GC sees valid UART. RXINV stays off —
  // RX pair (A/B) is correct. TODO: fix Y/Z routing in next board revision.
  Serial2.begin(SERIAL_BAUD_RATE);
  Serial2.setTimeout(100);
  static uint8_t rxBuf[RX_BUF_SIZE];
  Serial2.addMemoryForRead(rxBuf, RX_BUF_SIZE);
  static uint8_t txBuf[TX_BUF_SIZE];
  Serial2.addMemoryForWrite(txBuf, TX_BUF_SIZE);

  // Secondary crossover (direct TTL UART from V2 Serial6: V2 pin 24 → V1 pin
  // 21, V2 pin 25 ← V1 pin 20). RS-485 transceiver bypassed on this bus.
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

  sh.setup();
  sScanner.setup();
  fScanner.setup();

  pinMode(PIN_DISARM, OUTPUT);
  pinMode(PIN_ARM, OUTPUT);
  digitalWrite(PIN_DISARM, HIGH);

  // Wire BB IO and load persisted config. Controllers come up DISABLED
  // regardless of what EEPROM contained — config is restored but the state
  // machine starts safe.
  bbLox.bindIO(bbSetChannel, bbEmit);
  bbFuel.bindIO(bbSetChannel, bbEmit);
  bbLox.forceSafe();
  bbFuel.forceSafe();
  bbLoadEeprom(bbLox, bbFuel);

  Serial2.println("Panda Initialized!");
}

void loop() {
  char idChar;

  // ── Secondary bus: drain PT forwards from V2 ────────────────────────
  thXover.poll();
  if (thXover.isPacketReady()) {
    char *xPacket = thXover.takePacket();
    if (xPacket[0] == PT_IDENTIFIER) {
      parseV2PtPacket(xPacket);
    }
    size_t n = strnlen(xPacket, RX_BUF_SIZE);
    memset(xPacket, 0, n);
  }

  // ── Primary bus: commands from GC ───────────────────────────────────
  th.poll();
  if (th.isPacketReady()) {
    char *rxPacket = th.takePacket();
    Serial.println(rxPacket);
    idChar = rxPacket[0];

    if (idChar == 's') {
      sh.setCommand(rxPacket);
    } else if (idChar == 'S') {
      char channelChar = rxPacket[1];
      char stateChar = rxPacket[2];
      unsigned channel, state;
      if (channelChar >= '0' && channelChar <= '9')
        channel = channelChar - '0';
      else
        channel = 10 + (toupper(channelChar) - 'A');
      state = stateChar - '0';

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
    } else if (idChar == 'a') {
      digitalWrite(PIN_DISARM, LOW);
      digitalWrite(PIN_ARM, HIGH);
      gArmed = true;
      Serial2.println("Arming!");
    } else if (idChar == 'r') {
      digitalWrite(PIN_ARM, LOW);
      digitalWrite(PIN_DISARM, HIGH);
      gArmed = false;
      bbLox.forceSafe();
      bbFuel.forceSafe();
      sh.cancelExecution();
      sh.setAllChannelsOff();
      Serial2.println("Disarming!");
      Serial2.println("SEQ_ABORT: Outputs de-energized");
      if (sh.hasSequence()) {
        Serial2.print("SEQ_READY:");
        Serial2.println(sh.getLastCommand());
      }
    } else if (idChar == 'f') {
      if (!sh.hasSequence()) {
        Serial2.println("SEQ_ERROR: No sequence loaded");
      } else {
        Serial2.print("SEQ_EXEC_START:count=");
        Serial2.print(sh.getNumCommands());
        const char *lastCmd = sh.getLastCommand();
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
    // Bang-bang commands
    else if (idChar == 'B')
      handleB(rxPacket);
    else if (idChar == 'V')
      handleV(rxPacket);
    else if (idChar == 'M')
      handleM(rxPacket);
    else if (idChar == 'b')
      handleLowerB(rxPacket);
    else if (idChar == 'v')
      handleLowerV(rxPacket);
    else if (idChar == 'x')
      handleLowerX(rxPacket);

    size_t n = strnlen(rxPacket, 256);
    memset(rxPacket, 0, n);
  }

  sh.update();

  // ── Bang-bang step ───────────────────────────────────────────────────
  bbLox.update(gArmed);
  bbFuel.update(gArmed);

  // ========== DAQ ==========
  sScanner.update();
  fScanner.update();

  float sData[NUM_DC_CHANNELS] = {0},
        lctcData[NUM_LC_CHANNELS + NUM_TC_CHANNELS];
  sScanner.getSOutput(sData);
  fScanner.getLCTCOutput(lctcData);

  char sPacket[512], lctcPacket[512], ptPacket[512];
  th.toCSVRow(sData, S_IDENTIFIER, NUM_DC_CHANNELS, sPacket, 512, 5);
  th.toCSVRow(lctcData, LCTC_IDENTIFIER, NUM_TC_CHANNELS + NUM_LC_CHANNELS,
              lctcPacket, 512, 5);
  th.toCSVRow(v2PtData, PT_IDENTIFIER, NUM_PT_CHANNELS, ptPacket, 512, 5);

  static uint32_t lastHexDiagMs = 0;
  const uint32_t now = millis();
  if (now - lastHexDiagMs >= 1000UL) {
    printPacketHexBrief("LCTC", lctcPacket, sizeof(lctcPacket));
    printPacketHexBrief("S", sPacket, sizeof(sPacket));
    printPacketHexBrief("PT", ptPacket, sizeof(ptPacket));
    lastHexDiagMs = now;
  }

  Serial2.print(lctcPacket);
  Serial2.print(sPacket);
  Serial2.print(ptPacket);

  printBbHeartbeat(bbLox);
  printBbHeartbeat(bbFuel);
}
