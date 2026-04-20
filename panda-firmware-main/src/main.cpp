#include <FlexSerial.h>
#include <SPI.h>
#include <Wire.h>

#include "hardware-configs/pins.hpp"

// #include "scanners/Scanner.hpp"
#include "scanners/FScanner.hpp" // Fluids DAQ
#include "scanners/SScanner.hpp" // Solenoid current DAQ

#include "telemetry/TelemetryHandler.hpp"
#include "dc-controllers/SequenceHandler.hpp"
#include "board-functions/ArmingController.hpp"

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
 * SERIAL PROTOCOL (Baud rate: 115200)
 * 
 * Command Format:
 * 'a' - Arm
 * 'd' - Disarm
 * 's<Channel Identifier in Hex><State>.<Five digit delay in milliseconds>' - Solenoid Sequence
 * 'S<Channel Identifier in Hex><State>' - Solenoid command
 * 
 * 'B'<Channel Identifier in Hex><State> - Bang-Bang channel activation/deactivation
 * 'b'<Channel Identifier in Hex>.<Lower Deadband in raw voltage>.<Upper Deadband in raw voltage>.<Min off time in milliseconds>.<Min on time in milliseconds>
 * 
 * Telemetry Format:
 * 'p<Voltage across shunt resistor>' x16 - Pressure Transducer Packet
 * 's<Voltage across shunt resistor>' x12 - Solenoid Current Packet
 * 't<Conditioned thermocouple voltage>' x6 - Thermocouple Voltage Packet
 * 'l<Conditioned load cell voltage>' x6 - Load Cell Voltage Packet
 * 
 */

TelemetryHandler th(Serial2, 1024);
SequenceHandler sh;
ArmingController ac(PIN_ARM, PIN_DISARM);

SScanner sScanner(sADCPins.cs, sADCPins.irq, SPI, SPISettingsDefault);
FScanner fScanner(ptADCPins.cs, ptADCPins.irq, SPI1, SPISettingsDefault);

void setup() {
  // put your setup code here, to run once:
  Serial2.begin(SERIAL_BAUD_RATE); //RS-485 bus 1
  Serial2.setTimeout(100);
  
  // Teensy has a preset RX buffer limit of 64 bytes, this was causing our autosequencer issues
  static uint8_t rxBuf[RX_BUF_SIZE];
  Serial2.addMemoryForRead(rxBuf, RX_BUF_SIZE);

  static uint8_t txBuf[TX_BUF_SIZE]; // Literally downloading more memory
  Serial2.addMemoryForWrite(txBuf, TX_BUF_SIZE);

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

  Serial2.println("Panda Initialized!");

}

void loop() {
  // // put your main code here, to run repeatedly:

  char idChar;

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
        sh.channelArr[channel - 1].setState(state);
        Serial2.print("Solenoid Command: ");
        Serial2.print(channel);
        Serial2.print(" | ");
        Serial2.println(state);
      }

    }

    else if (idChar == 'a') {
      // Arm
      // ac.setState(ArmingController::ARM);

      digitalWrite(PIN_DISARM, LOW);
      digitalWrite(PIN_ARM, HIGH);
      Serial2.println("Arming!");

    }

    else if (idChar == 'r') {
      // Disarm
      // ac.setState(ArmingController::DISARM);
      digitalWrite(PIN_ARM, LOW);
      digitalWrite(PIN_DISARM, HIGH);
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

    // Resetting all incoming buffers
    size_t n = strnlen(rxPacket, 256);   // your rxBuf capacity
    memset(rxPacket, 0, n);
    // memset(rxPacket, 0, sizeof(rxPacket));
    // packetReady = false;
  }

  // ac.update();
  sh.update();


  // ========== DAQ ==========
  sScanner.update();
  fScanner.update();

  float sData[NUM_DC_CHANNELS] = {0}, lctcData[NUM_LC_CHANNELS + NUM_TC_CHANNELS];
  sScanner.getSOutput(sData);
  fScanner.getLCTCOutput(lctcData);

  char sPacket[512], lctcPacket[512];

  th.toCSVRow(sData, S_IDENTIFIER, NUM_DC_CHANNELS, sPacket, 512, 5);
  th.toCSVRow(lctcData, LCTC_IDENTIFIER, NUM_TC_CHANNELS + NUM_LC_CHANNELS, lctcPacket, 512, 5);

  // uint32_t current = millis();
  Serial2.print(lctcPacket);
  // Serial.print(lctcPacket);
  Serial2.print(sPacket);

  // uint32_t tTime = millis() - current;
  // Serial.print("Transmission time: ");
  // Serial.println(tTime);
}


