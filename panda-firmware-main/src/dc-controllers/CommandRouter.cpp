#include "dc-controllers/CommandRouter.hpp"

void CommandRouter::setup() {
    ac = ArmingController(PIN_ARM, PIN_DISARM);
    sh.setup();
    ac.setup();
}

void CommandRouter::setCmd(char* cmd_) {cmd = cmd_;}

void CommandRouter::update() {
    char idChar = cmd[0];
    // Feed cmd into command handler
    if (idChar == 's') {
      Serial.print("Sequence packet: ");
      Serial.println(cmd);
      sh.setCommand(cmd);
    }

    else if (idChar == 'S') {

      char channelChar = cmd[1];
      char stateChar = cmd[2];

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
        
        Serial.print("Solenoid Command: ");
        Serial.print(channel);
        Serial.print(" | ");
        Serial.println(state);
      }

    }

    else if (idChar == 'a') {
      // Arm
      ac.setState(ArmingController::ARM);
      Serial.println("Arming!");

    }

    else if (idChar == 'r') {
      // Disarm
      ac.setState(ArmingController::DISARM);
      Serial.println("Disarming!");
    }

    else if (idChar == 'f') {
      Serial.println(cmd);
      sh.execute(true);
    }
}