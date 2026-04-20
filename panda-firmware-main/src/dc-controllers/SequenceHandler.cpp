#include "dc-controllers/SequenceHandler.hpp"
#include <cstring>

void SequenceHandler::setup() {
    for (int i = 0; i < NUM_DC_CHANNELS; i++) {
        uint8_t pin = PINS_DC_CHANNELS[i];
        channelArr[i] = DCChannel(pin);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    
    }
}

void SequenceHandler::resetCommand() {
    memset(sequenceArr, 0, sizeof(sequenceArr));
    numCommands = 0;
    currentIndex = 0;
    inDelay = false;
    isActive = false;
}

void SequenceHandler::printCurrentCommand(void) {

    for (int i = 0; i < numCommands; i++) {
        Serial.print("  ["); Serial.print(i); Serial.print("] ");
        Serial.print("chan=");  Serial.print(sequenceArr[i].channel);
        Serial.print(", state=");  Serial.print(sequenceArr[i].state);
        Serial.print(", delay=");  Serial.println(sequenceArr[i].delay);
    } 
    
}

bool SequenceHandler::pollCommand(void) {
    if (numCommands < 0) {
        return true;
    }
    else {
        return false;
    }

}

void SequenceHandler::setCommand(char* command) {

    /*
    Command example: s11.01000,s10.00000,sA1.02000,sA0.00000
    Function: Channel 1 is on for one second, off's, then Channel 10 is on for 2 seconds, then off's

    **** Using milliseconds here. microseconds are cringe

    */

    numCommands    = 0;
    currentIndex   = 0;
    isActive       = false;
    sTimer         = 0;

    resetCommand();
    lastCommandValid = false;
    lastCommand[0] = '\0';

    char original[256];
    strncpy(original, command, sizeof(original) - 1);
    original[sizeof(original) - 1] = '\0';
    size_t origLen = strlen(original);
    while (origLen > 0 && (original[origLen - 1] == '\n' || original[origLen - 1] == '\r')) {
        original[--origLen] = '\0';
    }

    char buf[256];
    strncpy(buf, original, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    size_t lenCommand = strlen(command);

    for (int i = 0; i < lenCommand; i++) {
        if (buf[i] == 's') {
            numCommands++;
        }
    }

    unsigned channel, state, delay;
    char* token = strtok(buf, ",");

    if (token && sscanf(token + 1, "%1x%1u.%5u", &channel, &state, &delay) == 3) {
        sequenceArr[0].channel = channel;
        sequenceArr[0].state = state;
        sequenceArr[0].delay = delay;

        Serial2.print("SEQ_TOKEN idx=0");
        Serial2.print(" chan=");
        Serial2.print(sequenceArr[0].channel);
        Serial2.print(" state=");
        Serial2.print(sequenceArr[0].state ? "ON" : "OFF");
        Serial2.print(" duration=");
        Serial2.println(sequenceArr[0].delay);
    }

    for (int i = 1; i < numCommands; i++) {
        token = strtok(nullptr, ",");
        if (token && sscanf(token + 1, "%1x%1u.%5u", &channel, &state, &delay) == 3) {
            sequenceArr[i].channel = channel;
            sequenceArr[i].state = state;
            sequenceArr[i].delay = delay;

            Serial2.print("SEQ_TOKEN idx=");
            Serial2.print(i);
            Serial2.print(" chan=");
            Serial2.print(sequenceArr[i].channel);
            Serial2.print(" state=");
            Serial2.print(sequenceArr[i].state ? "ON" : "OFF");
            Serial2.print(" duration=");
            Serial2.println(sequenceArr[i].delay);
        }
    }

    if (numCommands > 0) {
        strncpy(lastCommand, original, sizeof(lastCommand) - 1);
        lastCommand[sizeof(lastCommand) - 1] = '\0';
        lastCommandValid = true;
        Serial2.print("SEQ_ACK:count=");
        Serial2.print(numCommands);
        Serial2.print(",raw=");
        Serial2.println(lastCommand);
    } else {
        Serial2.println("SEQ_ERROR: Failed to parse sequence");
    }


    // int commandLength = strlen(command);
    // int commandArrayIndex = 0;
    // for (int i = 0; i < commandLength; i++) {
    // if (command[i] == 's') {
    //     if (command[i+1] >= '0' && command[i+1] <= '9') {
    //     solenoidChannels[commandArrayIndex] = command[i + 1] - '0';
    //     }
    //     if (command[i+1] >= 'A' && command[i+1] <= 'F') {
    //     solenoidChannels[commandArrayIndex] = command[i + 1] - 'A' + 10;
    //     }

    // numCommands++;

    //     solenoidStates[commandArrayIndex] = command[i + 2] - '0';
    //     //Serial.println(command[i + 2]);
    // }
    // if (command[i] == '.') {
    //     solenoidDelays[commandArrayIndex] = 10000 * (command[i+1] - '0') + 1000 * (command[i+2] - '0') + 100 * (command[i+3] - '0') + 10 * (command[i+4] - '0') + 1 * (command[i+5] - '0');
    //     //Serial.println(solenoidDelays[commandArrayIndex]);
    // }
    // if (command[i] == ',') {
    //     commandArrayIndex++;
    // }
    // }


}

void SequenceHandler::execute(bool sequenceState) {
    if (sequenceState == true) {
        if (numCommands == 0) {
            Serial2.println("SEQ_ERROR: Execute requested with no commands");
            return;
        }
        isActive = true;
        // sTimer = solenoidDelays[0] + 1;
        sTimer = 0;
        currentIndex = 0;
    }
    else {
        isActive = false;
    }
}

void SequenceHandler::cancelExecution(void) {
    isActive = false;
    inDelay = false;
    currentIndex = 0;
}

void SequenceHandler::setAllChannelsOff(void) {
    for (int i = 0; i < NUM_DC_CHANNELS; i++) {
        channelArr[i].setState(false);
    }
}

// TODO: Solenoid delays is shifted right 1
void SequenceHandler::update() {

    // uint8_t dcChannels[12] = {34, 35, 36, 37, 38, 39, 40, 41, 17, 16, 15, 14};

    // No commands and not firing
    if (numCommands == 0 || isActive == false) {
        sTimer = 0;
        return;
    }

    if (inDelay == false) {
        uint8_t chan = sequenceArr[currentIndex].channel;
        if (chan >= 1 && chan <= NUM_DC_CHANNELS) {
            channelArr[chan - 1].setState(sequenceArr[currentIndex].state);
        }

        Serial2.print("SEQ_STEP:index=");
        Serial2.print(currentIndex);
        Serial2.print(",chan=");
        Serial2.print(chan);
        Serial2.print(",state=");
        Serial2.print(sequenceArr[currentIndex].state ? "ON" : "OFF");
        Serial2.print(",duration=");
        Serial2.println(sequenceArr[currentIndex].delay);

        sTimer = 0;
        inDelay = true;
    }

    else if (sTimer >= sequenceArr[currentIndex].delay) {
        inDelay = false;
        currentIndex++;

        if (currentIndex >= numCommands) {
            currentIndex = 0;
            inDelay = false;
            isActive = false;
            Serial2.println("SEQ_EXEC_COMPLETE");
            if (lastCommandValid) {
                Serial2.print("SEQ_READY:");
                Serial2.println(lastCommand);
            }
        }
    }
}