#pragma once

#include <Arduino.h>
#include "elapsedMillis.h"
#include "hardware-configs/BoardConfig.hpp"

class ArmingController {
    
    private:

    uint8_t armPin, disarmPin;
    elapsedMillis pulseTimer = 0, waitTimer = 0;

    bool justChanged = false;

    int pulseTime = 1000; // ms
    int waitTime = 1000; // ms


    public:

    enum State {
        DISARM,
        ARM,
    };

    enum Phase { // State changes only happen in IDLE. WAITING ensures a minimum time between state changes
        IN_PULSE,
        IDLE,
        WAITING
    };

    enum Result { // IGNORED if a state change is requested while in waiting or in pulse, ACCEPTED if otherwise
        IGNORED,
        ACCEPTED
    };

    State state = DISARM;
    Phase phase = IDLE;

    ArmingController(uint8_t armPin_, uint8_t disarmPin_) : armPin(armPin_), disarmPin(disarmPin_) {}

    void setup();
    Result setState(State state_);
    // void arm();
    // void disarm();

    void update();

};