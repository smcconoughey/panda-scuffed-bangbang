#pragma once
#include "ArmingController.hpp"

void ArmingController::setup() {
    pinMode(armPin, OUTPUT);
    pinMode(disarmPin, OUTPUT);

    state = DISARM;

    // Optional pulse to disarm
    digitalWrite(disarmPin, HIGH);
    delay(500);
    digitalWrite(disarmPin, LOW);

    // digitalWrite(armPin, LOW);
    // digitalWrite(disarmPin, LOW);

    phase = IDLE;
}

ArmingController::Result ArmingController::setState(State state_) {
    if (phase != IDLE) {return Result::IGNORED;}

    digitalWrite(armPin, LOW);
    digitalWrite(disarmPin, LOW);

    justChanged = true;


    Serial.println("Changing State!");
    phase = IN_PULSE;
    state = state_;
    pulseTimer = 0;
    return Result::ACCEPTED;
}

void ArmingController::update() {

    switch (phase) {
        case IDLE:
        // Serial.println("Idling...");
        break;

        case IN_PULSE:
        // Serial.println("In pulse...");
        if (justChanged) {
              digitalWrite((state == ARM) ? armPin : disarmPin, HIGH);
              justChanged = false;
        }
        if (pulseTimer >= pulseTime) {
            Serial.println("Pulse done!");

            digitalWrite((state == ARM) ? armPin : disarmPin, LOW);
            waitTimer = 0;
            phase = WAITING;
        }
        break;

        case WAITING:
        // Serial.println("waiting...");
        digitalWrite(armPin, LOW);
        digitalWrite(disarmPin, LOW);
        if (waitTimer >= waitTime) {
            Serial.println("Wait done!");
            phase = IDLE;
        }
        break;

    }

}