#pragma once
#include <Arduino.h>

/**
 * Header file containing teensy 4.1 pin assignments
 */

struct ADCPins {
    uint8_t irq;
    uint8_t cs;
    uint8_t mosi;
    uint8_t miso;
    uint8_t sck;
 };

static constexpr uint8_t UART1Pins[2] = {8, 7};
static constexpr uint8_t UART2Pins[2] = {20, 21};

// Arming pins
static constexpr uint8_t PIN_ARM = 32;
static constexpr uint8_t PIN_DISARM = 33;

static constexpr ADCPins sADCPins = {9, 10, 11, 12, 13};
static constexpr ADCPins ptADCPins = {2, 0, 26, 1, 27};

static constexpr uint8_t ptMux[4] = {31, 30, 29, 28}; // Mux pins for PTs
static constexpr uint8_t lctcMux[4] = {3, 4, 6, 5}; // Mux pins for both LCs and TCs
static constexpr uint8_t sMux[4] = {18, 19, 23, 22}; // Solenoid current mux pins

// DC Channel pins. In order of channels 1 to 12
static constexpr uint8_t PINS_DC_CHANNELS[12] = {34, 35, 36, 37, 38, 39, 40, 41, 17, 16, 15, 14};