#pragma once
#include <Wire.h>

// Driver header file for the MCP9802A0 temperature sensor
class MCP9802A0 {

    public:

    void initialize(void);
    float getTemp(void);
    uint16_t temperature;

};