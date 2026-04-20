#pragma once
#include <Arduino.h>
#include "drivers/MCP3561.hpp"
#include "hardware-configs/BoardConfig.hpp"
#include "hardware-configs/pins.hpp"

class Scanner {

    protected:

    enum State : uint8_t {
        IDLE,
        WAIT_MUX,
        WAIT_CONV
    };

    

    public:
    
    virtual ~Scanner() = default;
    virtual void update() = 0;
    virtual void setup() = 0;

};