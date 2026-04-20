#pragma once

#include "dc-controllers/SequenceHandler.hpp"
#include "board-functions/ArmingController.hpp"
#include "hardware-configs/BoardConfig.hpp"
#include "hardware-configs/pins.hpp"

class CommandRouter {

    private: 

    SequenceHandler sh;
    ArmingController ac;

    char* cmd;

    public:

    void setup();
    void update();
    void setCmd(char *cmd_);


};