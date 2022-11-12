#pragma once

#include <stddef.h>

namespace simulatedrobot
{
    class SimulatedRobot;
}

namespace arenamodelview
{

    extern bool running;

    void initModelView();

    void mainLoop(simulatedrobot::SimulatedRobot **, const size_t);

}
