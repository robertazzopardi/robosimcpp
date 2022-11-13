#pragma once

#include <stddef.h>
#include <vector>
#include "SimulatedRobot.h"

namespace arenamodelview
{

    extern bool running;

    void initModelView();

    void mainLoop(const std::vector<simulatedrobot::SimulatedRobot> &);

}
