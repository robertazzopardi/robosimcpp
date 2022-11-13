#pragma once

#include "SimulatedRobot.h"
#include <stddef.h>
#include <vector>

namespace arenamodelview
{

extern bool running;

void initModelView();

void mainLoop(const std::vector<simulatedrobot::SimulatedRobot> &);

} // namespace arenamodelview
