#pragma once

#include "SimulatedRobot.h"
#include <memory>
#include <stddef.h>
#include <vector>

namespace arenamodelview
{

extern bool running;

void initModelView();

void mainLoop(const std::vector<std::shared_ptr<simulatedrobot::SimulatedRobot>> &);

} // namespace arenamodelview