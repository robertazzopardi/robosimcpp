#pragma once

#include "SimulatedRobot.h"
#include <memory>
#include <stddef.h>
#include <vector>

namespace arenamodelview
{

void initModelView();

void renderLoop(const std::vector<std::shared_ptr<simulatedrobot::SimulatedRobot>> &, bool *);

void cleanUp();

} // namespace arenamodelview
