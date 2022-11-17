#include "EnvController.h"
#include "ArenaModel.h"
#include "ArenaModelView.h"
#include "RobotMonitor.h"
#include "SimulatedRobot.h"
#include <iostream>
#include <memory>
#include <thread>
#include <type_traits>

using robosim::robotmonitor::RobotMonitor;
using simulatedrobot::SimulatedRobot;

namespace robosim::envcontroller
{

EnvController::EnvController(const char *configFile) : running(true)
{
    arenamodel::makeModel(configFile);

    arenamodel::toString();
}

EnvController::EnvController(int rows, int cols) : running(true)
{
    arenamodel::makeModel(rows, cols);

    arenamodel::toString();
}

void EnvController::run()
{
    arenamodelview::initModelView();

    std::vector<std::shared_ptr<SimulatedRobot>> simulatedRobots;
    std::vector<std::thread> threads;

    for (const auto &monitor : robots)
    {
        threads.push_back(std::thread(&RobotMonitor::run, monitor));
        threads.push_back(std::thread(&SimulatedRobot::run, monitor->getRobot(), &running));

        simulatedRobots.push_back(monitor->getRobot());
    }

    arenamodelview::renderLoop(simulatedRobots, &running);

    arenamodelview::cleanUp();

    for (size_t i = 0; i < threads.size(); i++)
    {
        threads[i].join();
    }
}

float EnvController::getCellWidth() const
{
    return arenamodel::cellWidth;
}

float EnvController::getCellRadius() const
{
    return arenamodel::cellWidth / 2;
};

bool EnvController::isRunning() const
{
    return running;
}

void EnvController::stop()
{
    running = false;
}

} // namespace robosim::envcontroller
