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

template <typename... Args> static inline void init(const std::vector<RobotPtr> &robots, int robotSpeed, Args... args)
{
    arenamodel::makeModel(args...);

    for (const auto &monitor : robots)
    {
        monitor->setRobot(robotSpeed);
    }

    arenamodel::toString();
}

EnvController::EnvController(const char *confFileName, int robotSpeed) : running(true)
{
    init(robots, robotSpeed, confFileName);
}

EnvController::EnvController(int rows, int cols, int robotSpeed) : running(true)
{
    init(robots, robotSpeed, rows, cols);
}

void EnvController::startSimulation()
{
    arenamodelview::initModelView();

    std::vector<std::shared_ptr<SimulatedRobot>> simulatedRobots;
    std::vector<std::thread> threads;

    for (const auto &monitor : robots)
    {
        threads.push_back(std::thread(&RobotMonitor::run, monitor, &running));
        threads.push_back(std::thread(&SimulatedRobot::run, monitor->getRobot(), &running));

        simulatedRobots.push_back(monitor->getRobot());
    }

    arenamodelview::mainLoop(simulatedRobots, &running);

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
