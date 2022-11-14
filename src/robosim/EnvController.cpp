#include "EnvController.h"
#include "ArenaModel.h"
#include "ArenaModelView.h"
#include "RobotMonitor.h"
#include "SimulatedRobot.h"
#include <memory>
#include <thread>
#include <type_traits>

using robosim::robotmonitor::RobotMonitor;
using simulatedrobot::SimulatedRobot;

namespace robosim::envcontroller
{

MonitorVec robots;

namespace
{

template <typename... Args> void init(int robotSpeed, Args... args)
{
    arenamodel::makeModel(args...);

    for (const auto &monitor : robots)
    {
        monitor->setRobot(robotSpeed);
    }

    arenamodel::toString();
}

} // namespace

void EnvController(const char *confFileName, int robotSpeed)
{
    init(robotSpeed, confFileName);
}

void EnvController(int rows, int cols, int robotSpeed)
{
    init(robotSpeed, rows, cols);
}

void startSimulation()
{
    arenamodelview::initModelView();

    std::vector<std::shared_ptr<SimulatedRobot>> simulatedRobots;
    std::vector<std::thread> threads;

    for (auto monitor : robots)
    {
        threads.push_back(std::thread(&RobotMonitor::run, monitor, &arenamodelview::running));
        threads.push_back(std::thread(&SimulatedRobot::run, monitor->getRobot()));

        simulatedRobots.push_back(monitor->getRobot());
    }

    arenamodelview::mainLoop(simulatedRobots);

    for (size_t i = 0; i < threads.size(); i++)
    {
        threads[i].join();
    }
}

float getCellWidth()
{
    return arenamodel::cellWidth;
}

float getCellRadius()
{
    return arenamodel::cellWidth / 2;
};

bool isRunning()
{
    return arenamodelview::running;
}

void stop()
{
    arenamodelview::running = false;
}

} // namespace robosim::envcontroller
