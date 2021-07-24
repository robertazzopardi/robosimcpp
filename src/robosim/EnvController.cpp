/**
 * @file EnvController.cpp
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief
 * @version 0.1
 * @date 2021-07-07
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "EnvController.h"
#include "ArenaModel.h"
#include "ArenaModelView.h"
#include "Casting.h"
#include "RobotMonitor.h"
#include "SimulatedRobot.h"
#include <memory>
#include <thread>
#include <type_traits>
#include <vector>

// using robosim::envcontroller::EnvController;
using robosim::robotmonitor::RobotMonitor;
using simulatedrobot::SimulatedRobot;

constexpr auto SimRobot = typecasting::cast<SimulatedRobot *>;

namespace robosim::envcontroller {

namespace {

robotmonitor::MonitorVec myMonitors;

// Store as void smart pointer type, purely to hide the declarations
std::shared_ptr<void> view;

template <typename... Args>
void init(const robotmonitor::MonitorVec &monitors, int robotSpeed,
          Args... args) {
    arenamodel::makeModel(args...);

    myMonitors = monitors;

    for (const auto &monitor : myMonitors) {
        monitor->setRobot(robotSpeed);
    }

    arenamodel::toString();
}

} // namespace

// template <typename... Args>
// void robosim::EnvController(const MonitorVec &monitors, int robotSpeed,
//                             Args... args) {
//     // init(monitors, robotSpeed, args...);
//     arenamodel::makeModel(args...);

//     myMonitors = monitors;

//     for (const auto &monitor : myMonitors) {
//         monitor->setRobot(robotSpeed);
//     }

//     arenamodel::toString();
// }

void EnvController(const robotmonitor::MonitorVec &monitors,
                   const char *confFileName, int robotSpeed) {
    init(monitors, robotSpeed, confFileName);
}

void EnvController(const robotmonitor::MonitorVec &monitors, int rows, int cols,
                   int robotSpeed) {
    init(monitors, robotSpeed, rows, cols);
}

void startSimulation() {
    arenamodelview::initModelView();

    std::vector<SimulatedRobot *> sims;

    for (auto monitor : myMonitors) {
        std::thread(&RobotMonitor::run, monitor, &arenamodelview::running)
            .detach();
        std::thread(&SimulatedRobot::run, SimRobot(monitor->getRobot()))
            .detach();

        sims.push_back(SimRobot(monitor->getRobot()));
    }

    arenamodelview::mainLoop(sims);
}

float getCellWidth() { return arenamodel::cellWidth; }

float getCellRadius() { return arenamodel::cellWidth / 2; };

bool &isRunning() { return arenamodelview::running; }

} // namespace robosim::envcontroller
