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
#include "RobotMonitor.h"
#include "SimulatedRobot.h"
#include <memory>
#include <thread>
#include <type_traits>
#include <vector>

// using robosim::envcontroller::EnvController;
using robosim::robotmonitor::RobotMonitor;
using simulatedrobot::SimulatedRobot;

namespace robosim::envcontroller {

MonitorVec robots;

namespace {

// Store as void smart pointer type, purely to hide the declarations
std::shared_ptr<void> view;

template <typename... Args> void init(int robotSpeed, Args... args) {
    arenamodel::makeModel(args...);

    for (const auto &monitor : robots) {
        monitor->setRobot(robotSpeed);
    }

    arenamodel::toString();
}

} // namespace

void EnvController(const char *confFileName, int robotSpeed) {
    init(robotSpeed, confFileName);
}

void EnvController(int rows, int cols, int robotSpeed) {
    init(robotSpeed, rows, cols);
}

void startSimulation() {
    arenamodelview::initModelView();

    std::vector<SimulatedRobot *> sims;

    for (auto monitor : robots) {
        std::thread(&RobotMonitor::run, monitor, &arenamodelview::running)
            .detach();
        std::thread(&SimulatedRobot::run,
                    static_cast<SimulatedRobot *>(monitor->getRobot()))
            .detach();

        sims.push_back(static_cast<SimulatedRobot *>(monitor->getRobot()));
    }

    arenamodelview::mainLoop(sims.data(), sims.size());
}

float getCellWidth() { return arenamodel::cellWidth; }

float getCellRadius() { return arenamodel::cellWidth / 2; };

bool &isRunning() { return arenamodelview::running; }

void updateRunning() { arenamodelview::running = false; }

} // namespace robosim::envcontroller
