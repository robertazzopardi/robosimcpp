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

using robosim::EnvController;
using robosim::RobotMonitor;
using simulatedrobot::SimulatedRobot;

constexpr auto SimRobot = typecasting::cast<SimulatedRobot *>;

namespace robosim {

namespace {

MonitorVec myMonitors;

// Store as void smart pointer type, purely to hide the declarations
std::shared_ptr<void> view;

template <typename... Args>
void init(const MonitorVec &monitors, Args... args) {
    arenamodel::makeModel(args...);

    myMonitors = monitors;

    for (const auto &monitor : myMonitors) {
        monitor->setRobot();
    }

    arenamodel::toString();
}

} // namespace

void EnvController(const MonitorVec &monitors, const char *confFileName) {
    init(monitors, confFileName);
}

void EnvController(const MonitorVec &monitors, int rows, int cols) {
    init(monitors, rows, cols);
}

void startSimulation() {
    arenamodelview::initModelView();

    for (auto monitor : myMonitors) {
        std::thread(&RobotMonitor::run, monitor, &arenamodelview::running)
            .detach();
        std::thread(&SimulatedRobot::run, SimRobot(monitor->getRobot()))
            .detach();
    }

    arenamodelview::mainLoop(myMonitors);
}

} // namespace robosim
