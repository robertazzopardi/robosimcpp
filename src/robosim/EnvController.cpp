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
#include <thread>
#include <type_traits>
#include <vector>

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using robosim::EnvController;
using robosim::RobotMonitor;
using simulatedrobot::SimulatedRobot;

constexpr auto View = typecasting::make_ptr<ArenaModelView>;
constexpr auto ModelView = typecasting::cast_ptr<ArenaModelView>;
constexpr auto SimRobot = typecasting::cast<SimulatedRobot *>;

EnvController::EnvController(const char *confFileName,
                             const MonitorVec &monitors) {

    ArenaModel::makeModel(confFileName);

    init(monitors);
}

EnvController::EnvController(int rows, int cols, const MonitorVec &monitors) {

    ArenaModel::makeModel(rows, cols);

    init(monitors);
}

EnvController::~EnvController() {}

void EnvController::init(const MonitorVec &monitors) {
    myMonitors = monitors;

    for (const auto &monitor : myMonitors) {
        monitor->setRobot();
    }

    view = View();

    ArenaModel::toString();
}

void EnvController::startSimulation() {

    for (auto monitor : myMonitors) {
        std::thread(&RobotMonitor::run, monitor, &ArenaModelView::running)
            .detach();
        std::thread(&SimulatedRobot::run, SimRobot(monitor->getRobot()))
            .detach();
    }

    ModelView(view)->mainLoop(myMonitors);
}
