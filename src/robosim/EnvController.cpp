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
#include "Common.h"
#include "RobotMonitor.h"
#include "SimulatedRobot.h"
#include <future>
#include <iostream>
#include <type_traits>

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using robosim::EnvController;
using robosim::RobotMonitor;
using simulatedrobot::SimulatedRobot;

constexpr auto Arena =
    typecasting::make_ptr<ArenaModel, const char *, int, int>;
constexpr auto View =
    typecasting::make_ptr<ArenaModelView, ArenaModel *, SimulatedRobot *>;
constexpr auto ModelView = typecasting::cast_ptr<ArenaModelView>;
constexpr auto SimRobot = typecasting::cast<SimulatedRobot *>;
constexpr auto AModel = typecasting::cast<ArenaModel *>;

EnvController::EnvController(const char *confFileName, int cols, int rows,
                             RobotMonitor *monitor) {
    model = Arena(confFileName, cols, rows);

    myMonitor = monitor;
    myMonitor->setArenaModel(model);

    view = View(AModel(model.get()), SimRobot(myMonitor->getRobot()));

    std::cout << AModel(model.get())->toString() << std::endl;
}

EnvController::~EnvController() {}

void EnvController::updateEnv() {
    auto resRobotMonitor =
        std::async(&RobotMonitor::run, myMonitor, &ArenaModelView::running);
    auto resSimulatedRobot =
        std::async(&SimulatedRobot::run, SimRobot(myMonitor->getRobot()),
                   ModelView(view).get());

    ModelView(view)->mainLoop();
}
