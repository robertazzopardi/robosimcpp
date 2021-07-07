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
#include <future>
#include <iostream>
#include <type_traits>

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using robosim::EnvController;
using robosim::RobotMonitor;
using simulatedrobot::SimulatedRobot;

EnvController::EnvController(const char *confFileName, int cols, int rows,
                             RobotMonitor *monitor) {
    model = std::make_shared<ArenaModel>(confFileName, cols, rows);

    myMonitor = monitor;
    myMonitor->setArenaModel(model);

    view = std::make_shared<ArenaModelView>(
        static_cast<ArenaModel *>(model.get()),
        static_cast<SimulatedRobot *>(myMonitor->getRobot()));

    std::cout << static_cast<ArenaModel *>(model.get())->toString()
              << std::endl;
}

EnvController::~EnvController() {}

void EnvController::updateEnv() {
    auto resRobotMonitor =
        std::async(&RobotMonitor::run, myMonitor, &ArenaModelView::running);
    auto resSimulatedRobot =
        std::async(&SimulatedRobot::run,
                   static_cast<SimulatedRobot *>(myMonitor->getRobot()),
                   std::static_pointer_cast<ArenaModelView>(view).get());

    std::static_pointer_cast<ArenaModelView>(view)->mainLoop();
}
