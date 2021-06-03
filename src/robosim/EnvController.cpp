#include "EnvController.h"
#include "ArenaModel.h"
#include "ArenaModelView.h"
#include "RobotMonitor.h"
#include "SimulatedRobot.h"
#include <iostream>
#include <thread>
#include <type_traits>

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using robosim::EnvController;
using robosim::RobotMonitor;
using simulatedrobot::SimulatedRobot;

EnvController::EnvController(const char *confFileName, int cols, int rows, RobotMonitor *monitor) {
    model = std::make_shared<ArenaModel>(confFileName, cols, rows);

    myMonitor = monitor;
    myMonitor->setArenaModel(model);

    view = std::make_shared<ArenaModelView>(static_cast<ArenaModel *>(model.get()), static_cast<SimulatedRobot *>(myMonitor->getRobot()));

    std::cout << static_cast<ArenaModel *>(model.get())->toString() << std::endl;
}

EnvController::~EnvController() {
}

void EnvController::updateEnv() {
    auto running = true;

    std::thread monitorThread(&RobotMonitor::run, myMonitor, &running);
    std::thread robotThread(&SimulatedRobot::run, static_cast<SimulatedRobot *>(myMonitor->getRobot()), &running);

    std::static_pointer_cast<ArenaModelView>(view)->mainLoop();

    // Stop the thread first, then wait for it to quit
    running = false;
    robotThread.join();
    monitorThread.join();
}
