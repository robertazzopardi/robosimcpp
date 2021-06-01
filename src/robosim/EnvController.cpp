#include "EnvController.h"
#include "ArenaModel.h"
#include "ArenaModelView.h"
#include "RobotMonitor.h"
#include "SimulatedRobot.h"
#include <iostream>
#include <stddef.h>
#include <thread>

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using robosim::EnvController;
using robosim::RobotMonitor;
using simulatedrobot::SimulatedRobot;

EnvController::EnvController(const char *confFileName, int cols, int rows, RobotMonitor *monitor) {
    model = new ArenaModel(confFileName, cols, rows); // ArenaModel
    myMonitor = monitor;
    myMonitor->setArenaModel(static_cast<ArenaModel *>(model));
    view = new ArenaModelView(static_cast<ArenaModel *>(model), static_cast<SimulatedRobot *>(myMonitor->getRobot())); // ArenaModelView

    std::cout << static_cast<ArenaModel *>(model)->toString() << std::endl;
}

EnvController::~EnvController() {
    delete static_cast<ArenaModelView *>(view);
    view = NULL;

    delete static_cast<ArenaModel *>(model);
    model = NULL;
}

void EnvController::updateEnv() {
    bool running = true;

    std::thread monitorThread(&RobotMonitor::run, myMonitor, &running);
    std::thread robotThread(&SimulatedRobot::run, static_cast<SimulatedRobot *>(myMonitor->getRobot()), &running);

    static_cast<ArenaModelView *>(view)->mainLoop();

    // Stop the thread first, then wait for it to quit
    running = false;
    robotThread.join();
    monitorThread.join();
}

// void somefunction(void (*fptr)(void *, bool *), void *context, bool *running) {
//     fptr(context, running);
// }

// void forwarder(void *context, bool *running) {
//     static_cast<RobotMonitor *>(context)->run(running);
// }

// somefunction(&forwarder, robotMonitor, &running);
