#include "EnvController.h"
#include "ArenaModel.h"
#include "ArenaModelView.h"
#include "RobotMonitor.h"
#include "SimulatedRobot.h"
#include <memory>
#include <thread>

using robosim::robotmonitor::RobotMonitor;
using simulatedrobot::SimulatedRobot;

namespace robosim::envcontroller
{

EnvController::EnvController(const char *configFile) : running(true)
{
    arenamodel::makeModel(configFile);

    arenamodel::toString();
}

EnvController::EnvController(int rows, int cols) : running(true)
{
    arenamodel::makeModel(rows, cols);

    arenamodel::toString();
}

void EnvController::run()
{
    arenamodelview::initModelView();

    std::vector<std::shared_ptr<SimulatedRobot>> simulatedRobots;
    std::vector<std::thread> threads;

    for (const auto &monitor : robots)
    {
        if (*monitor->run_func != nullptr) {
            threads.emplace_back(std::thread(&RobotMonitor::callRunFunc, monitor));
        } else {
            threads.emplace_back(std::thread(&RobotMonitor::run, monitor));
        }
        
        threads.emplace_back(std::thread(&SimulatedRobot::run, monitor->getRobot(), &running));

        simulatedRobots.push_back(monitor->getRobot());
    }

    arenamodelview::renderLoop(simulatedRobots, &running);

    arenamodelview::cleanUp();

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

std::vector<std::shared_ptr<robosim::robotmonitor::RobotMonitor>> EnvController::getRobots() const
{
    return robots;
}

void EnvController::makeRobotsWithFunc(size_t count, size_t speed, const Colour &colour, void (*run)(RobotMonitor *))
{
    for (size_t i = 0; i < count; i++)
    {
        std::shared_ptr<RobotMonitor> robotMonitor = //
            std::make_shared<RobotMonitor>(false, colour, &running);
        robotMonitor->setRobot(speed);
        robotMonitor->run_func = run;
        robots.emplace_back(robotMonitor);
    }
}

} // namespace robosim::envcontroller

extern "C" {
    using robosim::envcontroller::EnvController;

    // Create an instance of MyClass
    EnvController* EnvController_new(const char *config) {
        return new EnvController(config);
    }

    // Call the add method
    // int MyClass_add(MyClass* instance, int y) {
    //     return instance->add(y);
    // }
    
    void EnvController_run(EnvController *env) {
        env->run();
    }

    void EnvController_makeRobotsWithFunc(EnvController *env, size_t count, size_t speed, const Colour &colour, void (*run)(RobotMonitor *)) {
        env->makeRobotsWithFunc(count, speed, colour, run);
    }

    // Destroy the instance
    void EnvController_delete(EnvController* env) {
        delete env;
    }
}
