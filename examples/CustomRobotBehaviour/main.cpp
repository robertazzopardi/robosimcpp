#include <cstdlib>
#include <iostream>
#include "../../include/robosim/RobotMonitor.h"
#include "../../include/robosim/EnvController.h"

using robosim::robotmonitor::RobotMonitor;
using robosim::envcontroller::EnvController;

const size_t DEG_90 = 90;
const size_t ROBOT_SPEED = 50;

class Robot : public RobotMonitor
{
  public:
    // Inherit constructor (essentially super the RobotMonitor constructor)
    Robot(bool verbose, Colour colour, bool *running)
        : RobotMonitor(verbose, colour, running)
    {
    }

    // Override run, to implement the robots behaviour
    void run() override
    {
        std::cout << "Starting Robot: " << serialNumber << '\n';

        while (*running)
        {
            travel();

            rotate(DEG_90);

            debug();
        }
    }
};

void run(RobotMonitor* robot) {
    std::cout << "Starting Robot: " << robot->serialNumber << '\n';
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Please provide 1 configuration file (.txt)" << '\n';
        return EXIT_FAILURE;
    }

    char *configFile = argv[1];

    EnvController env(configFile);
    // EnvController env(10, 10);

    env.makeRobots<Robot>(3, ROBOT_SPEED, OFF_BLACK);
    // env.makeRobotsWithFunc(3, ROBOT_SPEED, OFF_BLACK, run);

    env.run();

    return EXIT_SUCCESS;
}
