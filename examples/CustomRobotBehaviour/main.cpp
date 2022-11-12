#include <robosim/robosim.h>
#include <iostream>

class Robot : public robosim::robotmonitor::RobotMonitor
{
public:
    // Inherit constructor (essentially super the RobotMonitor constructor)
    Robot(bool verbose, colour::Colour colour)
        : robosim::robotmonitor::RobotMonitor(verbose, colour) {}

    // Override run, to implement the robots behaviour
    void run(bool *running)
    {
        std::cout << "Starting Robot: " << serialNumber << std::endl;

        while (*running)
        {
            travel();

            rotate(90);
            // setDirection(-90);
            debug();
        }
    }
};

int main(int argc, char **argv)
{
    // obvious error handling here
    char *configFile = argv[1];

    robosim::envcontroller::makeRobots<Robot>(3, colour::OFF_BLACK);

    robosim::envcontroller::EnvController(configFile, 50);
    // robosim::envcontroller::EnvController(10, 10, 50);

    robosim::envcontroller::startSimulation();
}
