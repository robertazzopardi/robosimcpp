#include <cstdlib>
#include <iostream>
#include <robosim/robosim.h>

class Robot : public robosim::robotmonitor::RobotMonitor
{
  public:
    // Inherit constructor (essentially super the RobotMonitor constructor)
    Robot(bool verbose, colour::Colour colour) : robosim::robotmonitor::RobotMonitor(verbose, colour)
    {
    }

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
    if (argc != 2)
    {
        return EXIT_FAILURE;
    }

    // obvious error handling here
    char *configFile = argv[1];

    robosim::envcontroller::EnvController env(configFile, 50);
    // robosim::envcontroller::EnvController env(10, 10, 50);

    env.makeRobots<Robot>(3, colour::OFF_BLACK);

    env.startSimulation();

    return EXIT_SUCCESS;
}
