#include <cstdlib>
#include <iostream>
#include <robosim/robosim.h>

class Robot : public robosim::robotmonitor::RobotMonitor
{
  public:
    // Inherit constructor (essentially super the RobotMonitor constructor)
    Robot(bool verbose, colour::Colour colour, bool *running)
        : robosim::robotmonitor::RobotMonitor(verbose, colour, running)
    {
    }

    // Override run, to implement the robots behaviour
    void run()
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
        std::cout << "Please provide 1 configuration file (.txt)" << std::endl;
        return EXIT_FAILURE;
    }

    char *configFile = argv[1];

    robosim::envcontroller::EnvController env(configFile);
    // robosim::envcontroller::EnvController env(10, 10);

    env.makeRobots<Robot>(3, 50, colour::OFF_BLACK);

    env.run();

    return EXIT_SUCCESS;
}
