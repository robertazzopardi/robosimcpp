#pragma once

#include "Colour.h"
#include "RobotMonitor.h"
#include <memory>
#include <stddef.h>
#include <vector>

namespace robosim::envcontroller
{

class EnvController
{
  private:
    std::vector<std::shared_ptr<robotmonitor::RobotMonitor>> robots;
    bool running;

  public:
    /**
     * Initialise the Environment Controller from a specified config file
     *
     * @param robots vector of robots
     * @param configFileName file path of the environment configuration file
     *
     */
    EnvController(const char *);

    /**
     * Create an Environment with given cell width and height, with a border of
     * obstacles
     *
     * @param robots vector of robots
     * @param rows number of rows
     * @param cols number of columns
     *
     */
    // void EnvController(const MonitorVec &, int, int, int);
    EnvController(int, int);

    /**
     * Creates array of robots of type Robot monitor, with specified colour
     *
     * Usage makeRobots<RobotMonitor Type>(1, colour::RED);
     *
     */
    template <typename RobotType = robotmonitor::RobotMonitor>
    void makeRobots(size_t count, uint32_t speed, const colour::Colour &colour)
    {
        for (size_t i = 0; i < count; i++)
        {
            std::shared_ptr<robotmonitor::RobotMonitor> robotMonitor =
                std::make_shared<RobotType>(false, colour, &running);
            robotMonitor->setRobot(speed);
            robots.push_back(robotMonitor);
        }
    }

    /**
     * Begin the simulation
     */
    void run();

    /**
     * Returns the diameter of a grid cell
     */
    float getCellWidth() const;

    /**
     * Returns the radius of a grid cell
     */
    float getCellRadius() const;

    /**
     * Returns whether the simulation is running
     */
    bool isRunning() const;

    /**
     * Ends the Simulation at anytime
     */
    void stop();
};

} // namespace robosim::envcontroller
