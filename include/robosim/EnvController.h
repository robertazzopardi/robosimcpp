#pragma once

#include "Colour.h"
#include "RobotMonitor.h"
#include <memory>
#include <stddef.h>
#include <vector>

namespace robosim::envcontroller
{

using RobotPtr = std::shared_ptr<robotmonitor::RobotMonitor>;

class EnvController
{
  private:
    std::vector<RobotPtr> robots;
    bool running;

  public:
    /**
     * Initialise the Environment Controller from a specified config file
     *
     * @param robots vector of robots
     * @param configFileName file path of the environment configuration file
     *
     */
    EnvController(const char *, int);

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
    EnvController(int, int, int);

    /**
     * Creates array of robots of type Robot monitor, with specified colour
     *
     * Usage makeRobots<RobotMonitor Type>(1, colour::RED);
     *
     */
    template <typename RobotType = robotmonitor::RobotMonitor>
    void makeRobots(const size_t count, const colour::Colour colour)
    {
        for (size_t i = 0; i < count; i++)
        {
            robots.push_back(std::make_shared<RobotType>(false, colour));
        }
    }

    /**
     * Begin the simulation
     */
    void startSimulation();

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
