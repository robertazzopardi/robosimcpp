#pragma once

#include "Colour.h"
#include "RobotMonitor.h"
#include <cstddef>
#include <memory>
#include <vector>

using robosim::robotmonitor::RobotMonitor;

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
     * Usage makeRobots<RobotMonitor Type>(1, RED);
     *
     */
    template <typename RobotType = RobotMonitor> void makeRobots(size_t count, size_t speed, const Colour &colour)
    {
        for (size_t i = 0; i < count; i++)
        {
            std::shared_ptr<RobotMonitor> robotMonitor = std::make_shared<RobotType>(false, colour, &running);
            robotMonitor->setRobot(speed);
            robots.emplace_back(robotMonitor);
        }
    }

    void makeRobotsWithFunc(size_t, size_t, const Colour &, void (*run)(RobotMonitor *));

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

    std::vector<std::shared_ptr<robosim::robotmonitor::RobotMonitor>> getRobots() const;
};

} // namespace robosim::envcontroller
