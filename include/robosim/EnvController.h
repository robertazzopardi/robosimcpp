#pragma once

#include "Colour.h"
#include "RobotMonitor.h"
#include <memory>
#include <stddef.h>
#include <vector>

namespace robosim::envcontroller
{

    using RobotPtr = std::shared_ptr<robotmonitor::RobotMonitor>;
    using MonitorVec = std::vector<RobotPtr>;

    extern MonitorVec robots;

    /**
     * Creates array of robots of type Robot monitor, with specified colour
     *
     * Usage makeRobots<RobotMonitor Type>(1, colour::RED);
     *
     */
    template <typename Type = robotmonitor::RobotMonitor>
    static inline void makeRobots(size_t size, colour::Colour colour)
    {
        for (size_t i = 0; i < size; i++)
        {
            robots.push_back(std::make_shared<Type>(false, colour));
        }
    }

    /**
     * Initialise the Environment Controller from a specified config file
     *
     * @param robots vector of robots
     * @param configFileName file path of the environment configuration file
     *
     */
    void EnvController(const char *, int);

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
    void EnvController(int, int, int);

    /**
     * Begin the simulation
     */
    void startSimulation();

    /**
     * Returns the diameter of a grid cell
     */
    float getCellWidth();

    /**
     * Returns the radius of a grid cell
     */
    float getCellRadius();

    /**
     * Returns whether the simulation is running
     */
    bool isRunning();

    /**
     * Ends the Simulation at anytime
     */
    void stop();

}
