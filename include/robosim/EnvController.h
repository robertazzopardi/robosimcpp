/**
 * @file EnvController.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief Sets up and manages the environment
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __ENV_CONTROLLER_H__
#define __ENV_CONTROLLER_H__

#include "RobotMonitor.h"

namespace robosim {

namespace envcontroller {

/**
 * Initialise the Environment Controller
 *
 * @param robots vector of robots
 * @param speed robots speed
 * @param args config file path for pre set environments or the width and height
 * for an only bordered arena
 */
// template <typename... Args>
// void EnvController(const MonitorVec &, int, Args...);

/**
 * Initialise the Environment Controller from a specified config file
 *
 * @param robots vector of robots
 * @param configFileName file path of the environment configuration file
 *
 */
void EnvController(const robotmonitor::MonitorVec &, const char *, int);

/**
 * Create an Environment with given cell width and height, with a border of
 * obstacles
 *
 * @param robots vector of robots
 * @param rows number of rows
 * @param cols number of columns
 *
 */
void EnvController(const robotmonitor::MonitorVec &, int, int, int);

/**
 * Begin the simulation
 */
void startSimulation();

float getCellWidth();

float getCellRadius();

bool &isRunning();

} // namespace envcontroller

} // namespace robosim

#endif // !__ENV_CONTROLLER_H__
