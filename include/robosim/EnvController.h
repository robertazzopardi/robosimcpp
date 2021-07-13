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

/**
 * Initialise the Environment Controller from a specified config file
 *
 * @param robots vector of robots
 * @param configFileName file path of the environment configuration file
 *
 */
void EnvController(const MonitorVec &, const char *);

/**
 * Create an Environment with given cell width and height, with a border of
 * obstacles
 *
 * @param robots vector of robots
 * @param rows number of rows
 * @param cols number of columns
 *
 */
void EnvController(const MonitorVec &, int, int);

/**
 * Begin the simulation
 */
void startSimulation();

} // namespace robosim

#endif // !__ENV_CONTROLLER_H__
