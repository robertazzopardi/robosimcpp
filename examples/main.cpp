/**
 * @file main.cpp
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief
 * @version 0.1
 * @date 2021-07-05
 *
 * @copyright Copyright (c) 2021
 *
 * Compile This File With:
 * clang++ -o main main.cpp -I/usr/local/include/robosim
 * -L/usr/local/lib -lrobosim
 * ./main
 *
 */
// clang++ -Wall -Werror -Wextra -std=c++20 -o main main.cpp
// -I/usr/local/include/robosim -L/usr/local/lib -lrobosim; ./main

#include <iostream>
#include <random>
#include <robosim.h>

#define CONFIG_NAME "./defaultConfig.txt"

class Robot : public robosim::RobotMonitor {
  public:
    // Inherit constructor (essentially super the RobotMonitor constructor)
    Robot(bool verbose) : robosim::RobotMonitor(verbose) {}

    // Override run, to implement the robots behaviour
    void run(bool *running) {
        std::cout << "Starting Robot: " << serialNumber << std::endl;

        while (*running) {
            travel();

            rotate(90);
            // setDirection(-90);
            debug();
        }
    }
};

int main(void) {
    robosim::MonitorVec robots = robosim::getRobots<Robot>(1);
    // robosim::MonitorVec robots = robosim::getRobots<Robot>(2);

    robosim::EnvController env(CONFIG_NAME, robots);
    // robosim::EnvController env(5, 5, robots);

    for (auto &robot : robots) {
        robot->setTravelSpeed(50);
    }

    env.startSimulation();

    return 0;
}
