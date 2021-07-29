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

class Robot : public robosim::robotmonitor::RobotMonitor {
  public:
    // Inherit constructor (essentially super the RobotMonitor constructor)
    Robot(bool verbose, colour::Colour colour)
        : robosim::robotmonitor::RobotMonitor(verbose, colour) {}

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
    robosim::envcontroller::makeRobots<Robot>(3, colour::OFF_BLACK);

    robosim::envcontroller::EnvController(CONFIG_NAME, 50);
    // robosim::envcontroller::EnvController(10, 10, 50);

    robosim::envcontroller::startSimulation();

    return 0;
}
