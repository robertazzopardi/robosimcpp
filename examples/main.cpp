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
    Robot(bool verbose, colour::Colour colour)
        : robosim::RobotMonitor(verbose, colour) {}

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
    auto robots = robosim::getRobots<Robot>(1, colour::OFF_BLACK);

    robosim::EnvController(robots, CONFIG_NAME, 50);
    // robosim::EnvController(robots, 10, 10, 50);

    robosim::startSimulation();

    return 0;
}
