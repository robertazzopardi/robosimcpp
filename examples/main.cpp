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

#include <iostream>
#include <robosim.h>

#define CONFIG_NAME "./defaultConfig.txt"

class Robot : public robosim::RobotMonitor {
  public:
    // Inherit constructor (essentially super the RobotMonitor constructor)
    Robot(bool verbose) : robosim::RobotMonitor(verbose) {}

    // Override the run method
    // The code here will be executed by the robot
    void run(bool *running) {
        while (*running) {
            std::cout << "Hello Robot " << std::endl;
            travel();
            rotate(90);
            // debug();
        }
    }
};

int main(void) {
    Robot robot(false);

    robosim::EnvController env(CONFIG_NAME, 10, 10, &robot);

    robot.setTravelSpeed(50);

    env.updateEnv();

    return 0;
}
