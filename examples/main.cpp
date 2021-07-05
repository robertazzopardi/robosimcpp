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
 * clang++ -Wall -Werror -Wextra -std=c++20 -o main main.cpp -I/usr/local/include/robosim -L/usr/local/lib -lrobosim
 *
 *
 */

#include <iostream>
#include <robosim.h>

static constexpr auto CONFIG_NAME = "./defaultConfig.txt";

class Robot : public robosim::RobotMonitor {
  public:
    Robot(int delay, bool verbose) : robosim::RobotMonitor(delay, verbose) {}
    void run(bool *in) {
        std::cout << "Hello Robot " << *in << std::endl;
        travel();
        // rotate(90);
    }
};

int main() {
    Robot robot(1000, false);
    robosim::EnvController env(CONFIG_NAME, 7, 7, &robot);

    env.updateEnv();

    return 0;
}
