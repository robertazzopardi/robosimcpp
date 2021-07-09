/**
 * @file main.cpp
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief
 * @version 0.1
 * @date 2021-07-05
 *
 * @copyright Copyright (c) 2021
 *
 * Example from the RoboSim page
 *
 * Compile This File With:
 * clang++ -o main main.cpp -I/usr/local/include/robosim
 * -L/usr/local/lib -lrobosim
 * ./main
 *
 */

#include <iostream>
#include <robosim.h>

using robosim::EnvController;
using robosim::RobotMonitor;

int main(void) {

  RobotMonitor monitor(true);
  EnvController controller("./defaultConfig.txt", 7, 7, &monitor);

  // Passed in RobotMonitor constructor instead
  // monitor.monitorRobotStatus(true);
  // monitor.setTravelSpeed(100);

  monitor.setDirection(0);

  while (monitor.getUSenseRange() > 700) {
    monitor.travel();
  }
  monitor.rotate(90);

  while (!monitor.isBumperPressed()) {
    monitor.travel();
    if (monitor.getCSenseColor() == colour::GREEN) {
      std::cout << "Found green cell at location (" << monitor.getX() << ", "
                << monitor.getY() << ") with heading " << monitor.getHeading()
                << std::endl;
    }
  }

  return 0;
}
