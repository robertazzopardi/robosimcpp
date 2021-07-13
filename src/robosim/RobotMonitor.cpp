/**
 * @file RobotMonitor.cpp
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief
 * @version 0.1
 * @date 2021-07-07
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "RobotMonitor.h"
#include "ArenaModelView.h"
#include "Casting.h"
#include "Colour.h"
#include "SimulatedRobot.h"
#include <SDL_timer.h>
#include <iostream>
#include <string>
#include <type_traits>

namespace arenamodel {
class ArenaModel;
}

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using robosim::RobotMonitor;
using simulatedrobot::SimulatedRobot;

constexpr auto Sim = typecasting::cast_ptr<SimulatedRobot>;
constexpr auto MSim = typecasting::make_ptr<SimulatedRobot, bool>;

constexpr auto DELAY = 100;

int RobotMonitor::robotCount = 1;

RobotMonitor::RobotMonitor(bool verbose) {
    this->verbose = verbose;
    serialNumber = robotCount++;
}

RobotMonitor::~RobotMonitor() {}

void *RobotMonitor::getRobot() { return robot.get(); }

void RobotMonitor::setRobot() { robot = MSim(true); }

bool RobotMonitor::setTravelSpeed(int travelSpeed) {
    return Sim(robot)->setTravelSpeed(travelSpeed);
}

template <typename Condition> void RobotMonitor::wait(Condition condition) {
    while (ArenaModelView::running && condition()) {
        SDL_Delay(DELAY);
    }
}

void RobotMonitor::travel() {
    Sim(robot)->travel();
    wait([robot = robot] { return !Sim(robot)->isAtDestination(); });
}

void RobotMonitor::rotate(int degrees) {
    Sim(robot)->rotate(degrees);
    wait([robot = robot] { return !Sim(robot)->isAtRotation(); });
}

void RobotMonitor::setDirection(int degrees) {
    if (Sim(robot)->setDirection(degrees)) {
        wait([robot = robot, degrees = degrees] {
            return Sim(robot)->getDirection() != degrees;
        });
    }
}

/**
 * =========================================================================
 * API Pose Methods
 * =========================================================================
 * int getX(); // Get X location on Map int getY();
 * // Get Y location on Map int getHeading()
 * // Get heading angle wrt map
 */

int RobotMonitor::getX() { return Sim(robot)->getX(); }

int RobotMonitor::getY() { return Sim(robot)->getY(); }

int RobotMonitor::getHeading() { return Sim(robot)->getHeading(); }

/**
 * =========================================================================
 * API Sensor Methods
 * =========================================================================
 * Boolean isBumperPressed();					// True if the
 * robot is adjacent to an obstacle
 *
 * int getUSenseRange();						//
 * Return distance to nearest object int getDirection();
 * // Return angle of sensor
 * int setDirection();							// Set
 * the direction of the sensor
 *
 * Color getCSenseColor();						// Get
 * the current sensed
 */

bool RobotMonitor::isBumperPressed() { return Sim(robot)->isBumperPressed(); }

colour::Colour RobotMonitor::getCSenseColor() {
    return Sim(robot)->getCSenseColor();
}

int RobotMonitor::getUSenseRange() { return Sim(robot)->getUSenseRange(); }

int RobotMonitor::getDirection() { return Sim(robot)->getDirection(); }

int RobotMonitor::getTravelSpeed() { return Sim(robot)->getTravelSpeed(); }

// =========================================================================================
/**
 * Turn on verbose diagnostics to check status of the robot.  Output will appear
 * in stdout.
 * @param verbose is either true (enable diagnostics) or false (disable
 * diagnostics)
 */
// void RobotMonitor::monitorRobotStatus(bool verbose) {
//     this->verbose = verbose;
// }

// =========================================================================================

void RobotMonitor::run(bool *running) {
    wait([this, running]() {
        if (verbose)
            debug();
        return *running;
    });
}

void RobotMonitor::debug() {
    auto c = getCSenseColor();

    std::cout << "Debug Robot " << serialNumber << std::endl;
    std::cout << "Pose: (" << getX() << "," << getY() << ") with heading "
              << getHeading() << std::endl;
    std::cout << "with a current travel speed of " << getTravelSpeed()
              << "mm per second" << std::endl;
    std::cout << "Bumper is pressed: " << (isBumperPressed() ? "true" : "false")
              << std::endl;
    std::cout << "Colour Sensor: (" << std::to_string(c.r) << ", "
              << std::to_string(c.g) << ", " << std::to_string(c.b) << ") "
              << std::endl;
    std::cout << "Range Sensor: " << getUSenseRange() << " with direction "
              << getDirection() << std::endl;
    std::cout << "==========================================================="
                 "======="
              << std::endl;
}
