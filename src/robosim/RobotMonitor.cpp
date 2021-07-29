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
#include "Colour.h"
#include "EnvController.h"
#include "SimulatedRobot.h"
#include <SDL_timer.h>
#include <iostream>
#include <string>
#include <type_traits>

namespace {

using simulatedrobot::SimulatedRobot;

template <typename T> inline auto cast_ptr(std::shared_ptr<void> ptr) {
    return std::static_pointer_cast<T>(ptr);
}

constexpr auto Sim = cast_ptr<SimulatedRobot>;

constexpr auto DELAY = 100;

template <typename Condition> void wait(Condition condition) {
    while (arenamodelview::running && condition()) {
        SDL_Delay(DELAY);
    }
}

} // namespace

namespace robosim::robotmonitor {

// MonitorVec robots;

int RobotMonitor::robotCount = 1;

RobotMonitor::RobotMonitor(bool verbose, colour::Colour colour) {
    this->verbose = verbose;
    serialNumber = robotCount++;
    this->colour = colour;
}

RobotMonitor::~RobotMonitor() {}

void *RobotMonitor::getRobot() { return robot.get(); }

void RobotMonitor::setRobot(int robotSpeed) {
    robot = std::make_shared<SimulatedRobot>(true, colour);
    Sim(robot)->setTravelSpeed(robotSpeed);
}

bool RobotMonitor::setTravelSpeed(int travelSpeed) {
    return Sim(robot)->setTravelSpeed(travelSpeed);
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

void RobotMonitor::setPose(int x, int y, int heading) {
    Sim(robot)->setPose(x, y, heading);
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

    std::cout << "Debug Robot " << serialNumber << "\nPose: (" << getX() << ","
              << getY() << ") with heading " << getHeading()
              << "\nwith a current travel speed of " << getTravelSpeed()
              << "mm per second\nBumper is pressed: "
              << (isBumperPressed() ? "true" : "false") << "\nColour Sensor: ("
              << std::to_string(c.r) << ", " << std::to_string(c.g) << ", "
              << std::to_string(c.b) << ")\nRange Sensor: " << getUSenseRange()
              << " with direction " << getDirection()
              << "\n==========================================================="
                 "=======\n";
}

int RobotMonitor::getGridX() {
    return (int)((((double)getX() / envcontroller::getCellWidth()) * 2) - 1) /
           2;
}

int RobotMonitor::getGridY() {
    return (int)((((double)getY() / envcontroller::getCellWidth()) * 2) - 1) /
           2;
}

} // namespace robosim::robotmonitor
