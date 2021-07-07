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
#include "SimulatedRobot.h"
#include <SDL_pixels.h>
#include <chrono>
#include <exception>
#include <iostream>
#include <thread>
#include <type_traits>

namespace arenamodel {
class ArenaModel;
}

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using robosim::RobotMonitor;
using simulatedrobot::SimulatedRobot;

RobotMonitor::RobotMonitor(int delay, bool verbose) {
    this->delay = delay;
    this->verbose = verbose;
}

RobotMonitor::~RobotMonitor() {}

void *RobotMonitor::getRobot() { return robot.get(); }

void RobotMonitor::setArenaModel(std::shared_ptr<void> model) {
    auto modeltmp = std::static_pointer_cast<ArenaModel>(model).get();
    robot = std::make_shared<SimulatedRobot>(modeltmp);
}

bool RobotMonitor::setTravelSpeed(int travelSpeed) {
    return std::static_pointer_cast<SimulatedRobot>(robot)->setTravelSpeed(
        travelSpeed);
}

void RobotMonitor::travel() {
    std::static_pointer_cast<SimulatedRobot>(robot)->travel();
    while (
        ArenaModelView::running &&
        !std::static_pointer_cast<SimulatedRobot>(robot)->isAtDestination()) {
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
        }
    }
}

void RobotMonitor::rotate(int degrees) {
    std::static_pointer_cast<SimulatedRobot>(robot)->rotate(degrees);
    while (ArenaModelView::running &&
           !std::static_pointer_cast<SimulatedRobot>(robot)->isAtRotation()) {
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
        }
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

int RobotMonitor::getX() {
    return std::static_pointer_cast<SimulatedRobot>(robot)->getX();
}

int RobotMonitor::getY() {
    return std::static_pointer_cast<SimulatedRobot>(robot)->getY();
}

int RobotMonitor::getHeading() {
    return std::static_pointer_cast<SimulatedRobot>(robot)->getHeading();
}

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

bool RobotMonitor::isBumperPressed() {
    return std::static_pointer_cast<SimulatedRobot>(robot)->isBumperPressed();
}

SDL_Color RobotMonitor::getCSenseColor() {
    return std::static_pointer_cast<SimulatedRobot>(robot)->getCSenseColor();
}

int RobotMonitor::getUSenseRange() {
    return std::static_pointer_cast<SimulatedRobot>(robot)->getUSenseRange();
}

void RobotMonitor::setDirection(int degrees) {
    if (std::static_pointer_cast<SimulatedRobot>(robot)->setDirection(
            degrees)) {
        while (
            ArenaModelView::running &&
            std::static_pointer_cast<SimulatedRobot>(robot)->getDirection() !=
                degrees) {
            try {
                std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            } catch (const std::exception &e) {
                std::cerr << e.what() << '\n';
            }
        }
    }
}

int RobotMonitor::getDirection() {
    return std::static_pointer_cast<SimulatedRobot>(robot)->getDirection();
}

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
    auto r = std::static_pointer_cast<SimulatedRobot>(robot);
    while (*running) {
        if (verbose) {
            std::cout << "Pose: (" << r->getX() << "," << r->getY()
                      << ") with heading " << r->getHeading() << std::endl;
            std::cout << "with a current travel speed of "
                      << r->getTravelSpeed() << "mm per second" << std::endl;
            std::cout << "Bumper is pressed: " << r->isBumperPressed()
                      << std::endl;
            std::cout << "Colour Sensor: " << r->getCSenseColor().r << ""
                      << r->getCSenseColor().g << "" << r->getCSenseColor().b
                      << "" << std::endl;
            std::cout << "Range Sensor: " << r->getUSenseRange()
                      << " with direction " << r->getDirection() << std::endl;
            std::cout
                << "==========================================================="
                   "======="
                << std::endl;
        }
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
        }
    }
}
