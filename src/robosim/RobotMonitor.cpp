#include "RobotMonitor.h"
#include "SimulatedRobot.h"
#include <SDL_pixels.h>
#include <chrono>
#include <exception>
#include <iostream>
#include <thread>

namespace arenamodel {
class ArenaModel;
}

using robosim::RobotMonitor;
using simulatedrobot::SimulatedRobot;

RobotMonitor::RobotMonitor(int delay, bool verbose) {
    this->delay = delay;
    this->verbose = verbose;
}

RobotMonitor::~RobotMonitor() {
    delete static_cast<SimulatedRobot *>(robot);
    robot = nullptr;
}

void *RobotMonitor::getRobot() {
    return robot;
}

void RobotMonitor::setArenaModel(void *model) {
    robot = new SimulatedRobot(static_cast<arenamodel::ArenaModel *>(model));
}

bool RobotMonitor::setTravelSpeed(int travelSpeed) {
    return static_cast<SimulatedRobot *>(robot)->setTravelSpeed(travelSpeed);
}

void RobotMonitor::travel() {
    static_cast<SimulatedRobot *>(robot)->travel();
    std::cout << static_cast<SimulatedRobot *>(robot)->isAtDestination() << std::endl;
    // while (!static_cast<SimulatedRobot *>(robot)->isAtDestination()) {
    //     try {
    //         std::cout << "Travelling" << std::endl;
    //         std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    //     } catch (const std::exception &e) {
    //         std::cerr << e.what() << '\n';
    //     }
    // }
}

void RobotMonitor::rotate(int degrees) {
    static_cast<SimulatedRobot *>(robot)->rotate(degrees);
    while (!static_cast<SimulatedRobot *>(robot)->isAtRotation()) {
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
 * int getX();									// Get X location on Map
 * int getY();									// Get Y location on Map
 * int getHeading()								// Get heading angle wrt map
 */

/**
 * Obtains the current position of the robot in the x axis (in mm)
 * @return the x location on the map
 */
int RobotMonitor::getX() {
    return static_cast<SimulatedRobot *>(robot)->getX();
}

/**
 * Obtains the current position of the robot in the y axis (in mm)
 * @return the y location on the map
 */
int RobotMonitor::getY() {
    return static_cast<SimulatedRobot *>(robot)->getY();
}

/**
 * Obtains the current position of the robot
 * The heading is in the nearest degree, such that
 * 0 is along the y axis, and increasing
 * values rotate in a clockwise direction
 * @return the heading
 */
int RobotMonitor::getHeading() {
    return static_cast<SimulatedRobot *>(robot)->getHeading();
}

/**
 * =========================================================================
 * API Sensor Methods
 * =========================================================================
 * Boolean isBumperPressed();					// True if the robot is adjacent to an obstacle
 *
 * int getUSenseRange();						// Return distance to nearest object
 * int getDirection();							// Return angle of sensor
 * int setDirection();							// Set the direction of the sensor
 *
 * Color getCSenseColor();						// Get the current  sensed
 */

/**
 * Check if the bumper is pressed.  This will be true if the robot
 * tried to move into an obstacle.  If the robot moves away from an
 * obstacle successfully, then the value is false.
 * @return bumper status
 */
bool RobotMonitor::isBumperPressed() {
    return static_cast<SimulatedRobot *>(robot)->isBumperPressed();
}

/**
 * Check the  of the cell beneath the center of the robot.
 * @return a Color object
 */
SDL_Color RobotMonitor::getCSenseColor() {
    return static_cast<SimulatedRobot *>(robot)->getCSenseColor();
}

/**
 * Get the range of the nearest object in the direction of the sensor.
 * This senses an object that would result in a collision with the robot
 * if it were to move in the direction of the sensor (i.e. the cone of the
 * sensor is parallel with a width being that of the robot diameter).
 * The maximum range is 2550mm.
 * @return range to nearest object in the direction of the sensor in mm
 */
int RobotMonitor::getUSenseRange() {
    return static_cast<SimulatedRobot *>(robot)->getUSenseRange();
}

/**
 * Set the desired direction of the sensor, as an offset of the heading
 * of the robot.  Valid ranges are -90 (i.e. looking left) to 90 (i.e. looking right).
 * Any angle between these ranges can be selected, and the method will block until
 * the sensor is pointing in this direction.
 * @param angle (in degrees) from the heading of the robot
 */
void RobotMonitor::setDirection(int degrees) {
    if (static_cast<SimulatedRobot *>(robot)->setDirection(degrees)) {
        while (static_cast<SimulatedRobot *>(robot)->getDirection() != degrees) {
            try {
                std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            } catch (const std::exception &e) {
                std::cerr << e.what() << '\n';
            }
        }
    }
}

/**
 * Get the current direction (as an offset to the robot heading) of
 * the sensor.
 * @return the angle of the sensor, where 0 is in the direction of the robot;
 * and any value in the range -90 (i.e. looking left) to 90 (i.e. looking right)
 */
int RobotMonitor::getDirection() {
    return static_cast<SimulatedRobot *>(robot)->getDirection();
}

// =========================================================================================
/**
 * Turn on verbose diagnostics to check status of the robot.  Output will appear in stdout.
 * @param verbose is either true (enable diagnostics) or false (disable diagnostics)
 */
// void RobotMonitor::monitorRobotStatus(bool verbose) {
//     this->verbose = verbose;
// }

// =========================================================================================
/**
 * The monitor writes various bits of robot state to the screen, then sleeps.
 */
void RobotMonitor::run(bool *running) {
    while (*running) {
        if (verbose) {
            std::cout << "Pose: (" << static_cast<SimulatedRobot *>(robot)->getX() << "," << static_cast<SimulatedRobot *>(robot)->getY() << ") with heading " << static_cast<SimulatedRobot *>(robot)->getHeading() << std::endl;
            std::cout << "with a current travel speed of " << static_cast<SimulatedRobot *>(robot)->getTravelSpeed() << "mm per second" << std::endl;
            std::cout << "Bumper is pressed: " << static_cast<SimulatedRobot *>(robot)->isBumperPressed() << std::endl;
            std::cout << "Colour Sensor: " << static_cast<SimulatedRobot *>(robot)->getCSenseColor().r << "" << static_cast<SimulatedRobot *>(robot)->getCSenseColor().g << "" << static_cast<SimulatedRobot *>(robot)->getCSenseColor().b << "" << std::endl;
            std::cout << "Range Sensor: " << static_cast<SimulatedRobot *>(robot)->getUSenseRange() << " with direction " << static_cast<SimulatedRobot *>(robot)->getDirection() << std::endl;
            std::cout << "==================================================================" << std::endl;
        }
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
        }
    }
}
