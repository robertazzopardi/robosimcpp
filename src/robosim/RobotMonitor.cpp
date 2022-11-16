#include <SDL_timer.h>

#include <iostream>
#include <string>
#include <type_traits>

#include "ArenaModelView.h"
#include "Colour.h"
#include "EnvController.h"
#include "RobotMonitor.h"
#include "SimulatedRobot.h"

namespace
{
constexpr auto DELAY = 100;

// template <typename Condition> void wait(Condition condition)
// {
//    while (arenamodelview::running && condition())
//    {
//        SDL_Delay(DELAY);
//    }
//}

} // namespace

namespace robosim::robotmonitor
{

uint8_t RobotMonitor::robotCount = 0;

RobotMonitor::RobotMonitor()
{
}

RobotMonitor::RobotMonitor(bool verbose, colour::Colour colour)
{
    this->verbose = verbose;
    serialNumber = ++robotCount;
    this->colour = colour;
}

RobotMonitor::~RobotMonitor()
{
}

std::shared_ptr<simulatedrobot::SimulatedRobot> RobotMonitor::getRobot()
{
    return robot;
}

void RobotMonitor::setRobot(int robotSpeed)
{
    // robot = std::make_shared<SimulatedRobot>(true, colour);
    robot = std::make_shared<simulatedrobot::SimulatedRobot>(simulatedrobot::SimulatedRobot(true, colour));
    robot->setTravelSpeed(robotSpeed);
}

bool RobotMonitor::setTravelSpeed(int travelSpeed)
{
    return robot->setTravelSpeed(travelSpeed);
}

void RobotMonitor::travel()
{
    robot->travel();
    while (arenamodelview::running && !robot->isAtDestination())
    {
        SDL_Delay(DELAY);
    }
}

void RobotMonitor::rotate(int degrees)
{
    robot->rotate(degrees);
    while (arenamodelview::running && !robot->isAtRotation())
    {
        SDL_Delay(DELAY);
    }
}

void RobotMonitor::setDirection(int degrees)
{
    if (robot->setDirection(degrees))
    {
        while (arenamodelview::running && robot->getDirection() != degrees)
        {
	   SDL_Delay(DELAY);
        }
    }
}

void RobotMonitor::setPose(int x, int y, int heading)
{
    robot->setPose(x, y, heading);
}

/**
 * =========================================================================
 * API Pose Methods
 * =========================================================================
 * int getX(); // Get X location on Map int getY();
 * // Get Y location on Map int getHeading()
 * // Get heading angle wrt map
 */

int RobotMonitor::getX()
{
    return robot->getX();
}

int RobotMonitor::getY()
{
    return robot->getY();
}

int RobotMonitor::getHeading()
{
    return robot->getHeading();
}

/**
 * =========================================================================
 * API Sensor Methods
 * =========================================================================
 * Boolean isBumperPressed();					// True
 * if the robot is adjacent to an obstacle
 *
 * int getUSenseRange();						//
 * Return distance to nearest object int getDirection();
 * // Return angle of sensor
 * int setDirection(); // Set the direction of the sensor
 *
 * Color getCSenseColor();						// Get
 * the current sensed
 */

bool RobotMonitor::isBumperPressed()
{
    return robot->isBumperPressed();
}

colour::Colour RobotMonitor::getCSenseColor()
{
    return robot->getCSenseColor();
}

int RobotMonitor::getUSenseRange()
{
    return robot->getUSenseRange();
}

int RobotMonitor::getDirection()
{
    return robot->getDirection();
}

int RobotMonitor::getTravelSpeed()
{
    return robot->getTravelSpeed();
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

void RobotMonitor::run(bool *running)
{
    while (arenamodelview::running && *running)
    {
        if (verbose)
        {
	   debug();
        }
        SDL_Delay(DELAY);
    }
}

void RobotMonitor::debug()
{
    colour::Colour c = getCSenseColor();

    std::cout << "Debug Robot " << serialNumber << "\nPose: (" << getX() << "," << getY() << ") with heading "
	     << getHeading() << "\nwith a current travel speed of " << getTravelSpeed()
	     << "mm per second\nBumper is pressed: " << (isBumperPressed() ? "true" : "false") << "\nColour Sensor: ("
	     << std::to_string(c.r) << ", " << std::to_string(c.g) << ", " << std::to_string(c.b)
	     << ")\nRange Sensor: " << getUSenseRange() << " with direction " << getDirection()
	     << "\n==========================================================="
	        "=======\n";
}

int32_t RobotMonitor::getGridX()
{
    return static_cast<int>((((static_cast<double>(getX() / envcontroller::getCellWidth())) * 2) - 1) / 2);
}

int32_t RobotMonitor::getGridY()
{
    return static_cast<int>((((static_cast<double>(getY() / envcontroller::getCellWidth())) * 2) - 1) / 2);
}

} // namespace robosim::robotmonitor
