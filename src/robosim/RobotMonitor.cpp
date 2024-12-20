#include "RobotMonitor.h"
#include "ArenaModel.h"
#include "Colour.h"
#include "SimulatedRobot.h"
#include <SDL_timer.h>
#include <iostream>
#include <string>

namespace
{

const uint32_t DELAY = 100;

} // namespace

namespace robosim::robotmonitor
{

uint8_t RobotMonitor::robotCount = 0;

RobotMonitor::RobotMonitor() = default;

RobotMonitor::RobotMonitor(bool verbose, Colour colour, bool *running)
{
    this->verbose = verbose;
    serialNumber = ++robotCount;
    this->colour = colour;
    this->running = running;
}

RobotMonitor::~RobotMonitor() = default;

std::shared_ptr<simulatedrobot::SimulatedRobot> RobotMonitor::getRobot() const
{
    return robot;
}

void RobotMonitor::setRobot(size_t robotSpeed)
{
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
    while (*running && !robot->isAtDestination())
    {
        SDL_Delay(DELAY);
    }
}

void RobotMonitor::rotate(int degrees)
{
    robot->rotate(degrees);
    while (*running && !robot->isAtRotation())
    {
        SDL_Delay(DELAY);
    }
}

void RobotMonitor::setDirection(int degrees)
{
    if (robot->setDirection(degrees))
    {
        while (*running && robot->getDirection() != degrees)
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

int RobotMonitor::getX() const
{
    return robot->getX();
}

int RobotMonitor::getY() const
{
    return robot->getY();
}

int RobotMonitor::getHeading() const
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

bool RobotMonitor::isBumperPressed() const
{
    return robot->isBumperPressed();
}

Colour RobotMonitor::getCSenseColor() const
{
    return robot->getCSenseColor();
}

int RobotMonitor::getUSenseRange() const
{
    return robot->getUSenseRange();
}

int RobotMonitor::getDirection() const
{
    return robot->getDirection();
}

int RobotMonitor::getTravelSpeed() const
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

void RobotMonitor::run()
{
    while (*running)
    {
        if (verbose)
        {
            debug();
        }
        SDL_Delay(DELAY);
    }
}

void RobotMonitor::callRunFunc() 
{
    if (run_func == nullptr) {
        std::cout << "RobotMonitor:run_func not set" << '\n';
        return;
    }

    run_func(this);   
}

void RobotMonitor::debug() const
{
    Colour c = getCSenseColor();

    std::cout << "Debug Robot " << serialNumber << "\nPose: (" << getX() << "," << getY() << ") with heading "
              << getHeading() << "\nwith a current travel speed of " << getTravelSpeed()
              << "mm per second\nBumper is pressed: " << (isBumperPressed() ? "true" : "false") << "\nColour Sensor: ("
              << std::to_string(c.r) << ", " << std::to_string(c.g) << ", " << std::to_string(c.b)
              << ")\nRange Sensor: " << getUSenseRange() << " with direction " << getDirection()
              << "\n==========================================================="
                 "=======\n";
}

int32_t RobotMonitor::getGridX() const
{
    return static_cast<int>((((static_cast<double>(getX() / arenamodel::cellWidth)) * 2) - 1) / 2);
}

int32_t RobotMonitor::getGridY() const
{
    return static_cast<int>((((static_cast<double>(getY() / arenamodel::cellWidth)) * 2) - 1) / 2);
}

} // namespace robosim::robotmonitor
