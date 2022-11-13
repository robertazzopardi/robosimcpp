#include "SimulatedRobot.h"
#include "ArenaModel.h"
#include "ArenaModelView.h"
#include "Colour.h"
#include "MyGridCell.h"
#include <SDL_rect.h>
#include <SDL_timer.h>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <random>
#include <stdlib.h>
#include <string>
#include <vector>

constexpr auto DEGREES_TO_RADIANS = 0.017453292519943295;

// Physical Characteristics of the Robot

// Leftmost angle that the sensor can be set
constexpr auto LEFT_SENSORANGLE = -90;
// Rightmost angle that the sensor can be set
constexpr auto RIGHT_SENSORANGLE = 90;
// Min speed of robot in mm per second
constexpr auto LOWER_TRAVELSPEED = 10;
// Max speed of robot in mm per second
constexpr auto UPPER_TRAVELSPEED = 100;
// The robot is modeled as a circle with this radius in mm.
constexpr auto RADIUS_ROBOTBODY = 100;
// The robot sensor modeled as a circle with this radius in mm.
constexpr auto RADIUS_SENSORBODY = 20;
// Update Delay is 0.1 second, to simplify update calculations
constexpr auto UPDATE_DELAY = 100;
// Rotation speed should be fixed & independent of travel speed
constexpr auto ROTATION_SPEED = 50;
// Maximum range of the sensor in mm
constexpr auto US_SENSOR_MAX_RANGE = 2550;
constexpr auto UPDATE_RATE = UPDATE_DELAY / 1000.0;

using mygridcell::OccupancyType;
using simulatedrobot::SimulatedRobot;

namespace
{
static std::random_device rd;
static std::mt19937 mt(rd());
} // namespace

SimulatedRobot::SimulatedRobot()
{
}

SimulatedRobot::SimulatedRobot(bool randomLocation, colour::Colour colour)
{
    static std::uniform_int_distribution<int> distX(1, arenamodel::grid[0].size() - 1);
    static std::uniform_int_distribution<int> distY(1, arenamodel::grid.size() - 1);

    if (randomLocation)
    {
        int x = 0, y = 0;

        do
        {
            x = distX(mt);
            y = distY(mt);
        } while (arenamodel::getOccupancy(x, y) != OccupancyType::EMPTY);

        arenamodel::setOccupancy({x, y, OccupancyType::ROBOT});

        auto r = arenamodel::cellWidth / 2;
        xLocation = (x * arenamodel::cellWidth) + r;
        yLocation = (y * arenamodel::cellWidth) + r;
    }
    else
    {
        auto center = static_cast<int>(3 * arenamodel::cellWidth / 2);

        // Position the robot in the center of the (1,1) cell
        xLocation = center; // center of (1, 1)
        yLocation = center; // center of (1, 1)
    }

    setTravelSpeed(LOWER_TRAVELSPEED); // i->e-> no speed->
    setHeading(0);                     // i->e-> due north

    rotationSpeedPerUpdate = static_cast<double>(ROTATION_SPEED / 10);
    // Set any other parameters

    // Create the robots render object
    auto r = arenamodel::cellWidth / 3;
    robotRender.body.r = r;
    auto rs = r / 6;
    robotRender.sensor.r = rs;

    // robotRender.bodyColour = colour;
    robotRender.bodyColour = colour;

    update();
}

SimulatedRobot::~SimulatedRobot()
{
}

// =======================================================================

uint32_t SimulatedRobot::getRobotBodySize()
{
    return RADIUS_ROBOTBODY;
}

uint32_t SimulatedRobot::getSensorBodySize()
{
    return RADIUS_SENSORBODY;
}

/* =========================================================================
 * Pose Methods
 * =========================================================================
 */

int SimulatedRobot::getHeading()
{
    return heading;
}

double SimulatedRobot::getHeadingInRadians()
{
    return headingInRadians;
}

double SimulatedRobot::getDirectionInRadians()
{
    return currentSensorAngle * DEGREES_TO_RADIANS;
}

void SimulatedRobot::setHeading(int heading)
{
    this->heading = heading;

    headingInRadians = heading * DEGREES_TO_RADIANS;
}

int SimulatedRobot::getX()
{
    return xLocation;
}

int SimulatedRobot::getY()
{
    return yLocation;
}

void SimulatedRobot::setPose(int x, int y, int heading)
{
    // Note that we don't yet do bounds checking, and thus the robot could be
    // misplaced.
    xLocation = x;
    yLocation = y;
    setHeading(heading);
}

/* =========================================================================
 * Locomotion Methods
 * =========================================================================
 */

bool SimulatedRobot::setTravelSpeed(int travelSpeed)
{
    if (travelSpeed < LOWER_TRAVELSPEED || travelSpeed > UPPER_TRAVELSPEED)
    {
        std::cerr << "Invalid Travel Speed - setTravelSpeed(" << std::to_string(travelSpeed) << ")";
        return false;
    }
    this->travelSpeed = travelSpeed; // This is the speed per second (i.e. 1000 time units)
    travelSpeedPerUpdate = travelSpeed * UPDATE_RATE;

    return true;
}

int SimulatedRobot::getTravelSpeed()
{
    return travelSpeed;
}

void SimulatedRobot::travel()
{
    currentDistanceToDestination += arenamodel::cellWidth;
}

bool SimulatedRobot::isAtDestination()
{
    return currentDistanceToDestination == 0;
}

void SimulatedRobot::rotate(int degrees)
{
    currentAngleToNewHeading += degrees;
}

bool SimulatedRobot::isAtRotation()
{
    return currentAngleToNewHeading == 0;
}

/* =========================================================================
 * Sensor Methods
 * =========================================================================
 */

bool SimulatedRobot::isBumperPressed()
{
    return bumperPressed;
}

colour::Colour SimulatedRobot::getCSenseColor()
{
    double cellWidth = arenamodel::cellWidth;
    double colPos = floor(getX() / cellWidth);
    double rowPos = floor(getY() / cellWidth);

    mygridcell::OccupancyType occupancy = arenamodel::getOccupancy(rowPos, colPos);
    switch (occupancy)
    {
    case OccupancyType::RED:
        return colour::RED;
    case OccupancyType::GREEN:
        return colour::GREEN;
    case OccupancyType::BLUE:
        return colour::BLUE;
    case OccupancyType::EMPTY:
    case OccupancyType::OBSTACLE:
    case OccupancyType::ROBOT:
    case OccupancyType::UNKNOWN:
    default:
        return colour::WHITE;
    }
}

// =====================================================================
// setDirection()
// getDirection()

bool SimulatedRobot::setDirection(int degrees)
{
    if (degrees < LEFT_SENSORANGLE || degrees > RIGHT_SENSORANGLE)
    {
        std::cerr << "Invalid Sensor Angle - setDirection(" << std::to_string(degrees) << ")";
        return false;
    }
    sensorDirection = degrees;

    return true;
}

int SimulatedRobot::getDirection()
{
    // NOTE that this returns the current sensor angle, not the desired angle
    return currentSensorAngle;
}

int SimulatedRobot::getUSenseRange()
{
    double sensorH = (getDirection() + getHeading()) * DEGREES_TO_RADIANS; // sensor direction in radians

    int range;
    for (range = 0; range < US_SENSOR_MAX_RANGE; range += 10)
    {
        double xDelta = round(sin(sensorH) * range);
        double yDelta = round(cos(sensorH) * range);
        if (isColliding(xLocation, yLocation, xDelta, yDelta))
        {
            break;
        }
    }

    return range;
}

// =======================================================================
// Obstacle Detection

bool SimulatedRobot::isColliding(int xPos, int yPos, int xDelta, int yDelta)
{
    int xBound;                      // boundary of the robot in the x axis (either +ve or -ve)
    int yBound;                      // boundary of the robot in the y axis (either +ve or -ve)
    bool collision = false;          // collision flag
    float w = arenamodel::cellWidth; // cell width
    float h = arenamodel::cellWidth; // cell height

    int32_t x = xPos + xDelta;
    int32_t y = yPos + yDelta;

    double r = (arenamodel::cellWidth / 3) * .9;

    if (xDelta >= 0)
    {
        xBound = x + r; // check on the right most part of the robot
    }
    else
    {
        xBound = x - r; // check on the left most part of the robot
    }

    if (yDelta >= 0)
    {                   // check up / up left / left
        yBound = y + r; // check on the bottom most part of the robot
    }
    else
    {                   // check left / down left / down
        yBound = y - r; // check on the top most part of the robot
    }

    if (arenamodel::getOccupancy(xBound / w, yPos / h) == OccupancyType::OBSTACLE ||   // Check left/right - x axis only
        arenamodel::getOccupancy(xBound / w, yBound / h) == OccupancyType::OBSTACLE || // Check diagonal - x/y axis
        arenamodel::getOccupancy(xPos / w, yBound / h) == OccupancyType::OBSTACLE      // Check up/down - y axis only
    )
    {
        collision = true;
    }
    // We don't check for out of bounds (i.e. hitting the wall...)!
    // This is currently managed by assuming obstacles around the arena

    return collision;
}

// =======================================================================
// Robot Update

void SimulatedRobot::update()
{
    // parameters
    double x = getX();
    double y = getY();

    double angle = getHeadingInRadians();

    double scale = robotRender.body.r * .9;

    double rx = x + sin(angle) * scale;
    double ry = y + cos(angle) * scale;

    // body
    robotRender.body.x = x;
    robotRender.body.y = y;

    // heading line
    robotRender.radius.x = rx;
    robotRender.radius.y = ry;

    double sensorAngle = getDirectionInRadians() + angle;

    double sx = x + sin(sensorAngle) * scale;
    double sy = y + cos(sensorAngle) * scale;

    robotRender.sensor.x = sx;
    robotRender.sensor.y = sy;
}

simulatedrobot::RobotRender SimulatedRobot::getRenderObject()
{
    return robotRender;
}

void SimulatedRobot::run()
{
    while (arenamodelview::running)
    {
        double deltaDist = 0;                            // Represents the distance to travel
        double deltaRotation = 0;                        // Represents the rotation distance to rotate
        double travelSegment = travelSpeedPerUpdate;     // We track movement in ints!
        double rotationSegment = rotationSpeedPerUpdate; // We track movement in ints!

        // Are we moving?

        if (currentDistanceToDestination != 0)
        {
            if (currentDistanceToDestination < 0)
            {
                // Need to invert travelSegment to go in the right direction
                travelSegment = -travelSegment;
            }

            if (abs(currentDistanceToDestination) < abs(travelSegment))
            {
                // We have less than the travelSpeed to the destination, so just
                // move to destination
                deltaDist = currentDistanceToDestination;
                // No more destination to go
                currentDistanceToDestination = 0;
            }
            else
            {
                deltaDist = travelSegment;
                currentDistanceToDestination -= travelSegment;
            }

            // Need to check if we are about to run into an obstacle
            double xDelta = sin(headingInRadians) * deltaDist;
            double yDelta = cos(headingInRadians) * deltaDist;

            // Check for collisions
            bumperPressed = isColliding(xLocation, yLocation, xDelta, yDelta);
            if (!bumperPressed)
            {
                // Update x & y position
                xLocation += xDelta;
                yLocation += yDelta;
            }
        }

        // =======================================================================
        // Are we also rotating?
        if (currentAngleToNewHeading != 0)
        {
            if (currentAngleToNewHeading < rotationSegment)
            {
                // Need to invert travelSegment to go in the right direction
                rotationSegment = -rotationSegment;
            }

            if (abs(currentAngleToNewHeading) < abs(rotationSegment))
            {
                deltaRotation = currentAngleToNewHeading;
                currentAngleToNewHeading = 0;
            }
            else
            {
                deltaRotation = rotationSegment;
                currentAngleToNewHeading -= rotationSegment;
            }
            // Update Rotation
            setHeading(getHeading() + deltaRotation);
        }

        // =======================================================================
        // Is the sensor in the correct place
        if (currentSensorAngle != sensorDirection)
        {
            // we need to move it
            if (sensorDirection > currentSensorAngle)
            {
                if (sensorDirection - currentSensorAngle > rotationSegment)
                {
                    currentSensorAngle += rotationSegment;
                }
                else
                {
                    currentSensorAngle = sensorDirection;
                }
            }
            else
            {
                if (currentSensorAngle - sensorDirection > rotationSegment)
                {
                    currentSensorAngle -= rotationSegment;
                }
                else
                {
                    currentSensorAngle = sensorDirection;
                }
            }
        }

        update();

        SDL_Delay(UPDATE_DELAY);
    }
}
