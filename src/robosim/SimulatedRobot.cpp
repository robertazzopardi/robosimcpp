#include "SimulatedRobot.h"
#include "ArenaModel.h"
#include "MyGridCell.h"
#include "SDLColours.h"
#include <SDL_pixels.h>
#include <chrono>
#include <exception>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <thread>

#define DEGREES_TO_RADIANS 0.017453292519943295

using arenamodel::ArenaModel;
using mygridcell::OccupancyType;
using simulatedrobot::SimulatedRobot;

SimulatedRobot::SimulatedRobot(ArenaModel *model) {
    this->model = model;

    // Position the robot in the center of the (1,1) cell
    xLocation = (int)(3 * model->getCellWidth()) >> 1; // center of (1, 1)
    yLocation = (int)(3 * model->getCellWidth()) >> 1; // center of (1, 1)
    setHeading(0);                                     // i->e-> due north
    setTravelSpeed(LOWER_TRAVELSPEED);                 // i->e-> no speed->
    rotationSpeedPerUpdate = ROTATION_SPEED * (UPDATE_DELAY / 1000.0);
    // Set any other parameters
    bumperPressed = false;
}

SimulatedRobot::~SimulatedRobot() {
}

// =======================================================================

int SimulatedRobot::getRobotBodySize() {
    return RADIUS_ROBOTBODY;
}

int SimulatedRobot::getSensorBodySize() {
    return RADIUS_SENSORBODY;
}

/* =========================================================================
 * Pose Methods
 * =========================================================================
 */

/**
 * Pose: obtains the current position of the robot
 * The heading is in the nearest degree, such that
 * 0 is along the y axis, and increasing
 * values rotate in a clockwise direction
 * @return the heading
 */
int SimulatedRobot::getHeading() {
    return heading;
}

/**
 * Pose: obtains the current position of the robot
 * The headingInRadians is in radians, such that
 * 0 is along the y axis, and increasing
 * values rotate in a clockwise direction
 * @return the heading in radians
 */
double SimulatedRobot::getHeadingInRadians() {
    return headingInRadians;
}

/**
 * Pose: sets the heading of the robot
 * Note that this is , but should be used to
 * ensure that headingInRadians is also updated.
 * @param heading direction (heading) of the robot in degrees
 */
void SimulatedRobot::setHeading(int heading) {
    this->heading = heading;

    headingInRadians = heading * DEGREES_TO_RADIANS;
}

/**
 * Pose: obtains the current position of the robot (in mm)
 * @return the x location on the map
 */
int SimulatedRobot::getX() {
    return xLocation;
}

/**
 * Pose: obtains the current position of the robot (in mm)
 * @return the y location on the map
 */
int SimulatedRobot::getY() {
    return yLocation;
}

/**
 * Pose: sets the current pose of the robot
 * @param travelSpeed the travelSpeed to set
 */
void SimulatedRobot::setPose(int x, int y, int heading) {
    // Note that we don't yet do bounds checking, and thus the robot could be misplaced.
    xLocation = x;
    yLocation = y;
    setHeading(heading);
}

/* =========================================================================
 * Locomotion Methods
 * =========================================================================
 */

/**
 * Locomotion: sets the current speed of the robot and
 * (implicitly) the rotation speed.
 * @param travelSpeed the travelSpeed to set
 * @return true if the travel speed could be successfully set
 */
bool SimulatedRobot::setTravelSpeed(int travelSpeed) {
    if (travelSpeed < LOWER_TRAVELSPEED || travelSpeed > UPPER_TRAVELSPEED) {
        std::cerr << "Invalid Travel Speed - setTravelSpeed(" << std::to_string(travelSpeed) << ")";
        return false;
    }
    this->travelSpeed = travelSpeed; // This is the speed per second (i.e. 1000 time units)
    travelSpeedPerUpdate = travelSpeed * ((double)UPDATE_DELAY / 1000.0);

    return true;
}

/**
 * Locomotion: gets the current speed of the robot
 * @return the travelSpeed
 */
int SimulatedRobot::getTravelSpeed() {
    return travelSpeed;
}

/**
 * Travel a defined distance then stop
 * @param distance the distance to travel in mm
 */
void SimulatedRobot::travel() {
    currentDistanceToDestination += model->getCellWidth();
}

/**
 * Return true if there is no further to travel
 */
bool SimulatedRobot::isAtDestination() {
    return currentDistanceToDestination == 0;
}

/**
 * Travel a defined distance then stop
 * @param distance the distance to travel in mm
 */

void SimulatedRobot::rotate(int degrees) {
    currentAngleToNewHeading += degrees;
}

/**
 * Return true if there is no further to travel
 */

bool SimulatedRobot::isAtRotation() {
    return currentAngleToNewHeading == 0;
}

/* =========================================================================
 * Sensor Methods
 * =========================================================================
 */

/**
 * Check if the bumper is pressed.  This will be true if the robot
 * tried to move into an obstacle.  If the robot moves away from an
 * obstacle successfully, then the value is false.
 * @return bumper status
 */

bool SimulatedRobot::isBumperPressed() {
    return bumperPressed;
}

/**
 * Returns the color of the current cell where the center of the robot
 * is.  If the cell is empty, then the  returned is WHITE.  Currently
 * the sensor will detect Color.RED, Color.GREEN and Color.BLUE
 * @return Color
 */

SDL_Color SimulatedRobot::getCSenseColor() {
    using namespace sdlcolours::colour;

    int colPos = getX() / model->getCellWidth();
    int rowPos = getY() / model->getCellWidth();

    switch (model->getOccupancy(colPos, rowPos)) {
    case OccupancyType::RED:
        return RED;
    case OccupancyType::GREEN:
        return GREEN;
    case OccupancyType::BLUE:
        return BLUE;

    case OccupancyType::EMPTY:
    case OccupancyType::OBSTACLE:
    case OccupancyType::ROBOT:
    case OccupancyType::UNKNOWN:
    default:
        break; // do nothing
    }

    return WHITE;
}

// =====================================================================
// setDirection()
// getDirection()

bool SimulatedRobot::setDirection(int degrees) {
    if (degrees < LEFT_SENSORANGLE || degrees > RIGHT_SENSORANGLE) {
        std::cerr << "Invalid Sensor Angle - setDirection(" << std::to_string(degrees) << ")";
        return false;
    }
    sensorDirection = degrees;
    return true;
}

int SimulatedRobot::getDirection() {
    // NOTE that this returns the current sensor angle, not the desired angle
    return currentSensorAngle;
}

/**
 * float getUSenseRange();					// Return distance to nearest object
 */
int SimulatedRobot::getUSenseRange() {
    double sensorH = (getDirection() + getHeading()) * DEGREES_TO_RADIANS; // sensor direction in radians

    int range;
    for (range = 0; range < US_SENSOR_MAX_RANGE; range += 10) {
        int xDelta = round(sin(sensorH) * range);
        int yDelta = round(cos(sensorH) * range);
        if (isColliding(xLocation, yLocation, xDelta, yDelta)) {
            break;
        }
    }
    return range;
}

// =======================================================================
// Obstacle Detection
bool SimulatedRobot::isColliding(int xPos, int yPos, int xDelta, int yDelta) {
    int xBound;                    // boundary of the robot in the x axis (either +ve or -ve)
    int yBound;                    // boundary of the robot in the y axis (either +ve or -ve)
    bool collision = false;        // collision flag
    int w = model->getCellWidth(); // cell width
    int h = model->getCellWidth(); // cell height

    if (xDelta >= 0) {
        xBound = xPos + xDelta + RADIUS_ROBOTBODY;     // check on the right most part of the robot
        if (yDelta >= 0) {                             // check up / up right / right
            yBound = yPos + yDelta + RADIUS_ROBOTBODY; // check on the bottom most part of the robot
        } else {                                       // check right /right down / down
            yBound = yPos + yDelta - RADIUS_ROBOTBODY; // check on the top most part of the robot
        }
    } else {
        xBound = xPos + xDelta - RADIUS_ROBOTBODY;     // check on the left most part of the robot
        if (yDelta >= 0) {                             // check up / up left / left
            yBound = yPos + yDelta + RADIUS_ROBOTBODY; // check on the bottom most part of the robot
        } else {                                       // check left / down left / down
            yBound = yPos + yDelta - RADIUS_ROBOTBODY; // check on the top most part of the robot
        }
    }
    if (
        model->getOccupancy(xBound / w, yPos / h) == OccupancyType::OBSTACLE ||   // Check left/right - x axis only
        model->getOccupancy(xBound / w, yBound / h) == OccupancyType::OBSTACLE || // Check diagonal - x/y axis
        model->getOccupancy(xPos / w, yBound / h) == OccupancyType::OBSTACLE      // Check up/down - y axis only
    ) {
        collision = true;
    }
    // We don't check for out of bounds (i.e. hitting the wall...)!
    // This is currently managed by assuming obstacles around the arena

    return collision;
}

// =======================================================================
// Robot Update

/**
 * Updates the position of the robot in response to any locomotion instruction
 * Currently assumes that the update delay is 1 second, to simplify speed calculations
 */

void SimulatedRobot::run(bool *alive) {
    while (*alive) {
        int deltaDist = 0;                                   // Represents the distance to travel
        int deltaRotation = 0;                               // Represents the rotation distance to rotate
        int travelSegment = round(travelSpeedPerUpdate);     // We track movement in ints!
        int rotationSegment = round(rotationSpeedPerUpdate); // We track movement in ints!

        // Are we moving?
        if (currentDistanceToDestination != 0) {
            if (currentDistanceToDestination < 0) {
                // Need to invert travelSegment to go in the right direction
                travelSegment = -travelSegment;
            }

            if (abs(currentDistanceToDestination) < abs(travelSegment)) {
                // We have less than the travelSpeed to the destination, so just move to destination
                deltaDist = currentDistanceToDestination;
                // No more destination to go
                currentDistanceToDestination = 0;
            } else {
                deltaDist = travelSegment;
                currentDistanceToDestination -= travelSegment;
            }

            // Need to check if we are about to run into an obstacle
            int xDelta = round(sin(headingInRadians) * deltaDist);
            int yDelta = round(cos(headingInRadians) * deltaDist);

            // Check for collisions
            // bumperPressed = isColliding(xLocation, yLocation, xDelta, yDelta);
            if (!bumperPressed) {
                // Update x & y position
                xLocation += xDelta;
                yLocation += yDelta;
            }
        }

        // =======================================================================
        // Are we also rotating?
        if (currentAngleToNewHeading != 0) {
            if (currentAngleToNewHeading < rotationSegment) {
                // Need to invert travelSegment to go in the right direction
                rotationSegment = -rotationSegment;
            }

            if (abs(currentAngleToNewHeading) < abs(rotationSegment)) {
                deltaRotation = currentAngleToNewHeading;
                currentAngleToNewHeading = 0;
            } else {
                deltaRotation = rotationSegment;
                currentAngleToNewHeading -= rotationSegment;
            }
            // Update Rotation
            setHeading(getHeading() + deltaRotation);
        }

        // =======================================================================
        // Is the sensor in the correct place
        if (currentSensorAngle != sensorDirection) {
            // we need to move it
            if (sensorDirection > currentSensorAngle) {
                if ((sensorDirection - currentSensorAngle) > rotationSegment) {
                    currentSensorAngle += rotationSegment;
                } else {
                    currentSensorAngle = sensorDirection;
                }
            } else {
                if ((currentSensorAngle - sensorDirection) > rotationSegment) {
                    currentSensorAngle -= rotationSegment;
                } else {
                    currentSensorAngle = sensorDirection;
                }
            }
        }

        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_DELAY));
        } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
        }
    }
}
