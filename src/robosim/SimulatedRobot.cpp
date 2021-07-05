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

static constexpr auto DEGREES_TO_RADIANS = 0.017453292519943295;

// Physical Characteristics of the Robot
static constexpr auto LEFT_SENSORANGLE = -90;     // Leftmost angle that the sensor can be set
static constexpr auto RIGHT_SENSORANGLE = 90;     // Rightmost angle that the sensor can be set
static constexpr auto LOWER_TRAVELSPEED = 10;     // Min speed of robot in mm per second
static constexpr auto UPPER_TRAVELSPEED = 100;    // Max speed of robot in mm per second
static constexpr auto RADIUS_ROBOTBODY = 100;     // The robot is modeled as a circle with this radius in mm.
static constexpr auto RADIUS_SENSORBODY = 20;     // The robot sensor modeled as a circle with this radius in mm.
static constexpr auto UPDATE_DELAY = 100;         // Update Delay is 0.1 second, to simplify update calculations
static constexpr auto ROTATION_SPEED = 50;        // rotation speed should be fixed & independent of travel speed
static constexpr auto US_SENSOR_MAX_RANGE = 2550; // Max range in mm
static constexpr auto UPDATE_RATE = UPDATE_DELAY / 1000.0;

using arenamodel::ArenaModel;
using mygridcell::OccupancyType;
using simulatedrobot::SimulatedRobot;

SimulatedRobot::SimulatedRobot(ArenaModel *model) : instance{} {
    this->model = model;

    auto center = (int)(3 * model->getCellWidth()) >> 1;
    // Position the robot in the center of the (1,1) cell
    instance.xLocation = center;       // center of (1, 1)
    instance.yLocation = center;       // center of (1, 1)
    setTravelSpeed(LOWER_TRAVELSPEED); // i->e-> no speed->
    setHeading(0);                     // i->e-> due north
    instance.rotationSpeedPerUpdate = ROTATION_SPEED * UPDATE_RATE;
    // Set any other parameters
}

SimulatedRobot::~SimulatedRobot() {
}

// =======================================================================

auto SimulatedRobot::getRobotBodySize() {
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
    return instance.heading;
}

/**
 * Pose: obtains the current position of the robot
 * The headingInRadians is in radians, such that
 * 0 is along the y axis, and increasing
 * values rotate in a clockwise direction
 * @return the heading in radians
 */
double SimulatedRobot::getHeadingInRadians() {
    return instance.headingInRadians;
}

/**
 * Pose: sets the heading of the robot
 * Note that this is , but should be used to
 * ensure that headingInRadians is also updated.
 * @param heading direction (heading) of the robot in degrees
 */
void SimulatedRobot::setHeading(int heading) {
    // std::cout << heading << std::endl;
    instance.heading = heading;

    instance.headingInRadians = heading * DEGREES_TO_RADIANS;
}

/**
 * Pose: obtains the current position of the robot (in mm)
 * @return the x location on the map
 */
int SimulatedRobot::getX() {
    return instance.xLocation;
}

/**
 * Pose: obtains the current position of the robot (in mm)
 * @return the y location on the map
 */
int SimulatedRobot::getY() {
    return instance.yLocation;
}

/**
 * Pose: sets the current pose of the robot
 * @param travelSpeed the travelSpeed to set
 */
void SimulatedRobot::setPose(int x, int y, int heading) {
    // Note that we don't yet do bounds checking, and thus the robot could be misplaced.
    instance.xLocation = x;
    instance.yLocation = y;
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
    instance.travelSpeed = travelSpeed; // This is the speed per second (i.e. 1000 time units)
    instance.travelSpeedPerUpdate = travelSpeed * UPDATE_RATE;

    return true;
}

/**
 * Locomotion: gets the current speed of the robot
 * @return the travelSpeed
 */
int SimulatedRobot::getTravelSpeed() {
    return instance.travelSpeed;
}

/**
 * Travel a defined distance then stop
 * @param distance the distance to travel in mm
 */
void SimulatedRobot::travel() {
    instance.currentDistanceToDestination += model->getCellWidth();
}

/**
 * Return true if there is no further to travel
 */
bool SimulatedRobot::isAtDestination() {
    return instance.currentDistanceToDestination == 0;
}

/**
 * Travel a defined distance then stop
 * @param distance the distance to travel in mm
 */

void SimulatedRobot::rotate(int degrees) {
    instance.currentAngleToNewHeading += degrees;
}

/**
 * Return true if there is no further to travel
 */
bool SimulatedRobot::isAtRotation() {
    return instance.currentAngleToNewHeading == 0;
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
    return instance.bumperPressed;
}

/**
 * Returns the color of the current cell where the center of the robot
 * is.  If the cell is empty, then the  returned is WHITE.  Currently
 * the sensor will detect Color.RED, Color.GREEN and Color.BLUE
 * @return Color
 */
SDL_Color SimulatedRobot::getCSenseColor() {
    using namespace sdlcolours;

    auto colPos = getX() / model->getCellWidth();
    auto rowPos = getY() / model->getCellWidth();

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
    instance.sensorDirection = degrees;
    return true;
}

int SimulatedRobot::getDirection() {
    // NOTE that this returns the current sensor angle, not the desired angle
    return instance.currentSensorAngle;
}

/**
 * float getUSenseRange();					// Return distance to nearest object
 */
int SimulatedRobot::getUSenseRange() {
    auto sensorH = (getDirection() + getHeading()) * DEGREES_TO_RADIANS; // sensor direction in radians

    int range;
    for (range = 0; range < US_SENSOR_MAX_RANGE; range += 10) {
        auto xDelta = round(sin(sensorH) * range);
        auto yDelta = round(cos(sensorH) * range);
        if (isColliding(instance.xLocation, instance.yLocation, xDelta, yDelta)) {
            break;
        }
    }
    return range;
}

// =======================================================================
// Obstacle Detection
bool SimulatedRobot::isColliding(int xPos, int yPos, int xDelta, int yDelta) {
    int xBound;                     // boundary of the robot in the x axis (either +ve or -ve)
    int yBound;                     // boundary of the robot in the y axis (either +ve or -ve)
    auto collision = false;         // collision flag
    auto w = model->getCellWidth(); // cell width
    auto h = model->getCellWidth(); // cell height

    if (xDelta >= 0) {
        xBound = xPos + xDelta + RADIUS_ROBOTBODY; // check on the right most part of the robot
    } else {
        xBound = xPos + xDelta - RADIUS_ROBOTBODY; // check on the left most part of the robot
    }
    if (yDelta >= 0) {                             // check up / up left / left
        yBound = yPos + yDelta + RADIUS_ROBOTBODY; // check on the bottom most part of the robot
    } else {                                       // check left / down left / down
        yBound = yPos + yDelta - RADIUS_ROBOTBODY; // check on the top most part of the robot
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
    auto rotationSegment = round(instance.rotationSpeedPerUpdate); // We track movement in ints!
    while (*alive) {
        auto deltaDist = 0;                                        // Represents the distance to travel
        auto deltaRotation = 0;                                    // Represents the rotation distance to rotate
        auto travelSegment = round(instance.travelSpeedPerUpdate); // We track movement in ints!
        std::cout << rotationSegment << std::endl;

        // Are we moving?
        if (instance.currentDistanceToDestination != 0) {
            if (instance.currentDistanceToDestination < 0) {
                // Need to invert travelSegment to go in the right direction
                travelSegment = -travelSegment;
            }

            if (abs(instance.currentDistanceToDestination) < abs(travelSegment)) {
                // We have less than the travelSpeed to the destination, so just move to destination
                deltaDist = instance.currentDistanceToDestination;
                // No more destination to go
                instance.currentDistanceToDestination = 0;
            } else {
                deltaDist = travelSegment;
                instance.currentDistanceToDestination -= travelSegment;
            }

            // Need to check if we are about to run into an obstacle
            auto xDelta = round(sin(instance.headingInRadians) * deltaDist);
            auto yDelta = round(cos(instance.headingInRadians) * deltaDist);

            // Check for collisions
            // bumperPressed = isColliding(xLocation, yLocation, xDelta, yDelta);
            if (!instance.bumperPressed) {
                // Update x & y position
                instance.xLocation += xDelta;
                instance.yLocation += yDelta;
            }
        }

        // =======================================================================
        // Are we also rotating?
        if (instance.currentAngleToNewHeading != 0) {
            if (instance.currentAngleToNewHeading < rotationSegment) {
                // Need to invert travelSegment to go in the right direction
                rotationSegment = -rotationSegment;
            }

            if (abs(instance.currentAngleToNewHeading) < abs(rotationSegment)) {
                deltaRotation = instance.currentAngleToNewHeading;
                instance.currentAngleToNewHeading = 0;
            } else {
                deltaRotation = rotationSegment;
                instance.currentAngleToNewHeading -= rotationSegment;
            }
            // Update Rotation
            setHeading(getHeading() + deltaRotation);
        }

        // =======================================================================
        // Is the sensor in the correct place
        if (instance.currentSensorAngle != instance.sensorDirection) {
            // we need to move it
            if (instance.sensorDirection > instance.currentSensorAngle) {
                if (instance.sensorDirection - instance.currentSensorAngle > rotationSegment) {
                    instance.currentSensorAngle += rotationSegment;
                } else {
                    instance.currentSensorAngle = instance.sensorDirection;
                }
            } else {
                if (instance.currentSensorAngle - instance.sensorDirection > rotationSegment) {
                    instance.currentSensorAngle -= rotationSegment;
                } else {
                    instance.currentSensorAngle = instance.sensorDirection;
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
