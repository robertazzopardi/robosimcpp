/**
 * @file SimulatedRobot.cpp
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief
 * @version 0.1
 * @date 2021-07-07
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "SimulatedRobot.h"
#include "ArenaModel.h"
#include "ArenaModelView.h"
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

// Java method of converting degrees to radians. Multiply degrees by this
static constexpr auto DEGREES_TO_RADIANS = 0.017453292519943295;

// Physical Characteristics of the Robot
static constexpr auto LEFT_SENSORANGLE =
    -90; // Leftmost angle that the sensor can be set
static constexpr auto RIGHT_SENSORANGLE =
    90; // Rightmost angle that the sensor can be set
static constexpr auto LOWER_TRAVELSPEED =
    10; // Min speed of robot in mm per second
static constexpr auto UPPER_TRAVELSPEED =
    100; // Max speed of robot in mm per second
static constexpr auto RADIUS_ROBOTBODY =
    100; // The robot is modeled as a circle with this radius in mm.
static constexpr auto RADIUS_SENSORBODY =
    20; // The robot sensor modeled as a circle with this radius in mm.
static constexpr auto UPDATE_DELAY =
    100; // Update Delay is 0.1 second, to simplify update calculations
static constexpr auto ROTATION_SPEED =
    50; // rotation speed should be fixed & independent of travel speed
static constexpr auto US_SENSOR_MAX_RANGE = 2550; // Max range in mm
static constexpr auto UPDATE_RATE = UPDATE_DELAY / 1000.0;

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using mygridcell::OccupancyType;
using simulatedrobot::SimulatedRobot;

SimulatedRobot::SimulatedRobot(ArenaModel *model) : attributes{} {
    this->model = model;

    auto center = static_cast<int>((3 * model->getCellWidth()) / 2);
    // Position the robot in the center of the (1,1) cell
    attributes.xLocation = center;     // center of (1, 1)
    attributes.yLocation = center;     // center of (1, 1)
    setTravelSpeed(LOWER_TRAVELSPEED); // i->e-> no speed->
    setHeading(0);                     // i->e-> due north
    // attributes.rotationSpeedPerUpdate = ROTATION_SPEED * UPDATE_RATE;
    attributes.rotationSpeedPerUpdate =
        static_cast<double>(ROTATION_SPEED / 10);
    // Set any other parameters
}

SimulatedRobot::~SimulatedRobot() {}

// =======================================================================

auto SimulatedRobot::getRobotBodySize() { return RADIUS_ROBOTBODY; }

int SimulatedRobot::getSensorBodySize() { return RADIUS_SENSORBODY; }

/* =========================================================================
 * Pose Methods
 * =========================================================================
 */

int SimulatedRobot::getHeading() { return attributes.heading; }

double SimulatedRobot::getHeadingInRadians() {
    return attributes.headingInRadians;
}

void SimulatedRobot::setHeading(int heading) {
    attributes.heading = heading;

    attributes.headingInRadians = heading * DEGREES_TO_RADIANS;
}

int SimulatedRobot::getX() { return attributes.xLocation; }

int SimulatedRobot::getY() { return attributes.yLocation; }

void SimulatedRobot::setPose(int x, int y, int heading) {
    // Note that we don't yet do bounds checking, and thus the robot could be
    // misplaced.
    attributes.xLocation = x;
    attributes.yLocation = y;
    setHeading(heading);
}

/* =========================================================================
 * Locomotion Methods
 * =========================================================================
 */

bool SimulatedRobot::setTravelSpeed(int travelSpeed) {
    if (travelSpeed < LOWER_TRAVELSPEED || travelSpeed > UPPER_TRAVELSPEED) {
        std::cerr << "Invalid Travel Speed - setTravelSpeed("
                  << std::to_string(travelSpeed) << ")";
        return false;
    }
    attributes.travelSpeed =
        travelSpeed; // This is the speed per second (i.e. 1000 time units)
    attributes.travelSpeedPerUpdate = travelSpeed * UPDATE_RATE;

    return true;
}

int SimulatedRobot::getTravelSpeed() { return attributes.travelSpeed; }

void SimulatedRobot::travel() {
    attributes.currentDistanceToDestination += model->getCellWidth();
}

bool SimulatedRobot::isAtDestination() {
    return attributes.currentDistanceToDestination == 0;
}

void SimulatedRobot::rotate(int degrees) {
    attributes.currentAngleToNewHeading += degrees;
}

bool SimulatedRobot::isAtRotation() {
    return attributes.currentAngleToNewHeading == 0;
}

/* =========================================================================
 * Sensor Methods
 * =========================================================================
 */

bool SimulatedRobot::isBumperPressed() { return attributes.bumperPressed; }

SDL_Color SimulatedRobot::getCSenseColor() {
    auto colPos = getX() / model->getCellWidth();
    auto rowPos = getY() / model->getCellWidth();

    switch (model->getOccupancy(colPos, rowPos)) {
    case OccupancyType::RED:
        return sdlcolours::RED;
    case OccupancyType::GREEN:
        return sdlcolours::GREEN;
    case OccupancyType::BLUE:
        return sdlcolours::BLUE;
    case OccupancyType::EMPTY:
    case OccupancyType::OBSTACLE:
    case OccupancyType::ROBOT:
    case OccupancyType::UNKNOWN:
    default:
        break; // do nothing
    }

    return sdlcolours::WHITE;
}

// =====================================================================
// setDirection()
// getDirection()

bool SimulatedRobot::setDirection(int degrees) {
    if (degrees < LEFT_SENSORANGLE || degrees > RIGHT_SENSORANGLE) {
        std::cerr << "Invalid Sensor Angle - setDirection("
                  << std::to_string(degrees) << ")";
        return false;
    }
    attributes.sensorDirection = degrees;
    return true;
}

int SimulatedRobot::getDirection() {
    // NOTE that this returns the current sensor angle, not the desired angle
    return attributes.currentSensorAngle;
}

int SimulatedRobot::getUSenseRange() {
    auto sensorH = (getDirection() + getHeading()) *
                   DEGREES_TO_RADIANS; // sensor direction in radians

    int range;
    for (range = 0; range < US_SENSOR_MAX_RANGE; range += 10) {
        auto xDelta = round(sin(sensorH) * range);
        auto yDelta = round(cos(sensorH) * range);
        if (isColliding(attributes.xLocation, attributes.yLocation, xDelta,
                        yDelta)) {
            break;
        }
    }

    return range;
}

// =======================================================================
// Obstacle Detection

bool SimulatedRobot::isColliding(int xPos, int yPos, int xDelta, int yDelta) {
    int xBound; // boundary of the robot in the x axis (either +ve or -ve)
    int yBound; // boundary of the robot in the y axis (either +ve or -ve)
    auto collision = false;         // collision flag
    auto w = model->getCellWidth(); // cell width
    auto h = model->getCellWidth(); // cell height

    if (xDelta >= 0) {
        xBound = xPos + xDelta +
                 RADIUS_ROBOTBODY; // check on the right most part of the robot
    } else {
        xBound = xPos + xDelta -
                 RADIUS_ROBOTBODY; // check on the left most part of the robot
    }
    if (yDelta >= 0) { // check up / up left / left
        yBound = yPos + yDelta +
                 RADIUS_ROBOTBODY; // check on the bottom most part of the robot
    } else {                       // check left / down left / down
        yBound = yPos + yDelta -
                 RADIUS_ROBOTBODY; // check on the top most part of the robot
    }

    if (model->getOccupancy(xBound / w, yPos / h) ==
            OccupancyType::OBSTACLE || // Check left/right - x axis only
        model->getOccupancy(xBound / w, yBound / h) ==
            OccupancyType::OBSTACLE || // Check diagonal - x/y axis
        model->getOccupancy(xPos / w, yBound / h) ==
            OccupancyType::OBSTACLE // Check up/down - y axis only
    ) {
        collision = true;
    }
    // We don't check for out of bounds (i.e. hitting the wall...)!
    // This is currently managed by assuming obstacles around the arena

    return collision;
}

// =======================================================================
// Robot Update

void SimulatedRobot::run(arenamodelview::ArenaModelView *view) {
    // while (view->running) {
    while (ArenaModelView::running) {
        auto deltaDist = 0;     // Represents the distance to travel
        auto deltaRotation = 0; // Represents the rotation distance to rotate
        auto travelSegment = round(
            attributes.travelSpeedPerUpdate); // We track movement in ints!
        auto rotationSegment = round(
            attributes.rotationSpeedPerUpdate); // We track movement in ints!

        // Are we moving?

        if (attributes.currentDistanceToDestination != 0) {
            if (attributes.currentDistanceToDestination < 0) {
                // Need to invert travelSegment to go in the right direction
                travelSegment = -travelSegment;
            }

            if (abs(attributes.currentDistanceToDestination) <
                abs(travelSegment)) {
                // We have less than the travelSpeed to the destination, so just
                // move to destination
                deltaDist = attributes.currentDistanceToDestination;
                // No more destination to go
                attributes.currentDistanceToDestination = 0;
            } else {
                deltaDist = travelSegment;
                attributes.currentDistanceToDestination -= travelSegment;
            }
            // Need to check if we are about to run into an obstacle
            auto xDelta = round(sin(attributes.headingInRadians) * deltaDist);
            auto yDelta = round(cos(attributes.headingInRadians) * deltaDist);

            // Check for collisions
            // attributes.bumperPressed = isColliding(attributes.xLocation,
            // attributes.yLocation, xDelta, yDelta);
            if (!attributes.bumperPressed) {
                // Update x & y position
                attributes.xLocation += xDelta;
                attributes.yLocation += yDelta;
            }
        }

        // =======================================================================
        // Are we also rotating?
        if (attributes.currentAngleToNewHeading != 0) {
            if (attributes.currentAngleToNewHeading < rotationSegment) {
                // Need to invert travelSegment to go in the right direction
                rotationSegment = -rotationSegment;
            }

            if (abs(attributes.currentAngleToNewHeading) <
                abs(rotationSegment)) {
                deltaRotation = attributes.currentAngleToNewHeading;
                attributes.currentAngleToNewHeading = 0;
            } else {
                deltaRotation = rotationSegment;
                attributes.currentAngleToNewHeading -= rotationSegment;
            }
            // Update Rotation
            setHeading(getHeading() + deltaRotation);
        }

        // =======================================================================
        // Is the sensor in the correct place
        if (attributes.currentSensorAngle != attributes.sensorDirection) {
            // we need to move it
            if (attributes.sensorDirection > attributes.currentSensorAngle) {
                if ((attributes.sensorDirection -
                     attributes.currentSensorAngle) > rotationSegment) {
                    attributes.currentSensorAngle += rotationSegment;
                } else {
                    attributes.currentSensorAngle = attributes.sensorDirection;
                }
            } else {
                if ((attributes.currentSensorAngle -
                     attributes.sensorDirection) > rotationSegment) {
                    attributes.currentSensorAngle -= rotationSegment;
                } else {
                    attributes.currentSensorAngle = attributes.sensorDirection;
                }
            }
        }

        view->update();

        try {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(UPDATE_DELAY));
        } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
        }
    }
}
