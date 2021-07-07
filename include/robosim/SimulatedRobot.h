/**
 * @file SimulatedRobot.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief Definition of a simulated robot
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __SIMULATED_ROBOT_H__
#define __SIMULATED_ROBOT_H__

struct SDL_Color;

namespace arenamodel {
class ArenaModel;
} // namespace arenamodel

namespace arenamodelview {
class ArenaModelView;
}

namespace simulatedrobot {

class SimulatedRobot {
  private:
    arenamodel::ArenaModel *model; // Model of the Arena

    // The instance parameters for the characteristics of our simulated robot
    struct Attributes {
        int travelSpeed;                  // mm per second.
        double travelSpeedPerUpdate;      // mm per update.
        double rotationSpeedPerUpdate;    // Rotation should be
                                          // ROTATION_SPEED_COEFFICIENT *
                                          // travelSpeedPerUpdate
        int currentDistanceToDestination; // in mm
        int currentAngleToNewHeading;     // rounded to nearest degree in range
                                          // [-360..360]
        double headingInRadians; // pose angle from origin in radians, looking
                                 // along y axis
        int xLocation; // x location in current coordinate system (bottom left
                       // is 0,0)
        int yLocation; // y location in current coordinate system (bottom left
                       // is 0,0)
        int heading;   // angle from origin in degrees, looking along y axis
        bool bumperPressed;     // True if the robot last tried to move into an
                                // obstacle, false otherwise
        int sensorDirection;    // Angle (in degrees) from the robot heading
                                // [-90..+90]
        int currentSensorAngle; // rounded to nearest degree in range [-90..+90]
    } attributes;

    bool isColliding(int, int, int, int);

    /**
     * Pose: sets the heading of the robot
     * Note that this is , but should be used to
     * ensure that headingInRadians is also updated.
     * @param heading direction (heading) of the robot in degrees
     */
    void setHeading(int);

  public:
    // bool shouldStop = false;

    SimulatedRobot(arenamodel::ArenaModel *);
    ~SimulatedRobot();

    auto getRobotBodySize();
    int getSensorBodySize();

    /**
     * Pose: obtains the current position of the robot
     * The heading is in the nearest degree, such that
     * 0 is along the y axis, and increasing
     * values rotate in a clockwise direction
     * @return the heading
     */
    int getHeading();

    /**
     * Pose: obtains the current position of the robot
     * The headingInRadians is in radians, such that
     * 0 is along the y axis, and increasing
     * values rotate in a clockwise direction
     * @return the heading in radians
     */
    double getHeadingInRadians();

    /**
     * Pose: obtains the current position of the robot (in mm)
     * @return the x location on the map
     */
    int getX();

    /**
     * Pose: obtains the current position of the robot (in mm)
     * @return the y location on the map
     */
    int getY();

    /**
     * Pose: sets the current pose of the robot
     * @param travelSpeed the travelSpeed to set
     */
    void setPose(int, int, int);

    /**
     * Locomotion: sets the current speed of the robot and
     * (implicitly) the rotation speed.
     * @param travelSpeed the travelSpeed to set
     * @return true if the travel speed could be successfully set
     */
    bool setTravelSpeed(int);

    /**
     * Locomotion: gets the current speed of the robot
     * @return the travelSpeed
     */
    int getTravelSpeed();

    /**
     * Travel a defined distance then stop
     * @param distance the distance to travel in mm
     */
    void travel();

    /**
     * Return true if there is no further to travel
     */
    bool isAtDestination();

    /**
     * Travel a defined distance then stop
     * @param distance the distance to travel in mm
     */
    void rotate(int);

    /**
     * Return true if there is no further to travel
     */
    bool isAtRotation();

    /**
     * Check if the bumper is pressed.  This will be true if the robot
     * tried to move into an obstacle.  If the robot moves away from an
     * obstacle successfully, then the value is false.
     * @return bumper status
     */
    bool isBumperPressed();

    /**
     * Returns the color of the current cell where the center of the robot
     * is.  If the cell is empty, then the  returned is WHITE.  Currently
     * the sensor will detect Color.RED, Color.GREEN and Color.BLUE
     * @return Color
     */
    SDL_Color getCSenseColor();

    bool setDirection(int);
    int getDirection();

    /**
     * float getUSenseRange();
     * Return distance to nearest object
     */
    int getUSenseRange();

    /**
     * Updates the position of the robot in response to any locomotion
     * instruction Currently assumes that the update delay is 1 second, to
     * simplify speed calculations
     */
    void run(arenamodelview::ArenaModelView *);
};

} // namespace simulatedrobot

#endif // !__SIMULATED_ROBOT_H__