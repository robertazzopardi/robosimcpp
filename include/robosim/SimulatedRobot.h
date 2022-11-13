#pragma once

#include "Colour.h"
#include <SDL2/SDL_rect.h>
#include <SDL2/SDL_stdinc.h>

namespace simulatedrobot
{

  struct Circle
  {
    Sint16 x;
    Sint16 y;
    Sint16 r;
  };

  struct RobotRender
  {
    Circle body;
    Circle sensor;
    SDL_FPoint radius;
    colour::Colour bodyColour;
  };

  class SimulatedRobot
  {
  private:
    int travelSpeed = 0;                  // mm per second.
    double travelSpeedPerUpdate = 0;      // mm per update.
    double rotationSpeedPerUpdate = 0;    // Rotation should be
                                          // ROTATION_SPEED_COEFFICIENT *
                                          // travelSpeedPerUpdate
    int currentDistanceToDestination = 0; // in mm
    int currentAngleToNewHeading = 0;     // rounded to nearest degree in range
                                          // [-360..360]
    double headingInRadians = 0;          // pose angle from origin in radians, looking
                                          // along y axis
    int xLocation = 0;                    // x location in current coordinate system (bottom left
                                          // is 0,0)
    int yLocation = 0;                    // y location in current coordinate system (bottom left
                                          // is 0,0)
    int heading = 0;                      // angle from origin in degrees, looking along y axis
    bool bumperPressed = 0;               // True if the robot last tried to move into an
                                          // obstacle, false otherwise
    int sensorDirection = 0;              // Angle (in degrees) from the robot heading
                                          // [-90..+90]
    int currentSensorAngle = 0;           // rounded to nearest degree in range
                                          // [-90..+90]

    bool isColliding(int, int, int, int);

    // std::unique_ptr<RobotRender> robotRender;
    RobotRender robotRender;

    /**
     * Pose: sets the heading of the robot
     * Note that this is , but should be used to
     * ensure that headingInRadians is also updated.
     * @param heading direction (heading) of the robot in degrees
     */
    void setHeading(int);

  public:
    // bool shouldStop = false;

    // SimulatedRobot(arenamodel::ArenaModel *);
    SimulatedRobot();
    SimulatedRobot(bool, colour::Colour);
    ~SimulatedRobot();

    uint32_t getRobotBodySize();
    uint32_t getSensorBodySize();

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
     * Get the direction the sensor is facing in degrees
     */
    int getDirection();

    /**
     * Get the direction the sensor is facing in radians
     */
    double getDirectionInRadians();

    /**
     * Turn Robot by the specified degrees
     */
    bool setDirection(int);

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
    colour::Colour getCSenseColor();

    /**
     * float getUSenseRange();
     * Return distance to nearest object
     */
    int getUSenseRange();

    // Update the Robot
    void update();

    // RobotRender *getRenderObject();
    RobotRender getRenderObject();

    /**
     * Updates the position of the robot in response to any locomotion
     * instruction Currently assumes that the update delay is 1 second, to
     * simplify speed calculations
     */
    void run();
  };

}