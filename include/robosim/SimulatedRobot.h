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

#ifndef __SIMULATED_ROBOT__
#define __SIMULATED_ROBOT__

struct SDL_Color;

namespace arenamodel {
class ArenaModel;
} // namespace arenamodel

namespace simulatedrobot {

class SimulatedRobot {
  private:
    arenamodel::ArenaModel *model; // Model of the Arena

    // The instance parameters for the characteristics of our simulated robot
    struct Instance {
        int travelSpeed;                  // mm per second.
        double travelSpeedPerUpdate;      // mm per update.
        double rotationSpeedPerUpdate;    // Rotation should be ROTATION_SPEED_COEFFICIENT * travelSpeedPerUpdate
        int currentDistanceToDestination; // in mm
        int currentAngleToNewHeading;     // rounded to nearest degree in range [-360..360]
        double headingInRadians;          // pose angle from origin in radians, looking along y axis
        int xLocation;                    // x location in current coordinate system (bottom left is 0,0)
        int yLocation;                    // y location in current coordinate system (bottom left is 0,0)
        int heading;                      // angle from origin in degrees, looking along y axis
        bool bumperPressed;               // True if the robot last tried to move into an obstacle, false otherwise
        int sensorDirection;              // Angle (in degrees) from the robot heading [-90..+90]
        int currentSensorAngle;           // rounded to nearest degree in range [-90..+90]
    } instance;

    bool isColliding(int, int, int, int);
    void setHeading(int);

  public:
    SimulatedRobot(arenamodel::ArenaModel *);
    ~SimulatedRobot();

    auto getRobotBodySize();
    int getSensorBodySize();
    int getHeading();
    double getHeadingInRadians();
    int getX();
    int getY();
    void setPose(int, int, int);
    bool setTravelSpeed(int);
    int getTravelSpeed();
    void travel();
    bool isAtDestination();
    void rotate(int);
    bool isAtRotation();
    bool isBumperPressed();
    SDL_Color getCSenseColor();
    bool setDirection(int);
    int getDirection();
    int getUSenseRange();
    void run(bool *);
};

} // namespace simulatedrobot

#endif // !__SIMULATED_ROBOT__
