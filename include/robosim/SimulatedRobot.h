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

typedef struct SDL_Color SDL_Color;

namespace arenamodel {
class ArenaModel;
} // namespace arenamodel

namespace simulatedrobot {

class SimulatedRobot {
  private:
    arenamodel::ArenaModel *model; // Model of the Arena

    // Physical Characteristics of the Robot
    static const int LEFT_SENSORANGLE = -90; // Leftmost angle that the sensor can be set
    static const int RIGHT_SENSORANGLE = 90; // Rightmost angle that the sensor can be set
    static const int LOWER_TRAVELSPEED = 10;
    static const int UPPER_TRAVELSPEED = 100;    // Max speed of robot in mm per second
    static const int RADIUS_ROBOTBODY = 100;     // The robot is modeled as a circle with this radius in mm.
    static const int RADIUS_SENSORBODY = 20;     // The robot sensor modeled as a circle with this radius in mm.
    static const int UPDATE_DELAY = 100;         // Update Delay is 0.1 second, to simplify update calculations
    static const int ROTATION_SPEED = 50;        // rotation speed should be fixed & independent of travel speed
    static const int US_SENSOR_MAX_RANGE = 2550; // Max range in mm

    // The instance parameters for the characteristics of our simulated robot
    // Locomotion
    int travelSpeed = 0;                  // mm per second.
    double travelSpeedPerUpdate = 0;      // mm per update.
    double rotationSpeedPerUpdate = 0;    // Rotation should be ROTATION_SPEED_COEFFICIENT * travelSpeedPerUpdate
    int currentDistanceToDestination = 0; // in mm
    int currentAngleToNewHeading = 0;     // rounded to nearest degree in range [-360..360]
    double headingInRadians = 0;          // pose angle from origin in radians, looking along y axis

    // Pose
    int xLocation = 0; // x location in current coordinate system (bottom left is 0,0)
    int yLocation = 0; // y location in current coordinate system (bottom left is 0,0)
    int heading = 0;   // angle from origin in degrees, looking along y axis

    // Sensors - Bumper Sensor
    bool bumperPressed; // True if the robot last tried to move into an obstacle, false otherwise

    // Sensors - UltraSound
    int sensorDirection = 0;    // Angle (in degrees) from the robot heading [-90..+90]
    int currentSensorAngle = 0; // rounded to nearest degree in range [-90..+90]

    bool isColliding(int, int, int, int);
    void setHeading(int);

    // bool running = true;

  public:
    SimulatedRobot(arenamodel::ArenaModel *);
    ~SimulatedRobot();

    int getRobotBodySize();
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
