#pragma once

#include "Colour.h"
#include <memory>

namespace simulatedrobot
{
class SimulatedRobot;
}

namespace robosim::robotmonitor
{

class RobotMonitor
{
  protected:
    bool *running;

  private:
    static uint8_t robotCount;

    bool verbose;
    int getTravelSpeed() const;

    Colour colour;
    std::shared_ptr<simulatedrobot::SimulatedRobot> robot;

  public:
    int serialNumber;

    void (*run_func)(RobotMonitor *);
    void callRunFunc();

    RobotMonitor();
    RobotMonitor(bool, Colour, bool *);

    virtual ~RobotMonitor();

    void setRobot(size_t);

    std::shared_ptr<simulatedrobot::SimulatedRobot> getRobot() const;

    /**
     * Update the robots pose
     */
    void setPose(int, int, int);

    /**
     * Sets the robots travel speed
     */
    bool setTravelSpeed(int);

    /**
     * The robot travels one cell forward in this current heading
     */
    void travel();

    /**
     * Rotates the robot by the specified degrees (in degrees)
     *
     * @param degrees the amount to rotate by
     */
    void rotate(int);

    /**
     * Obtains the current position of the robot in the x axis (in mm)
     * @return the x location on the map
     */
    int getX() const;

    /**
     * Obtains the current position of the robot in the y axis (in mm)
     * @return the y location on the map
     */
    int getY() const;

    /**
     * Obtains the current position of the robot
     * The heading is in the nearest degree, such that
     * 0 is along the y axis, and increasing
     * values rotate in a clockwise direction
     * @return the heading
     */
    int getHeading() const;

    /**
     * Check if the bumper is pressed.  This will be true if the robot
     * tried to move into an obstacle.  If the robot moves away from an
     * obstacle successfully, then the value is false.
     * @return bumper status
     */
    bool isBumperPressed() const;

    /**
     * Check the  of the cell beneath the center of the robot.
     * @return a Color object
     */
    Colour getCSenseColor() const;

    /**
     * Get the range of the nearest object in the direction of the sensor.
     * This senses an object that would result in a collision with the robot
     * if it were to move in the direction of the sensor (i.e. the cone of
     * the sensor is parallel with a width being that of the robot
     * diameter). The maximum range is 2550mm.
     * @return range to nearest object in the direction of the sensor in mm
     */
    int getUSenseRange() const;

    /**
     * Set the desired direction of the sensor, as an offset of the heading
     * of the robot.  Valid ranges are -90 (i.e. looking left) to 90 (i.e.
     * looking right). Any angle between these ranges can be selected, and
     * the method will block until the sensor is pointing in this direction.
     * @param angle (in degrees) from the heading of the robot
     */
    void setDirection(int);

    /**
     * Get the current direction (as an offset to the robot heading) of
     * the sensor.
     * @return the angle of the sensor, where 0 is in the direction of the
     * robot; and any value in the range -90 (i.e. looking left) to 90 (i.e.
     * looking right)
     */
    int getDirection() const;

    /**
     * Logs the robots current observations and properties
     */
    void debug() const;

    /**
     * If this method is not overridden then the monitor writes various bits
     * of robot state to the screen, then sleeps.
     *
     * @param running Pointer to whether the simulation is still running
     */
    virtual void run();

    /*
     * Get the robots X position on the Grid
     */
    int getGridX() const;

    /*
     * Get the robots Y position on the Grid
     */
    int getGridY() const;
};

} // namespace robosim::robotmonitor
