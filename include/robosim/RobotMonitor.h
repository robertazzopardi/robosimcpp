/**
 * @file RobotMonitor.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief This code is based on the RobotMonitor.java that is provided as part
 * of the source for the COMP329 lectures on Threads & Multi-tasking in Robots.
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __ROBOT_MONITOR_H__
#define __ROBOT_MONITOR_H__

#include <memory>

namespace colour {
struct Colour;
} // namespace colour

namespace robosim {

class RobotMonitor {
  private:
    // stored as void, purely to hide simulated robot
    std::shared_ptr<void> robot;

    // int delay;
    bool verbose;

    template <typename fn> void wait(fn);

    int getTravelSpeed();

  public:
    RobotMonitor(bool);
    ~RobotMonitor();

    void setArenaModel(std::shared_ptr<void>);

    void *getRobot();

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
    int getX();

    /**
     * Obtains the current position of the robot in the y axis (in mm)
     * @return the y location on the map
     */
    int getY();

    /**
     * Obtains the current position of the robot
     * The heading is in the nearest degree, such that
     * 0 is along the y axis, and increasing
     * values rotate in a clockwise direction
     * @return the heading
     */
    int getHeading();

    /**
     * Check if the bumper is pressed.  This will be true if the robot
     * tried to move into an obstacle.  If the robot moves away from an
     * obstacle successfully, then the value is false.
     * @return bumper status
     */
    bool isBumperPressed();

    /**
     * Check the  of the cell beneath the center of the robot.
     * @return a Color object
     */
    colour::Colour getCSenseColor();

    /**
     * Get the range of the nearest object in the direction of the sensor.
     * This senses an object that would result in a collision with the robot
     * if it were to move in the direction of the sensor (i.e. the cone of the
     * sensor is parallel with a width being that of the robot diameter).
     * The maximum range is 2550mm.
     * @return range to nearest object in the direction of the sensor in mm
     */
    int getUSenseRange();

    /**
     * Set the desired direction of the sensor, as an offset of the heading
     * of the robot.  Valid ranges are -90 (i.e. looking left) to 90 (i.e.
     * looking right). Any angle between these ranges can be selected, and the
     * method will block until the sensor is pointing in this direction.
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
    int getDirection();

    /**
     * If this method is not overridden then the monitor writes various bits of
     * robot state to the screen, then sleeps.
     */
    virtual void run(bool *);

    void debug();
};

} // namespace robosim

#endif // !__ROBOT_MONITOR_H__
