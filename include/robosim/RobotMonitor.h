/**
 * @file RobotMonitor.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief This code is based on the RobotMonitor.java that is provided as part of the source for the COMP329 lectures on Threads & Multi-tasking in Robots.
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __ROBOT_MONITOR__
#define __ROBOT_MONITOR__

typedef struct SDL_Color SDL_Color;

namespace robosim {

class RobotMonitor {
  private:
    // stored as void, purely to hide simulated robot
    void *robot;
    int delay;
    bool verbose;

  public:
    RobotMonitor(int, bool);
    ~RobotMonitor();

    void setArenaModel(void *);
    void *getRobot();

    bool setTravelSpeed(int);
    void travel();
    void rotate(int);
    int getX();
    int getY();
    int getHeading();
    bool isBumperPressed();
    SDL_Color getCSenseColor();
    int getUSenseRange();
    void setDirection(int);
    int getDirection();

    virtual void run(bool *);
};

} // namespace robosim

#endif // !__ROBOT_MONITOR__
