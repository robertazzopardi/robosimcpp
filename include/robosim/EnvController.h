/**
 * @file EnvController.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief Sets up and manages the environment
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __ENV_CONTROLLER__
#define __ENV_CONTROLLER__

#include <memory>

namespace robosim {

class RobotMonitor;

class EnvController {
  private:
    robosim::RobotMonitor *myMonitor;

    // Store as void smart pointer type, purely to hide the declarations
    std::shared_ptr<void> model;
    std::shared_ptr<void> view;

  public:
    EnvController(const char *, int, int, RobotMonitor *);
    ~EnvController();

    void updateEnv();
};

} // namespace robosim

#endif // !__ENV_CONTROLLER__
