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

#ifndef __ENV_CONTROLLER_H__
#define __ENV_CONTROLLER_H__

#include "RobotMonitor.h"
#include <memory>

namespace robosim {

class EnvController {
  private:
    MonitorVec myMonitors;

    // Store as void smart pointer type, purely to hide the declarations
    std::shared_ptr<void> view;

    void init(const MonitorVec &);

  public:
    EnvController(const char *, const MonitorVec &);
    EnvController(int, int, const MonitorVec &);

    ~EnvController();

    void startSimulation();
};

} // namespace robosim

#endif // !__ENV_CONTROLLER_H__
