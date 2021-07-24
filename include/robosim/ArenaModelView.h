/**
 * @file ArenaModelView.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief A Simple GUI displaying the arena model
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __ARENA_MODEL_VIEW_H__
#define __ARENA_MODEL_VIEW_H__

#include <vector>

namespace robosim {}

namespace simulatedrobot {
class SimulatedRobot;
}

namespace arenamodelview {

extern bool running;

void initModelView();

void mainLoop(const std::vector<simulatedrobot::SimulatedRobot *> &);

} // namespace arenamodelview

#endif // !__ARENA_MODEL_VIEW_H__
