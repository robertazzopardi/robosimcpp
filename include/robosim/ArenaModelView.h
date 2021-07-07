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

#include <memory>
#include <vector>

struct SDL_Window;
struct SDL_Renderer;
struct SDL_Color;

struct RobotRender;

namespace simulatedrobot {
class SimulatedRobot;
} // namespace simulatedrobot

namespace arenamodel {
class ArenaModel;
} // namespace arenamodel

struct RenderObjects;

namespace arenamodelview {

class ArenaModelView {
  private:
    arenamodel::ArenaModel *model;
    simulatedrobot::SimulatedRobot *robot;
    std::unique_ptr<RobotRender> robotRender;
    std::unique_ptr<RenderObjects> renderObjects;

    SDL_Window *window;
    SDL_Renderer *renderer;

    int pointCount;

    void buildGui();

    template <typename F, typename FF, typename V>
    void drawC(F, FF, std::vector<V>, SDL_Color);

  public:
    static bool running;

    ArenaModelView(arenamodel::ArenaModel *, simulatedrobot::SimulatedRobot *);
    ~ArenaModelView();

    void update();
    void mainLoop();
};

} // namespace arenamodelview

#endif // !__ARENA_MODEL_VIEW_H__
