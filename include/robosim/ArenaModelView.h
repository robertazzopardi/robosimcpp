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

#ifndef __ARENA_MODEL_VIEW__
#define __ARENA_MODEL_VIEW__

#include <memory>
#include <vector>

struct SDL_Window;
struct SDL_Renderer;
struct SDL_Color;
union SDL_Event;

struct Robot;

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
    std::unique_ptr<Robot> robotRender;
    std::unique_ptr<RenderObjects> renderObjects;

    SDL_Window *window;
    SDL_Renderer *renderer;

    int pointCount;

    static bool handleEvents(SDL_Event *);

    void buildGui();

    void update();

    template <typename F, typename FF, typename V>
    void drawC(F, FF, std::vector<V>, SDL_Color);

  public:
    ArenaModelView(arenamodel::ArenaModel *, simulatedrobot::SimulatedRobot *);
    ~ArenaModelView();

    void mainLoop();
};

} // namespace arenamodelview

#endif // !__ARENA_MODEL_VIEW__
