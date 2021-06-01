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

typedef struct SDL_Window SDL_Window;
typedef struct SDL_Renderer SDL_Renderer;
typedef struct SDL_FPoint SDL_FPoint;
typedef struct SDL_FRect SDL_FRect;
typedef union SDL_Event SDL_Event;

typedef struct Robot Robot;

namespace simulatedrobot {
class SimulatedRobot;
} // namespace simulatedrobot

namespace arenamodel {
class ArenaModel;
} // namespace arenamodel

namespace arenamodelview {

class ArenaModelView {
  private:
    arenamodel::ArenaModel *model;
    simulatedrobot::SimulatedRobot *robot;

    SDL_Window *window;
    SDL_Renderer *renderer;

    SDL_FPoint *points;
    SDL_FRect *obstacles;
    SDL_FRect *reds;
    SDL_FRect *greens;
    SDL_FRect *blues;

    int pointCount;

    static bool handleEvents(SDL_Event *);

    void buildGui();

    Robot *robotRender;
    void update();

  public:
    ArenaModelView(arenamodel::ArenaModel *, simulatedrobot::SimulatedRobot *);
    ~ArenaModelView();

    void setRobot(simulatedrobot::SimulatedRobot *);
    void mainLoop();
};

} // namespace arenamodelview

#endif // !__ARENA_MODEL_VIEW__
