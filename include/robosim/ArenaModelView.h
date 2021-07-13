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

#include "RobotMonitor.h"
#include <memory>
#include <vector>

struct SDL_Window;
struct SDL_Renderer;

namespace colour {
struct Colour;
}

struct RenderObjects;

namespace arenamodelview {

class ArenaModelView {
  private:
    std::unique_ptr<RenderObjects> renderObjects;

    SDL_Window *window;
    SDL_Renderer *renderer;

    void buildGui();

    template <typename Function, typename V>
    void renderColourDraw(Function, colour::Colour, std::vector<V>);

  public:
    static bool running;

    ArenaModelView();
    ~ArenaModelView();

    void mainLoop(const robosim::MonitorVec &);
};

} // namespace arenamodelview

#endif // !__ARENA_MODEL_VIEW_H__
