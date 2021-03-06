/**
 * @file ArenaModelView.cpp
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief
 * @version 0.1
 * @date 2021-07-07
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "ArenaModelView.h"
#include "ArenaModel.h"
#include "Colour.h"
#include "MyGridCell.h"
#include "SimulatedRobot.h"
#include <SDL.h>
#include <SDL2_gfxPrimitives.h>
#include <SDL_error.h>
#include <SDL_events.h>
#include <SDL_hints.h>
#include <SDL_rect.h>
#include <SDL_render.h>
#include <SDL_timer.h>
#include <SDL_video.h>
#include <iostream>
#include <stdlib.h>
#include <vector>

namespace robosim {

namespace robotmonitor {
class RobotMonitor;
}

}  // namespace robosim

static constexpr auto WINDOW_TITLE = "RoboSim";
static constexpr auto FRAME_DELAY = 1000 / 60;

using mygridcell::OccupancyType;
// using robosim::robotmonitor::RobotMonitor;
using simulatedrobot::SimulatedRobot;

namespace arenamodelview {

bool running = true;

namespace {

struct RenderObjects {
    std::vector<SDL_FRect> points;
    std::vector<SDL_FRect> obstacles;
    std::vector<SDL_FRect> reds;
    std::vector<SDL_FRect> greens;
    std::vector<SDL_FRect> blues;
};

static RenderObjects renderObjects;

static SDL_Window *window;
static SDL_Renderer *renderer;

static inline SDL_FRect makeFRect(const size_t col, const size_t row) {
    return {col * arenamodel::cellWidth, row * arenamodel::cellWidth,
            arenamodel::cellWidth, arenamodel::cellWidth};
}

void buildGui() {

    auto w = arenamodel::grid[0].size() * arenamodel::cellWidth;
    auto h = arenamodel::grid.size() * arenamodel::cellWidth;

    // vertical lines
    for (size_t row = 0; row < arenamodel::grid[0].size(); row += 2) {
        renderObjects.points.push_back(
            {row * arenamodel::cellWidth, 0, arenamodel::cellWidth, h});
    }

    // Set up obstacles on the grid
    for (size_t row = 0; row < arenamodel::grid.size(); row++) {

        // Add horizontal lines
        // Check if number is not odd,
        // to mimic the loop stride of the vertical lines
        if (!(row & 1)) {
            renderObjects.points.push_back(
                {0, row * arenamodel::cellWidth, w, arenamodel::cellWidth});
        }

        for (size_t col = 0; col < arenamodel::grid[row].size(); col++) {
            switch (arenamodel::grid[row][col].getCellType()) {
            case OccupancyType::OBSTACLE:
                renderObjects.obstacles.push_back(makeFRect(col, row));
                break;
            case OccupancyType::RED:
                renderObjects.reds.push_back(makeFRect(col, row));
                break;
            case OccupancyType::GREEN:
                renderObjects.greens.push_back(makeFRect(col, row));
                break;
            case OccupancyType::BLUE:
                renderObjects.blues.push_back(makeFRect(col, row));
                break;
            case OccupancyType::ROBOT:
            case OccupancyType::EMPTY:
            case OccupancyType::UNKNOWN:
            default:
                break;
            }
        }
    }
}

template <typename Function, typename V>
static constexpr void renderColourDraw(Function renderFunction,
                                       colour::Colour c, std::vector<V> prims) {
    SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
    renderFunction(renderer, prims.data(), prims.size());
}

static inline void cleanUp() {
    // Destroy renderer
    SDL_DestroyRenderer(renderer);
    renderer = nullptr;

    // Destroy window
    SDL_DestroyWindow(window);
    window = nullptr;

    // Quit SDL
    SDL_Quit();
}

}  // namespace

void mainLoop(simulatedrobot::SimulatedRobot **robots,
              const size_t robotCount) {
    using namespace colour;

    SDL_Event event;

    // Annimation loop
    while (running) {
        // Handle events
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_QUIT:
                running = false;
                break;
            }
        }

        // Draw background
        SDL_SetRenderDrawColor(renderer, OFF_WHITE.r, OFF_WHITE.g, OFF_WHITE.b,
                               OFF_WHITE.a);

        // Clears the screen
        SDL_RenderClear(renderer);

        // DRAW LOGIC

        // Draw grid cells
        renderColourDraw(SDL_RenderFillRectsF, OBSTACLE,
                         renderObjects.obstacles);
        renderColourDraw(SDL_RenderFillRectsF, RED, renderObjects.reds);
        renderColourDraw(SDL_RenderFillRectsF, GREEN, renderObjects.greens);
        renderColourDraw(SDL_RenderFillRectsF, BLUE, renderObjects.blues);

        // Draw Lines
        renderColourDraw(SDL_RenderDrawRectsF, LINE_BLUE, renderObjects.points);

        // Draw Robots
        // for (auto robot : robots) {
        for (size_t i = 0; i < robotCount; i++) {

            auto renderObject = robots[i]->getRenderObject();

            filledCircleRGBA(renderer, renderObject.body.x, renderObject.body.y,
                             renderObject.body.r, renderObject.bodyColour.r,
                             renderObject.bodyColour.g,
                             renderObject.bodyColour.b,
                             renderObject.bodyColour.a);

            SDL_SetRenderDrawColor(renderer, OFF_WHITE.r, OFF_WHITE.g,
                                   OFF_WHITE.b, OFF_WHITE.a);
            SDL_RenderDrawLineF(renderer, renderObject.body.x,
                                renderObject.body.y, renderObject.radius.x,
                                renderObject.radius.y);

            filledCircleRGBA(renderer, renderObject.sensor.x,
                             renderObject.sensor.y, renderObject.sensor.r,
                             OFF_WHITE.r, OFF_WHITE.g, OFF_WHITE.b,
                             OFF_WHITE.a);
        }

        // Draw to screen
        SDL_RenderPresent(renderer);

        // Frame delay
        SDL_Delay(FRAME_DELAY);
    }

    cleanUp();
}

void initModelView() {
    if (SDL_Init(SDL_INIT_EVERYTHING)) {
        std::cout << "error initializing SDL: %s\n"
                  << SDL_GetError() << std::endl;
        std::exit(EXIT_FAILURE);
    }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");

    window = SDL_CreateWindow(
        WINDOW_TITLE, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        arenamodel::grid[0].size() * arenamodel::cellWidth,
        arenamodel::grid.size() * arenamodel::cellWidth, 0);

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // renderObjects = std::make_unique<RenderObjects>();

    buildGui();
}

}  // namespace arenamodelview
