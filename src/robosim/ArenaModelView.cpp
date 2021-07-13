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
#include "RobotMonitor.h"
#include "SimulatedRobot.h"
#include <SDL.h>
#include <SDL2_gfxPrimitives.h>
#include <SDL_error.h>
#include <SDL_events.h>
#include <SDL_rect.h>
#include <SDL_render.h>
#include <SDL_timer.h>
#include <SDL_video.h>
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <type_traits>
#include <vector>

static constexpr auto WINDOW_TITLE = "RoboSim";
static constexpr auto FRAME_DELAY = 1000 / 60;

using mygridcell::OccupancyType;
using robosim::RobotMonitor;
using simulatedrobot::SimulatedRobot;

namespace arenamodelview {

namespace {

struct RenderObjects {
    std::vector<SDL_FRect> points;
    std::vector<SDL_FRect> obstacles;
    std::vector<SDL_FRect> reds;
    std::vector<SDL_FRect> greens;
    std::vector<SDL_FRect> blues;
};

std::unique_ptr<RenderObjects> renderObjects;

SDL_Window *window;
SDL_Renderer *renderer;

void buildGui() {

    auto w = arenamodel::grid[0].size() * arenamodel::cellWidth;
    auto h = arenamodel::grid.size() * arenamodel::cellWidth;

    // vertical lines
    for (size_t row = 0; row < arenamodel::grid[0].size(); row += 2) {
        renderObjects->points.push_back(
            {row * arenamodel::cellWidth, 0, arenamodel::cellWidth, h});
    }

    // horizontal lines
    for (size_t row = 0; row < arenamodel::grid.size(); row += 2) {
        renderObjects->points.push_back(
            {0, row * arenamodel::cellWidth, w, arenamodel::cellWidth});
    }

    // Set up obstacles on the grid
    for (size_t row = 0; row < arenamodel::grid.size(); row++) {
        for (size_t col = 0; col < arenamodel::grid[row].size(); col++) {
            SDL_FRect fRect = {col * arenamodel::cellWidth,
                               row * arenamodel::cellWidth,
                               arenamodel::cellWidth, arenamodel::cellWidth};

            switch (arenamodel::grid[row][col].getCellType()) {
            case OccupancyType::OBSTACLE:
                renderObjects->obstacles.push_back(fRect);
                break;
            case OccupancyType::RED:
                renderObjects->reds.push_back(fRect);
                break;
            case OccupancyType::GREEN:
                renderObjects->greens.push_back(fRect);
                break;
            case OccupancyType::BLUE:
                renderObjects->blues.push_back(fRect);
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

static void cleanUp() {
    // Destroy renderer
    SDL_DestroyRenderer(renderer);
    renderer = nullptr;

    // Destroy window
    SDL_DestroyWindow(window);
    window = nullptr;

    // Quit SDL
    SDL_Quit();
}

} // namespace

void mainLoop(const std::vector<robosim::RobotPtr> &monitors) {
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
            default:
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
                         renderObjects->obstacles);
        renderColourDraw(SDL_RenderFillRectsF, RED, renderObjects->reds);
        renderColourDraw(SDL_RenderFillRectsF, GREEN, renderObjects->greens);
        renderColourDraw(SDL_RenderFillRectsF, BLUE, renderObjects->blues);

        // Draw Lines
        renderColourDraw(SDL_RenderDrawRectsF, LINE_BLUE,
                         renderObjects->points);

        // Draw Robots
        for (auto monitor : monitors) {
            auto renderObject =
                static_cast<SimulatedRobot *>(monitor->getRobot())
                    ->getRenderObject();

            filledCircleRGBA(renderer, renderObject.body.x, renderObject.body.y,
                             renderObject.body.r, OFF_BLACK.r, OFF_BLACK.g,
                             OFF_BLACK.b, OFF_BLACK.a);

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
        std::exit(-1);
    }

    window = SDL_CreateWindow(
        WINDOW_TITLE, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        arenamodel::grid[0].size() * arenamodel::cellWidth,
        arenamodel::grid.size() * arenamodel::cellWidth, 0);

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    renderObjects = std::make_unique<RenderObjects>();

    buildGui();
}

} // namespace arenamodelview
