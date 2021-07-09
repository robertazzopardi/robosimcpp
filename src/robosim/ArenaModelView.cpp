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
#include <SDL_rect.h>
#include <SDL_render.h>
#include <SDL_stdinc.h>
#include <SDL_timer.h>
#include <SDL_video.h>
#include <iostream>
#include <math.h>
#include <stddef.h>

#define WINDOW_TITLE "RoboSim"
static constexpr auto FRAME_DELAY = 1000 / 60;

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using mygridcell::OccupancyType;
using simulatedrobot::SimulatedRobot;

typedef struct {
    Sint16 x;
    Sint16 y;
    Sint16 r;
} Circle;

struct RobotRender {
    Circle body;
    Circle sensor;
    SDL_FPoint radius;
};

struct RenderObjects {
    std::vector<SDL_FRect> points;
    std::vector<SDL_FRect> obstacles;
    std::vector<SDL_FRect> reds;
    std::vector<SDL_FRect> greens;
    std::vector<SDL_FRect> blues;
};

bool ArenaModelView::running = true;

ArenaModelView::ArenaModelView(ArenaModel *model, SimulatedRobot *robot) {
    this->model = model;
    this->robot = robot;

    if (SDL_Init(SDL_INIT_EVERYTHING)) {
        std::cout << "error initializing SDL: %s\n"
                  << SDL_GetError() << std::endl;
    }

    robotRender = std::make_unique<RobotRender>();
    renderObjects = std::make_unique<RenderObjects>();

    window = SDL_CreateWindow(
        WINDOW_TITLE, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        model->getArenaWidthInCells() * model->getCellWidth(),
        model->getArenaHeightInCells() * model->getCellWidth(), 0);

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    buildGui();
}

ArenaModelView::~ArenaModelView() {
    // Destroy renderer
    SDL_DestroyRenderer(renderer);

    // Destroy window
    SDL_DestroyWindow(window);

    window = nullptr;
    renderer = nullptr;

    // Quit SDL
    SDL_Quit();
}

void ArenaModelView::buildGui() {

    auto cellWidth = model->getCellWidth();

    auto grid = model->getGrid();

    auto h = model->getArenaHeightInCells() * cellWidth;
    auto w = model->getArenaWidthInCells() * cellWidth;

    for (int row = 0; row < model->getArenaWidthInCells(); row += 2) {
        // vertical
        renderObjects->points.push_back({row * cellWidth, 0, cellWidth, h});
    }

    for (int row = 0; row < model->getArenaHeightInCells(); row += 2) {
        // horizontal
        renderObjects->points.push_back({0, row * cellWidth, w, cellWidth});
    }

    // renderObjects->points.push_back({0, 0, w, h});

    // Set up obstacles on the grid
    for (size_t row = 0; row < grid.size(); row++) {
        for (size_t col = 0; col < grid.at(row).size(); col++) {
            SDL_FRect fRect = {col * cellWidth, row * cellWidth, cellWidth,
                               cellWidth};

            switch (grid.at(row).at(col).getCellType()) {
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

    auto r = cellWidth / 3;
    robotRender->body.r = r;

    auto rs = r / 6;
    robotRender->sensor.r = rs;

    update();
}

void ArenaModelView::update() {
    // parameters
    auto x = robot->getX();
    auto y = robot->getY();

    auto angle = robot->getHeadingInRadians();

    auto scale = robotRender->body.r * .9;

    auto rx = x + sin(angle) * scale;
    auto ry = y + cos(angle) * scale;

    // body
    robotRender->body.x = x;
    robotRender->body.y = y;

    // heading line
    robotRender->radius.x = rx;
    robotRender->radius.y = ry;

    // sensor
    auto sangle = robot->getDirectionInRadians() + angle;

    auto sx = x + sin(sangle) * scale;
    auto sy = y + cos(sangle) * scale;

    robotRender->sensor.x = sx;
    robotRender->sensor.y = sy;
}

template <typename Function, typename V>
void ArenaModelView::renderColourDraw(Function renderFunction, colour::Colour c,
                                      std::vector<V> prims) {
    SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
    renderFunction(renderer, prims.data(), prims.size());
}

void ArenaModelView::mainLoop() {
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
        SDL_SetRenderDrawColor(renderer, colour::OFF_WHITE.r,
                               colour::OFF_WHITE.g, colour::OFF_WHITE.b,
                               colour::OFF_WHITE.a);

        // Clears the screen
        SDL_RenderClear(renderer);

        // DRAW LOGIC

        // Draw grid cells
        renderColourDraw(SDL_RenderFillRectsF, colour::OBSTACLE,
                         renderObjects->obstacles);
        renderColourDraw(SDL_RenderFillRectsF, colour::RED,
                         renderObjects->reds);
        renderColourDraw(SDL_RenderFillRectsF, colour::GREEN,
                         renderObjects->greens);
        renderColourDraw(SDL_RenderFillRectsF, colour::BLUE,
                         renderObjects->blues);

        // Draw Lines
        renderColourDraw(SDL_RenderDrawRectsF, colour::LINE_BLUE,
                         renderObjects->points);

        // Draw Robots
        filledCircleRGBA(renderer, robotRender->body.x, robotRender->body.y,
                         robotRender->body.r, colour::OFF_BLACK.r,
                         colour::OFF_BLACK.g, colour::OFF_BLACK.b,
                         colour::OFF_BLACK.a);
        SDL_SetRenderDrawColor(renderer, colour::OFF_WHITE.r,
                               colour::OFF_WHITE.g, colour::OFF_WHITE.b,
                               colour::OFF_WHITE.a);
        SDL_RenderDrawLineF(renderer, robotRender->body.x, robotRender->body.y,
                            robotRender->radius.x, robotRender->radius.y);

        filledCircleRGBA(renderer, robotRender->sensor.x, robotRender->sensor.y,
                         robotRender->sensor.r, colour::OFF_WHITE.r,
                         colour::OFF_WHITE.g, colour::OFF_WHITE.b,
                         colour::OFF_WHITE.a);

        // Draw to screen
        SDL_RenderPresent(renderer);

        // Frame delay
        SDL_Delay(FRAME_DELAY);
    }
}
