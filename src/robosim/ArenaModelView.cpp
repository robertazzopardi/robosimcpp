#include "ArenaModelView.h"
#include "ArenaModel.h"
#include "MyGridCell.h"
#include "SDLColours.h"
#include "SimulatedRobot.h"
#include <SDL.h>
#include <SDL2_gfxPrimitives.h>
#include <SDL_error.h>
#include <SDL_events.h>
#include <SDL_keyboard.h>
#include <SDL_keycode.h>
#include <SDL_pixels.h>
#include <SDL_rect.h>
#include <SDL_render.h>
#include <SDL_stdinc.h>
#include <SDL_timer.h>
#include <SDL_video.h>
#include <iostream>
#include <math.h>

static constexpr auto SIZE = 800;
static constexpr auto WINDOW_TITLE = "RoboSim";
static constexpr auto FRAME_DELAY = 1000 / 60;
// static constexpr auto DEGREES_TO_RADIANS = 0.017453292519943295;

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using mygridcell::OccupancyType;
using simulatedrobot::SimulatedRobot;
namespace rc = sdlcolours;

typedef struct {
    Sint16 x;
    Sint16 y;
    Sint16 r;
} Circle;

struct Robot {
    Circle body;
    Circle sensor;
    SDL_FPoint center;
    SDL_FPoint radius;
};

struct RenderObjects {
    std::vector<SDL_FRect> points;
    std::vector<SDL_FRect> obstacles;
    std::vector<SDL_FRect> reds;
    std::vector<SDL_FRect> greens;
    std::vector<SDL_FRect> blues;
};

ArenaModelView::ArenaModelView(ArenaModel *model, SimulatedRobot *robot) {
    this->model = model;
    this->robot = robot;

    pointCount = (model->getArenaWidthInCells() * 4) + 4;

    if (SDL_Init(SDL_INIT_EVERYTHING)) {
        std::cout << "error initializing SDL: %s\n"
                  << SDL_GetError() << std::endl;
    }

    robotRender = std::make_unique<Robot>();
    renderObjects = std::make_unique<RenderObjects>();

    window = SDL_CreateWindow(WINDOW_TITLE, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SIZE, SIZE, 0);

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
    // Set up grid lines
    for (auto i = 0; i < pointCount / 4; i++) {
        auto dxy = i * model->getCellWidth();
        renderObjects->points.push_back({dxy, 0, SIZE, SIZE});
        renderObjects->points.push_back({0, dxy, SIZE, SIZE});
    }

    auto xy = model->getCellWidth();
    for (auto y = 0; y < model->getArenaHeightInCells(); y++) {
        for (auto x = 0; x < model->getArenaWidthInCells(); x++) {
            switch (model->getOccupancy(x, y)) {
            case OccupancyType::OBSTACLE:
                renderObjects->obstacles.push_back({(x * xy), (y * xy), xy, xy});
                break;
            case OccupancyType::RED:
                renderObjects->reds.push_back({(x * xy), (y * xy), xy, xy});
                break;
            case OccupancyType::GREEN:
                renderObjects->greens.push_back({(x * xy), (y * xy), xy, xy});
                break;
            case OccupancyType::BLUE:
                renderObjects->blues.push_back({(x * xy), (y * xy), xy, xy});
                break;
            case OccupancyType::ROBOT:
            case OccupancyType::EMPTY:
            case OccupancyType::UNKNOWN:
            default:
                break;
            }
        }
    }

    auto r = round(model->getCellWidth() * 0.3);
    robotRender->body.r = r;

    auto rs = round(model->getCellWidth() * 0.045);
    robotRender->sensor.r = rs;
}

void ArenaModelView::update() {
    // parameters
    auto x = robot->getX();
    auto y = robot->getY();

    auto robotH = round(robot->getHeadingInRadians()); // heading angle in radians

    auto headingX = x + sin(robotH) * robotRender->body.r;
    auto headingY = y + cos(robotH) * robotRender->body.r;

    auto scale = robotRender->body.r * .9;
    auto rx = x + sin(robotH) * scale;
    auto ry = y + cos(robotH) * scale;

    // body circle
    robotRender->body.x = x;
    robotRender->body.y = y;

    // ------------------------
    // Need to indicate heading
    robotRender->center.x = x;
    robotRender->center.y = y;
    robotRender->radius.x = headingX;
    robotRender->radius.y = headingY;

    // ------------------------
    // Draw the sensor as a smaller circle
    robotRender->sensor.x = rx;
    robotRender->sensor.y = ry;
}

template <typename F, typename FF, typename V>
void ArenaModelView::drawC(F fn, FF fnfn, std::vector<V> prims, SDL_Color c) {
    fn(renderer, c.r, c.g, c.b, c.a);
    fnfn(renderer, prims.data(), prims.size());
}

// template <typename F>
// void drawG(F fn) {}

void ArenaModelView::mainLoop() {
    SDL_Event event = {0};

    // Annimation loop
    while (handleEvents(&event)) {

        // UPDATE LOGIC
        update();
        // UPDATE LOGIC

        // Draw background
        SDL_SetRenderDrawColor(renderer, rc::OFF_WHITE.r, rc::OFF_WHITE.g, rc::OFF_WHITE.b, rc::OFF_WHITE.a);

        // Clears the screen
        SDL_RenderClear(renderer);

        // DRAW LOGIC

        // Draw grid cells
        drawC(SDL_SetRenderDrawColor, SDL_RenderFillRectsF, renderObjects->obstacles, rc::OBSTACLE);
        drawC(SDL_SetRenderDrawColor, SDL_RenderFillRectsF, renderObjects->reds, rc::RED);
        drawC(SDL_SetRenderDrawColor, SDL_RenderFillRectsF, renderObjects->greens, rc::GREEN);
        drawC(SDL_SetRenderDrawColor, SDL_RenderFillRectsF, renderObjects->blues, rc::BLUE);

        // Draw Lines
        drawC(SDL_SetRenderDrawColor, SDL_RenderDrawRectsF, renderObjects->points, rc::LINE_BLUE);

        // Draw Robots
        filledCircleRGBA(renderer, robotRender->body.x, robotRender->body.y, robotRender->body.r, rc::OFF_BLACK.r, rc::OFF_BLACK.g, rc::OFF_BLACK.b, rc::OFF_BLACK.a);
        SDL_SetRenderDrawColor(renderer, rc::OFF_WHITE.r, rc::OFF_WHITE.g, rc::OFF_WHITE.b, rc::OFF_WHITE.a);
        SDL_RenderDrawLineF(renderer, robotRender->center.x, robotRender->center.y, robotRender->radius.x, robotRender->radius.y);
        filledCircleRGBA(renderer, robotRender->sensor.x, robotRender->sensor.y, robotRender->sensor.r, rc::OFF_WHITE.r, rc::OFF_WHITE.g, rc::OFF_WHITE.b, rc::OFF_WHITE.a);

        // DRAW LOGIC

        // Triggers the double buffers for multiple rendering
        SDL_RenderPresent(renderer);

        // Frame delay
        SDL_Delay(FRAME_DELAY);
    }
}

bool ArenaModelView::handleEvents(SDL_Event *event) {
    while (SDL_PollEvent(event)) {
        switch (event->type) {
        case SDL_QUIT:
            return false;
        case SDL_KEYDOWN:
            switch (event->key.keysym.sym) {
            case SDLK_ESCAPE:
                return false;
            default:
                break;
            }
            break;
        default:
            break;
        }
    }
    return true;
}
