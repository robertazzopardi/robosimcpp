#include "ArenaModelView.h"
#include "ArenaModel.h"
#include "MyGridCell.h"
#include "SDLColours.h"
#include "SimulatedRobot.h"
#include <SDL.h>
#include <SDL2_gfxPrimitives.h>
#include <SDL_Rect.h>
#include <SDL_error.h>
#include <SDL_events.h>
#include <SDL_keyboard.h>
#include <SDL_keycode.h>
#include <SDL_pixels.h>
#include <SDL_render.h>
#include <SDL_stdinc.h>
#include <SDL_timer.h>
#include <SDL_video.h>
#include <iostream>
#include <math.h>

#define SIZE 800
#define WINDOW_TITLE "RoboSim"
#define SCALE_FACTOR 0.25
#define DEGREES_TO_RADIANS 0.017453292519943295

#define LAY_COLOUR(c) c.r, c.g, c.b, c.a
#define LAY_POINTS(p, i) p[i].x, p[i].y, p[i + 1].x, p[i + 1].y
#define LAY_CIRCLE(c) c.x, c.y, c.r
#define LAY_LINE(p1, p2) p1.x, p1.y, p2.x, p2.y

using arenamodel::ArenaModel;
using arenamodelview::ArenaModelView;
using mygridcell::OccupancyType;
using simulatedrobot::SimulatedRobot;
namespace rc = sdlcolours::colour;

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

static const Uint32 FRAME_DELAY = 1000 / 60;

ArenaModelView::ArenaModelView(ArenaModel *model, SimulatedRobot *robot) {
    this->model = model;
    this->robot = robot;

    pointCount = (model->getArenaWidthInCells() * 4) + 4;

    if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
        std::cout << "error initializing SDL: %s\n"
                  << SDL_GetError() << std::endl;
    }

    points = new SDL_FPoint[pointCount << 1];
    obstacles = new SDL_FRect[model->getObstacleCount() + 1];
    reds = new SDL_FRect[model->getRedCount() + 1];
    greens = new SDL_FRect[model->getGreenCount() + 1];
    blues = new SDL_FRect[model->getBlueCount() + 1];

    robotRender = new Robot();

    window = SDL_CreateWindow(WINDOW_TITLE, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SIZE, SIZE, 0);

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    buildGui();

    // update();
}

ArenaModelView::~ArenaModelView() {
    // Free the points array
    delete[] points;
    points = nullptr;

    // Free the cells
    delete[] obstacles;
    delete[] reds;
    delete[] greens;
    delete[] blues;
    obstacles = nullptr;
    reds = nullptr;
    greens = nullptr;
    blues = nullptr;

    delete robotRender;
    robotRender = nullptr;

    obstacles = nullptr;
    reds = nullptr;
    greens = nullptr;
    blues = nullptr;

    // Destroy renderer
    SDL_DestroyRenderer(renderer);

    // Destroy window
    SDL_DestroyWindow(window);

    window = nullptr;
    renderer = nullptr;

    // Quit SDL
    SDL_Quit();
}

void ArenaModelView::setRobot(SimulatedRobot *robot) {
    this->robot = robot;
}

void ArenaModelView::buildGui() {
    // Set up grid lines
    int index = 0;
    for (int i = 0; i < pointCount / 4; i++) {
        float dxy = i * model->getCellWidth();
        points[index++] = {dxy, 0};
        points[index++] = {dxy, SIZE};
        points[index++] = {0, dxy};
        points[index++] = {SIZE, dxy};
    }
    std::cout << pointCount << " " << index << std::endl;

    points[index - 1].y -= 1;
    points[index - 2].y -= 1;
    points[index - 3].x -= 1;
    points[index - 4].x -= 1;

    // Set up grid cells
    int oindex = 0;
    int rindex = 0;
    int gindex = 0;
    int bindex = 0;

    float xy = model->getCellWidth();
    for (int y = 0; y < model->getArenaHeightInCells(); y++) {
        for (int x = 0; x < model->getArenaWidthInCells(); x++) {
            switch (model->getOccupancy(x, y)) {
            case OccupancyType::OBSTACLE:
                obstacles[oindex++] = {(x * xy), (y * xy), xy, xy};
                break;
            case OccupancyType::RED:
                reds[rindex++] = {(x * xy), (y * xy), xy, xy};
                break;
            case OccupancyType::GREEN:
                greens[gindex++] = {(x * xy), (y * xy), xy, xy};
                break;
            case OccupancyType::BLUE:
                blues[bindex++] = {(x * xy), (y * xy), xy, xy};
                break;
            case OccupancyType::ROBOT:
            case OccupancyType::EMPTY:
            case OccupancyType::UNKNOWN:
            default:
                break;
            }
        }
    }

    Sint16 r = round(model->getCellWidth() * 0.3);
    robotRender->body.r = r;
    Sint16 rs = round(model->getCellWidth() * 0.045);
    robotRender->sensor.r = rs;
}

void ArenaModelView::update() {
    // parameters
    Sint16 x = robot->getX();
    Sint16 y = robot->getY();

    Sint16 robotH = round(robot->getHeadingInRadians()); // heading angle in radians

    float headingX = x + sin(robotH) * robotRender->body.r;
    float headingY = y + cos(robotH) * robotRender->body.r;

    float scale = robotRender->body.r * .9;
    Sint16 rx = x + sin(robotH) * scale;
    Sint16 ry = y + cos(robotH) * scale;

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

void ArenaModelView::mainLoop() {
    SDL_Event event = {0};

    // Annimation loop
    while (handleEvents(&event)) {

        // UPDATE LOGIC
        update();
        // UPDATE LOGIC

        // Draw background
        SDL_SetRenderDrawColor(renderer, LAY_COLOUR(rc::OFF_WHITE));

        // Clears the screen
        SDL_RenderClear(renderer);

        // DRAW LOGIC

        // Draw grid cells
        SDL_SetRenderDrawColor(renderer, LAY_COLOUR(rc::OBSTACLE));
        SDL_RenderFillRectsF(renderer, obstacles, model->getObstacleCount());
        SDL_SetRenderDrawColor(renderer, LAY_COLOUR(rc::RED));
        SDL_RenderFillRectsF(renderer, reds, model->getRedCount());
        SDL_SetRenderDrawColor(renderer, LAY_COLOUR(rc::GREEN));
        SDL_RenderFillRectsF(renderer, greens, model->getGreenCount());
        SDL_SetRenderDrawColor(renderer, LAY_COLOUR(rc::BLUE));
        SDL_RenderFillRectsF(renderer, blues, model->getBlueCount());

        // Draw Lines
        SDL_SetRenderDrawColor(renderer, LAY_COLOUR(rc::LINE_BLUE));
        for (int i = 0; i < pointCount; i += 2)
            SDL_RenderDrawLine(renderer, LAY_POINTS(points, i));

        // Draw Robots
        filledCircleRGBA(renderer, LAY_CIRCLE(robotRender->body), LAY_COLOUR(rc::OFF_BLACK));
        SDL_SetRenderDrawColor(renderer, LAY_COLOUR(rc::OFF_WHITE));
        SDL_RenderDrawLineF(renderer, LAY_LINE(robotRender->center, robotRender->radius));
        filledCircleRGBA(renderer, LAY_CIRCLE(robotRender->sensor), LAY_COLOUR(rc::OFF_WHITE));

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
