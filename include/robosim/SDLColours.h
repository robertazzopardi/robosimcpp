/**
 * @file OccupancyType.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief Enum defining the objects that can be in a grid cell
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __SDL_COLOURS__
#define __SDL_COLOURS__

#include <SDL_Pixels.h>

namespace sdlcolours {

static constexpr SDL_Color RED = {255, 0, 0, SDL_ALPHA_OPAQUE};
static constexpr SDL_Color GREEN = {0, 255, 0, SDL_ALPHA_OPAQUE};
static constexpr SDL_Color BLUE = {0, 0, 255, SDL_ALPHA_OPAQUE};
static constexpr SDL_Color BLACK = {0, 0, 0, SDL_ALPHA_OPAQUE};
static constexpr SDL_Color WHITE = {255, 255, 255, SDL_ALPHA_OPAQUE};
static constexpr SDL_Color OFF_WHITE = {245, 245, 245, SDL_ALPHA_OPAQUE};
static constexpr SDL_Color OFF_BLACK = {48, 48, 48, SDL_ALPHA_OPAQUE};
static constexpr SDL_Color LINE_BLUE = {0, 100, 255, SDL_ALPHA_OPAQUE};
static constexpr SDL_Color OBSTACLE = {120, 120, 120, SDL_ALPHA_OPAQUE};

} // namespace sdlcolours

#endif // !__SDL_COLOURS__
