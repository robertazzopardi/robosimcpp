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

#ifndef __COMMON_H__
#define __COMMON_H__

#include <SDL_Pixels.h>
#include <memory>

namespace sdlcolours {

constexpr SDL_Color RED = {255, 0, 0, SDL_ALPHA_OPAQUE};
constexpr SDL_Color GREEN = {0, 255, 0, SDL_ALPHA_OPAQUE};
constexpr SDL_Color BLUE = {0, 0, 255, SDL_ALPHA_OPAQUE};
constexpr SDL_Color BLACK = {0, 0, 0, SDL_ALPHA_OPAQUE};
constexpr SDL_Color WHITE = {255, 255, 255, SDL_ALPHA_OPAQUE};
constexpr SDL_Color OFF_WHITE = {245, 245, 245, SDL_ALPHA_OPAQUE};
constexpr SDL_Color OFF_BLACK = {48, 48, 48, SDL_ALPHA_OPAQUE};
constexpr SDL_Color LINE_BLUE = {0, 100, 255, SDL_ALPHA_OPAQUE};
constexpr SDL_Color OBSTACLE = {120, 120, 120, SDL_ALPHA_OPAQUE};

} // namespace sdlcolours

namespace typecasting {
template <typename T> auto cast(void *ptr) { return static_cast<T>(ptr); }

template <typename T> auto cast_ptr(std::shared_ptr<void> ptr) {
    return std::static_pointer_cast<T>(ptr);
}

template <typename T, typename... Args> auto make_ptr(Args... args) {
    return std::make_shared<T>(args...);
}
} // namespace typecasting

#endif // !__COMMON_H__
