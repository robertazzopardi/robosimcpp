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

#include <string>

namespace colour {

constexpr auto OPAQUE = 255;

struct Colour {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;

    inline auto operator==(const Colour &oc) const {
        return r == oc.r && g == oc.g && b == oc.b && a == oc.a;
    }
};

constexpr Colour RED = {255, 0, 0, OPAQUE};
constexpr Colour GREEN = {0, 255, 0, OPAQUE};
constexpr Colour BLUE = {0, 0, 255, OPAQUE};
constexpr Colour BLACK = {0, 0, 0, OPAQUE};
constexpr Colour WHITE = {255, 255, 255, OPAQUE};
constexpr Colour OFF_WHITE = {245, 245, 245, OPAQUE};
constexpr Colour OFF_BLACK = {48, 48, 48, OPAQUE};
constexpr Colour LINE_BLUE = {0, 100, 255, OPAQUE};
constexpr Colour OBSTACLE = {120, 120, 120, OPAQUE};

} // namespace colour

#endif // !__COMMON_H__
