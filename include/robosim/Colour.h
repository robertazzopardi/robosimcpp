#pragma once

#include <stdint.h>

namespace colour
{

constexpr uint8_t OPAQUE = 255;

struct Colour
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;

    bool operator==(const Colour &oc) const
    {
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
constexpr Colour OFF_RED = {255, 76, 76, OPAQUE};
constexpr Colour LINE_BLUE = {0, 100, 255, OPAQUE};
constexpr Colour OBSTACLE = {120, 120, 120, OPAQUE};

} // namespace colour
