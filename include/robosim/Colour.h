#pragma once

#ifdef __cplusplus
extern "C"
{
#endif /* ifdef __cplusplus */

    struct Colour
    {
        int r;
        int g;
        int b;
        int a;

#ifdef __cplusplus
        bool operator==(const Colour &oc) const
        {
            return r == oc.r && g == oc.g && b == oc.b && a == oc.a;
        }
#endif /* ifdef __cplusplus */
    };

    const int OPAQUE = 255;
    const Colour RED = {255, 0, 0, OPAQUE};
    const Colour GREEN = {0, 255, 0, OPAQUE};
    const Colour BLUE = {0, 0, 255, OPAQUE};
    const Colour BLACK = {0, 0, 0, OPAQUE};
    const Colour WHITE = {255, 255, 255, OPAQUE};
    const Colour OFF_WHITE = {245, 245, 245, OPAQUE};
    const Colour OFF_BLACK = {48, 48, 48, OPAQUE};
    const Colour OFF_RED = {255, 76, 76, OPAQUE};
    const Colour LINE_BLUE = {0, 100, 255, OPAQUE};
    const Colour OBSTACLE = {120, 120, 120, OPAQUE};

#ifdef __cplusplus
}
#endif /* ifdef __cplusplus */
