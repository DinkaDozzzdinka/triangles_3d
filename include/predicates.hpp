#pragma once

#include "geometry2d.hpp"

namespace triangles
{
    enum class Sign
    {
        Negative,
        Zero,
        Positive
    };

    // Finite coordinates are rescaled exactly by a common power of two.
    // Throws invalid_argument for non-finite inputs and range_error if
    // nonzero coordinate exponents span more than 256 binary orders.
    Sign Orient2DSign(const Point2& a, const Point2& b, const Point2& c);
}