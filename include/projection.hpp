#pragma once

#include "geometry.hpp"
#include "geometry2d.hpp"

namespace triangles
{
    Point2 ProjectTo2D(const Point3& point, Axis dropped_axis);
    Triangle2 ProjectTriangleTo2D(const Triangle3& triangle, Axis dropped_axis);
}