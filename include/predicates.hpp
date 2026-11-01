#pragma once

#include "geometry.hpp"
#include "geometry2d.hpp"

namespace triangles
{
    enum class Sign
    {
        Negative,
        Zero,
        Positive
    };


    Sign Orient2DSign(const Point2& a, const Point2& b, const Point2& c);
    Sign Orient3DSign(const Point3& a, const Point3& b, const Point3& c, const Point3& point);

    struct PlaneSideValues
    {
        double start;
        double end;
    };

    PlaneSideValues ScaledPlaneSideValues(const Segment3& segment, const Triangle3& plane);

}
