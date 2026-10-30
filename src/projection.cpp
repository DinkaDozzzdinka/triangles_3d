#include "projection.hpp"

#include <cstddef>
#include <stdexcept>


namespace triangles
{
    Point2 ProjectTo2D(const Point3& point, Axis dropped_axis)
    {
        if (dropped_axis == Axis::X)
            return {point.cord_y, point.cord_z};
        if (dropped_axis == Axis::Y)
            return {point.cord_x, point.cord_z};
        if (dropped_axis == Axis::Z)
            return {point.cord_x, point.cord_y};

        throw std::invalid_argument("Unknown projection axis");
    }

    Triangle2 ProjectTriangleTo2D(const Triangle3& triangle, Axis dropped_axis)
    {
        Triangle2 triangle2{};

        for (std::size_t i = 0; i < triangle.vertices.size(); ++i)
            triangle2.vertices2[i] = ProjectTo2D(triangle.vertices[i], dropped_axis);

        return triangle2;
    }

}