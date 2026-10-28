#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>


namespace triangles
{
    Vec3 operator-(const Point3& lhs, const Point3& rhs)
    {
        Vec3 res_vector{};
        res_vector.cord_x = lhs.cord_x - rhs.cord_x;
        res_vector.cord_y = lhs.cord_y - rhs.cord_y;
        res_vector.cord_z = lhs.cord_z - rhs.cord_z;
        return res_vector;
    }

    Vec3 operator+(const Vec3& lhs, const Vec3& rhs)
    {
        return {
            lhs.cord_x + rhs.cord_x,
            lhs.cord_y + rhs.cord_y,
            lhs.cord_z + rhs.cord_z
        };
    }

    Point3 operator+(const Point3& point, const Vec3& displacement)
    {
        return {
            point.cord_x + displacement.cord_x,
            point.cord_y + displacement.cord_y,
            point.cord_z + displacement.cord_z
        };
    }

    double Dot(const Vec3& lhs, const Vec3& rhs)
    {
        return (lhs.cord_x*rhs.cord_x + lhs.cord_y*rhs.cord_y + lhs.cord_z*rhs.cord_z);
    }

    Vec3 Cross(const Vec3& lhs, const Vec3& rhs)
    {
        return {
            lhs.cord_y * rhs.cord_z - lhs.cord_z * rhs.cord_y,
            lhs.cord_z * rhs.cord_x - lhs.cord_x * rhs.cord_z,
            lhs.cord_x * rhs.cord_y - lhs.cord_y * rhs.cord_x
        };
    }

    double Length(const Vec3& vector)
    {
        return std::hypot(vector.cord_x, vector.cord_y, vector.cord_z);
    }

    Vec3 Normalize(const Vec3& vector)
    {
        if (!std::isfinite(vector.cord_x) || !std::isfinite(vector.cord_y) || !std::isfinite(vector.cord_z))
        {
            throw std::invalid_argument("Cannot normalize non-finite coordinates");
        }

        const double scale = std::max({std::abs(vector.cord_x), std::abs(vector.cord_y), std::abs(vector.cord_z)});
        if (scale == 0.0)
            throw std::domain_error("Cannot normalize a zero vector");

        const Vec3 scaled{
            vector.cord_x / scale,
            vector.cord_y / scale,
            vector.cord_z / scale
        };
        const double length = Length(scaled);
        return {
            scaled.cord_x / length,
            scaled.cord_y / length,
            scaled.cord_z / length
        };
    }

    Vec3 UnitNormal(const Point3& a, const Point3& b, const Point3& c)
    {
        const Vec3 ab = Normalize(b - a);
        const Vec3 ac = Normalize(c - a);
        return Normalize(Cross(ab, ac));
    }

    Vec3 UnitNormal(const Triangle3& triangle)
    {
        return UnitNormal(triangle.vertices[0], triangle.vertices[1], triangle.vertices[2]);
    }
    double Area(const Triangle3& triangle)
    {
        return 0.5*Length(Cross(triangle.vertices[1] - triangle.vertices[0], triangle.vertices[2] - triangle.vertices[0]));
    }

    Aabb3 Bounds(const Triangle3& triangle)
    {
        Point3 point_min = {std::min({triangle.vertices[0].cord_x, triangle.vertices[1].cord_x, triangle.vertices[2].cord_x}), std::min({triangle.vertices[0].cord_y, triangle.vertices[1].cord_y, triangle.vertices[2].cord_y}), std::min({triangle.vertices[0].cord_z, triangle.vertices[1].cord_z, triangle.vertices[2].cord_z})};
        Point3 point_max = {std::max({triangle.vertices[0].cord_x, triangle.vertices[1].cord_x, triangle.vertices[2].cord_x}), std::max({triangle.vertices[0].cord_y, triangle.vertices[1].cord_y, triangle.vertices[2].cord_y}), std::max({triangle.vertices[0].cord_z, triangle.vertices[1].cord_z, triangle.vertices[2].cord_z})};
        return {point_min, point_max};
    }

}
