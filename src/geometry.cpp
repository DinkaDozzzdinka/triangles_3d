#include "geometry.hpp"
#include "predicates.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>


namespace triangles
{
    namespace
    {
        bool PointLess(const Point3& first, const Point3& second)
        {
            if (first.cord_x != second.cord_x)
                return first.cord_x < second.cord_x;
            if (first.cord_y != second.cord_y)
                return first.cord_y < second.cord_y;
            return first.cord_z < second.cord_z;
        }
    }

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

    double SignedDistanceToPlane(const Point3& point, const Triangle3& triangle)
    {
        Vec3 normal = UnitNormal(triangle);
        return Dot(normal, point - triangle.vertices[0]);
    }

    Axis DominantAxis(const Vec3& normal)
    {
        auto max_component = std::max({std::abs(normal.cord_x), std::abs(normal.cord_y), std::abs(normal.cord_z)});
        if (max_component == std:: abs(normal.cord_x))
            return Axis::X;
        else if (max_component == std:: abs(normal.cord_y))
                return Axis::Y;
        else return Axis::Z;
    }

    TriangleKind ClassifyTriangle(const Triangle3& triangle)
    {
        for (const Point3& point : triangle.vertices)
        {
            if (!std::isfinite(point.cord_x) || !std::isfinite(point.cord_y)
                || !std::isfinite(point.cord_z))
                throw std::invalid_argument("Triangle classification requires finite coordinates");
        }

        const Point3& a = triangle.vertices[0];
        const Point3& b = triangle.vertices[1];
        const Point3& c = triangle.vertices[2];
        const bool a_equals_b = a.cord_x == b.cord_x
            && a.cord_y == b.cord_y && a.cord_z == b.cord_z;
        const bool a_equals_c = a.cord_x == c.cord_x
            && a.cord_y == c.cord_y && a.cord_z == c.cord_z;

        if (a_equals_b && a_equals_c)
            return TriangleKind::Point;

        const Sign xy = Orient2DSign({a.cord_x, a.cord_y}, {b.cord_x, b.cord_y}, {c.cord_x, c.cord_y});
        const Sign yz = Orient2DSign({a.cord_y, a.cord_z}, {b.cord_y, b.cord_z}, {c.cord_y, c.cord_z});
        const Sign zx = Orient2DSign({a.cord_z, a.cord_x}, {b.cord_z, b.cord_x}, {c.cord_z, c.cord_x});

        if (xy != Sign::Zero || yz != Sign::Zero || zx != Sign::Zero)
            return TriangleKind::Triangle;
        return TriangleKind::Segment;
    }

    Segment3 DegenerateTriangleSegment(const Triangle3& triangle)
    {
        if (ClassifyTriangle(triangle) == TriangleKind::Triangle)
            throw std::invalid_argument("A nondegenerate triangle cannot be represented by a segment");

        const auto endpoints = std::minmax_element(triangle.vertices.begin(),
                                                  triangle.vertices.end(), PointLess);
        return {*endpoints.first, *endpoints.second};
    }

    bool PointOnSegment(const Point3& point, const Segment3& segment)
    {
        const Triangle3 points{{segment.start, segment.end, point}};
        if (ClassifyTriangle(points) == TriangleKind::Triangle)
            return false;

        return std::min(segment.start.cord_x, segment.end.cord_x) <= point.cord_x
            && point.cord_x <= std::max(segment.start.cord_x, segment.end.cord_x)
            && std::min(segment.start.cord_y, segment.end.cord_y) <= point.cord_y
            && point.cord_y <= std::max(segment.start.cord_y, segment.end.cord_y)
            && std::min(segment.start.cord_z, segment.end.cord_z) <= point.cord_z
            && point.cord_z <= std::max(segment.start.cord_z, segment.end.cord_z);
    }

}
