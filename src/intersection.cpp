#include "intersection.hpp"
#include "projection.hpp"
#include "geometry2d.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <stdexcept>

namespace triangles
{
    namespace
    {
        long double PlaneSideValue(const Point3& point, const Triangle3& triangle)
        {
            const long double a_x = triangle.vertices[0].cord_x;
            const long double a_y = triangle.vertices[0].cord_y;
            const long double a_z = triangle.vertices[0].cord_z;
            const long double b_x = triangle.vertices[1].cord_x;
            const long double b_y = triangle.vertices[1].cord_y;
            const long double b_z = triangle.vertices[1].cord_z;
            const long double c_x = triangle.vertices[2].cord_x;
            const long double c_y = triangle.vertices[2].cord_y;
            const long double c_z = triangle.vertices[2].cord_z;
            const long double p_x = point.cord_x;
            const long double p_y = point.cord_y;
            const long double p_z = point.cord_z;

            const long double ab_x = b_x - a_x;
            const long double ab_y = b_y - a_y;
            const long double ab_z = b_z - a_z;
            const long double ac_x = c_x - a_x;
            const long double ac_y = c_y - a_y;
            const long double ac_z = c_z - a_z;
            const long double ap_x = p_x - a_x;
            const long double ap_y = p_y - a_y;
            const long double ap_z = p_z - a_z;

            return (ab_y * ac_z - ab_z * ac_y) * ap_x
                 + (ab_z * ac_x - ab_x * ac_z) * ap_y
                 + (ab_x * ac_y - ab_y * ac_x) * ap_z;
        }
    }

    bool CoplanarTrianglesIntersect(const Triangle3& lhs,const Triangle3& rhs)
    {
        Vec3 normal = UnitNormal(lhs);
        Axis dropped_axis = DominantAxis(normal);
        Triangle2 lhs2 = ProjectTriangleTo2D (lhs, dropped_axis);
        Triangle2 rhs2 = ProjectTriangleTo2D (rhs, dropped_axis);
        return TrianglesIntersect(lhs2, rhs2);

    }

    Point3 SegmentPlaneIntersection(const Segment3& segment, const Triangle3& plane_triangle)
    {
        const double start_distance = SignedDistanceToPlane(segment.start, plane_triangle);
        const double end_distance = SignedDistanceToPlane(segment.end, plane_triangle);

        if (!std::isfinite(start_distance) || !std::isfinite(end_distance))
            throw std::invalid_argument("Segment distances to the plane must be finite");

        const bool opposite_sides = (start_distance > 0.0 && end_distance < 0.0) || (start_distance < 0.0 && end_distance > 0.0);
        if (!opposite_sides)
            throw std::invalid_argument("Segment endpoints must lie strictly on opposite sides of the plane");

        const double scale = std::max(std::abs(start_distance), std::abs(end_distance));
        const double start_scaled = start_distance / scale;
        const double end_scaled = end_distance / scale;
        const double t = start_scaled / (start_scaled - end_scaled);
        const double remaining = 1.0 - t;

        return {
            remaining * segment.start.cord_x + t * segment.end.cord_x,
            remaining * segment.start.cord_y + t * segment.end.cord_y,
            remaining * segment.start.cord_z + t * segment.end.cord_z
        };
    }

    std::optional<Segment3> TrianglePlaneIntersection(const Triangle3& triangle, const Triangle3& plane_triangle)
    {
        std::array<double, 3> distances{};
        for (std::size_t i = 0; i < triangle.vertices.size(); ++i)
        {
            distances[i] = SignedDistanceToPlane(triangle.vertices[i], plane_triangle);
            if (!std::isfinite(distances[i]))
                throw std::invalid_argument("Triangle distances to the plane must be finite");
        }

        if (distances[0] == 0.0 && distances[1] == 0.0 && distances[2] == 0.0)
            throw std::invalid_argument("Coplanar triangles require a separate intersection check");

        std::array<Point3, 2> section_points{};
        std::size_t point_count = 0;

        for (std::size_t i = 0; i < triangle.vertices.size(); ++i)
        {
            if (distances[i] == 0.0)
                section_points[point_count++] = triangle.vertices[i];
        }

        for (std::size_t i = 0; i < triangle.vertices.size(); ++i)
        {
            const std::size_t next = (i + 1) % triangle.vertices.size();
            const bool opposite_sides = (distances[i] > 0.0 && distances[next] < 0.0) || (distances[i] < 0.0 && distances[next] > 0.0);
            if (opposite_sides)
            {
                const Segment3 edge{triangle.vertices[i], triangle.vertices[next]};
                section_points[point_count++] = SegmentPlaneIntersection(edge, plane_triangle);
            }
        }

        if (point_count == 0)
            return std::nullopt;
        if (point_count == 1)
            return Segment3{section_points[0], section_points[0]};
        return Segment3{section_points[0], section_points[1]};
    }

    bool TrianglesIntersect(const Triangle3& lhs, const Triangle3& rhs)
    {
        const Vec3 normal = UnitNormal(lhs);
        bool coplanar = true;
        for (const Point3& vertex : rhs.vertices)
        {
            const long double side = PlaneSideValue(vertex, lhs);
            if (!std::isfinite(side))
                throw std::invalid_argument("Triangle plane values must be finite");
            if (side != 0.0L)
                coplanar = false;
        }

        if (coplanar)
            return CoplanarTrianglesIntersect(lhs, rhs);

        const std::optional<Segment3> section = TrianglePlaneIntersection(rhs, lhs);
        if (!section)
            return false;

        const Axis dropped_axis = DominantAxis(normal);
        const Triangle2 projected_triangle = ProjectTriangleTo2D(lhs, dropped_axis);
        const Segment2 projected_section{
            ProjectTo2D(section->start, dropped_axis),
            ProjectTo2D(section->end, dropped_axis)
        };

        return SegmentIntersectsTriangle(projected_section, projected_triangle);
    }

}