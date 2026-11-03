#include "intersection.hpp"
#include "predicates.hpp"
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
        bool SamePoint(const Point3& lhs, const Point3& rhs)
        {
            return lhs.cord_x == rhs.cord_x && lhs.cord_y == rhs.cord_y && lhs.cord_z == rhs.cord_z;
        }

        void ValidateSegment(const Segment3& segment)
        {
            for (const Point3& point : {segment.start, segment.end})
            {
                if (!std::isfinite(point.cord_x) || !std::isfinite(point.cord_y) || !std::isfinite(point.cord_z))
                    throw std::invalid_argument("Segment coordinates must be finite");
            }
        }

        Segment2 ProjectSegmentTo2D(const Segment3& segment, Axis dropped_axis)
        {
            return {ProjectTo2D(segment.start, dropped_axis), ProjectTo2D(segment.end, dropped_axis)};
        }

        std::array<Sign, 3> TriangleVertexSides(const Triangle3& triangle, const Triangle3& plane)
        {
            return {
                Orient3DSign(plane.vertices[0], plane.vertices[1], plane.vertices[2], triangle.vertices[0]),
                Orient3DSign(plane.vertices[0], plane.vertices[1], plane.vertices[2], triangle.vertices[1]),
                Orient3DSign(plane.vertices[0], plane.vertices[1], plane.vertices[2], triangle.vertices[2])
            };
        }

        bool AllSidesEqual(const std::array<Sign, 3>& sides, Sign value)
        {
            return sides[0] == value && sides[1] == value && sides[2] == value;
        }

        bool StrictlyOnOneSide(const std::array<Sign, 3>& sides)
        {
            return AllSidesEqual(sides, Sign::Positive) || AllSidesEqual(sides, Sign::Negative);
        }

        bool OppositeSides(Sign first, Sign second)
        {
            return (first == Sign::Positive && second == Sign::Negative)
                || (first == Sign::Negative && second == Sign::Positive);
        }

        Axis PlaneProjectionAxis(const Triangle3& plane)
        {
            for (const Axis axis : {Axis::X, Axis::Y, Axis::Z})
            {
                const Triangle2 projected = ProjectTriangleTo2D(plane, axis);
                if (Orient2DSign(projected.vertices2[0], projected.vertices2[1],
                                 projected.vertices2[2]) != Sign::Zero)
                    return axis;
            }
            throw std::domain_error("A degenerate triangle does not define a plane");
        }
    }

    bool PointInTriangle(const Point3& point, const Triangle3& triangle)
    {
        if (ClassifyTriangle(triangle) != TriangleKind::Triangle)
            return PointOnSegment(point, DegenerateTriangleSegment(triangle));

        if (Orient3DSign(triangle.vertices[0], triangle.vertices[1], triangle.vertices[2], point) != Sign::Zero)
            return false;

        const Axis dropped_axis = PlaneProjectionAxis(triangle);
        return PointInTriangle(ProjectTo2D(point, dropped_axis),
                               ProjectTriangleTo2D(triangle, dropped_axis));
    }

    bool CoplanarTrianglesIntersect(const Triangle3& lhs,const Triangle3& rhs)
    {
        const Axis dropped_axis = PlaneProjectionAxis(lhs);
        Triangle2 lhs2 = ProjectTriangleTo2D (lhs, dropped_axis);
        Triangle2 rhs2 = ProjectTriangleTo2D (rhs, dropped_axis);
        return TrianglesIntersect(lhs2, rhs2);

    }

    bool SegmentsIntersect(const Segment3& lhs, const Segment3& rhs)
    {
        ValidateSegment(lhs);
        ValidateSegment(rhs);
        if (SamePoint(lhs.start, lhs.end))
            return PointOnSegment(lhs.start, rhs);
        if (SamePoint(rhs.start, rhs.end))
            return PointOnSegment(rhs.start, lhs);

        if (Orient3DSign(lhs.start, lhs.end, rhs.start, rhs.end) != Sign::Zero)
            return false;

        for (const Point3& point : {rhs.start, rhs.end})
        {
            const Triangle3 plane{{lhs.start, lhs.end, point}};
            if (ClassifyTriangle(plane) == TriangleKind::Triangle)
            {
                const Axis dropped_axis = PlaneProjectionAxis(plane);
                return SegmentsIntersect(ProjectSegmentTo2D(lhs, dropped_axis),
                                         ProjectSegmentTo2D(rhs, dropped_axis));
            }
        }

        return PointOnSegment(lhs.start, rhs) || PointOnSegment(lhs.end, rhs)
            || PointOnSegment(rhs.start, lhs) || PointOnSegment(rhs.end, lhs);
    }

    bool SegmentIntersectsTriangle(const Segment3& segment, const Triangle3& triangle)
    {
        ValidateSegment(segment);
        const TriangleKind kind = ClassifyTriangle(triangle);
        if (SamePoint(segment.start, segment.end))
            return PointInTriangle(segment.start, triangle);
        if (kind != TriangleKind::Triangle)
            return SegmentsIntersect(segment, DegenerateTriangleSegment(triangle));

        const auto& a = triangle.vertices[0];
        const auto& b = triangle.vertices[1];
        const auto& c = triangle.vertices[2];
        const Sign start_side = Orient3DSign(a, b, c, segment.start);
        const Sign end_side = Orient3DSign(a, b, c, segment.end);
        if (start_side == Sign::Zero && end_side == Sign::Zero)
        {
            const Axis dropped_axis = PlaneProjectionAxis(triangle);
            return SegmentIntersectsTriangle(ProjectSegmentTo2D(segment, dropped_axis),
                                             ProjectTriangleTo2D(triangle, dropped_axis));
        }
        if (start_side == Sign::Zero)
            return PointInTriangle(segment.start, triangle);
        if (end_side == Sign::Zero)
            return PointInTriangle(segment.end, triangle);
        if (!OppositeSides(start_side, end_side))
            return false;

        const Sign ab = Orient3DSign(segment.start, segment.end, a, b);
        const Sign bc = Orient3DSign(segment.start, segment.end, b, c);
        const Sign ca = Orient3DSign(segment.start, segment.end, c, a);
        const bool has_positive = ab == Sign::Positive || bc == Sign::Positive || ca == Sign::Positive;
        const bool has_negative = ab == Sign::Negative || bc == Sign::Negative || ca == Sign::Negative;
        return !(has_positive && has_negative);
    }

    Point3 SegmentPlaneIntersection(const Segment3& segment, const Triangle3& plane_triangle)
    {
        PlaneProjectionAxis(plane_triangle);
        const PlaneSideValues values = ScaledPlaneSideValues(segment, plane_triangle);
        const double start_value = values.start;
        const double end_value = values.end;

        const bool opposite_sides = (start_value > 0.0 && end_value < 0.0) || (start_value < 0.0 && end_value > 0.0);
        if (!opposite_sides)
            throw std::invalid_argument("Segment endpoints must lie strictly on opposite sides of the plane");

        const double scale = std::max(std::abs(start_value), std::abs(end_value));
        const double start_scaled = start_value / scale;
        const double end_scaled = end_value / scale;
        const double sum = std::abs(start_scaled) + std::abs(end_scaled);
        const double t = std::abs(start_scaled) / sum;
        const double remaining = std::abs(end_scaled) / sum;

        return {
            remaining * segment.start.cord_x + t * segment.end.cord_x,
            remaining * segment.start.cord_y + t * segment.end.cord_y,
            remaining * segment.start.cord_z + t * segment.end.cord_z
        };
    }

    std::optional<Segment3> TrianglePlaneIntersection(const Triangle3& triangle, const Triangle3& plane_triangle)
    {
        PlaneProjectionAxis(plane_triangle);
        const auto sides = TriangleVertexSides(triangle, plane_triangle);

        if (AllSidesEqual(sides, Sign::Zero))
            throw std::invalid_argument("Coplanar triangles require a separate intersection check");

        std::array<Point3, 2> section_points{};
        std::size_t point_count = 0;

        for (std::size_t i = 0; i < triangle.vertices.size(); ++i)
        {
            if (sides[i] == Sign::Zero)
                section_points[point_count++] = triangle.vertices[i];
        }

        for (std::size_t i = 0; i < triangle.vertices.size(); ++i)
        {
            const std::size_t next = (i + 1) % triangle.vertices.size();
            if (OppositeSides(sides[i], sides[next]))
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
        const TriangleKind lhs_kind = ClassifyTriangle(lhs);
        const TriangleKind rhs_kind = ClassifyTriangle(rhs);
        if (lhs_kind != TriangleKind::Triangle)
            return SegmentIntersectsTriangle(DegenerateTriangleSegment(lhs), rhs);
        if (rhs_kind != TriangleKind::Triangle)
            return SegmentIntersectsTriangle(DegenerateTriangleSegment(rhs), lhs);

        const auto rhs_sides = TriangleVertexSides(rhs, lhs);
        if (AllSidesEqual(rhs_sides, Sign::Zero))
            return CoplanarTrianglesIntersect(lhs, rhs);
        if (StrictlyOnOneSide(rhs_sides))
            return false;

        const auto lhs_sides = TriangleVertexSides(lhs, rhs);
        if (StrictlyOnOneSide(lhs_sides))
            return false;

        for (std::size_t i = 0; i < lhs.vertices.size(); ++i)
        {
            const std::size_t next = (i + 1) % lhs.vertices.size();
            if (SegmentIntersectsTriangle({lhs.vertices[i], lhs.vertices[next]}, rhs)
                || SegmentIntersectsTriangle({rhs.vertices[i], rhs.vertices[next]}, lhs))
                return true;
        }
        return false;
    }


    std::vector<bool> FindIntersectionsBruteForce(const std::vector<Triangle3>& triangles)
    {
        std::vector<bool> flags(triangles.size(), false);

        for (std::size_t i = 0; i < triangles.size(); ++i)
        {
            for (std::size_t j = i + 1; j < triangles.size(); ++j)
            {
                if (TrianglesIntersect(triangles[i], triangles[j]))
                {
                    flags[i] = true;
                    flags[j] = true;
                }
            }
        }

        return flags;
    }

}
