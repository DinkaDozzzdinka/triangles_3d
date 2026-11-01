#pragma once

#include <optional>
#include <vector>
#include "geometry.hpp"

namespace triangles
{
    // Boundary included. Degenerate triangles are treated as points/segments.
    // Finite coordinates and the orientation predicates' exponent limits apply.
    bool PointInTriangle(const Point3& point, const Triangle3& triangle);
    // Closed segments; zero-length segments and triangle degeneracies are supported.
    // Input restrictions are the same as for PointInTriangle.
    bool SegmentsIntersect(const Segment3& lhs, const Segment3& rhs);
    bool SegmentIntersectsTriangle(const Segment3& segment, const Triangle3& triangle);
    // Requires nondegenerate triangles in a common plane.
    bool CoplanarTrianglesIntersect(const Triangle3& lhs, const Triangle3& rhs);
    Point3 SegmentPlaneIntersection(const Segment3& segment,const Triangle3& plane_triangle);
    std::optional<Segment3> TrianglePlaneIntersection(const Triangle3& triangle, const Triangle3& plane_triangle);
    // Contacts count as intersection; handles point/segment triangle degeneracies.
    bool TrianglesIntersect(const Triangle3& lhs, const Triangle3& rhs);
    std::vector<bool> FindIntersectionsBruteForce(const std::vector<Triangle3>& triangles);

}
