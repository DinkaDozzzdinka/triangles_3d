#pragma once

#include <optional>
#include <vector>
#include "geometry.hpp"

namespace triangles
{
    bool PointInTriangle(const Point3& point, const Triangle3& triangle);
  
    bool SegmentsIntersect(const Segment3& lhs, const Segment3& rhs);
    bool SegmentIntersectsTriangle(const Segment3& segment, const Triangle3& triangle);

    bool CoplanarTrianglesIntersect(const Triangle3& lhs, const Triangle3& rhs);
    Point3 SegmentPlaneIntersection(const Segment3& segment,const Triangle3& plane_triangle);
    std::optional<Segment3> TrianglePlaneIntersection(const Triangle3& triangle, const Triangle3& plane_triangle);

    bool TrianglesIntersect(const Triangle3& lhs, const Triangle3& rhs);
    std::vector<bool> FindIntersectionsBruteForce(const std::vector<Triangle3>& triangles);

}
