#include "intersection.hpp"
#include "projection.hpp"
#include "geometry2d.hpp"

namespace triangles
{
    bool CoplanarTrianglesIntersect(const Triangle3& lhs,const Triangle3& rhs)
    {
        Vec3 normal = UnitNormal(lhs);
        Axis dropped_axis = DominantAxis(normal);
        Triangle2 lhs2 = ProjectTriangleTo2D (lhs, dropped_axis);
        Triangle2 rhs2 = ProjectTriangleTo2D (rhs, dropped_axis);
        return TrianglesIntersect(lhs2, rhs2);

    }
}