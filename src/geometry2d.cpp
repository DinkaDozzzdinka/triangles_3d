#include <algorithm>
#include "geometry2d.hpp"


namespace triangles
{

    double Orient2D(const Point2& a, const Point2& b, const Point2& c)
    {
        return ((b.cord_x - a.cord_x) * (c.cord_y - a.cord_y) - (b.cord_y - a.cord_y) * (c.cord_x - a.cord_x));
    }

    bool PointOnSegment(const Point2& c, const Point2& a, const Point2& b)
    {
        if (Orient2D(a, b, c) != 0.0)
            return false;

        return std::min(a.cord_x, b.cord_x) <= c.cord_x
            && c.cord_x <= std::max(a.cord_x, b.cord_x)
            && std::min(a.cord_y, b.cord_y) <= c.cord_y
            && c.cord_y <= std::max(a.cord_y, b.cord_y);
    }

    bool SegmentsIntersect(const Segment2& lhs, const Segment2& rhs)
    {
        const double o1 = Orient2D(lhs.start, lhs.end, rhs.start);
        const double o2 = Orient2D(lhs.start, lhs.end, rhs.end);
        const double o3 = Orient2D(rhs.start, rhs.end, lhs.start);
        const double o4 = Orient2D(rhs.start, rhs.end, lhs.end);

        if ((o1 == 0.0 && PointOnSegment (rhs.start, lhs.start, lhs.end)|| o2 == 0.0 && PointOnSegment (rhs.end, lhs.start, lhs.end) || o3 == 0.0 && PointOnSegment (lhs.start, rhs.start, rhs.end) || o4 == 0.0 && PointOnSegment (lhs.end, rhs.start, rhs.end)))
            return true;
        else if (((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) && ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0)))
            return true;
        return false;
    }

    bool PointInTriangle(const Point2& point, const Triangle2& triangle)
    {
        Point2 a = triangle.vertices2[0];
        Point2 b = triangle.vertices2[1];
        Point2 c = triangle.vertices2[2];
        if ((Orient2D(a, b, point) >= 0 && Orient2D(b, c, point) >= 0 && Orient2D(c, a, point) >= 0) || (Orient2D(a, b, point) <= 0 && Orient2D(b, c, point) <= 0 && Orient2D(c, a, point) <= 0)) 
            return true;
        return false;
    }

}