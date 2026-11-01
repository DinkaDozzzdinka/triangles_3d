#include <algorithm>
#include <cstddef>
#include "geometry2d.hpp"
#include "predicates.hpp"


namespace triangles
{

    double Orient2D(const Point2& a, const Point2& b, const Point2& c)
    {
        const long double ax = a.cord_x;
        const long double bx = b.cord_x;
        const long double cx = c.cord_x;
        const long double ay = a.cord_y;
        const long double by = b.cord_y;
        const long double cy = c.cord_y;
        const long double ab_x = bx - ax;
        const long double ac_x = cx - ax;
        const long double ac_y = cy - ay;
        const long double ab_y = by - ay;

        return (ab_x * ac_y - ab_y * ac_x);
    }

    bool PointOnSegment(const Point2& c, const Point2& a, const Point2& b)
    {
        if (Orient2DSign(a, b, c) != Sign::Zero)
            return false;

        return std::min(a.cord_x, b.cord_x) <= c.cord_x
            && c.cord_x <= std::max(a.cord_x, b.cord_x)
            && std::min(a.cord_y, b.cord_y) <= c.cord_y
            && c.cord_y <= std::max(a.cord_y, b.cord_y);
    }

    bool SegmentsIntersect(const Segment2& lhs, const Segment2& rhs)
    {
        const Sign o1 = Orient2DSign(lhs.start, lhs.end, rhs.start);
        const Sign o2 = Orient2DSign(lhs.start, lhs.end, rhs.end);
        const Sign o3 = Orient2DSign(rhs.start, rhs.end, lhs.start);
        const Sign o4 = Orient2DSign(rhs.start, rhs.end, lhs.end);

        if ((o1 == Sign::Zero && PointOnSegment(rhs.start, lhs.start, lhs.end))
         || (o2 == Sign::Zero && PointOnSegment(rhs.end, lhs.start, lhs.end))
         || (o3 == Sign::Zero && PointOnSegment(lhs.start, rhs.start, rhs.end))
         || (o4 == Sign::Zero && PointOnSegment(lhs.end, rhs.start, rhs.end)))
            return true;

        const bool rhs_straddles = (o1 == Sign::Positive && o2 == Sign::Negative)
                               || (o1 == Sign::Negative && o2 == Sign::Positive);
        const bool lhs_straddles = (o3 == Sign::Positive && o4 == Sign::Negative)
                               || (o3 == Sign::Negative && o4 == Sign::Positive);
        return rhs_straddles && lhs_straddles;
    }

    bool PointInTriangle(const Point2& point, const Triangle2& triangle)
    {
        const Point2& a = triangle.vertices2[0];
        const Point2& b = triangle.vertices2[1];
        const Point2& c = triangle.vertices2[2];
        const Sign ab = Orient2DSign(a, b, point);
        const Sign bc = Orient2DSign(b, c, point);
        const Sign ca = Orient2DSign(c, a, point);

        const bool no_negative = ab != Sign::Negative
                              && bc != Sign::Negative
                              && ca != Sign::Negative;
        const bool no_positive = ab != Sign::Positive
                              && bc != Sign::Positive
                              && ca != Sign::Positive;
        return no_negative || no_positive;
    }

    bool TrianglesIntersect(const Triangle2& lhs, const Triangle2& rhs)
    {
        for (std::size_t i = 0; i < lhs.vertices2.size(); ++i)
        {
            const Segment2 first{
                lhs.vertices2[i],
                lhs.vertices2[(i + 1) % lhs.vertices2.size()]
            };

            for (std::size_t j = 0; j < rhs.vertices2.size(); ++j)
            {
                const Segment2 second{
                    rhs.vertices2[j],
                    rhs.vertices2[(j + 1) % rhs.vertices2.size()]
                };

                if (SegmentsIntersect(first, second))
                    return true;
            }
        }

        for (const Point2& vertex : lhs.vertices2)
        {
            if (PointInTriangle(vertex, rhs))
                return true;
        }

        for (const Point2& vertex : rhs.vertices2)
        {
            if (PointInTriangle(vertex, lhs))
                return true;
        }

        return false;
    }

    bool SegmentIntersectsTriangle(const Segment2& segment, const Triangle2& triangle)
    {
        if (PointInTriangle(segment.start, triangle) || PointInTriangle(segment.end, triangle))
            return true;

        for (std::size_t i = 0; i < triangle.vertices2.size(); ++i)
        {
            const Segment2 edge{
                triangle.vertices2[i],
                triangle.vertices2[(i + 1) % triangle.vertices2.size()]
            };
            if (SegmentsIntersect(segment, edge))
                return true;
        }

        return false;
    }

}