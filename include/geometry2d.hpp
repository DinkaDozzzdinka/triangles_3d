#pragma once
#include <array>


namespace triangles 
{
    struct Point2
    {
        double cord_x = 0.0;
        double cord_y = 0.0;
    };

    struct Segment2
    {
        Point2 start;
        Point2 end;
    };

    struct Triangle2
    {
        std:: array<Point2, 3> vertices2;
    };

    
    double Orient2D(const Point2& a, const Point2& b, const Point2& c);
    bool PointOnSegment(const Point2& point, const Point2& a, const Point2& b);
    bool SegmentsIntersect(const Segment2& lhs, const Segment2& rhs);
    bool PointInTriangle(const Point2& point, const Triangle2& triangle);
}