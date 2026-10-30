#pragma once

#include <array>


namespace triangles
{
    struct Point3
    {
        double cord_x = 0.0;
        double cord_y = 0.0;
        double cord_z = 0.0;
    };

    struct Vec3
    {
        double cord_x = 0.0;
        double cord_y = 0.0;
        double cord_z = 0.0;
    };

    struct Segment3
    {
        Point3 start;
        Point3 end;
    };

    struct Triangle3
    {
        std:: array<Point3, 3> vertices;
    };

    struct Aabb3
    {
        Point3 minimum{};
        Point3 maximum{};
    };

    enum class Axis
    {
        X,
        Y,
        Z
    };

    
    
    Vec3 operator-(const Point3& lhs, const Point3& rhs);
    Vec3 operator+(const Vec3& lhs, const Vec3& rhs);
    Point3 operator+(const Point3& point, const Vec3& displacement);
    double Dot(const Vec3& lhs, const Vec3& rhs);
    Vec3 Cross(const Vec3& lhs, const Vec3& rhs);
    double Length(const Vec3& vector);
    Vec3 Normalize(const Vec3& vector);
    Vec3 UnitNormal(const Point3& a, const Point3& b, const Point3& c);
    Vec3 UnitNormal(const Triangle3& triangle);
    double Area(const Triangle3& triangle);
    Aabb3 Bounds(const Triangle3& triangle);
    double SignedDistanceToPlane(const Point3& point,const Triangle3& triangle);
    Axis DominantAxis(const Vec3& normal);

}
