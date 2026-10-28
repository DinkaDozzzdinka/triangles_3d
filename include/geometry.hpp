#pragma once

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


    Vec3 operator-(const Point3& lhs, const Point3& rhs);

    Vec3 operator+(const Vec3& lhs, const Vec3& rhs);
    Point3 operator+(const Point3& point, const Vec3& displacement);

}
