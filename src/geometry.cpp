#include "geometry.hpp"


namespace triangles
{
    Vec3 operator-(const Point3& lhs, const Point3& rhs)
    {
        Vec3 res_vector{};
        res_vector.cord_x = lhs.cord_x - rhs.cord_x;
        res_vector.cord_y = lhs.cord_y - rhs.cord_y;
        res_vector.cord_z = lhs.cord_z - rhs.cord_z;
        return res_vector;
    }

    Vec3 operator+(const Vec3& lhs, const Vec3& rhs)
    {
        return {
            lhs.cord_x + rhs.cord_x,
            lhs.cord_y + rhs.cord_y,
            lhs.cord_z + rhs.cord_z
        };
    }

    Point3 operator+(const Point3& point, const Vec3& displacement)
    {
        return {
            point.cord_x + displacement.cord_x,
            point.cord_y + displacement.cord_y,
            point.cord_z + displacement.cord_z
        };
    }

}
