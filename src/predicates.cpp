#include "predicates.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

extern "C"
{
    void exactinit(void);
    double orient2d(double* a, double* b, double* c);
    double orient3d(double* a, double* b, double* c, double* point);
    double orient3dexact(double* a, double* b, double* c, double* point);
}

namespace triangles
{
    namespace
    {
        constexpr int maximum_exponent_spread = 256;

        struct PredicatesInitialization
        {
            PredicatesInitialization()
            {
                exactinit();
            }
        };

        void InitializePredicates()
        {
            static const PredicatesInitialization initialization;
        }

        template <std::size_t Size>
        bool NormalizeCoordinates(std::array<double, Size>& coordinates)
        {
            int minimum_exponent = std::numeric_limits<int>::max();
            int maximum_exponent = std::numeric_limits<int>::lowest();
            bool has_nonzero_coordinate = false;

            for (const double coordinate : coordinates)
            {
                if (!std::isfinite(coordinate))
                    throw std::invalid_argument("Orientation requires finite coordinates");
                if (coordinate != 0.0)
                {
                    const int exponent = std::ilogb(coordinate);
                    minimum_exponent = std::min(minimum_exponent, exponent);
                    maximum_exponent = std::max(maximum_exponent, exponent);
                    has_nonzero_coordinate = true;
                }
            }

            if (!has_nonzero_coordinate)
                return false;
            if (maximum_exponent - minimum_exponent > maximum_exponent_spread)
                throw std::range_error("Orientation coordinate exponents span more than 256 binary orders");

            // Exact common power-of-two scaling preserves the orientation sign.
            for (double& coordinate : coordinates)
                coordinate = std::scalbn(coordinate, -maximum_exponent);
            return true;
        }

        Sign DeterminantSign(double determinant)
        {
            if (!std::isfinite(determinant))
                throw std::overflow_error("Orientation backend returned a non-finite result");
            if (determinant > 0.0)
                return Sign::Positive;
            if (determinant < 0.0)
                return Sign::Negative;
            return Sign::Zero;
        }
    }

    Sign Orient2DSign(const Point2& a, const Point2& b, const Point2& c)
    {
        std::array<double, 6> coordinates{
            a.cord_x, a.cord_y, b.cord_x, b.cord_y, c.cord_x, c.cord_y
        };
        if (!NormalizeCoordinates(coordinates))
            return Sign::Zero;

        InitializePredicates();
        return DeterminantSign(orient2d(coordinates.data(), coordinates.data() + 2, coordinates.data() + 4));
    }

    Sign Orient3DSign(const Point3& a, const Point3& b, const Point3& c, const Point3& point)
    {
        std::array<double, 12> coordinates{
            a.cord_x, a.cord_y, a.cord_z,
            b.cord_x, b.cord_y, b.cord_z,
            c.cord_x, c.cord_y, c.cord_z,
            point.cord_x, point.cord_y, point.cord_z
        };
        if (!NormalizeCoordinates(coordinates))
            return Sign::Zero;

        InitializePredicates();
        // The C routine has the opposite convention to ((b-a) x (c-a)) dot (point-a).
        return DeterminantSign(-orient3d(coordinates.data(), coordinates.data() + 3, coordinates.data() + 6, coordinates.data() + 9));
    }

    PlaneSideValues ScaledPlaneSideValues(const Segment3& segment, const Triangle3& plane)
    {
        std::array<double, 15> coordinates{
            plane.vertices[0].cord_x, plane.vertices[0].cord_y, plane.vertices[0].cord_z,
            plane.vertices[1].cord_x, plane.vertices[1].cord_y, plane.vertices[1].cord_z,
            plane.vertices[2].cord_x, plane.vertices[2].cord_y, plane.vertices[2].cord_z,
            segment.start.cord_x, segment.start.cord_y, segment.start.cord_z,
            segment.end.cord_x, segment.end.cord_y, segment.end.cord_z
        };

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            std::array<double, 5> column{};
            for (std::size_t point = 0; point < column.size(); ++point)
                column[point] = coordinates[3 * point + axis];
            NormalizeCoordinates(column);
            for (std::size_t point = 0; point < column.size(); ++point)
                coordinates[3 * point + axis] = column[point];
        }

        InitializePredicates();
        const double start = -orient3dexact(coordinates.data(), coordinates.data() + 3,
                                            coordinates.data() + 6, coordinates.data() + 9);
        const double end = -orient3dexact(coordinates.data(), coordinates.data() + 3,
                                          coordinates.data() + 6, coordinates.data() + 12);
        DeterminantSign(start);
        DeterminantSign(end);
        return {start, end};
    }
}
