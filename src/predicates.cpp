#include "predicates.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

extern "C"
{
    void exactinit(void);
    double orient2d(double* a, double* b, double* c);
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
    }

    Sign Orient2DSign(const Point2& a, const Point2& b, const Point2& c)
    {
        const std::array<double, 6> coordinates{
            a.cord_x, a.cord_y, b.cord_x, b.cord_y, c.cord_x, c.cord_y
        };
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
            return Sign::Zero;
        if (maximum_exponent - minimum_exponent > maximum_exponent_spread)
            throw std::range_error("Orientation coordinate exponents span more than 256 binary orders");

        // Exact common power-of-two scaling keeps the C backend away from
        // overflow and underflow without changing the orientation sign.
        double pa[2]{std::scalbn(a.cord_x, -maximum_exponent),
                     std::scalbn(a.cord_y, -maximum_exponent)};
        double pb[2]{std::scalbn(b.cord_x, -maximum_exponent),
                     std::scalbn(b.cord_y, -maximum_exponent)};
        double pc[2]{std::scalbn(c.cord_x, -maximum_exponent),
                     std::scalbn(c.cord_y, -maximum_exponent)};

        InitializePredicates();
        const double determinant = orient2d(pa, pb, pc);
        if (!std::isfinite(determinant))
            throw std::overflow_error("Orientation backend returned a non-finite result");
        if (determinant > 0.0)
            return Sign::Positive;
        if (determinant < 0.0)
            return Sign::Negative;
        return Sign::Zero;
    }
}
