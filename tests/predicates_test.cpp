#include "predicates.hpp"
#include "geometry2d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace triangles
{
    namespace
    {
        void ExpectDifficultOrientationInEveryOrder(double n)
        {
            SCOPED_TRACE(::testing::Message() << "n=" << n);
            const Point2 a{0.0, 0.0};
            const Point2 b{n + 1.0, n};
            const Point2 c{n, n - 1.0};

            EXPECT_EQ(Orient2DSign(a, b, c), Sign::Negative);
            EXPECT_EQ(Orient2DSign(b, c, a), Sign::Negative);
            EXPECT_EQ(Orient2DSign(c, a, b), Sign::Negative);
            EXPECT_EQ(Orient2DSign(b, a, c), Sign::Positive);
            EXPECT_EQ(Orient2DSign(a, c, b), Sign::Positive);
            EXPECT_EQ(Orient2DSign(c, b, a), Sign::Positive);
        }

        void ExpectMembershipInEveryOrder(const Point2& point, const Triangle2& triangle, bool expected)
        {
            std::array<std::size_t, 3> order{0, 1, 2};
            do
            {
                SCOPED_TRACE(::testing::Message() << "vertex order: "
                             << order[0] << order[1] << order[2]);
                const Triangle2 permuted{{triangle.vertices2[order[0]],
                                          triangle.vertices2[order[1]],
                                          triangle.vertices2[order[2]]}};
                EXPECT_EQ(PointInTriangle(point, permuted), expected);
            } while (std::next_permutation(order.begin(), order.end()));
        }
    }

    TEST(Orient2DSignTest, DistinguishesLeftAndRight)
    {
        EXPECT_EQ(Orient2DSign({0.0, 0.0}, {2.0, 0.0}, {0.0, 1.0}), Sign::Positive);
        EXPECT_EQ(Orient2DSign({0.0, 0.0}, {2.0, 0.0}, {0.0, -1.0}), Sign::Negative);
    }

    TEST(Orient2DSignTest, RecognizesCollinearPointsOnLineAndExtensions)
    {
        EXPECT_EQ(Orient2DSign({0.0, 0.0}, {2.0, 2.0}, {1.0, 1.0}), Sign::Zero);
        EXPECT_EQ(Orient2DSign({0.0, 0.0}, {2.0, 2.0}, {-1.0, -1.0}), Sign::Zero);
        EXPECT_EQ(Orient2DSign({0.0, 0.0}, {2.0, 2.0}, {3.0, 3.0}), Sign::Zero);
    }

    TEST(Orient2DSignTest, RecognizesRepeatedPoints)
    {
        const Point2 a{1.0, 2.0};
        const Point2 b{3.0, 4.0};
        EXPECT_EQ(Orient2DSign(a, a, b), Sign::Zero);
        EXPECT_EQ(Orient2DSign(a, b, a), Sign::Zero);
        EXPECT_EQ(Orient2DSign(a, b, b), Sign::Zero);
        EXPECT_EQ(Orient2DSign({0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}), Sign::Zero);
    }

    TEST(Orient2DSignTest, HandlesCancellationAtTwoToThe27InEveryOrder)
    {
        ExpectDifficultOrientationInEveryOrder(134217728.0);
    }

    TEST(Orient2DSignTest, HandlesCancellationBeyondLongDoublePrecisionInEveryOrder)
    {
        ExpectDifficultOrientationInEveryOrder(1099511627776.0);
    }

    TEST(Orient2DSignTest, HandlesCancellationAtTwoToThe52InEveryOrder)
    {
        ExpectDifficultOrientationInEveryOrder(4503599627370496.0);
    }

    TEST(Orient2DSignTest, TranslationPreservesSign)
    {
        EXPECT_EQ(Orient2DSign({0.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}), Sign::Positive);
        EXPECT_EQ(Orient2DSign({16.0, -32.0}, {18.0, -32.0}, {16.0, -30.0}), Sign::Positive);
    }

    TEST(Orient2DSignTest, ExactPowerOfTwoRescalingPreservesSign)
    {
        for (const int exponent : {-900, -500, 0, 500, 900})
        {
            SCOPED_TRACE(exponent);
            const double scale = std::scalbn(1.0, exponent);
            EXPECT_EQ(Orient2DSign({0.0, 0.0}, {scale, 0.0}, {0.0, scale}), Sign::Positive);
        }
    }

    TEST(Orient2DSignTest, RescalesExtremeFiniteCoordinatesBeforeArithmetic)
    {
        const double maximum = std::numeric_limits<double>::max();
        EXPECT_EQ(Orient2DSign({-maximum, 0.0}, {maximum, 0.0}, {0.0, maximum}), Sign::Positive);
    }

    TEST(Orient2DSignTest, RescalesSubnormalCoordinatesBeforeArithmetic)
    {
        const double tiny = std::numeric_limits<double>::denorm_min();
        EXPECT_EQ(Orient2DSign({0.0, 0.0}, {tiny, 0.0}, {0.0, tiny}), Sign::Positive);
    }

    TEST(Orient2DSignTest, RejectsUnsupportedExponentSpread)
    {
        const double maximum = std::numeric_limits<double>::max();
        const double tiny = std::numeric_limits<double>::denorm_min();
        EXPECT_THROW(Orient2DSign({0.0, 0.0}, {maximum, 0.0}, {0.0, tiny}), std::range_error);
    }

    TEST(Orient2DSignTest, ChecksSupportedExponentSpreadBoundary)
    {
        EXPECT_EQ(Orient2DSign({0.0, 0.0}, {1.0, 0.0}, {0.0, std::scalbn(1.0, -256)}), Sign::Positive);
        EXPECT_THROW(Orient2DSign({0.0, 0.0}, {1.0, 0.0}, {0.0, std::scalbn(1.0, -257)}), std::range_error);
    }

    TEST(Orient2DSignTest, RejectsNaNInEveryPointAndCoordinate)
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const Point2 invalidPoints[]{{nan, 0.0}, {0.0, nan}};
        const Point2 a{0.0, 0.0};
        const Point2 b{2.0, 0.0};
        const Point2 c{0.0, 2.0};
        for (const Point2& invalid : invalidPoints)
        {
            EXPECT_THROW(Orient2DSign(invalid, b, c), std::invalid_argument);
            EXPECT_THROW(Orient2DSign(a, invalid, c), std::invalid_argument);
            EXPECT_THROW(Orient2DSign(a, b, invalid), std::invalid_argument);
        }
    }

    TEST(Orient2DSignTest, RejectsInfinityInEveryPointAndCoordinate)
    {
        const double infinity = std::numeric_limits<double>::infinity();
        const Point2 invalidPoints[]{{infinity, 0.0}, {-infinity, 0.0}, {0.0, infinity}, {0.0, -infinity}};
        const Point2 a{0.0, 0.0};
        const Point2 b{2.0, 0.0};
        const Point2 c{0.0, 2.0};
        for (const Point2& invalid : invalidPoints)
        {
            EXPECT_THROW(Orient2DSign(invalid, b, c), std::invalid_argument);
            EXPECT_THROW(Orient2DSign(a, invalid, c), std::invalid_argument);
            EXPECT_THROW(Orient2DSign(a, b, invalid), std::invalid_argument);
        }
    }

    TEST(Robust2DIntegrationTest, PointOnSegmentRejectsFalseCollinearity)
    {
        const double n = 1099511627776.0;
        const Point2 a{0.0, 0.0};
        const Point2 b{n + 1.0, n};
        const Point2 point{n, n - 1.0};
        EXPECT_FALSE(PointOnSegment(point, a, b));
        EXPECT_FALSE(PointOnSegment(point, b, a));
    }

    TEST(Robust2DIntegrationTest, SegmentsRejectNearlyCollinearGap)
    {
        const double n = 1099511627776.0;
        const Segment2 first{{0.0, 0.0}, {n + 1.0, n}};
        const Segment2 second{{n, n - 1.0}, {n + 1.0, n - 1.0}};
        EXPECT_FALSE(SegmentsIntersect(first, second));
        EXPECT_FALSE(SegmentsIntersect(second, first));
    }

    TEST(Robust2DIntegrationTest, DistinguishesCrossingAndParallelSegmentsAtTinyScale)
    {
        const double scale = std::scalbn(1.0, -900);
        const Segment2 first{{0.0, 0.0}, {scale, scale}};
        const Segment2 crossing{{0.0, scale}, {scale, 0.0}};
        const Segment2 parallel{{0.0, scale}, {scale, 2.0 * scale}};
        EXPECT_TRUE(SegmentsIntersect(first, crossing));
        EXPECT_FALSE(SegmentsIntersect(first, parallel));
    }

    TEST(Robust2DIntegrationTest, RejectsPointJustOutsideVeryThinTriangle)
    {
        const double n = 1099511627776.0;
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{n + 1.0, n}, Point2{0.0, 1.0}}};
        ExpectMembershipInEveryOrder({n, n - 1.0}, triangle, false);
    }

    TEST(Robust2DIntegrationTest, IncludesPointOnVeryThinTriangleEdge)
    {
        const double n = 1099511627776.0;
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{n + 1.0, n}, Point2{0.0, 1.0}}};
        ExpectMembershipInEveryOrder({(n + 1.0) / 2.0, n / 2.0}, triangle, true);
    }

    TEST(Robust2DIntegrationTest, RejectsTrianglesSeparatedByNearCollinearEdge)
    {
        const double n = 1099511627776.0;
        const Triangle2 first{{Point2{0.0, 0.0}, Point2{n + 1.0, n}, Point2{0.0, 1.0}}};
        const Triangle2 second{{Point2{n, n - 1.0}, Point2{n + 1.0, n - 1.0}, Point2{n + 1.0, n - 2.0}}};
        EXPECT_FALSE(TrianglesIntersect(first, second));
        EXPECT_FALSE(TrianglesIntersect(second, first));
    }

    TEST(Robust2DIntegrationTest, SmallGapIsNotReplacedByGeometricTolerance)
    {
        const double gap = std::scalbn(1.0, -100);
        const Triangle2 first{{Point2{0.0, 0.0}, Point2{2.0, 0.0}, Point2{0.0, 2.0}}};
        const Triangle2 second{{Point2{0.0, -gap}, Point2{2.0, -gap}, Point2{0.0, -2.0}}};
        EXPECT_FALSE(TrianglesIntersect(first, second));
        EXPECT_FALSE(TrianglesIntersect(second, first));
    }

    TEST(Robust2DIntegrationTest, SegmentTriangleRejectsNearlyCollinearGap)
    {
        const double n = 1099511627776.0;
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{n + 1.0, n}, Point2{0.0, 1.0}}};
        const Segment2 segment{{n, n - 1.0}, {n + 1.0, n - 1.0}};
        EXPECT_FALSE(SegmentIntersectsTriangle(segment, triangle));
        EXPECT_FALSE(SegmentIntersectsTriangle({segment.end, segment.start}, triangle));
    }
}
