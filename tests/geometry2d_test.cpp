#include "geometry2d.hpp"

#include <algorithm>
#include <array>
#include <cstddef>

#include <gtest/gtest.h>

namespace triangles
{
    TEST(Orient2DTest, PointToTheLeftHasPositiveOrientation)
    {
        EXPECT_DOUBLE_EQ(Orient2D({0.0, 0.0}, {2.0, 0.0}, {0.0, 1.0}), 2.0);
    }

    TEST(Orient2DTest, PointToTheRightHasNegativeOrientation)
    {
        EXPECT_DOUBLE_EQ(Orient2D({0.0, 0.0}, {2.0, 0.0}, {0.0, -1.0}), -2.0);
    }

    TEST(Orient2DTest, CollinearPointsHaveZeroOrientationEvenOutsideSegment)
    {
        const Point2 a{0.0, 0.0};
        const Point2 b{2.0, 0.0};

        EXPECT_DOUBLE_EQ(Orient2D(a, b, {1.0, 0.0}), 0.0);
        EXPECT_DOUBLE_EQ(Orient2D(a, b, {-1.0, 0.0}), 0.0);
        EXPECT_DOUBLE_EQ(Orient2D(a, b, {3.0, 0.0}), 0.0);
    }

    TEST(Orient2DTest, HandlesVerticalDirectedLine)
    {
        const Point2 a{0.0, 0.0};
        const Point2 b{0.0, 2.0};

        EXPECT_DOUBLE_EQ(Orient2D(a, b, {-1.0, 0.0}), 2.0);
        EXPECT_DOUBLE_EQ(Orient2D(a, b, {1.0, 0.0}), -2.0);
    }

    TEST(Orient2DTest, HandlesObliqueLineAwayFromOrigin)
    {
        EXPECT_DOUBLE_EQ(Orient2D({1.0, 2.0}, {4.0, 3.0}, {2.0, 6.0}), 11.0);
        EXPECT_DOUBLE_EQ(Orient2D({1.0, 2.0}, {4.0, 3.0}, {7.0, 4.0}), 0.0);
    }

    TEST(Orient2DTest, HandlesNegativeCoordinates)
    {
        EXPECT_DOUBLE_EQ(Orient2D({-5.0, -4.0}, {-1.0, -2.0}, {-4.0, 0.0}), 14.0);
    }

    TEST(Orient2DTest, HandlesFractionalCoordinates)
    {
        EXPECT_DOUBLE_EQ(Orient2D({0.25, 0.5}, {0.75, 0.5}, {0.25, 0.75}), 0.125);
    }

    TEST(Orient2DTest, RepeatedPointsHaveZeroOrientation)
    {
        const Point2 a{1.0, 2.0};
        const Point2 b{4.0, 3.0};

        EXPECT_DOUBLE_EQ(Orient2D(a, a, b), 0.0);
        EXPECT_DOUBLE_EQ(Orient2D(a, b, a), 0.0);
        EXPECT_DOUBLE_EQ(Orient2D(a, b, b), 0.0);
        EXPECT_DOUBLE_EQ(Orient2D(a, a, a), 0.0);
    }

    TEST(Orient2DTest, SwappingTwoPointsReversesSign)
    {
        const Point2 a{1.0, 2.0};
        const Point2 b{4.0, 3.0};
        const Point2 c{2.0, 6.0};
        const double orientation = Orient2D(a, b, c);

        EXPECT_DOUBLE_EQ(Orient2D(b, a, c), -orientation);
        EXPECT_DOUBLE_EQ(Orient2D(a, c, b), -orientation);
        EXPECT_DOUBLE_EQ(Orient2D(c, b, a), -orientation);
    }

    TEST(Orient2DTest, CyclicPermutationPreservesOrientation)
    {
        const Point2 a{1.0, 2.0};
        const Point2 b{4.0, 3.0};
        const Point2 c{2.0, 6.0};
        const double orientation = Orient2D(a, b, c);

        EXPECT_DOUBLE_EQ(Orient2D(b, c, a), orientation);
        EXPECT_DOUBLE_EQ(Orient2D(c, a, b), orientation);
    }

    TEST(Orient2DTest, TranslatingAllPointsPreservesOrientation)
    {
        const double original = Orient2D({1.0, 2.0}, {4.0, 3.0}, {2.0, 6.0});
        const double translated = Orient2D({9.0, -14.0}, {12.0, -13.0}, {10.0, -10.0});

        EXPECT_DOUBLE_EQ(translated, original);
    }

    TEST(Orient2DTest, DoublingCoordinatesQuadruplesSignedArea)
    {
        const double original = Orient2D({1.0, 2.0}, {4.0, 3.0}, {2.0, 6.0});
        const double scaled = Orient2D({2.0, 4.0}, {8.0, 6.0}, {4.0, 12.0});

        EXPECT_DOUBLE_EQ(scaled, 4.0 * original);
    }

    TEST(PointOnSegmentTest, AcceptsInteriorPoint)
    {
        EXPECT_TRUE(PointOnSegment({1.0, 1.0}, {0.0, 0.0}, {2.0, 2.0}));
    }

    TEST(PointOnSegmentTest, IncludesBothEndpoints)
    {
        const Point2 a{1.0, 2.0};
        const Point2 b{4.0, 6.0};

        EXPECT_TRUE(PointOnSegment(a, a, b));
        EXPECT_TRUE(PointOnSegment(b, a, b));
    }

    TEST(PointOnSegmentTest, RejectsCollinearPointsBeyondBothEnds)
    {
        const Point2 a{0.0, 0.0};
        const Point2 b{2.0, 2.0};

        EXPECT_FALSE(PointOnSegment({-1.0, -1.0}, a, b));
        EXPECT_FALSE(PointOnSegment({3.0, 3.0}, a, b));
    }

    TEST(PointOnSegmentTest, RejectsOffLinePointsInsideBoundingBox)
    {
        const Point2 a{0.0, 0.0};
        const Point2 b{2.0, 2.0};

        EXPECT_FALSE(PointOnSegment({0.5, 1.5}, a, b));
        EXPECT_FALSE(PointOnSegment({1.5, 0.5}, a, b));
    }

    TEST(PointOnSegmentTest, HandlesHorizontalSegment)
    {
        const Point2 a{0.0, 2.0};
        const Point2 b{4.0, 2.0};

        EXPECT_TRUE(PointOnSegment({2.0, 2.0}, a, b));
        EXPECT_FALSE(PointOnSegment({-1.0, 2.0}, a, b));
        EXPECT_FALSE(PointOnSegment({5.0, 2.0}, a, b));
        EXPECT_FALSE(PointOnSegment({2.0, 3.0}, a, b));
    }

    TEST(PointOnSegmentTest, HandlesVerticalSegment)
    {
        const Point2 a{2.0, 0.0};
        const Point2 b{2.0, 4.0};

        EXPECT_TRUE(PointOnSegment({2.0, 2.0}, a, b));
        EXPECT_FALSE(PointOnSegment({2.0, -1.0}, a, b));
        EXPECT_FALSE(PointOnSegment({2.0, 5.0}, a, b));
        EXPECT_FALSE(PointOnSegment({3.0, 2.0}, a, b));
    }

    TEST(PointOnSegmentTest, HandlesDescendingSegment)
    {
        const Point2 a{0.0, 4.0};
        const Point2 b{4.0, 0.0};

        EXPECT_TRUE(PointOnSegment({1.0, 3.0}, a, b));
        EXPECT_FALSE(PointOnSegment({1.0, 1.0}, a, b));
        EXPECT_FALSE(PointOnSegment({5.0, -1.0}, a, b));
    }

    TEST(PointOnSegmentTest, ReversingEndpointsPreservesMembership)
    {
        const Point2 a{1.0, 2.0};
        const Point2 b{3.0, 6.0};
        const Point2 inside{2.0, 4.0};
        const Point2 beyond{4.0, 8.0};
        const Point2 offLine{2.0, 3.0};

        EXPECT_TRUE(PointOnSegment(inside, a, b));
        EXPECT_TRUE(PointOnSegment(inside, b, a));
        EXPECT_FALSE(PointOnSegment(beyond, a, b));
        EXPECT_FALSE(PointOnSegment(beyond, b, a));
        EXPECT_FALSE(PointOnSegment(offLine, a, b));
        EXPECT_FALSE(PointOnSegment(offLine, b, a));
    }

    TEST(PointOnSegmentTest, HandlesNegativeAndFractionalCoordinates)
    {
        const Point2 a{-2.5, -1.5};
        const Point2 b{-0.5, 0.5};

        EXPECT_TRUE(PointOnSegment({-1.5, -0.5}, a, b));
        EXPECT_FALSE(PointOnSegment({0.5, 1.5}, a, b));
        EXPECT_FALSE(PointOnSegment({-1.5, 0.0}, a, b));
    }

    TEST(PointOnSegmentTest, ZeroLengthSegmentIncludesOnlyItsPoint)
    {
        const Point2 a{1.0, 2.0};

        EXPECT_TRUE(PointOnSegment(a, a, a));
        EXPECT_FALSE(PointOnSegment({2.0, 2.0}, a, a));
        EXPECT_FALSE(PointOnSegment({1.0, 3.0}, a, a));
        EXPECT_FALSE(PointOnSegment({2.0, 3.0}, a, a));
    }

    TEST (SegmentsIntersectTest, SimpleCase)
    {   
        const Segment2 ab{{0.0, 0.0}, {5.0, 5.0}};
        const Segment2 cd{{4.0, 2.0}, {3.0, 7.0}};

        EXPECT_TRUE(SegmentsIntersect(ab, cd));
    }


    namespace
    {
        void ExpectIntersectionInAnyOrder(const Segment2& lhs, const Segment2& rhs, bool expected)
        {
            const Segment2 lhsOrders[]{lhs, {lhs.end, lhs.start}};
            const Segment2 rhsOrders[]{rhs, {rhs.end, rhs.start}};

            for (const Segment2& first : lhsOrders)
            {
                for (const Segment2& second : rhsOrders)
                {
                    SCOPED_TRACE(::testing::Message()
                        << "first: (" << first.start.cord_x << ", " << first.start.cord_y
                        << ") -> (" << first.end.cord_x << ", " << first.end.cord_y
                        << "), second: (" << second.start.cord_x << ", " << second.start.cord_y
                        << ") -> (" << second.end.cord_x << ", " << second.end.cord_y << ")");
                    EXPECT_EQ(SegmentsIntersect(first, second), expected);
                    EXPECT_EQ(SegmentsIntersect(second, first), expected);
                }
            }
        }
    }

    TEST(SegmentsIntersectTest, HorizontalAndVerticalSegmentsCrossInsideBoth)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {4.0, 0.0}},
                                     {{2.0, -1.0}, {2.0, 1.0}}, true);
    }

    TEST(SegmentsIntersectTest, SeparatedParallelSegmentsDoNotIntersect)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {4.0, 0.0}},
                                     {{0.0, 1.0}, {4.0, 1.0}}, false);
    }

    TEST(SegmentsIntersectTest, CollinearSeparatedSegmentsDoNotIntersect)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {1.0, 0.0}},
                                     {{2.0, 0.0}, {3.0, 0.0}}, false);
    }

    TEST(SegmentsIntersectTest, CollinearSegmentsPartiallyOverlap)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {3.0, 0.0}},
                                     {{2.0, 0.0}, {5.0, 0.0}}, true);
    }

    TEST(SegmentsIntersectTest, OneSegmentIsContainedInTheOther)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {4.0, 4.0}},
                                     {{1.0, 1.0}, {3.0, 3.0}}, true);
    }

    TEST(SegmentsIntersectTest, CoincidentSegmentsIntersect)
    {
        ExpectIntersectionInAnyOrder({{-1.0, -2.0}, {3.0, 4.0}},
                                     {{-1.0, -2.0}, {3.0, 4.0}}, true);
    }

    TEST(SegmentsIntersectTest, NonCollinearSegmentsShareAnEndpoint)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {2.0, 0.0}},
                                     {{2.0, 0.0}, {3.0, 1.0}}, true);
    }

    TEST(SegmentsIntersectTest, CollinearSegmentsTouchAtAnEndpoint)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {2.0, 0.0}},
                                     {{2.0, 0.0}, {4.0, 0.0}}, true);
    }

    TEST(SegmentsIntersectTest, EndpointTouchesInteriorOfOtherSegment)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {4.0, 0.0}},
                                     {{2.0, 0.0}, {2.0, 2.0}}, true);
    }

    TEST(SegmentsIntersectTest, EndpointOnLineExtensionDoesNotImplyIntersection)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {2.0, 0.0}},
                                     {{3.0, 0.0}, {3.0, 1.0}}, false);
    }

    TEST(SegmentsIntersectTest, LinesCrossOutsideFirstSegment)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {1.0, 0.0}},
                                     {{2.0, -1.0}, {2.0, 1.0}}, false);
    }

    TEST(SegmentsIntersectTest, LinesCrossOutsideBothSegments)
    {
        ExpectIntersectionInAnyOrder({{0.0, 0.0}, {1.0, 0.0}},
                                     {{2.0, 1.0}, {2.0, 2.0}}, false);
    }

    TEST(SegmentsIntersectTest, HandlesNegativeAndFractionalCoordinates)
    {
        ExpectIntersectionInAnyOrder({{-1.5, -1.5}, {0.5, 0.5}},
                                     {{-1.5, 0.5}, {0.5, -1.5}}, true);
    }

    TEST(SegmentsIntersectTest, ZeroLengthSegmentInsideOtherSegmentIntersects)
    {
        ExpectIntersectionInAnyOrder({{1.0, 1.0}, {1.0, 1.0}},
                                     {{0.0, 0.0}, {2.0, 2.0}}, true);
    }

    TEST(SegmentsIntersectTest, ZeroLengthSegmentOnLineExtensionDoesNotIntersect)
    {
        ExpectIntersectionInAnyOrder({{3.0, 3.0}, {3.0, 3.0}},
                                     {{0.0, 0.0}, {2.0, 2.0}}, false);
    }

    TEST(SegmentsIntersectTest, ZeroLengthSegmentOffLineDoesNotIntersect)
    {
        ExpectIntersectionInAnyOrder({{1.0, 0.0}, {1.0, 0.0}},
                                     {{0.0, 0.0}, {2.0, 2.0}}, false);
    }

    TEST(SegmentsIntersectTest, CoincidentZeroLengthSegmentsIntersect)
    {
        ExpectIntersectionInAnyOrder({{1.0, 2.0}, {1.0, 2.0}},
                                     {{1.0, 2.0}, {1.0, 2.0}}, true);
    }

    TEST(SegmentsIntersectTest, DistinctZeroLengthSegmentsDoNotIntersect)
    {
        ExpectIntersectionInAnyOrder({{1.0, 2.0}, {1.0, 2.0}},
                                     {{3.0, 4.0}, {3.0, 4.0}}, false);
    }

    TEST (PointInTriangleTest, BaseCase)
    {   
        const Point2 point{-1.0, -1.0};
        const Triangle2 triangle{Point2{0.0, 0.0}, Point2{2.0, 0.0}, Point2{0.0, 2.0}};
        EXPECT_FALSE(PointInTriangle(point, triangle));
    }


    TEST(PointInTriangleTest, AcceptsInteriorPoint)
    {
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{2.0, 0.0}, Point2{0.0, 2.0}}};

        EXPECT_TRUE(PointInTriangle({0.5, 0.5}, triangle));
    }

    TEST(PointInTriangleTest, IncludesEachEdge)
    {
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{2.0, 0.0}, Point2{0.0, 2.0}}};

        EXPECT_TRUE(PointInTriangle({1.0, 0.0}, triangle));
        EXPECT_TRUE(PointInTriangle({1.0, 1.0}, triangle));
        EXPECT_TRUE(PointInTriangle({0.0, 1.0}, triangle));
    }

    TEST(PointInTriangleTest, IncludesEachVertex)
    {
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{2.0, 0.0}, Point2{0.0, 2.0}}};

        for (const Point2& vertex : triangle.vertices2)
            EXPECT_TRUE(PointInTriangle(vertex, triangle));
    }

    TEST(PointInTriangleTest, RejectsPointsAcrossEachEdge)
    {
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{2.0, 0.0}, Point2{0.0, 2.0}}};

        EXPECT_FALSE(PointInTriangle({0.5, -0.5}, triangle));
        EXPECT_FALSE(PointInTriangle({1.5, 1.5}, triangle));
        EXPECT_FALSE(PointInTriangle({-0.5, 0.5}, triangle));
    }

    TEST(PointInTriangleTest, RejectsPointsOnSideExtensions)
    {
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{2.0, 0.0}, Point2{0.0, 2.0}}};

        EXPECT_FALSE(PointInTriangle({-1.0, 0.0}, triangle));
        EXPECT_FALSE(PointInTriangle({3.0, 0.0}, triangle));
        EXPECT_FALSE(PointInTriangle({0.0, -1.0}, triangle));
        EXPECT_FALSE(PointInTriangle({0.0, 3.0}, triangle));
        EXPECT_FALSE(PointInTriangle({-1.0, 3.0}, triangle));
        EXPECT_FALSE(PointInTriangle({3.0, -1.0}, triangle));
    }

    TEST(PointInTriangleTest, SupportsClockwiseWinding)
    {
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{0.0, 2.0}, Point2{2.0, 0.0}}};

        EXPECT_TRUE(PointInTriangle({0.5, 0.5}, triangle));
        EXPECT_TRUE(PointInTriangle({1.0, 1.0}, triangle));
        EXPECT_TRUE(PointInTriangle({0.0, 0.0}, triangle));
        EXPECT_FALSE(PointInTriangle({1.5, 1.5}, triangle));
        EXPECT_FALSE(PointInTriangle({-1.0, -1.0}, triangle));
    }

    TEST(PointInTriangleTest, AllVertexPermutationsPreserveMembership)
    {
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{2.0, 0.0}, Point2{0.0, 2.0}}};
        std::array<std::size_t, 3> order{0, 1, 2};

        do
        {
            SCOPED_TRACE(::testing::Message() << "vertex order: "
                         << order[0] << ", " << order[1] << ", " << order[2]);
            const Triangle2 permuted{{triangle.vertices2[order[0]],
                                     triangle.vertices2[order[1]],
                                     triangle.vertices2[order[2]]}};

            EXPECT_TRUE(PointInTriangle({0.5, 0.5}, permuted));
            EXPECT_TRUE(PointInTriangle({1.0, 0.0}, permuted));
            EXPECT_TRUE(PointInTriangle({1.0, 1.0}, permuted));
            EXPECT_TRUE(PointInTriangle({0.0, 1.0}, permuted));
            for (const Point2& vertex : triangle.vertices2)
                EXPECT_TRUE(PointInTriangle(vertex, permuted));
            EXPECT_FALSE(PointInTriangle({1.5, 1.5}, permuted));
            EXPECT_FALSE(PointInTriangle({-1.0, -1.0}, permuted));
            EXPECT_FALSE(PointInTriangle({3.0, 0.0}, permuted));
        } while (std::next_permutation(order.begin(), order.end()));
    }

    TEST(PointInTriangleTest, HandlesTranslatedTriangleWithNegativeCoordinates)
    {
        const Triangle2 triangle{{Point2{-4.0, -3.0}, Point2{0.0, -3.0}, Point2{-4.0, 1.0}}};

        EXPECT_TRUE(PointInTriangle({-3.0, -2.0}, triangle));
        EXPECT_TRUE(PointInTriangle({-2.0, -1.0}, triangle));
        EXPECT_FALSE(PointInTriangle({-1.0, 0.0}, triangle));
    }

    TEST(PointInTriangleTest, HandlesTriangleWithObliqueSides)
    {
        const Triangle2 triangle{{Point2{0.0, 0.0}, Point2{4.0, 2.0}, Point2{2.0, 4.0}}};

        EXPECT_TRUE(PointInTriangle({2.0, 2.0}, triangle));
        EXPECT_TRUE(PointInTriangle({2.0, 1.0}, triangle));
        EXPECT_TRUE(PointInTriangle({3.0, 3.0}, triangle));
        EXPECT_TRUE(PointInTriangle({1.0, 2.0}, triangle));
        EXPECT_FALSE(PointInTriangle({3.0, 0.0}, triangle));
    }

    TEST(PointInTriangleTest, HandlesFractionalCoordinates)
    {
        const Triangle2 triangle{{Point2{0.25, 0.25}, Point2{1.25, 0.25}, Point2{0.25, 1.25}}};

        EXPECT_TRUE(PointInTriangle({0.5, 0.5}, triangle));
        EXPECT_TRUE(PointInTriangle({0.75, 0.75}, triangle));
        EXPECT_FALSE(PointInTriangle({1.0, 1.0}, triangle));
    }

}
