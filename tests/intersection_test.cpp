#include "intersection.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace triangles
{
    namespace
    {
        void ExpectCoplanarIntersectionInAnyOrder(const Triangle3& lhs,
                                                 const Triangle3& rhs, bool expected)
        {
            std::array<std::size_t, 3> lhsOrder{0, 1, 2};
            do
            {
                const Triangle3 first{{lhs.vertices[lhsOrder[0]],
                                       lhs.vertices[lhsOrder[1]],
                                       lhs.vertices[lhsOrder[2]]}};
                std::array<std::size_t, 3> rhsOrder{0, 1, 2};
                do
                {
                    SCOPED_TRACE(::testing::Message()
                        << "first vertex order: " << lhsOrder[0] << lhsOrder[1] << lhsOrder[2]
                        << ", second vertex order: " << rhsOrder[0] << rhsOrder[1] << rhsOrder[2]);
                    const Triangle3 second{{rhs.vertices[rhsOrder[0]],
                                            rhs.vertices[rhsOrder[1]],
                                            rhs.vertices[rhsOrder[2]]}};

                    EXPECT_EQ(CoplanarTrianglesIntersect(first, second), expected);
                    EXPECT_EQ(CoplanarTrianglesIntersect(second, first), expected);
                } while (std::next_permutation(rhsOrder.begin(), rhsOrder.end()));
            } while (std::next_permutation(lhsOrder.begin(), lhsOrder.end()));
        }
    }

    TEST(CoplanarTrianglesIntersectTest, OverlapInHorizontalPlaneAwayFromOrigin)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 5.0}, Point3{4.0, 0.0, 5.0}, Point3{0.0, 4.0, 5.0}}};
        const Triangle3 second{{Point3{1.0, 1.0, 5.0}, Point3{5.0, 1.0, 5.0}, Point3{1.0, 5.0, 5.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, SeparatedTrianglesInHorizontalPlane)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 5.0}, Point3{2.0, 0.0, 5.0}, Point3{0.0, 2.0, 5.0}}};
        const Triangle3 second{{Point3{3.0, 3.0, 5.0}, Point3{5.0, 3.0, 5.0}, Point3{3.0, 5.0, 5.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, false);
    }

    TEST(CoplanarTrianglesIntersectTest, OverlapInVerticalPlaneWithConstantX)
    {
        const Triangle3 first{{Point3{5.0, 0.0, 0.0}, Point3{5.0, 4.0, 0.0}, Point3{5.0, 0.0, 4.0}}};
        const Triangle3 second{{Point3{5.0, 1.0, 1.0}, Point3{5.0, 5.0, 1.0}, Point3{5.0, 1.0, 5.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, SeparationInZIsPreservedInConstantXPlane)
    {
        const Triangle3 first{{Point3{5.0, 0.0, 0.0}, Point3{5.0, 2.0, 0.0}, Point3{5.0, 0.0, 2.0}}};
        const Triangle3 second{{Point3{5.0, 0.0, 3.0}, Point3{5.0, 2.0, 3.0}, Point3{5.0, 0.0, 5.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, false);
    }

    TEST(CoplanarTrianglesIntersectTest, OverlapInVerticalPlaneWithConstantY)
    {
        const Triangle3 first{{Point3{0.0, -3.0, 0.0}, Point3{4.0, -3.0, 0.0}, Point3{0.0, -3.0, 4.0}}};
        const Triangle3 second{{Point3{1.0, -3.0, 1.0}, Point3{5.0, -3.0, 1.0}, Point3{1.0, -3.0, 5.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, SeparationInZIsPreservedInConstantYPlane)
    {
        const Triangle3 first{{Point3{0.0, -3.0, 0.0}, Point3{2.0, -3.0, 0.0}, Point3{0.0, -3.0, 2.0}}};
        const Triangle3 second{{Point3{0.0, -3.0, 3.0}, Point3{2.0, -3.0, 3.0}, Point3{0.0, -3.0, 5.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, false);
    }

    TEST(CoplanarTrianglesIntersectTest, ObliquePlaneWithDominantZNormal)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 1.0}, Point3{0.0, 4.0, 1.0}}};
        const Triangle3 second{{Point3{1.0, 1.0, 0.5}, Point3{5.0, 1.0, 1.5}, Point3{1.0, 5.0, 1.5}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, ObliquePlaneWithDominantXNormal)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{1.0, 4.0, 0.0}, Point3{1.0, 0.0, 4.0}}};
        const Triangle3 second{{Point3{0.5, 1.0, 1.0}, Point3{1.5, 5.0, 1.0}, Point3{1.5, 1.0, 5.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, ObliquePlaneWithDominantYNormal)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 1.0, 0.0}, Point3{0.0, 1.0, 4.0}}};
        const Triangle3 second{{Point3{1.0, 0.5, 1.0}, Point3{5.0, 1.5, 1.0}, Point3{1.0, 1.5, 5.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, StrictContainmentInObliquePlane)
    {
        const Triangle3 outer{{Point3{0.0, 0.0, 0.0}, Point3{6.0, 0.0, -6.0}, Point3{0.0, 6.0, -6.0}}};
        const Triangle3 inner{{Point3{1.0, 1.0, -2.0}, Point3{2.0, 1.0, -3.0}, Point3{1.0, 2.0, -3.0}}};

        ExpectCoplanarIntersectionInAnyOrder(outer, inner, true);
    }

    TEST(CoplanarTrianglesIntersectTest, StarInObliquePlaneWithoutContainedVertices)
    {
        const Triangle3 first{{Point3{-2.0, -1.0, 3.0}, Point3{2.0, -1.0, -1.0}, Point3{0.0, 2.0, -2.0}}};
        const Triangle3 second{{Point3{-2.0, 1.0, 1.0}, Point3{2.0, 1.0, -3.0}, Point3{0.0, -2.0, 2.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, CoincidentTrianglesInObliquePlane)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, -2.0}, Point3{0.0, 2.0, -2.0}}};

        ExpectCoplanarIntersectionInAnyOrder(triangle, triangle, true);
    }

    TEST(CoplanarTrianglesIntersectTest, SingleSharedVertex)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{2.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{2.0, -2.0, 0.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, WholeSharedEdgeInObliquePlane)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 2.0}, Point3{0.0, 2.0, 2.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 2.0}, Point3{0.0, -2.0, -2.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, PartialSharedEdgeInVerticalPlane)
    {
        const Triangle3 first{{Point3{5.0, 0.0, 0.0}, Point3{5.0, 4.0, 0.0}, Point3{5.0, 0.0, 4.0}}};
        const Triangle3 second{{Point3{5.0, 1.0, 0.0}, Point3{5.0, 3.0, 0.0}, Point3{5.0, 1.0, -2.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, VertexTouchesInteriorOfEdgeInVerticalPlane)
    {
        const Triangle3 first{{Point3{0.0, -3.0, 0.0}, Point3{4.0, -3.0, 0.0}, Point3{0.0, -3.0, 4.0}}};
        const Triangle3 second{{Point3{2.0, -3.0, 0.0}, Point3{1.0, -3.0, -2.0}, Point3{3.0, -3.0, -2.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, true);
    }

    TEST(CoplanarTrianglesIntersectTest, SideExtensionsDoNotCountAsIntersection)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 5.0}, Point3{2.0, 0.0, 5.0}, Point3{0.0, 2.0, 5.0}}};
        const Triangle3 second{{Point3{3.0, 0.0, 5.0}, Point3{4.0, 0.0, 5.0}, Point3{3.0, 1.0, 5.0}}};

        ExpectCoplanarIntersectionInAnyOrder(first, second, false);
    }

    namespace
    {
        void ExpectSegmentPlaneIntersectionInAnyOrder(const Segment3& segment,
                                                     const Triangle3& plane,
                                                     const Point3& expected)
        {
            std::array<std::size_t, 3> order{0, 1, 2};
            const Segment3 directions[]{segment, {segment.end, segment.start}};
            do
            {
                SCOPED_TRACE(::testing::Message() << "plane vertex order: "
                             << order[0] << order[1] << order[2]);
                const Triangle3 permuted{{plane.vertices[order[0]],
                                          plane.vertices[order[1]],
                                          plane.vertices[order[2]]}};
                for (const Segment3& direction : directions)
                {
                    SCOPED_TRACE(::testing::Message()
                        << "segment starts at: " << direction.start.cord_x << ", "
                        << direction.start.cord_y << ", " << direction.start.cord_z);
                    const Point3 result = SegmentPlaneIntersection(direction, permuted);
                    EXPECT_NEAR(result.cord_x, expected.cord_x, 1e-12);
                    EXPECT_NEAR(result.cord_y, expected.cord_y, 1e-12);
                    EXPECT_NEAR(result.cord_z, expected.cord_z, 1e-12);
                }
            } while (std::next_permutation(order.begin(), order.end()));
        }
    }

    TEST(SegmentPlaneIntersectionTest, FindsAsymmetricIntersection)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{0.0, 0.0, 2.0}, {3.0, 0.0, -1.0}},
                                                 plane, {2.0, 0.0, 0.0});
    }

    TEST(SegmentPlaneIntersectionTest, FindsMidpointWhenDistancesAreEqual)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{0.0, 2.0, 4.0}, {4.0, 6.0, -4.0}},
                                                 plane, {2.0, 4.0, 0.0});
    }

    TEST(SegmentPlaneIntersectionTest, HandlesPlaneAwayFromOrigin)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 5.0}, Point3{2.0, 0.0, 5.0}, Point3{0.0, 2.0, 5.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{1.0, 2.0, 8.0}, {5.0, 6.0, 4.0}},
                                                 plane, {4.0, 5.0, 5.0});
    }

    TEST(SegmentPlaneIntersectionTest, HandlesVerticalPlaneWithConstantX)
    {
        const Triangle3 plane{{Point3{4.0, 0.0, 0.0}, Point3{4.0, 2.0, 0.0}, Point3{4.0, 0.0, 2.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{7.0, 1.0, 2.0}, {3.0, 5.0, 6.0}},
                                                 plane, {4.0, 4.0, 5.0});
    }

    TEST(SegmentPlaneIntersectionTest, HandlesVerticalPlaneWithConstantY)
    {
        const Triangle3 plane{{Point3{0.0, -3.0, 0.0}, Point3{0.0, -3.0, 2.0}, Point3{2.0, -3.0, 0.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{1.0, -1.0, 2.0}, {5.0, -5.0, 6.0}},
                                                 plane, {3.0, -3.0, 4.0});
    }

    TEST(SegmentPlaneIntersectionTest, HandlesObliquePlane)
    {
        const Triangle3 plane{{Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}, Point3{0.0, 0.0, 1.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}},
                                                 plane, {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0});
    }

    TEST(SegmentPlaneIntersectionTest, IntersectionCanLieOutsidePlaneTriangle)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{10.0, 10.0, 2.0}, {13.0, 10.0, -1.0}},
                                                 plane, {12.0, 10.0, 0.0});
    }

    TEST(SegmentPlaneIntersectionTest, HandlesLargeOppositeDistancesWithoutOverflow)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{0.0, 0.0, 1e308}, {4.0, 0.0, -1e308}},
                                                 plane, {2.0, 0.0, 0.0});
    }

    TEST(SegmentPlaneIntersectionTest, RejectsEndpointsOnSameSide)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Segment3 above{{0.0, 0.0, 1.0}, {1.0, 1.0, 2.0}};
        const Segment3 below{{0.0, 0.0, -1.0}, {1.0, 1.0, -2.0}};

        EXPECT_THROW(SegmentPlaneIntersection(above, plane), std::invalid_argument);
        EXPECT_THROW(SegmentPlaneIntersection(below, plane), std::invalid_argument);
    }

    TEST(SegmentPlaneIntersectionTest, RejectsEndpointOnPlaneUnderStrictCrossingContract)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Segment3 startsOnPlane{{0.0, 0.0, 0.0}, {1.0, 1.0, 2.0}};
        const Segment3 endsOnPlane{{1.0, 1.0, 2.0}, {0.0, 0.0, 0.0}};

        EXPECT_THROW(SegmentPlaneIntersection(startsOnPlane, plane), std::invalid_argument);
        EXPECT_THROW(SegmentPlaneIntersection(endsOnPlane, plane), std::invalid_argument);
    }

    TEST(SegmentPlaneIntersectionTest, RejectsCoplanarSegment)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Segment3 segment{{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}};

        EXPECT_THROW(SegmentPlaneIntersection(segment, plane), std::invalid_argument);
    }

    TEST(SegmentPlaneIntersectionTest, RejectsDegeneratePlaneTriangle)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}}};
        const Segment3 segment{{0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}};

        EXPECT_THROW(SegmentPlaneIntersection(segment, plane), std::domain_error);
    }

    TEST(SegmentPlaneIntersectionTest, RejectsNonFiniteDistance)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Segment3 segment{{0.0, 0.0, std::numeric_limits<double>::infinity()}, {0.0, 0.0, -1.0}};

        EXPECT_THROW(SegmentPlaneIntersection(segment, plane), std::invalid_argument);
    }


    namespace
    {
        bool SectionPointsAreNear(const Point3& lhs, const Point3& rhs)
        {
            return std::abs(lhs.cord_x - rhs.cord_x) <= 1e-12
                && std::abs(lhs.cord_y - rhs.cord_y) <= 1e-12
                && std::abs(lhs.cord_z - rhs.cord_z) <= 1e-12;
        }

        void ExpectTrianglePlaneSectionInAnyOrder(const Triangle3& triangle,
                                                 const Triangle3& plane,
                                                 const std::optional<Segment3>& expected)
        {
            std::array<std::size_t, 3> triangleOrder{0, 1, 2};
            do
            {
                const Triangle3 permutedTriangle{{triangle.vertices[triangleOrder[0]],
                                                 triangle.vertices[triangleOrder[1]],
                                                 triangle.vertices[triangleOrder[2]]}};
                std::array<std::size_t, 3> planeOrder{0, 1, 2};
                do
                {
                    SCOPED_TRACE(::testing::Message()
                        << "triangle vertex order: " << triangleOrder[0] << triangleOrder[1] << triangleOrder[2]
                        << ", plane vertex order: " << planeOrder[0] << planeOrder[1] << planeOrder[2]);
                    const Triangle3 permutedPlane{{plane.vertices[planeOrder[0]],
                                                  plane.vertices[planeOrder[1]],
                                                  plane.vertices[planeOrder[2]]}};
                    const auto actual = TrianglePlaneIntersection(permutedTriangle, permutedPlane);
                    ASSERT_EQ(actual.has_value(), expected.has_value());
                    if (actual)
                    {
                        const bool sameOrder = SectionPointsAreNear(actual->start, expected->start)
                                            && SectionPointsAreNear(actual->end, expected->end);
                        const bool reversedOrder = SectionPointsAreNear(actual->start, expected->end)
                                                && SectionPointsAreNear(actual->end, expected->start);
                        EXPECT_TRUE(sameOrder || reversedOrder)
                            << "actual section: (" << actual->start.cord_x << ", "
                            << actual->start.cord_y << ", " << actual->start.cord_z
                            << ") -> (" << actual->end.cord_x << ", "
                            << actual->end.cord_y << ", " << actual->end.cord_z << ")";
                    }
                } while (std::next_permutation(planeOrder.begin(), planeOrder.end()));
            } while (std::next_permutation(triangleOrder.begin(), triangleOrder.end()));
        }
    }

    TEST(TrianglePlaneIntersectionTest, TriangleStrictlyAbovePlaneHasNoSection)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 1.0}, Point3{2.0, 0.0, 2.0}, Point3{0.0, 2.0, 3.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane, std::nullopt);
    }

    TEST(TrianglePlaneIntersectionTest, TriangleStrictlyBelowPlaneHasNoSection)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, -1.0}, Point3{2.0, 0.0, -2.0}, Point3{0.0, 2.0, -3.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane, std::nullopt);
    }

    TEST(TrianglePlaneIntersectionTest, CrossingProducesTwoEdgeIntersectionPoints)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 2.0}, Point3{3.0, 0.0, -1.0}, Point3{0.0, 3.0, 2.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{2.0, 0.0, 0.0}, {2.0, 1.0, 0.0}});
    }

    TEST(TrianglePlaneIntersectionTest, VertexTouchFromAboveProducesZeroLengthSegment)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{1.0, 1.0, 0.0}, Point3{3.0, 1.0, 2.0}, Point3{1.0, 3.0, 3.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}});
    }

    TEST(TrianglePlaneIntersectionTest, VertexTouchFromBelowProducesZeroLengthSegment)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{1.0, 1.0, 0.0}, Point3{3.0, 1.0, -2.0}, Point3{1.0, 3.0, -3.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}});
    }

    TEST(TrianglePlaneIntersectionTest, EdgeOnPlaneWithThirdVertexAbove)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 3.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});
    }

    TEST(TrianglePlaneIntersectionTest, EdgeOnPlaneWithThirdVertexBelow)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, -3.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});
    }

    TEST(TrianglePlaneIntersectionTest, VertexOnPlaneAndOtherVerticesOnOppositeSides)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, -2.0}, Point3{0.0, 2.0, 2.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}});
    }

    TEST(TrianglePlaneIntersectionTest, HandlesHorizontalPlaneAwayFromOrigin)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 5.0}, Point3{2.0, 0.0, 5.0}, Point3{0.0, 2.0, 5.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 7.0}, Point3{3.0, 0.0, 4.0}, Point3{0.0, 3.0, 7.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{2.0, 0.0, 5.0}, {2.0, 1.0, 5.0}});
    }

    TEST(TrianglePlaneIntersectionTest, HandlesVerticalPlaneWithConstantX)
    {
        const Triangle3 plane{{Point3{4.0, 0.0, 0.0}, Point3{4.0, 2.0, 0.0}, Point3{4.0, 0.0, 2.0}}};
        const Triangle3 triangle{{Point3{6.0, 0.0, 0.0}, Point3{3.0, 3.0, 0.0}, Point3{6.0, 0.0, 3.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{4.0, 2.0, 0.0}, {4.0, 2.0, 1.0}});
    }

    TEST(TrianglePlaneIntersectionTest, HandlesVerticalPlaneWithConstantY)
    {
        const Triangle3 plane{{Point3{0.0, -3.0, 0.0}, Point3{2.0, -3.0, 0.0}, Point3{0.0, -3.0, 2.0}}};
        const Triangle3 triangle{{Point3{0.0, -1.0, 0.0}, Point3{3.0, -4.0, 0.0}, Point3{0.0, -1.0, 3.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{2.0, -3.0, 0.0}, {2.0, -3.0, 1.0}});
    }

    TEST(TrianglePlaneIntersectionTest, HandlesObliquePlane)
    {
        const Triangle3 plane{{Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}, Point3{0.0, 0.0, 1.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}, Point3{2.0, 0.0, 0.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}, {1.0, 0.0, 0.0}});
    }

    TEST(TrianglePlaneIntersectionTest, SectionCanLieOutsideFinitePlaneTriangle)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{10.0, 10.0, 2.0}, Point3{13.0, 10.0, -1.0}, Point3{10.0, 13.0, 2.0}}};

        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{12.0, 10.0, 0.0}, {12.0, 11.0, 0.0}});
    }

    TEST(TrianglePlaneIntersectionTest, RejectsFullyCoplanarTriangle)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{1.0, 1.0, 0.0}, Point3{3.0, 1.0, 0.0}, Point3{1.0, 3.0, 0.0}}};

        EXPECT_THROW(TrianglePlaneIntersection(triangle, plane), std::invalid_argument);
    }

    TEST(TrianglePlaneIntersectionTest, RejectsDegeneratePlaneTriangle)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 1.0}, Point3{2.0, 0.0, -1.0}, Point3{0.0, 2.0, 1.0}}};

        EXPECT_THROW(TrianglePlaneIntersection(triangle, plane), std::domain_error);
    }

    TEST(TrianglePlaneIntersectionTest, RejectsNonFiniteDistance)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, std::numeric_limits<double>::infinity()},
                                  Point3{2.0, 0.0, -1.0}, Point3{0.0, 2.0, 1.0}}};

        EXPECT_THROW(TrianglePlaneIntersection(triangle, plane), std::invalid_argument);
    }


    namespace
    {
        void Expect3DTriangleIntersectionInAnyOrder(const Triangle3& lhs,
                                                  const Triangle3& rhs, bool expected)
        {
            std::array<std::size_t, 3> lhsOrder{0, 1, 2};
            do
            {
                const Triangle3 first{{lhs.vertices[lhsOrder[0]],
                                       lhs.vertices[lhsOrder[1]],
                                       lhs.vertices[lhsOrder[2]]}};
                std::array<std::size_t, 3> rhsOrder{0, 1, 2};
                do
                {
                    SCOPED_TRACE(::testing::Message()
                        << "first vertex order: " << lhsOrder[0] << lhsOrder[1] << lhsOrder[2]
                        << ", second vertex order: " << rhsOrder[0] << rhsOrder[1] << rhsOrder[2]);
                    const Triangle3 second{{rhs.vertices[rhsOrder[0]],
                                            rhs.vertices[rhsOrder[1]],
                                            rhs.vertices[rhsOrder[2]]}};

                    EXPECT_EQ(TrianglesIntersect(first, second), expected);
                    EXPECT_EQ(TrianglesIntersect(second, first), expected);
                } while (std::next_permutation(rhsOrder.begin(), rhsOrder.end()));
            } while (std::next_permutation(lhsOrder.begin(), lhsOrder.end()));
        }
    }

    TEST(TrianglesIntersect3DTest, CoplanarTrianglesOverlap)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 5.0}, Point3{4.0, 0.0, 5.0}, Point3{0.0, 4.0, 5.0}}};
        const Triangle3 second{{Point3{1.0, 1.0, 5.0}, Point3{5.0, 1.0, 5.0}, Point3{1.0, 5.0, 5.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, CoplanarTrianglesAreSeparated)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 5.0}, Point3{2.0, 0.0, 5.0}, Point3{0.0, 2.0, 5.0}}};
        const Triangle3 second{{Point3{3.0, 3.0, 5.0}, Point3{5.0, 3.0, 5.0}, Point3{3.0, 5.0, 5.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, false);
    }

    TEST(TrianglesIntersect3DTest, CoplanarContainment)
    {
        const Triangle3 outer{{Point3{0.0, 0.0, 0.0}, Point3{6.0, 0.0, -6.0}, Point3{0.0, 6.0, -6.0}}};
        const Triangle3 inner{{Point3{1.0, 1.0, -2.0}, Point3{2.0, 1.0, -3.0}, Point3{1.0, 2.0, -3.0}}};
        Expect3DTriangleIntersectionInAnyOrder(outer, inner, true);
    }

    TEST(TrianglesIntersect3DTest, CoplanarOverlapInVerticalPlane)
    {
        const Triangle3 first{{Point3{5.0, 0.0, 0.0}, Point3{5.0, 4.0, 0.0}, Point3{5.0, 0.0, 4.0}}};
        const Triangle3 second{{Point3{5.0, 1.0, 1.0}, Point3{5.0, 5.0, 1.0}, Point3{5.0, 1.0, 5.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, CoplanarSeparationInVerticalPlane)
    {
        const Triangle3 first{{Point3{5.0, 0.0, 0.0}, Point3{5.0, 2.0, 0.0}, Point3{5.0, 0.0, 2.0}}};
        const Triangle3 second{{Point3{5.0, 0.0, 3.0}, Point3{5.0, 2.0, 3.0}, Point3{5.0, 0.0, 5.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, false);
    }

    TEST(TrianglesIntersect3DTest, ParallelPlanesWithIdenticalXYProjectionsDoNotIntersect)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, 5.0}, Point3{2.0, 0.0, 5.0}, Point3{0.0, 2.0, 5.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, false);
    }

    TEST(TrianglesIntersect3DTest, NonCoplanarSectionInsideTriangle)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        const Triangle3 second{{Point3{1.0, 1.0, 2.0}, Point3{1.0, 1.0, -2.0}, Point3{3.0, 1.0, 2.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, SectionCrossesTriangleWithBothEndpointsOutside)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{-1.0, 0.5, -1.0}, Point3{-1.0, 0.5, 1.0}, Point3{5.0, 0.5, 1.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, PlaneCrossingAndOverlappingProjectionsAreInsufficient)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        const Triangle3 second{{Point3{0.5, 0.5, 1.0}, Point3{1.0, 0.5, 1.0}, Point3{10.0, 10.0, -1.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, false);
    }

    TEST(TrianglesIntersect3DTest, NonCoplanarTrianglesShareSingleVertex)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, 0.0}, Point3{-1.0, 0.0, 1.0}, Point3{0.0, -1.0, 1.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, NonCoplanarTrianglesShareWholeEdge)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 0.0, 2.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, NonCoplanarTrianglesSharePartOfEdge)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        const Triangle3 second{{Point3{1.0, 0.0, 0.0}, Point3{3.0, 0.0, 0.0}, Point3{1.0, 0.0, 2.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, VertexTouchesOtherTriangleEdge)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        const Triangle3 second{{Point3{2.0, 0.0, 0.0}, Point3{2.0, -1.0, 1.0}, Point3{3.0, -1.0, 1.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, VertexTouchesOtherTriangleInterior)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        const Triangle3 second{{Point3{1.0, 1.0, 0.0}, Point3{2.0, 1.0, 1.0}, Point3{1.0, 2.0, 1.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, VertexTouchesPlaneOutsideOtherTriangle)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        const Triangle3 second{{Point3{5.0, 5.0, 0.0}, Point3{6.0, 5.0, 1.0}, Point3{5.0, 6.0, 1.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, false);
    }

    TEST(TrianglesIntersect3DTest, NonCoplanarIntersectionWithObliqueFirstPlane)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 4.0}, Point3{0.0, 4.0, 4.0}}};
        const Triangle3 second{{Point3{1.0, 1.0, 4.0}, Point3{1.0, 1.0, 0.0}, Point3{3.0, 1.0, 6.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, TwoDifferentVerticalPlanesIntersect)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}, Point3{0.0, 0.0, 4.0}}};
        const Triangle3 second{{Point3{-2.0, 1.0, 1.0}, Point3{2.0, 1.0, 1.0}, Point3{2.0, 1.0, 3.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    TEST(TrianglesIntersect3DTest, IntersectingVerticalPlanesDoNotImplyTriangleIntersection)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}, Point3{0.0, 0.0, 4.0}}};
        const Triangle3 second{{Point3{-2.0, 1.0, 5.0}, Point3{2.0, 1.0, 5.0}, Point3{2.0, 1.0, 7.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, false);
    }


    TEST(TrianglesIntersect3DTest, SmallPositiveGapDoesNotBecomeCoplanar)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, 1e-14}, Point3{2.0, 0.0, 1e-14}, Point3{0.0, 2.0, 1e-14}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, false);
    }


    TEST(FindIntersectionsBruteForceTest, EmptyInputProducesEmptyMask)
    {
        EXPECT_TRUE(FindIntersectionsBruteForce({}).empty());
    }

    TEST(FindIntersectionsBruteForceTest, SingleTriangleIsNotComparedWithItself)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const std::vector<bool> expected{false};

        EXPECT_EQ(FindIntersectionsBruteForce({triangle}), expected);
    }

    TEST(FindIntersectionsBruteForceTest, MarksBothTrianglesOfIntersectingPair)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        const Triangle3 second{{Point3{1.0, 1.0, 2.0}, Point3{1.0, 1.0, -2.0}, Point3{3.0, 1.0, 2.0}}};
        const std::vector<bool> expected{true, true};

        EXPECT_EQ(FindIntersectionsBruteForce({first, second}), expected);
    }

    TEST(FindIntersectionsBruteForceTest, SeparatedPairRemainsUnmarked)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{3.0, 3.0, 0.0}, Point3{5.0, 3.0, 0.0}, Point3{3.0, 5.0, 0.0}}};
        const std::vector<bool> expected{false, false};

        EXPECT_EQ(FindIntersectionsBruteForce({first, second}), expected);
    }

    TEST(FindIntersectionsBruteForceTest, PreservesIsolatedTriangleBeforeIntersectingPair)
    {
        const Triangle3 isolated{{Point3{10.0, 10.0, 0.0}, Point3{12.0, 10.0, 0.0}, Point3{10.0, 12.0, 0.0}}};
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{1.0, 0.0, 0.0}, Point3{3.0, 0.0, 0.0}, Point3{1.0, 2.0, 0.0}}};
        const std::vector<bool> expected{false, true, true};

        EXPECT_EQ(FindIntersectionsBruteForce({isolated, first, second}), expected);
    }

    TEST(FindIntersectionsBruteForceTest, AllSeparatedTrianglesRemainUnmarked)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, 2.0}, Point3{2.0, 0.0, 2.0}, Point3{0.0, 2.0, 2.0}}};
        const Triangle3 third{{Point3{0.0, 0.0, 4.0}, Point3{2.0, 0.0, 4.0}, Point3{0.0, 2.0, 4.0}}};
        const std::vector<bool> expected{false, false, false};

        EXPECT_EQ(FindIntersectionsBruteForce({first, second, third}), expected);
    }

    TEST(FindIntersectionsBruteForceTest, ContinuesAfterFirstMatchToMarkLaterTriangles)
    {
        const Triangle3 center{{Point3{0.0, 0.0, 0.0}, Point3{10.0, 0.0, 0.0}, Point3{0.0, 10.0, 0.0}}};
        const Triangle3 first{{Point3{1.0, 1.0, -1.0}, Point3{1.0, 1.0, 1.0}, Point3{2.0, 1.0, 1.0}}};
        const Triangle3 second{{Point3{1.0, 7.0, -1.0}, Point3{1.0, 7.0, 1.0}, Point3{2.0, 7.0, 1.0}}};
        const std::vector<bool> expected{true, true, true};

        ASSERT_FALSE(TrianglesIntersect(first, second));
        EXPECT_EQ(FindIntersectionsBruteForce({center, first, second}), expected);
    }

    TEST(FindIntersectionsBruteForceTest, HandlesSeparateIntersectingGroups)
    {
        const Triangle3 a{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 b{{Point3{1.0, 0.0, 0.0}, Point3{3.0, 0.0, 0.0}, Point3{1.0, 2.0, 0.0}}};
        const Triangle3 c{{Point3{10.0, 10.0, 0.0}, Point3{12.0, 10.0, 0.0}, Point3{10.0, 12.0, 0.0}}};
        const Triangle3 d{{Point3{11.0, 10.0, 0.0}, Point3{13.0, 10.0, 0.0}, Point3{11.0, 12.0, 0.0}}};
        const Triangle3 isolated{{Point3{0.0, 0.0, 5.0}, Point3{2.0, 0.0, 5.0}, Point3{0.0, 2.0, 5.0}}};
        const std::vector<bool> expected{true, true, true, true, false};

        EXPECT_EQ(FindIntersectionsBruteForce({a, b, c, d, isolated}), expected);
    }

    TEST(FindIntersectionsBruteForceTest, DuplicateTrianglesAtDifferentIndicesIntersect)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const std::vector<bool> expected{true, true, true};

        EXPECT_EQ(FindIntersectionsBruteForce({triangle, triangle, triangle}), expected);
    }

    TEST(FindIntersectionsBruteForceTest, TouchingTrianglesAreMarked)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, 0.0}, Point3{-1.0, 0.0, 1.0}, Point3{0.0, -1.0, 1.0}}};
        const std::vector<bool> expected{true, true};

        EXPECT_EQ(FindIntersectionsBruteForce({first, second}), expected);
    }

    TEST(FindIntersectionsBruteForceTest, PermutingInputPreservesMaskAssociation)
    {
        const std::array<Triangle3, 4> triangles{{
            Triangle3{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}},
            Triangle3{{Point3{1.0, 0.0, 0.0}, Point3{3.0, 0.0, 0.0}, Point3{1.0, 2.0, 0.0}}},
            Triangle3{{Point3{10.0, 10.0, 0.0}, Point3{12.0, 10.0, 0.0}, Point3{10.0, 12.0, 0.0}}},
            Triangle3{{Point3{0.0, 0.0, 5.0}, Point3{2.0, 0.0, 5.0}, Point3{0.0, 2.0, 5.0}}}
        }};
        const std::array<bool, 4> originalMask{true, true, false, false};
        std::array<std::size_t, 4> order{0, 1, 2, 3};
        do
        {
            SCOPED_TRACE(::testing::Message() << "input order: "
                         << order[0] << order[1] << order[2] << order[3]);
            std::vector<Triangle3> permuted;
            std::vector<bool> expected;
            for (const std::size_t index : order)
            {
                permuted.push_back(triangles[index]);
                expected.push_back(originalMask[index]);
            }
            EXPECT_EQ(FindIntersectionsBruteForce(permuted), expected);
        } while (std::next_permutation(order.begin(), order.end()));
    }


    TEST(Robust3DIntegrationTest, RejectsFalseCoplanarityBeyondLongDoublePrecision)
    {
        const double n = 1099511627776.0;
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 1.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, 1.0}, Point3{0.0, 0.0, 2.0}, Point3{n + 1.0, n, 1.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, false);
    }

    TEST(Robust3DIntegrationTest, RecognizesCoplanarContainmentWithLargeCoordinates)
    {
        const double n = 1099511627776.0;
        const Triangle3 outer{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 1.0}}};
        const Triangle3 inner{{Point3{n / 2.0 + 0.25, n / 2.0 - 0.25, 0.25},
                                Point3{3.0 * n / 4.0 + 0.5, 3.0 * n / 4.0 - 0.25, 0.25},
                                Point3{3.0 * n / 4.0 + 0.25, 3.0 * n / 4.0 - 0.5, 0.5}}};
        Expect3DTriangleIntersectionInAnyOrder(outer, inner, true);
    }

    TEST(Robust3DIntegrationTest, SmallPositiveGapRemainsSeparated)
    {
        const double gap = std::scalbn(1.0, -100);
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, gap}, Point3{2.0, 0.0, gap}, Point3{0.0, 2.0, gap}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, false);
    }

    TEST(SegmentPlaneIntersectionTest, RecoversCrossingWhenRoundedNormalLosesComponent)
    {
        const double n = 1099511627776.0;
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 1.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}},
                                                 plane, {0.0, 0.0, 0.0});
    }

    TEST(SegmentPlaneIntersectionTest, RecoversAsymmetricCrossingNearCancellation)
    {
        for (const int exponent : {27, 40, 52})
        {
            SCOPED_TRACE(exponent);
            const double n = std::scalbn(1.0, exponent);
            const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 1.0}}};
            ExpectSegmentPlaneIntersectionInAnyOrder({{0.0, 0.0, 3.0}, {0.0, 0.0, -1.0}},
                                                     plane, {0.0, 0.0, 0.0});
        }
    }

    TEST(SegmentPlaneIntersectionTest, UsesSharedScaleForBothEndpointValues)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{0.0, 0.0, 8.0}, {9.0, 0.0, -1.0}},
                                                 plane, {8.0, 0.0, 0.0});
    }

    TEST(SegmentPlaneIntersectionTest, ThinNondegenerateTriangleStillDefinesPlane)
    {
        const double n = 1099511627776.0;
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 0.0}}};
        ExpectSegmentPlaneIntersectionInAnyOrder({{0.0, 0.0, 2.0}, {3.0, 0.0, -1.0}},
                                                 plane, {2.0, 0.0, 0.0});
    }

    TEST(SegmentPlaneIntersectionTest, PreservesSmallWeightNearEndpoint)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}}};
        const Segment3 segment{{1e50, 0.0, 1e16}, {0.0, 0.0, -1.0}};
        for (const Segment3& direction : {segment, Segment3{segment.end, segment.start}})
        {
            const Point3 point = SegmentPlaneIntersection(direction, plane);
            EXPECT_TRUE(std::isfinite(point.cord_x));
            EXPECT_NEAR(point.cord_x / 1e34, 1.0, 1e-14);
            EXPECT_DOUBLE_EQ(point.cord_y, 0.0);
            EXPECT_NEAR(point.cord_z, 0.0, 1e-12);
        }
    }

    TEST(SegmentPlaneIntersectionTest, HandlesUniformlyTinyAndHugeScenes)
    {
        for (const int exponent : {-900, 900})
        {
            SCOPED_TRACE(exponent);
            const double scale = std::scalbn(1.0, exponent);
            const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{2.0 * scale, 0.0, 0.0}, Point3{0.0, 2.0 * scale, 0.0}}};
            const Segment3 segment{{0.0, 0.0, 2.0 * scale}, {3.0 * scale, 0.0, -scale}};
            const Point3 point = SegmentPlaneIntersection(segment, plane);
            EXPECT_NEAR(point.cord_x / scale, 2.0, 1e-12);
            EXPECT_DOUBLE_EQ(point.cord_y, 0.0);
            EXPECT_NEAR(point.cord_z / scale, 0.0, 1e-12);
        }
    }

    TEST(SegmentPlaneIntersectionTest, RejectsUnsupportedSpreadWithinOneAxis)
    {
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}}};
        const Segment3 segment{{0.0, 0.0, 1.0}, {0.0, 0.0, -std::scalbn(1.0, -257)}};
        EXPECT_THROW(SegmentPlaneIntersection(segment, plane), std::range_error);
    }

    TEST(SegmentPlaneIntersectionTest, RejectsNonFiniteCoordinatesInEveryInputPoint)
    {
        const Triangle3 originalPlane{{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}}};
        const Segment3 originalSegment{{0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}};
        for (std::size_t pointIndex = 0; pointIndex < 5; ++pointIndex)
        {
            Triangle3 plane = originalPlane;
            Segment3 segment = originalSegment;
            Point3& point = pointIndex < 3 ? plane.vertices[pointIndex]
                : (pointIndex == 3 ? segment.start : segment.end);
            point.cord_x = std::numeric_limits<double>::quiet_NaN();
            EXPECT_THROW(SegmentPlaneIntersection(segment, plane), std::invalid_argument);
        }
    }

    TEST(TrianglePlaneIntersectionTest, RecoversSectionWhenRoundedDistancesAreZero)
    {
        const double n = 1099511627776.0;
        const Point3 onPlane{n + 1.0, n, 0.0};
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, onPlane, Point3{n, n - 1.0, 1.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 1.0}, Point3{0.0, 0.0, -1.0}, onPlane}};
        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{0.0, 0.0, 0.0}, onPlane});
    }

    TEST(TrianglePlaneIntersectionTest, RetainsSingleVertexContactNearCancellation)
    {
        const double n = 1099511627776.0;
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 1.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 1.0}, Point3{-1.0, 0.0, 1.0}}};
        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
    }

    TEST(TrianglePlaneIntersectionTest, RetainsWholeEdgeContactNearCancellation)
    {
        const double n = 1099511627776.0;
        const Point3 onPlane{n + 1.0, n, 0.0};
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, onPlane, Point3{n, n - 1.0, 1.0}}};
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, onPlane, Point3{0.0, 0.0, 1.0}}};
        ExpectTrianglePlaneSectionInAnyOrder(triangle, plane,
                                            Segment3{{0.0, 0.0, 0.0}, onPlane});
    }

    TEST(TrianglePlaneIntersectionTest, RejectsExactCoplanarityNearCancellation)
    {
        const double n = 1099511627776.0;
        const Triangle3 plane{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 1.0}}};
        EXPECT_THROW(TrianglePlaneIntersection(plane, plane), std::invalid_argument);
    }

    TEST(Robust3DIntegrationTest, CrossingThroughIllConditionedPlaneIsFound)
    {
        const double n = 1099511627776.0;
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 1.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, 1.0}, Point3{0.0, 0.0, -1.0}, Point3{1.0, 0.0, 1.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
    }

    namespace
    {
        void ExpectPointInTriangle3DInEveryOrder(const Point3& point, const Triangle3& triangle,
                                                bool expected)
        {
            std::array<std::size_t, 3> order{0, 1, 2};
            do
            {
                SCOPED_TRACE(::testing::Message() << "vertex order: "
                             << order[0] << order[1] << order[2]);
                const Triangle3 permuted{{triangle.vertices[order[0]], triangle.vertices[order[1]],
                                           triangle.vertices[order[2]]}};
                EXPECT_EQ(PointInTriangle(point, permuted), expected);
            } while (std::next_permutation(order.begin(), order.end()));
        }
    }

    TEST(PointInTriangle3DTest, HorizontalInteriorAndExterior)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        ExpectPointInTriangle3DInEveryOrder({1.0, 1.0, 0.0}, triangle, true);
        ExpectPointInTriangle3DInEveryOrder({3.0, 3.0, 0.0}, triangle, false);
        ExpectPointInTriangle3DInEveryOrder({-1.0, 1.0, 0.0}, triangle, false);
    }

    TEST(PointInTriangle3DTest, IncludesAllEdgesAndVertices)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        for (const Point3& point : triangle.vertices)
            ExpectPointInTriangle3DInEveryOrder(point, triangle, true);
        for (const Point3& point : {Point3{2.0, 0.0, 0.0}, Point3{2.0, 2.0, 0.0}, Point3{0.0, 2.0, 0.0}})
            ExpectPointInTriangle3DInEveryOrder(point, triangle, true);
    }

    TEST(PointInTriangle3DTest, RejectsOffPlanePointsDespiteInteriorProjection)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        for (const double z : {1.0, -1.0, std::scalbn(1.0, -100)})
            ExpectPointInTriangle3DInEveryOrder({1.0, 1.0, z}, triangle, false);
    }

    TEST(PointInTriangle3DTest, HandlesVerticalPlanesAwayFromOrigin)
    {
        const Triangle3 xPlane{{Point3{4.0, 0.0, 0.0}, Point3{4.0, 4.0, 0.0}, Point3{4.0, 0.0, 4.0}}};
        ExpectPointInTriangle3DInEveryOrder({4.0, 1.0, 1.0}, xPlane, true);
        ExpectPointInTriangle3DInEveryOrder({4.0, 3.0, 3.0}, xPlane, false);
        ExpectPointInTriangle3DInEveryOrder({5.0, 1.0, 1.0}, xPlane, false);
        const Triangle3 yPlane{{Point3{0.0, -3.0, 0.0}, Point3{4.0, -3.0, 0.0}, Point3{0.0, -3.0, 4.0}}};
        ExpectPointInTriangle3DInEveryOrder({1.0, -3.0, 1.0}, yPlane, true);
        ExpectPointInTriangle3DInEveryOrder({3.0, -3.0, 3.0}, yPlane, false);
        ExpectPointInTriangle3DInEveryOrder({1.0, -2.0, 1.0}, yPlane, false);
    }

    TEST(PointInTriangle3DTest, HandlesObliquePlane)
    {
        const Triangle3 triangle{{Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}, Point3{0.0, 0.0, 4.0}}};
        ExpectPointInTriangle3DInEveryOrder({1.0, 1.0, 2.0}, triangle, true);
        ExpectPointInTriangle3DInEveryOrder({2.0, 2.0, 0.0}, triangle, true);
        ExpectPointInTriangle3DInEveryOrder({-1.0, 1.0, 4.0}, triangle, false);
        ExpectPointInTriangle3DInEveryOrder({1.0, 1.0, 1.0}, triangle, false);
    }

    TEST(PointInTriangle3DTest, CollapsedTriangleContainsOnlyItsOwnPoint)
    {
        const Point3 point{2.0, -3.0, 4.0};
        const Triangle3 triangle{{point, point, point}};
        ExpectPointInTriangle3DInEveryOrder(point, triangle, true);
        ExpectPointInTriangle3DInEveryOrder({3.0, -3.0, 4.0}, triangle, false);
    }

    TEST(PointInTriangle3DTest, CollinearTriangleContainsItsWholeSegment)
    {
        const Triangle3 triangle{{Point3{2.0, 4.0, 6.0}, Point3{0.0, 0.0, 0.0}, Point3{5.0, 10.0, 15.0}}};
        ExpectPointInTriangle3DInEveryOrder({3.0, 6.0, 9.0}, triangle, true);
        for (const Point3& vertex : triangle.vertices)
            ExpectPointInTriangle3DInEveryOrder(vertex, triangle, true);
        ExpectPointInTriangle3DInEveryOrder({6.0, 12.0, 18.0}, triangle, false);
        ExpectPointInTriangle3DInEveryOrder({3.0, 6.0, 8.0}, triangle, false);
    }

    TEST(PointInTriangle3DTest, RepeatedVertexProducesSegment)
    {
        const Point3 a{0.0, 0.0, 0.0};
        const Point3 b{4.0, 4.0, 4.0};
        const Triangle3 triangle{{a, a, b}};
        ExpectPointInTriangle3DInEveryOrder({2.0, 2.0, 2.0}, triangle, true);
        ExpectPointInTriangle3DInEveryOrder({5.0, 5.0, 5.0}, triangle, false);
        ExpectPointInTriangle3DInEveryOrder({2.0, 2.0, 3.0}, triangle, false);
    }

    TEST(PointInTriangle3DTest, PreservesThinTrianglesInEveryCoordinatePlane)
    {
        const double n = 1099511627776.0;
        Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 0.0}}};
        Point3 inside{n / 2.0 + 0.25, n / 2.0 - 0.25, 0.0};
        Point3 outside{n / 2.0 + 0.5, n / 2.0 - 0.25, 0.0};
        for (int rotation = 0; rotation < 3; ++rotation)
        {
            SCOPED_TRACE(rotation);
            ExpectPointInTriangle3DInEveryOrder(inside, triangle, true);
            ExpectPointInTriangle3DInEveryOrder(outside, triangle, false);
            for (const Point3& vertex : triangle.vertices)
                ExpectPointInTriangle3DInEveryOrder(vertex, triangle, true);
            for (auto& point : triangle.vertices)
                point = {point.cord_y, point.cord_z, point.cord_x};
            inside = {inside.cord_y, inside.cord_z, inside.cord_x};
            outside = {outside.cord_y, outside.cord_z, outside.cord_x};
        }
    }

    TEST(PointInTriangle3DTest, UsesRobustCoplanarityForThinObliqueTriangle)
    {
        const double n = 1099511627776.0;
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 1.0}}};
        ExpectPointInTriangle3DInEveryOrder({n / 2.0 + 0.25, n / 2.0 - 0.25, 0.25}, triangle, true);
        ExpectPointInTriangle3DInEveryOrder({n / 2.0 + 0.25, n / 2.0 - 0.25, 0.5}, triangle, false);
    }

    TEST(PointInTriangle3DTest, HandlesUniformlyTinyAndHugeScenes)
    {
        for (const int exponent : {-900, 900})
        {
            SCOPED_TRACE(exponent);
            const double scale = std::scalbn(1.0, exponent);
            const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0 * scale, 0.0, 0.0}, Point3{0.0, 4.0 * scale, 0.0}}};
            ExpectPointInTriangle3DInEveryOrder({scale, scale, 0.0}, triangle, true);
            ExpectPointInTriangle3DInEveryOrder({3.0 * scale, 3.0 * scale, 0.0}, triangle, false);
            ExpectPointInTriangle3DInEveryOrder({scale, scale, scale}, triangle, false);
        }
    }

    TEST(PointInTriangle3DTest, RejectsNonFiniteCoordinatesForEveryTriangleKind)
    {
        const Point3 a{0.0, 0.0, 0.0};
        const Point3 b{4.0, 0.0, 0.0};
        const Triangle3 cases[]{{{a, b, {0.0, 4.0, 0.0}}}, {{a, a, b}}, {{a, a, a}}};
        for (const Triangle3& original : cases)
        {
            for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                                       std::numeric_limits<double>::infinity(),
                                       -std::numeric_limits<double>::infinity()})
            {
                for (std::size_t index = 0; index < 4; ++index)
                {
                    for (std::size_t axis = 0; axis < 3; ++axis)
                    {
                        auto triangle = original;
                        Point3 query{1.0, 1.0, 0.0};
                        Point3& point = index < 3 ? triangle.vertices[index] : query;
                        const std::array<double*, 3> coordinates{&point.cord_x, &point.cord_y, &point.cord_z};
                        *coordinates[axis] = value;
                        EXPECT_THROW(PointInTriangle(query, triangle), std::invalid_argument);
                    }
                }
            }
        }
    }

    TEST(PointInTriangle3DTest, PreservesPredicateExponentLimits)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}}};
        const double tiny = std::scalbn(1.0, -256);
        ExpectPointInTriangle3DInEveryOrder({tiny, tiny, 0.0}, triangle, true);
        const Point3 unsupported{0.25, 0.25, std::scalbn(1.0, -257)};
        EXPECT_THROW(PointInTriangle(unsupported, triangle), std::range_error);
    }

    namespace
    {
        void Expect3DSegmentIntersectionInAnyOrder(const Segment3& lhs, const Segment3& rhs, bool expected)
        {
            for (const Segment3& first : {lhs, Segment3{lhs.end, lhs.start}})
            {
                for (const Segment3& second : {rhs, Segment3{rhs.end, rhs.start}})
                {
                    EXPECT_EQ(SegmentsIntersect(first, second), expected);
                    EXPECT_EQ(SegmentsIntersect(second, first), expected);
                }
            }
        }

        void ExpectSegmentTriangleIntersectionInAnyOrder(const Segment3& segment,
                                                         const Triangle3& triangle, bool expected)
        {
            std::array<std::size_t, 3> order{0, 1, 2};
            do
            {
                SCOPED_TRACE(::testing::Message() << "vertex order: " << order[0] << order[1] << order[2]);
                const Triangle3 permuted{{triangle.vertices[order[0]], triangle.vertices[order[1]],
                                           triangle.vertices[order[2]]}};
                EXPECT_EQ(SegmentIntersectsTriangle(segment, permuted), expected);
                EXPECT_EQ(SegmentIntersectsTriangle({segment.end, segment.start}, permuted), expected);
            } while (std::next_permutation(order.begin(), order.end()));
        }

        Triangle3 PointTriangle(const Point3& point)
        {
            return {{point, point, point}};
        }

        Triangle3 SegmentTriangle(const Segment3& segment)
        {
            return {{segment.end, segment.start, segment.end}};
        }
    }

    TEST(SegmentsIntersect3DTest, CoplanarCrossingsInCoordinateAndObliquePlanes)
    {
        Segment3 lhs{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
        Segment3 rhs{{0.0, -1.0, 0.0}, {0.0, 1.0, 0.0}};
        for (int rotation = 0; rotation < 3; ++rotation)
        {
            Expect3DSegmentIntersectionInAnyOrder(lhs, rhs, true);
            for (Point3* point : {&lhs.start, &lhs.end, &rhs.start, &rhs.end})
                *point = {point->cord_y, point->cord_z, point->cord_x};
        }
        Expect3DSegmentIntersectionInAnyOrder({{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}},
                                              {{-1.0, 1.0, 0.0}, {1.0, -1.0, 0.0}}, true);
    }

    TEST(SegmentsIntersect3DTest, IncludesEndpointAndTJunctionContacts)
    {
        const Segment3 lhs{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
        Expect3DSegmentIntersectionInAnyOrder(lhs, {{2.0, 0.0, 0.0}, {2.0, 1.0, 1.0}}, true);
        Expect3DSegmentIntersectionInAnyOrder(lhs, {{1.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}, true);
    }

    TEST(SegmentsIntersect3DTest, CollinearOverlapContainmentAndSeparation)
    {
        const Segment3 lhs{{0.0, 0.0, 0.0}, {4.0, 4.0, 4.0}};
        Expect3DSegmentIntersectionInAnyOrder(lhs, {{2.0, 2.0, 2.0}, {6.0, 6.0, 6.0}}, true);
        Expect3DSegmentIntersectionInAnyOrder(lhs, {{1.0, 1.0, 1.0}, {2.0, 2.0, 2.0}}, true);
        Expect3DSegmentIntersectionInAnyOrder(lhs, {{4.0, 4.0, 4.0}, {6.0, 6.0, 6.0}}, true);
        Expect3DSegmentIntersectionInAnyOrder(lhs, {{5.0, 5.0, 5.0}, {6.0, 6.0, 6.0}}, false);
        Expect3DSegmentIntersectionInAnyOrder(lhs, lhs, true);
    }

    TEST(SegmentsIntersect3DTest, RejectsSeparatedCoplanarAndParallelSegments)
    {
        const Segment3 lhs{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
        Expect3DSegmentIntersectionInAnyOrder(lhs, {{2.0, -1.0, 0.0}, {2.0, 1.0, 0.0}}, false);
        Expect3DSegmentIntersectionInAnyOrder(lhs, {{0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}}, false);
        Expect3DSegmentIntersectionInAnyOrder(lhs, {{0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}}, false);
    }

    TEST(SegmentsIntersect3DTest, RejectsSkewSegmentsWithCrossingProjections)
    {
        Expect3DSegmentIntersectionInAnyOrder({{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
                                              {{0.0, -1.0, 1.0}, {0.0, 1.0, 1.0}}, false);
    }

    TEST(SegmentsIntersect3DTest, HandlesZeroLengthSegments)
    {
        const Segment3 line{{0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}};
        Expect3DSegmentIntersectionInAnyOrder(line, {{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}}, true);
        Expect3DSegmentIntersectionInAnyOrder(line, {{3.0, 3.0, 3.0}, {3.0, 3.0, 3.0}}, false);
        Expect3DSegmentIntersectionInAnyOrder(line, {{1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}}, false);
        const Segment3 point{{1.0, 2.0, 3.0}, {1.0, 2.0, 3.0}};
        Expect3DSegmentIntersectionInAnyOrder(point, point, true);
        Expect3DSegmentIntersectionInAnyOrder(point, {{1.0, 2.0, 4.0}, {1.0, 2.0, 4.0}}, false);
    }

    TEST(SegmentsIntersect3DTest, ResolvesNearCollinearSeparation)
    {
        const double n = 1099511627776.0;
        Expect3DSegmentIntersectionInAnyOrder({{0.0, 0.0, 0.0}, {n + 1.0, n, 0.0}},
                                              {{n, n - 1.0, 0.0}, {n + 0.5, n - 0.5, 0.0}}, false);
    }

    TEST(SegmentsIntersect3DTest, HandlesUniformScales)
    {
        for (const int exponent : {-900, 900})
        {
            const double s = std::scalbn(1.0, exponent);
            const Segment3 lhs{{-s, 0.0, 0.0}, {s, 0.0, 0.0}};
            Expect3DSegmentIntersectionInAnyOrder(lhs, {{0.0, -s, 0.0}, {0.0, s, 0.0}}, true);
            Expect3DSegmentIntersectionInAnyOrder(lhs, {{0.0, -s, s}, {0.0, s, s}}, false);
        }
    }

    TEST(SegmentsIntersect3DTest, RejectsNonFiniteCoordinates)
    {
        for (const double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
        {
            for (std::size_t index = 0; index < 4; ++index)
            {
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    std::array<Point3, 4> points{Point3{0.0, 0.0, 0.0}, Point3{0.0, 0.0, 0.0},
                                                  Point3{-1.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}};
                    auto& p = points[index];
                    const std::array<double*, 3> coordinates{&p.cord_x, &p.cord_y, &p.cord_z};
                    *coordinates[axis] = invalid;
                    const Segment3 lhs{points[0], points[1]}, rhs{points[2], points[3]};
                    EXPECT_THROW(SegmentsIntersect(lhs, rhs), std::invalid_argument);
                }
            }
        }
    }

    TEST(SegmentsIntersect3DTest, PreservesExponentLimits)
    {
        const double tiny = std::scalbn(1.0, -257);
        const Segment3 lhs{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
        const Segment3 rhs{{0.0, tiny, 0.0}, {1.0, tiny, 0.0}};
        EXPECT_THROW(SegmentsIntersect(lhs, rhs), std::range_error);
    }

    TEST(SegmentIntersectsTriangle3DTest, StrictPlaneCrossings)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        ExpectSegmentTriangleIntersectionInAnyOrder({{1.0, 1.0, -1.0}, {1.0, 1.0, 1.0}}, triangle, true);
        ExpectSegmentTriangleIntersectionInAnyOrder({{3.0, 3.0, -1.0}, {3.0, 3.0, 1.0}}, triangle, false);
        ExpectSegmentTriangleIntersectionInAnyOrder({{0.0, 0.0, 1.0}, {10.0, 0.0, -1.0}}, triangle, false);
    }

    TEST(SegmentIntersectsTriangle3DTest, CrossingThroughEdgesAndVertices)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        ExpectSegmentTriangleIntersectionInAnyOrder({{2.0, 2.0, -1.0}, {2.0, 2.0, 1.0}}, triangle, true);
        ExpectSegmentTriangleIntersectionInAnyOrder({{0.0, 0.0, -1.0}, {0.0, 0.0, 1.0}}, triangle, true);
    }

    TEST(SegmentIntersectsTriangle3DTest, EndpointsOnPlane)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        ExpectSegmentTriangleIntersectionInAnyOrder({{1.0, 1.0, 0.0}, {1.0, 1.0, 1.0}}, triangle, true);
        ExpectSegmentTriangleIntersectionInAnyOrder({{5.0, 5.0, 0.0}, {1.0, 1.0, 1.0}}, triangle, false);
    }

    TEST(SegmentIntersectsTriangle3DTest, RejectsEndpointsStrictlyOnSameSide)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        ExpectSegmentTriangleIntersectionInAnyOrder({{1.0, 1.0, 1.0}, {1.0, 1.0, 2.0}}, triangle, false);
        ExpectSegmentTriangleIntersectionInAnyOrder({{1.0, 1.0, -1.0}, {1.0, 1.0, -2.0}}, triangle, false);
    }

    TEST(SegmentIntersectsTriangle3DTest, CoplanarContainmentCrossingAndContacts)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        ExpectSegmentTriangleIntersectionInAnyOrder({{1.0, 1.0, 0.0}, {2.0, 1.0, 0.0}}, triangle, true);
        ExpectSegmentTriangleIntersectionInAnyOrder({{-1.0, 1.0, 0.0}, {5.0, 1.0, 0.0}}, triangle, true);
        ExpectSegmentTriangleIntersectionInAnyOrder({{3.0, 3.0, 0.0}, {4.0, 4.0, 0.0}}, triangle, false);
        ExpectSegmentTriangleIntersectionInAnyOrder({{-1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, triangle, true);
        ExpectSegmentTriangleIntersectionInAnyOrder({{-1.0, -1.0, 0.0}, {0.0, 0.0, 0.0}}, triangle, true);
    }

    TEST(SegmentIntersectsTriangle3DTest, ZeroLengthSegments)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        ExpectSegmentTriangleIntersectionInAnyOrder({{1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}}, triangle, true);
        ExpectSegmentTriangleIntersectionInAnyOrder({{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}}, triangle, false);
        ExpectSegmentTriangleIntersectionInAnyOrder({{3.0, 3.0, 0.0}, {3.0, 3.0, 0.0}}, triangle, false);
    }

    TEST(SegmentIntersectsTriangle3DTest, DegenerateTriangles)
    {
        const Segment3 segment{{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
        ExpectSegmentTriangleIntersectionInAnyOrder(segment, PointTriangle({0.0, 0.0, 0.0}), true);
        ExpectSegmentTriangleIntersectionInAnyOrder(segment, PointTriangle({0.0, 1.0, 0.0}), false);
        ExpectSegmentTriangleIntersectionInAnyOrder(segment, SegmentTriangle({{0.0, -1.0, 0.0}, {0.0, 1.0, 0.0}}), true);
        ExpectSegmentTriangleIntersectionInAnyOrder(segment, SegmentTriangle({{0.0, -1.0, 1.0}, {0.0, 1.0, 1.0}}), false);
    }

    TEST(SegmentIntersectsTriangle3DTest, VerticalAndObliquePlanes)
    {
        Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        Segment3 segment{{1.0, 1.0, -1.0}, {1.0, 1.0, 1.0}};
        for (int rotation = 0; rotation < 3; ++rotation)
        {
            ExpectSegmentTriangleIntersectionInAnyOrder(segment, triangle, true);
            for (auto& p : triangle.vertices) p = {p.cord_y, p.cord_z, p.cord_x};
            for (Point3* p : {&segment.start, &segment.end}) *p = {p->cord_y, p->cord_z, p->cord_x};
        }
        const Triangle3 oblique{{Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}, Point3{0.0, 0.0, 4.0}}};
        ExpectSegmentTriangleIntersectionInAnyOrder({{0.0, 0.0, 0.0}, {4.0, 4.0, 4.0}}, oblique, true);
    }

    TEST(SegmentIntersectsTriangle3DTest, ThinTriangles)
    {
        const double n = 1099511627776.0;
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 0.0}}};
        ExpectSegmentTriangleIntersectionInAnyOrder({{n / 2.0 + 0.25, n / 2.0 - 0.25, -1.0},
                                                     {n / 2.0 + 0.25, n / 2.0 - 0.25, 1.0}}, triangle, true);
        ExpectSegmentTriangleIntersectionInAnyOrder({{n / 2.0 + 0.5, n / 2.0 - 0.25, -1.0},
                                                     {n / 2.0 + 0.5, n / 2.0 - 0.25, 1.0}}, triangle, false);
    }

    TEST(SegmentIntersectsTriangle3DTest, RationalIntersectionDoesNotRequireRoundedConstruction)
    {
        const Triangle3 triangle{{Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}, Point3{0.0, 0.0, 1.0}}};
        ExpectSegmentTriangleIntersectionInAnyOrder({{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}, triangle, true);
    }

    TEST(SegmentIntersectsTriangle3DTest, HandlesUniformScales)
    {
        for (const int exponent : {-900, 900})
        {
            const double s = std::scalbn(1.0, exponent);
            const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0 * s, 0.0, 0.0}, Point3{0.0, 4.0 * s, 0.0}}};
            ExpectSegmentTriangleIntersectionInAnyOrder({{s, s, -s}, {s, s, s}}, triangle, true);
            ExpectSegmentTriangleIntersectionInAnyOrder({{3.0 * s, 3.0 * s, -s}, {3.0 * s, 3.0 * s, s}}, triangle, false);
        }
    }

    TEST(SegmentIntersectsTriangle3DTest, RejectsNonFiniteInputs)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        const double invalid = std::numeric_limits<double>::quiet_NaN();
        const Segment3 segment{{0.0, 0.0, 0.0}, {invalid, 0.0, 0.0}};
        EXPECT_THROW(SegmentIntersectsTriangle(segment, triangle), std::invalid_argument);
        auto invalidTriangle = triangle;
        invalidTriangle.vertices[2].cord_z = invalid;
        EXPECT_THROW(SegmentIntersectsTriangle({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, invalidTriangle), std::invalid_argument);
    }

    TEST(DegenerateTrianglesIntersectTest, PointPoint)
    {
        const Triangle3 point = PointTriangle({1.0, 2.0, 3.0});
        Expect3DTriangleIntersectionInAnyOrder(point, point, true);
        Expect3DTriangleIntersectionInAnyOrder(point, PointTriangle({1.0, 2.0, 4.0}), false);
    }

    TEST(DegenerateTrianglesIntersectTest, PointSegment)
    {
        const Triangle3 segment = SegmentTriangle({{0.0, 0.0, 0.0}, {4.0, 4.0, 4.0}});
        Expect3DTriangleIntersectionInAnyOrder(PointTriangle({2.0, 2.0, 2.0}), segment, true);
        Expect3DTriangleIntersectionInAnyOrder(PointTriangle({0.0, 0.0, 0.0}), segment, true);
        Expect3DTriangleIntersectionInAnyOrder(PointTriangle({5.0, 5.0, 5.0}), segment, false);
        Expect3DTriangleIntersectionInAnyOrder(PointTriangle({2.0, 2.0, 3.0}), segment, false);
    }

    TEST(DegenerateTrianglesIntersectTest, PointTriangle)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        Expect3DTriangleIntersectionInAnyOrder(PointTriangle({1.0, 1.0, 0.0}), triangle, true);
        Expect3DTriangleIntersectionInAnyOrder(PointTriangle({2.0, 2.0, 0.0}), triangle, true);
        Expect3DTriangleIntersectionInAnyOrder(PointTriangle({3.0, 3.0, 0.0}), triangle, false);
        Expect3DTriangleIntersectionInAnyOrder(PointTriangle({1.0, 1.0, 1.0}), triangle, false);
    }

    TEST(DegenerateTrianglesIntersectTest, SegmentSegment)
    {
        const Triangle3 first = SegmentTriangle({{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});
        Expect3DTriangleIntersectionInAnyOrder(first, SegmentTriangle({{0.0, -1.0, 0.0}, {0.0, 1.0, 0.0}}), true);
        Expect3DTriangleIntersectionInAnyOrder(first, SegmentTriangle({{0.0, -1.0, 1.0}, {0.0, 1.0, 1.0}}), false);
        Expect3DTriangleIntersectionInAnyOrder(first, SegmentTriangle({{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}), true);
        Expect3DTriangleIntersectionInAnyOrder(first, SegmentTriangle({{2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}}), false);
    }

    TEST(DegenerateTrianglesIntersectTest, SegmentTriangle)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}};
        Expect3DTriangleIntersectionInAnyOrder(SegmentTriangle({{1.0, 1.0, -1.0}, {1.0, 1.0, 1.0}}), triangle, true);
        Expect3DTriangleIntersectionInAnyOrder(SegmentTriangle({{3.0, 3.0, -1.0}, {3.0, 3.0, 1.0}}), triangle, false);
        Expect3DTriangleIntersectionInAnyOrder(SegmentTriangle({{1.0, 1.0, 0.0}, {2.0, 1.0, 0.0}}), triangle, true);
    }

    TEST(DegenerateTrianglesIntersectTest, ThinCoplanarContainmentRemainsNondegenerate)
    {
        const double n = 1099511627776.0;
        const Triangle3 outer{{Point3{0.0, 0.0, 0.0}, Point3{n + 1.0, n, 0.0}, Point3{n, n - 1.0, 0.0}}};
        const Triangle3 inner{{Point3{n / 2.0 + 0.25, n / 2.0 - 0.25, 0.0},
                                Point3{3.0 * n / 4.0 + 0.5, 3.0 * n / 4.0 - 0.25, 0.0},
                                Point3{3.0 * n / 4.0 + 0.25, 3.0 * n / 4.0 - 0.5, 0.0}}};
        Expect3DTriangleIntersectionInAnyOrder(outer, inner, true);
    }

    TEST(DegenerateTrianglesIntersectTest, RationalBoundaryContactAndNearbySeparation)
    {
        const Triangle3 first{{Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}}};
        const Triangle3 second{{Point3{0.0, 0.0, 1.0}, Point3{1.0, 2.0, -2.0}, Point3{0.0, 0.0, 2.0}}};
        Expect3DTriangleIntersectionInAnyOrder(first, second, true);
        auto separated = second;
        separated.vertices[1].cord_y += std::scalbn(1.0, -40);
        Expect3DTriangleIntersectionInAnyOrder(first, separated, false);
    }

    TEST(DegenerateTrianglesIntersectTest, MixedSceneMaskMarksBothMembersAndPreservesIsolatedFigures)
    {
        std::vector<Triangle3> scene{
            {{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}},
            PointTriangle({1.0, 1.0, 0.0}),
            SegmentTriangle({{1.0, 1.0, -1.0}, {1.0, 1.0, 1.0}}),
            PointTriangle({10.0, 10.0, 10.0}),
            SegmentTriangle({{10.0, 10.0, 9.0}, {10.0, 10.0, 11.0}}),
            PointTriangle({30.0, 30.0, 30.0}),
            SegmentTriangle({{20.0, 20.0, 20.0}, {21.0, 20.0, 20.0}})
        };
        std::vector<bool> expected{true, true, true, true, true, false, false};
        EXPECT_EQ(FindIntersectionsBruteForce(scene), expected);
        std::reverse(scene.begin(), scene.end());
        std::reverse(expected.begin(), expected.end());
        EXPECT_EQ(FindIntersectionsBruteForce(scene), expected);
    }

    TEST(DegenerateTrianglesIntersectTest, RejectsNonFiniteCoordinatesBeforeDegenerateDispatch)
    {
        const Triangle3 point = PointTriangle({0.0, 0.0, 0.0});
        auto invalid = point;
        invalid.vertices[2].cord_z = std::numeric_limits<double>::infinity();
        EXPECT_THROW(TrianglesIntersect(point, invalid), std::invalid_argument);
        EXPECT_THROW(TrianglesIntersect(invalid, point), std::invalid_argument);
    }

}
