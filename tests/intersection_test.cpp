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

}
