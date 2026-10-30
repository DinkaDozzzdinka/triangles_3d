#include "intersection.hpp"

#include <algorithm>
#include <array>
#include <cstddef>

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
}
