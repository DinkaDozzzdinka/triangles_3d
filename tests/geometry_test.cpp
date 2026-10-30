#include "geometry.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

TEST(GeometryTests, SubtractsPoints)
{
    const triangles::Point3 a{1.0, 2.0, 3.0};
    const triangles::Point3 b{4.0, 6.0, 3.0};

    const triangles::Vec3 result = b - a;

    EXPECT_DOUBLE_EQ(result.cord_x, 3.0);
    EXPECT_DOUBLE_EQ(result.cord_y, 4.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 0.0);
}

TEST(GeometryTests, SubtractsPointFromItself)
{
    const triangles::Point3 point{2.0, -3.0, 7.0};

    const triangles::Vec3 result = point - point;

    EXPECT_DOUBLE_EQ(result.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 0.0);
}

TEST(GeometryTests, ReversesDirection)
{
    const triangles::Point3 a{1.0, 2.0, 3.0};
    const triangles::Point3 b{4.0, 6.0, 3.0};

    const triangles::Vec3 result = a - b;

    EXPECT_DOUBLE_EQ(result.cord_x, -3.0);
    EXPECT_DOUBLE_EQ(result.cord_y, -4.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 0.0);
}

TEST(GeometryTests, SubtractsNegativeCoordinates)
{
    const triangles::Point3 a{-4.0, -1.0, -7.0};
    const triangles::Point3 b{-1.0, -5.0, -2.0};

    const triangles::Vec3 result = b - a;

    EXPECT_DOUBLE_EQ(result.cord_x, 3.0);
    EXPECT_DOUBLE_EQ(result.cord_y, -4.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 5.0);
}

TEST(GeometryTests, SubtractsFractionalCoordinates)
{
    const triangles::Point3 a{0.25, -1.5, 2.75};
    const triangles::Point3 b{1.5, -0.25, 0.5};

    const triangles::Vec3 result = b - a;

    EXPECT_DOUBLE_EQ(result.cord_x, 1.25);
    EXPECT_DOUBLE_EQ(result.cord_y, 1.25);
    EXPECT_DOUBLE_EQ(result.cord_z, -2.25);
}

TEST(GeometryTests, DefaultsPointToOrigin)
{

    const triangles::Point3 point;

    EXPECT_DOUBLE_EQ(point.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(point.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(point.cord_z, 0.0);
}

TEST(GeometryTests, DefaultsVectorToZero)
{
    const triangles::Vec3 vector;

    EXPECT_DOUBLE_EQ(vector.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(vector.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(vector.cord_z, 0.0);
}

TEST(GeometryTests, AddsVectors)
{
    const triangles::Vec3 lhs{1.0, 2.0, 3.0};
    const triangles::Vec3 rhs{4.0, -2.0, 1.0};

    const triangles::Vec3 result = lhs + rhs;

    EXPECT_DOUBLE_EQ(result.cord_x, 5.0);
    EXPECT_DOUBLE_EQ(result.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 4.0);
}

TEST(GeometryTests, AddingZeroVectorPreservesVector)
{
    const triangles::Vec3 vector{1.25, -2.5, 3.75};
    const triangles::Vec3 zero{};

    const triangles::Vec3 result = vector + zero;

    EXPECT_DOUBLE_EQ(result.cord_x, 1.25);
    EXPECT_DOUBLE_EQ(result.cord_y, -2.5);
    EXPECT_DOUBLE_EQ(result.cord_z, 3.75);
}

TEST(GeometryTests, TranslatesPoint)
{
    const triangles::Point3 point{1.0, 2.0, 3.0};
    const triangles::Vec3 displacement{3.0, 4.0, 0.0};

    const triangles::Point3 result = point + displacement;

    EXPECT_DOUBLE_EQ(result.cord_x, 4.0);
    EXPECT_DOUBLE_EQ(result.cord_y, 6.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 3.0);
}

TEST(GeometryTests, AddingZeroVectorPreservesPoint)
{
    const triangles::Point3 point{-1.25, 2.5, -3.75};
    const triangles::Vec3 zero{};

    const triangles::Point3 result = point + zero;

    EXPECT_DOUBLE_EQ(result.cord_x, -1.25);
    EXPECT_DOUBLE_EQ(result.cord_y, 2.5);
    EXPECT_DOUBLE_EQ(result.cord_z, -3.75);
}

TEST(GeometryTests, ScalarProduct)
{
    const triangles::Vec3 vector1{1.0, 0.0, 0.0};
    const triangles::Vec3 vector2{0.0, 1.0, 0.0};

    const double result = triangles::Dot (vector1, vector2);

    EXPECT_DOUBLE_EQ(result, 0);

}


TEST(GeometryTests, DotsParallelUnitVectors)
{
    const triangles::Vec3 lhs{1.0, 0.0, 0.0};
    const triangles::Vec3 rhs{1.0, 0.0, 0.0};

    EXPECT_DOUBLE_EQ(triangles::Dot(lhs, rhs), 1.0);
}

TEST(GeometryTests, DotsOppositeUnitVectors)
{
    const triangles::Vec3 lhs{1.0, 0.0, 0.0};
    const triangles::Vec3 rhs{-1.0, 0.0, 0.0};

    EXPECT_DOUBLE_EQ(triangles::Dot(lhs, rhs), -1.0);
}

TEST(GeometryTests, DotWithItselfIsSquaredLength)
{
    const triangles::Vec3 vector{3.0, 4.0, 0.0};

    EXPECT_DOUBLE_EQ(triangles::Dot(vector, vector), 25.0);
}

TEST(GeometryTests, DotWithZeroIsZero)
{
    const triangles::Vec3 vector{-2.0, 5.0, 7.0};
    const triangles::Vec3 zero{};

    EXPECT_DOUBLE_EQ(triangles::Dot(vector, zero), 0.0);
    EXPECT_DOUBLE_EQ(triangles::Dot(zero, vector), 0.0);
}

TEST(GeometryTests, DotsMixedSignCoordinates)
{
    const triangles::Vec3 lhs{1.0, 2.0, 3.0};
    const triangles::Vec3 rhs{4.0, -5.0, 6.0};

    EXPECT_DOUBLE_EQ(triangles::Dot(lhs, rhs), 12.0);
}

TEST(GeometryTests, DotsFractionalCoordinates)
{
    const triangles::Vec3 lhs{0.5, -1.25, 2.0};
    const triangles::Vec3 rhs{1.5, 2.0, -0.25};

    EXPECT_DOUBLE_EQ(triangles::Dot(lhs, rhs), -2.25);
}

TEST(GeometryTests, CrossesBasisVectors)
{
    const triangles::Vec3 x{1.0, 0.0, 0.0};
    const triangles::Vec3 y{0.0, 1.0, 0.0};

    const triangles::Vec3 result = triangles::Cross(x, y);

    EXPECT_DOUBLE_EQ(result.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 1.0);
}

TEST(GeometryTests, SwappingCrossArgumentsChangesSign)
{
    const triangles::Vec3 lhs{1.0, 2.0, 3.0};
    const triangles::Vec3 rhs{4.0, -5.0, 6.0};

    const triangles::Vec3 result = triangles::Cross(rhs, lhs);

    EXPECT_DOUBLE_EQ(result.cord_x, -27.0);
    EXPECT_DOUBLE_EQ(result.cord_y, -6.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 13.0);
}

TEST(GeometryTests, CrossOfParallelVectorsIsZero)
{
    const triangles::Vec3 lhs{1.0, 2.0, 3.0};
    const triangles::Vec3 rhs{2.0, 4.0, 6.0};

    const triangles::Vec3 result = triangles::Cross(lhs, rhs);

    EXPECT_DOUBLE_EQ(result.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 0.0);
}

TEST(GeometryTests, CrossWithZeroIsZero)
{
    const triangles::Vec3 vector{2.0, -3.0, 5.0};
    const triangles::Vec3 zero{};

    const triangles::Vec3 right = triangles::Cross(vector, zero);
    const triangles::Vec3 left = triangles::Cross(zero, vector);

    EXPECT_DOUBLE_EQ(right.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(right.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(right.cord_z, 0.0);
    EXPECT_DOUBLE_EQ(left.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(left.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(left.cord_z, 0.0);
}

TEST(GeometryTests, CrossIsPerpendicularToBothVectors)
{
    const triangles::Vec3 lhs{1.0, 2.0, 3.0};
    const triangles::Vec3 rhs{4.0, -5.0, 6.0};

    const triangles::Vec3 result = triangles::Cross(lhs, rhs);

    EXPECT_DOUBLE_EQ(result.cord_x, 27.0);
    EXPECT_DOUBLE_EQ(result.cord_y, 6.0);
    EXPECT_DOUBLE_EQ(result.cord_z, -13.0);
    EXPECT_DOUBLE_EQ(triangles::Dot(result, lhs), 0.0);
    EXPECT_DOUBLE_EQ(triangles::Dot(result, rhs), 0.0);
}

TEST(GeometryTests, CrossesFractionalCoordinates)
{
    const triangles::Vec3 lhs{0.5, 1.0, -0.25};
    const triangles::Vec3 rhs{2.0, -0.5, 1.0};

    const triangles::Vec3 result = triangles::Cross(lhs, rhs);

    EXPECT_DOUBLE_EQ(result.cord_x, 0.875);
    EXPECT_DOUBLE_EQ(result.cord_y, -1.0);
    EXPECT_DOUBLE_EQ(result.cord_z, -2.25);
}

TEST(GeometryTests, LengthOfZeroIsZero)
{
    EXPECT_DOUBLE_EQ(triangles::Length(triangles::Vec3{}), 0.0);
}

TEST(GeometryTests, LengthUsesAllThreeCoordinates)
{
    const triangles::Vec3 vector{2.0, -3.0, 6.0};
    EXPECT_DOUBLE_EQ(triangles::Length(vector), 7.0);
}

TEST(GeometryTests, ComputesFractionalLength)
{
    const triangles::Vec3 vector{0.5, -1.0, 1.0};
    EXPECT_DOUBLE_EQ(triangles::Length(vector), 1.5);
}

TEST(GeometryTests, ComputesLargeLengthWithoutIntermediateOverflow)
{
    const triangles::Vec3 vector{3e200, 4e200, 0.0};
    EXPECT_NEAR(triangles::Length(vector) / 1e200, 5.0, 1e-14);
}

TEST(GeometryTests, ComputesSmallLengthWithoutIntermediateUnderflow)
{
    const triangles::Vec3 vector{3e-200, 4e-200, 0.0};
    EXPECT_NEAR(triangles::Length(vector) / 1e-200, 5.0, 1e-14);
}

TEST(GeometryTests, NormalizesVectorPreservingDirection)
{
    const triangles::Vec3 vector{-2.0, 3.0, -6.0};
    const triangles::Vec3 result = triangles::Normalize(vector);

    EXPECT_NEAR(result.cord_x, -2.0 / 7.0, 1e-14);
    EXPECT_NEAR(result.cord_y, 3.0 / 7.0, 1e-14);
    EXPECT_NEAR(result.cord_z, -6.0 / 7.0, 1e-14);
    EXPECT_NEAR(triangles::Length(result), 1.0, 1e-14);
}

TEST(GeometryTests, NormalizingUnitVectorPreservesIt)
{
    const triangles::Vec3 vector{0.0, -1.0, 0.0};
    const triangles::Vec3 result = triangles::Normalize(vector);

    EXPECT_DOUBLE_EQ(result.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_y, -1.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 0.0);
}

TEST(GeometryTests, NormalizesLargeFiniteVector)
{
    const double maximum = std::numeric_limits<double>::max();
    const triangles::Vec3 vector{maximum, maximum, 0.0};
    const triangles::Vec3 result = triangles::Normalize(vector);
    const double component = 1.0 / std::sqrt(2.0);

    EXPECT_NEAR(result.cord_x, component, 1e-14);
    EXPECT_NEAR(result.cord_y, component, 1e-14);
    EXPECT_DOUBLE_EQ(result.cord_z, 0.0);
    EXPECT_NEAR(triangles::Length(result), 1.0, 1e-14);
}

TEST(GeometryTests, NormalizesTinyVector)
{
    const triangles::Vec3 vector{1e-300, -1e-300, 0.0};
    const triangles::Vec3 result = triangles::Normalize(vector);
    const double component = 1.0 / std::sqrt(2.0);

    EXPECT_NEAR(result.cord_x, component, 1e-14);
    EXPECT_NEAR(result.cord_y, -component, 1e-14);
    EXPECT_DOUBLE_EQ(result.cord_z, 0.0);
    EXPECT_NEAR(triangles::Length(result), 1.0, 1e-14);
}

TEST(GeometryTests, NormalizationRejectsZeroVector)
{
    EXPECT_THROW(triangles::Normalize(triangles::Vec3{}), std::domain_error);
}

TEST(GeometryTests, NormalizationRejectsNonFiniteCoordinates)
{
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const triangles::Vec3 invalidVectors[] = {
        {infinity, 0.0, 0.0}, {0.0, nan, 0.0}, {0.0, 0.0, -infinity}
    };

    for (const auto& vector : invalidVectors)
        EXPECT_THROW(triangles::Normalize(vector), std::invalid_argument);
}

TEST(GeometryTests, ComputesTriangleUnitNormal)
{
    const triangles::Point3 a{0.0, 0.0, 0.0};
    const triangles::Point3 b{2.0, 0.0, 0.0};
    const triangles::Point3 c{0.0, 3.0, 0.0};
    const triangles::Vec3 result = triangles::UnitNormal(a, b, c);

    EXPECT_DOUBLE_EQ(result.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_z, 1.0);
    EXPECT_DOUBLE_EQ(triangles::Length(result), 1.0);
}

TEST(GeometryTests, ReversingTriangleWindingReversesUnitNormal)
{
    const triangles::Point3 a{0.0, 0.0, 0.0};
    const triangles::Point3 b{2.0, 0.0, 0.0};
    const triangles::Point3 c{0.0, 3.0, 0.0};
    const triangles::Vec3 result = triangles::UnitNormal(a, c, b);

    EXPECT_DOUBLE_EQ(result.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(result.cord_z, -1.0);
}

TEST(GeometryTests, UnitNormalRejectsCollinearPoints)
{
    const triangles::Point3 a{0.0, 0.0, 0.0};
    const triangles::Point3 b{1.0, 2.0, 3.0};
    const triangles::Point3 c{2.0, 4.0, 6.0};

    EXPECT_THROW(triangles::UnitNormal(a, b, c), std::domain_error);
}

TEST(GeometryTests, UnitNormalRejectsRepeatedVertex)
{
    const triangles::Point3 a{1.0, 2.0, 3.0};
    const triangles::Point3 b{4.0, 5.0, 6.0};

    EXPECT_THROW(triangles::UnitNormal(a, a, b), std::domain_error);
}

TEST(GeometryTests, UnitNormalHandlesLargeAndSmallEdges)
{
    const double scales[] = {1e-200, 1e200};
    for (const double scale : scales)
    {
        SCOPED_TRACE(scale);
        const triangles::Point3 a{0.0, 0.0, 0.0};
        const triangles::Point3 b{scale, 0.0, 0.0};
        const triangles::Point3 c{0.0, scale, 0.0};
        const triangles::Vec3 result = triangles::UnitNormal(a, b, c);

        EXPECT_DOUBLE_EQ(result.cord_x, 0.0);
        EXPECT_DOUBLE_EQ(result.cord_y, 0.0);
        EXPECT_DOUBLE_EQ(result.cord_z, 1.0);
    }
}

TEST(GeometryTests, TriangleOverloadComputesUnitNormal)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{2.0, 0.0, 0.0},
        triangles::Point3{0.0, 3.0, 0.0}
    }};

    const triangles::Vec3 normal = triangles::UnitNormal(triangle);

    EXPECT_DOUBLE_EQ(normal.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(normal.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(normal.cord_z, 1.0);
}

TEST(GeometryTests, TriangleOverloadHandlesObliquePlane)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{3.0, -2.0, 5.0},
        triangles::Point3{4.0, -2.0, 6.0},
        triangles::Point3{3.0, 0.0, 5.0}
    }};

    const triangles::Vec3 normal = triangles::UnitNormal(triangle);
    const double component = 1.0 / std::sqrt(2.0);

    EXPECT_NEAR(normal.cord_x, -component, 1e-14);
    EXPECT_NEAR(normal.cord_y, 0.0, 1e-14);
    EXPECT_NEAR(normal.cord_z, component, 1e-14);
    EXPECT_NEAR(triangles::Length(normal), 1.0, 1e-14);
}

TEST(GeometryTests, TriangleOverloadRespectsWinding)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{0.0, 3.0, 0.0},
        triangles::Point3{2.0, 0.0, 0.0}
    }};

    const triangles::Vec3 normal = triangles::UnitNormal(triangle);

    EXPECT_DOUBLE_EQ(normal.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(normal.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(normal.cord_z, -1.0);
}

TEST(GeometryTests, TriangleOverloadRejectsCollinearVertices)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{1.0, 2.0, 3.0},
        triangles::Point3{2.0, 4.0, 6.0}
    }};

    EXPECT_THROW(triangles::UnitNormal(triangle), std::domain_error);
}

TEST(GeometryTests, TriangleOverloadRejectsRepeatedVertex)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{1.0, 2.0, 3.0},
        triangles::Point3{1.0, 2.0, 3.0},
        triangles::Point3{4.0, 5.0, 6.0}
    }};

    EXPECT_THROW(triangles::UnitNormal(triangle), std::domain_error);
}

TEST(GeometryTests, ComputesRightTriangleArea)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{2.0, 0.0, 0.0},
        triangles::Point3{0.0, 3.0, 0.0}
    }};

    EXPECT_DOUBLE_EQ(triangles::Area(triangle), 3.0);
}

TEST(GeometryTests, ComputesObliqueTriangleArea)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{1.0, 0.0, 1.0},
        triangles::Point3{0.0, 2.0, 0.0}
    }};

    EXPECT_NEAR(triangles::Area(triangle), std::sqrt(2.0), 1e-14);
}

TEST(GeometryTests, TriangleAreaIsIndependentOfVertexOrder)
{
    const triangles::Point3 a{0.0, 0.0, 0.0};
    const triangles::Point3 b{2.0, 0.0, 0.0};
    const triangles::Point3 c{0.0, 3.0, 0.0};
    const triangles::Triangle3 original{{a, b, c}};
    const triangles::Triangle3 cyclic{{b, c, a}};
    const triangles::Triangle3 reversed{{a, c, b}};

    EXPECT_DOUBLE_EQ(triangles::Area(original), 3.0);
    EXPECT_DOUBLE_EQ(triangles::Area(cyclic), 3.0);
    EXPECT_DOUBLE_EQ(triangles::Area(reversed), 3.0);
}

TEST(GeometryTests, TriangleAreaIsIndependentOfTranslation)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{4.0, -2.0, 7.0},
        triangles::Point3{6.0, -2.0, 7.0},
        triangles::Point3{4.0, 1.0, 7.0}
    }};

    EXPECT_DOUBLE_EQ(triangles::Area(triangle), 3.0);
}

TEST(GeometryTests, CollinearTriangleAreaIsZero)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{1.0, 2.0, 3.0},
        triangles::Point3{2.0, 4.0, 6.0}
    }};

    EXPECT_DOUBLE_EQ(triangles::Area(triangle), 0.0);
}

TEST(GeometryTests, TriangleWithRepeatedVertexHasZeroArea)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{1.0, 2.0, 3.0},
        triangles::Point3{1.0, 2.0, 3.0},
        triangles::Point3{4.0, 5.0, 6.0}
    }};

    EXPECT_DOUBLE_EQ(triangles::Area(triangle), 0.0);
}

TEST(GeometryTests, ScalingTriangleDoublesEdgesAndQuadruplesArea)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{4.0, 0.0, 0.0},
        triangles::Point3{0.0, 6.0, 0.0}
    }};

    EXPECT_DOUBLE_EQ(triangles::Area(triangle), 12.0);
}

TEST(GeometryTests, BoundsSelectsExtremaIndependentlyForEachAxis)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{-2.0, 1.0, 4.0},
        triangles::Point3{3.0, -1.0, 2.0},
        triangles::Point3{0.0, 5.0, -3.0}
    }};
    const triangles::Aabb3 bounds = triangles::Bounds(triangle);

    EXPECT_DOUBLE_EQ(bounds.minimum.cord_x, -2.0);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_y, -1.0);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_z, -3.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_x, 3.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_y, 5.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_z, 4.0);

    for (const auto& point : triangle.vertices)
    {
        EXPECT_GE(point.cord_x, bounds.minimum.cord_x);
        EXPECT_GE(point.cord_y, bounds.minimum.cord_y);
        EXPECT_GE(point.cord_z, bounds.minimum.cord_z);
        EXPECT_LE(point.cord_x, bounds.maximum.cord_x);
        EXPECT_LE(point.cord_y, bounds.maximum.cord_y);
        EXPECT_LE(point.cord_z, bounds.maximum.cord_z);
    }
}

TEST(GeometryTests, BoundsHandlesAllNegativeCoordinates)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{-5.0, -2.0, -7.0},
        triangles::Point3{-1.0, -8.0, -3.0},
        triangles::Point3{-4.0, -6.0, -9.0}
    }};
    const triangles::Aabb3 bounds = triangles::Bounds(triangle);

    EXPECT_DOUBLE_EQ(bounds.minimum.cord_x, -5.0);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_y, -8.0);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_z, -9.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_x, -1.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_y, -2.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_z, -3.0);
}

TEST(GeometryTests, BoundsHandlesFractionalCoordinates)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{-0.5, 1.25, 3.5},
        triangles::Point3{0.25, -2.5, -1.75},
        triangles::Point3{2.75, 0.5, 0.0}
    }};
    const triangles::Aabb3 bounds = triangles::Bounds(triangle);

    EXPECT_DOUBLE_EQ(bounds.minimum.cord_x, -0.5);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_y, -2.5);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_z, -1.75);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_x, 2.75);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_y, 1.25);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_z, 3.5);
}

TEST(GeometryTests, BoundsIsIndependentOfVertexOrder)
{
    const std::array<triangles::Point3, 3> points{{
        {-2.0, 1.0, 4.0}, {3.0, -1.0, 2.0}, {0.0, 5.0, -3.0}
    }};
    std::array<int, 3> order{0, 1, 2};

    do
    {
        const triangles::Triangle3 triangle{{
            points[order[0]], points[order[1]], points[order[2]]
        }};
        const triangles::Aabb3 bounds = triangles::Bounds(triangle);

        EXPECT_DOUBLE_EQ(bounds.minimum.cord_x, -2.0);
        EXPECT_DOUBLE_EQ(bounds.minimum.cord_y, -1.0);
        EXPECT_DOUBLE_EQ(bounds.minimum.cord_z, -3.0);
        EXPECT_DOUBLE_EQ(bounds.maximum.cord_x, 3.0);
        EXPECT_DOUBLE_EQ(bounds.maximum.cord_y, 5.0);
        EXPECT_DOUBLE_EQ(bounds.maximum.cord_z, 4.0);
    }
    while (std::next_permutation(order.begin(), order.end()));
}

TEST(GeometryTests, BoundsAllowsZeroThickness)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{0.0, 0.0, 5.0},
        triangles::Point3{2.0, 0.0, 5.0},
        triangles::Point3{0.0, 3.0, 5.0}
    }};
    const triangles::Aabb3 bounds = triangles::Bounds(triangle);

    EXPECT_DOUBLE_EQ(bounds.minimum.cord_x, 0.0);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_z, 5.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_x, 2.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_y, 3.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_z, 5.0);
}

TEST(GeometryTests, BoundsOfPointTriangleHasCoincidentCorners)
{
    const triangles::Point3 point{-2.0, 4.0, 1.0};
    const triangles::Triangle3 triangle{{point, point, point}};
    const triangles::Aabb3 bounds = triangles::Bounds(triangle);

    EXPECT_DOUBLE_EQ(bounds.minimum.cord_x, -2.0);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_y, 4.0);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_z, 1.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_x, -2.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_y, 4.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_z, 1.0);
}

TEST(GeometryTests, BoundsHandlesCollinearVertices)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{1.0, 2.0, 3.0},
        triangles::Point3{-1.0, -2.0, -3.0},
        triangles::Point3{0.0, 0.0, 0.0}
    }};
    const triangles::Aabb3 bounds = triangles::Bounds(triangle);

    EXPECT_DOUBLE_EQ(bounds.minimum.cord_x, -1.0);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_y, -2.0);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_z, -3.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_x, 1.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_y, 2.0);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_z, 3.0);
}

TEST(GeometryTests, BoundsHandlesExtremeFiniteCoordinates)
{
    const double maximum = std::numeric_limits<double>::max();
    const triangles::Triangle3 triangle{{
        triangles::Point3{-maximum, 0.0, maximum},
        triangles::Point3{maximum, -maximum, 0.0},
        triangles::Point3{0.0, maximum, -maximum}
    }};
    const triangles::Aabb3 bounds = triangles::Bounds(triangle);

    EXPECT_DOUBLE_EQ(bounds.minimum.cord_x, -maximum);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_y, -maximum);
    EXPECT_DOUBLE_EQ(bounds.minimum.cord_z, -maximum);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_x, maximum);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_y, maximum);
    EXPECT_DOUBLE_EQ(bounds.maximum.cord_z, maximum);
}

TEST (SignedDistanceToPlaneTest, SimpleTest)
{
    const triangles:: Point3 point1{1.0, 1.0, 3.0};
    const triangles:: Point3 point2{1.0, 1.0, -2.0};
    const triangles:: Point3 point3{10.0, 10.0, 0.0};
    const triangles:: Triangle3 triangle{{
        triangles::Point3{0.0,0.0,0.0}, 
        triangles::Point3{2.0,0.0,0.0}, 
        triangles::Point3{0.0,2.0,0.0}
    }};

    EXPECT_DOUBLE_EQ(triangles:: SignedDistanceToPlane (point1, triangle), 3.0);
    EXPECT_DOUBLE_EQ(triangles:: SignedDistanceToPlane (point2, triangle), -2.0);
    EXPECT_DOUBLE_EQ(triangles:: SignedDistanceToPlane (point3, triangle), 0.0);

}

TEST(SignedDistanceToPlaneTest, HandlesPlaneAwayFromOrigin)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{1.0, 2.0, 5.0},
        triangles::Point3{3.0, 2.0, 5.0},
        triangles::Point3{1.0, 5.0, 5.0}
    }};

    EXPECT_DOUBLE_EQ(triangles::SignedDistanceToPlane({10.0, -8.0, 8.0}, triangle), 3.0);
    EXPECT_DOUBLE_EQ(triangles::SignedDistanceToPlane({-10.0, 8.0, 3.0}, triangle), -2.0);
}

TEST(SignedDistanceToPlaneTest, HandlesVerticalPlane)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{4.0, 1.0, 2.0},
        triangles::Point3{4.0, 3.0, 2.0},
        triangles::Point3{4.0, 1.0, 5.0}
    }};

    EXPECT_DOUBLE_EQ(triangles::SignedDistanceToPlane({7.0, 0.0, 0.0}, triangle), 3.0);
    EXPECT_DOUBLE_EQ(triangles::SignedDistanceToPlane({2.0, 0.0, 0.0}, triangle), -2.0);
    EXPECT_DOUBLE_EQ(triangles::SignedDistanceToPlane({4.0, -8.0, 12.0}, triangle), 0.0);
}

TEST(SignedDistanceToPlaneTest, HandlesObliquePlane)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{1.0, 0.0, 0.0},
        triangles::Point3{0.0, 1.0, 0.0},
        triangles::Point3{0.0, 0.0, 1.0}
    }};

    EXPECT_NEAR(triangles::SignedDistanceToPlane({2.0, 2.0, 2.0}, triangle),
                5.0 / std::sqrt(3.0), 1e-12);
    EXPECT_NEAR(triangles::SignedDistanceToPlane({0.0, 0.0, 0.0}, triangle),
                -1.0 / std::sqrt(3.0), 1e-12);
}

TEST(SignedDistanceToPlaneTest, ReturnsZeroOnVerticesAndPlaneOutsideTriangle)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{1.0, 2.0, 3.0},
        triangles::Point3{3.0, 2.0, 3.0},
        triangles::Point3{1.0, 4.0, 5.0}
    }};

    for (const triangles::Point3& vertex : triangle.vertices)
        EXPECT_NEAR(triangles::SignedDistanceToPlane(vertex, triangle), 0.0, 1e-12);
    EXPECT_NEAR(triangles::SignedDistanceToPlane({11.0, 12.0, 13.0}, triangle), 0.0, 1e-12);
}

TEST(SignedDistanceToPlaneTest, ReversingWindingReversesSign)
{
    const triangles::Point3 a{0.0, 0.0, 0.0};
    const triangles::Point3 b{2.0, 0.0, 0.0};
    const triangles::Point3 c{0.0, 2.0, 0.0};
    const triangles::Triangle3 original{{a, b, c}};
    const triangles::Triangle3 reversed{{a, c, b}};
    const triangles::Point3 point{1.0, 1.0, 3.0};

    EXPECT_DOUBLE_EQ(triangles::SignedDistanceToPlane(point, original), 3.0);
    EXPECT_DOUBLE_EQ(triangles::SignedDistanceToPlane(point, reversed), -3.0);
}

TEST(SignedDistanceToPlaneTest, CyclicPermutationPreservesSignedDistance)
{
    const triangles::Point3 a{1.0, 0.0, 0.0};
    const triangles::Point3 b{0.0, 1.0, 0.0};
    const triangles::Point3 c{0.0, 0.0, 1.0};
    const triangles::Triangle3 orders[]{{{a, b, c}}, {{b, c, a}}, {{c, a, b}}};

    for (const triangles::Triangle3& triangle : orders)
        EXPECT_NEAR(triangles::SignedDistanceToPlane({2.0, 2.0, 2.0}, triangle),
                    5.0 / std::sqrt(3.0), 1e-12);
}

TEST(SignedDistanceToPlaneTest, TriangleSizeDoesNotScaleDistance)
{
    const triangles::Triangle3 large{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{4.0, 0.0, 0.0},
        triangles::Point3{0.0, 6.0, 0.0}
    }};
    const triangles::Triangle3 small{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{0.25, 0.0, 0.0},
        triangles::Point3{0.0, 0.5, 0.0}
    }};

    EXPECT_DOUBLE_EQ(triangles::SignedDistanceToPlane({1.0, 1.0, 4.0}, large), 4.0);
    EXPECT_DOUBLE_EQ(triangles::SignedDistanceToPlane({1.0, 1.0, 4.0}, small), 4.0);
}

TEST(SignedDistanceToPlaneTest, TranslatingPointAndTrianglePreservesDistance)
{
    const triangles::Triangle3 original{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{2.0, 0.0, 0.0},
        triangles::Point3{0.0, 2.0, 2.0}
    }};
    const triangles::Triangle3 translated{{
        triangles::Point3{8.0, -4.0, 16.0},
        triangles::Point3{10.0, -4.0, 16.0},
        triangles::Point3{8.0, -2.0, 18.0}
    }};

    EXPECT_NEAR(triangles::SignedDistanceToPlane({1.0, 0.0, 2.0}, original),
                std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(triangles::SignedDistanceToPlane({9.0, -4.0, 18.0}, translated),
                std::sqrt(2.0), 1e-12);
}

TEST(SignedDistanceToPlaneTest, RejectsCollinearTriangle)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{1.0, 0.0, 0.0},
        triangles::Point3{2.0, 0.0, 0.0}
    }};

    EXPECT_THROW(triangles::SignedDistanceToPlane({0.0, 1.0, 0.0}, triangle), std::domain_error);
}

TEST(SignedDistanceToPlaneTest, RejectsTriangleWithRepeatedVertex)
{
    const triangles::Triangle3 triangle{{
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{0.0, 0.0, 0.0},
        triangles::Point3{0.0, 2.0, 0.0}
    }};

    EXPECT_THROW(triangles::SignedDistanceToPlane({0.0, 0.0, 1.0}, triangle), std::domain_error);
}

TEST(DominantAxisTest, SelectsXWhenItsMagnitudeIsLargest)
{
    EXPECT_EQ(triangles::DominantAxis({3.0, 1.0, 2.0}), triangles::Axis::X);
}

TEST(DominantAxisTest, SelectsNegativeYWhenItsMagnitudeIsLargest)
{
    EXPECT_EQ(triangles::DominantAxis({0.01, -0.9, 0.2}), triangles::Axis::Y);
}

TEST(DominantAxisTest, SelectsZWhenItsMagnitudeIsLargest)
{
    EXPECT_EQ(triangles::DominantAxis({1.0, 2.0, 3.0}), triangles::Axis::Z);
}

TEST(DominantAxisTest, SelectsNegativeXWhenItsMagnitudeIsLargest)
{
    EXPECT_EQ(triangles::DominantAxis({-3.0, 1.0, 2.0}), triangles::Axis::X);
}

TEST(DominantAxisTest, SelectsNegativeZWhenItsMagnitudeIsLargest)
{
    EXPECT_EQ(triangles::DominantAxis({1.0, 2.0, -3.0}), triangles::Axis::Z);
}

TEST(DominantAxisTest, HandlesAxisAlignedNormalsInBothDirections)
{
    EXPECT_EQ(triangles::DominantAxis({1.0, 0.0, 0.0}), triangles::Axis::X);
    EXPECT_EQ(triangles::DominantAxis({-1.0, 0.0, 0.0}), triangles::Axis::X);
    EXPECT_EQ(triangles::DominantAxis({0.0, 1.0, 0.0}), triangles::Axis::Y);
    EXPECT_EQ(triangles::DominantAxis({0.0, -1.0, 0.0}), triangles::Axis::Y);
    EXPECT_EQ(triangles::DominantAxis({0.0, 0.0, 1.0}), triangles::Axis::Z);
    EXPECT_EQ(triangles::DominantAxis({0.0, 0.0, -1.0}), triangles::Axis::Z);
}

TEST(DominantAxisTest, PrefersXWhenXAndYTieForMaximumMagnitude)
{
    EXPECT_EQ(triangles::DominantAxis({2.0, -2.0, 1.0}), triangles::Axis::X);
    EXPECT_EQ(triangles::DominantAxis({-2.0, 2.0, 1.0}), triangles::Axis::X);
}

TEST(DominantAxisTest, PrefersXWhenXAndZTieForMaximumMagnitude)
{
    EXPECT_EQ(triangles::DominantAxis({-3.0, 1.0, 3.0}), triangles::Axis::X);
}

TEST(DominantAxisTest, PrefersYWhenYAndZTieForMaximumMagnitude)
{
    EXPECT_EQ(triangles::DominantAxis({1.0, -3.0, 3.0}), triangles::Axis::Y);
}

TEST(DominantAxisTest, PrefersXWhenAllMagnitudesTie)
{
    EXPECT_EQ(triangles::DominantAxis({-2.0, 2.0, -2.0}), triangles::Axis::X);
}

TEST(DominantAxisTest, HandlesFractionalComponents)
{
    EXPECT_EQ(triangles::DominantAxis({0.125, -0.5, 0.25}), triangles::Axis::Y);
}

TEST(DominantAxisTest, HandlesExtremeFiniteComponents)
{
    const double maximum = std::numeric_limits<double>::max();

    EXPECT_EQ(triangles::DominantAxis({-maximum, maximum / 2.0, 1.0}), triangles::Axis::X);
    EXPECT_EQ(triangles::DominantAxis({1.0, -maximum, maximum / 2.0}), triangles::Axis::Y);
    EXPECT_EQ(triangles::DominantAxis({1.0, maximum / 2.0, -maximum}), triangles::Axis::Z);
}

TEST(DominantAxisTest, HandlesTinyNonzeroComponents)
{
    EXPECT_EQ(triangles::DominantAxis({1e-200, -3e-200, 2e-200}), triangles::Axis::Y);
}
