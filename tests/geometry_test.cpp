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

namespace
{
    void ExpectTriangleKindInEveryOrder(const triangles::Triangle3& triangle,
                                       triangles::TriangleKind expected)
    {
        std::array<std::size_t, 3> order{0, 1, 2};
        do
        {
            SCOPED_TRACE(::testing::Message() << "vertex order: "
                         << order[0] << order[1] << order[2]);
            const triangles::Triangle3 permuted{{triangle.vertices[order[0]],
                                                 triangle.vertices[order[1]],
                                                 triangle.vertices[order[2]]}};
            EXPECT_EQ(triangles::ClassifyTriangle(permuted), expected);
        } while (std::next_permutation(order.begin(), order.end()));
    }
}

TEST(ClassifyTriangleTest, CoincidentVerticesProducePoint)
{
    const triangles::Point3 point{-2.0, 3.0, 4.0};
    ExpectTriangleKindInEveryOrder({{point, point, point}}, triangles::TriangleKind::Point);
}

TEST(ClassifyTriangleTest, SignedZerosDenoteSamePoint)
{
    const triangles::Triangle3 triangle{{triangles::Point3{0.0, -0.0, 0.0},
                                         triangles::Point3{-0.0, 0.0, -0.0},
                                         triangles::Point3{0.0, 0.0, 0.0}}};
    ExpectTriangleKindInEveryOrder(triangle, triangles::TriangleKind::Point);
}

TEST(ClassifyTriangleTest, RepeatedVertexAndDistinctVertexProduceSegment)
{
    const triangles::Point3 a{1.0, -2.0, 3.0};
    const triangles::Point3 b{-4.0, 5.0, 6.0};
    ExpectTriangleKindInEveryOrder({{a, a, b}}, triangles::TriangleKind::Segment);
}

TEST(ClassifyTriangleTest, DistinctCollinearVerticesProduceSegment)
{
    const triangles::Triangle3 triangle{{triangles::Point3{-4.0, 2.0, 9.0},
                                         triangles::Point3{0.0, 4.0, 3.0},
                                         triangles::Point3{6.0, 7.0, -6.0}}};
    ExpectTriangleKindInEveryOrder(triangle, triangles::TriangleKind::Segment);
}

TEST(ClassifyTriangleTest, HandlesLinesParallelToEveryCoordinateAxis)
{
    triangles::Triangle3 triangle{{triangles::Point3{-2.0, 3.0, 4.0},
                                    triangles::Point3{0.0, 3.0, 4.0},
                                    triangles::Point3{5.0, 3.0, 4.0}}};
    for (int rotation = 0; rotation < 3; ++rotation)
    {
        SCOPED_TRACE(rotation);
        ExpectTriangleKindInEveryOrder(triangle, triangles::TriangleKind::Segment);
        for (auto& point : triangle.vertices)
            point = {point.cord_y, point.cord_z, point.cord_x};
    }
}

TEST(ClassifyTriangleTest, HandlesTrianglesInEveryCoordinatePlane)
{
    triangles::Triangle3 triangle{{triangles::Point3{0.0, 0.0, 0.0},
                                    triangles::Point3{2.0, 0.0, 0.0},
                                    triangles::Point3{0.0, 3.0, 0.0}}};
    for (int rotation = 0; rotation < 3; ++rotation)
    {
        SCOPED_TRACE(rotation);
        ExpectTriangleKindInEveryOrder(triangle, triangles::TriangleKind::Triangle);
        for (auto& point : triangle.vertices)
            point = {point.cord_y, point.cord_z, point.cord_x};
    }
}

TEST(ClassifyTriangleTest, HandlesObliqueTriangleAwayFromOrigin)
{
    const triangles::Triangle3 triangle{{triangles::Point3{5.0, -2.0, 7.0},
                                         triangles::Point3{6.0, -1.0, 7.0},
                                         triangles::Point3{5.0, -1.0, 8.0}}};
    ExpectTriangleKindInEveryOrder(triangle, triangles::TriangleKind::Triangle);
}

TEST(ClassifyTriangleTest, PreservesThinTriangleBeyondOrdinaryDeterminantPrecision)
{
    for (const int exponent : {27, 40, 52})
    {
        const double n = std::scalbn(1.0, exponent);
        triangles::Triangle3 triangle{{triangles::Point3{0.0, 0.0, 0.0},
                                        triangles::Point3{n + 1.0, n, 0.0},
                                        triangles::Point3{n, n - 1.0, 0.0}}};
        for (int rotation = 0; rotation < 3; ++rotation)
        {
            SCOPED_TRACE(::testing::Message() << "exponent: " << exponent
                         << ", rotation: " << rotation);
            ExpectTriangleKindInEveryOrder(triangle, triangles::TriangleKind::Triangle);
            for (auto& point : triangle.vertices)
                point = {point.cord_y, point.cord_z, point.cord_x};
        }
    }
}

TEST(ClassifyTriangleTest, NearbyDistinctVerticesAreNotMergedByEpsilon)
{
    const double tiny = std::scalbn(1.0, -100);
    const triangles::Triangle3 triangle{{triangles::Point3{0.0, 0.0, 0.0},
                                         triangles::Point3{tiny, 0.0, 0.0},
                                         triangles::Point3{0.0, tiny, 0.0}}};
    ExpectTriangleKindInEveryOrder(triangle, triangles::TriangleKind::Triangle);
    ExpectTriangleKindInEveryOrder({{triangle.vertices[0], triangle.vertices[0], triangle.vertices[1]}},
                                  triangles::TriangleKind::Segment);
}

TEST(ClassifyTriangleTest, HandlesUniformlyTinyAndHugeCoordinates)
{
    for (const int exponent : {-900, 900})
    {
        SCOPED_TRACE(exponent);
        const double scale = std::scalbn(1.0, exponent);
        const triangles::Point3 a{0.0, 0.0, 0.0};
        const triangles::Point3 b{scale, 2.0 * scale, 3.0 * scale};
        const triangles::Point3 c{2.0 * scale, 4.0 * scale, 6.0 * scale};
        ExpectTriangleKindInEveryOrder({{a, b, c}}, triangles::TriangleKind::Segment);
        ExpectTriangleKindInEveryOrder({{a, b, {2.0 * scale, 4.0 * scale, 7.0 * scale}}},
                                      triangles::TriangleKind::Triangle);
    }
}

TEST(ClassifyTriangleTest, RejectsNonFiniteCoordinatesInEveryVertexAndComponent)
{
    const triangles::Triangle3 original{{triangles::Point3{0.0, 0.0, 0.0},
                                         triangles::Point3{1.0, 0.0, 0.0},
                                         triangles::Point3{0.0, 1.0, 0.0}}};
    for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()})
    {
        for (std::size_t vertex = 0; vertex < 3; ++vertex)
        {
            for (std::size_t component = 0; component < 3; ++component)
            {
                auto triangle = original;
                auto& point = triangle.vertices[vertex];
                const std::array<double*, 3> coordinates{&point.cord_x, &point.cord_y, &point.cord_z};
                *coordinates[component] = value;
                EXPECT_THROW(triangles::ClassifyTriangle(triangle), std::invalid_argument);
            }
        }
        const triangles::Point3 invalid{value, value, value};
        const triangles::Triangle3 coincident{{invalid, invalid, invalid}};
        EXPECT_THROW(triangles::ClassifyTriangle(coincident), std::invalid_argument);
    }
}

TEST(ClassifyTriangleTest, SupportsExponentSpreadBoundary)
{
    const triangles::Triangle3 triangle{{triangles::Point3{0.0, 0.0, 0.0},
                                         triangles::Point3{1.0, 0.0, 0.0},
                                         triangles::Point3{0.0, std::scalbn(1.0, -256), 0.0}}};
    ExpectTriangleKindInEveryOrder(triangle, triangles::TriangleKind::Triangle);
}

TEST(ClassifyTriangleTest, RejectsUnsupportedExponentSpread)
{
    const triangles::Triangle3 triangle{{triangles::Point3{0.0, 0.0, 0.0},
                                         triangles::Point3{1.0, 0.0, 0.0},
                                         triangles::Point3{0.0, std::scalbn(1.0, -257), 0.0}}};
    EXPECT_THROW(triangles::ClassifyTriangle(triangle), std::range_error);
}

namespace
{
    void ExpectDegenerateSegmentInEveryOrder(const triangles::Triangle3& triangle,
                                            const triangles::Segment3& expected)
    {
        std::array<std::size_t, 3> order{0, 1, 2};
        do
        {
            SCOPED_TRACE(::testing::Message() << "vertex order: "
                         << order[0] << order[1] << order[2]);
            const triangles::Triangle3 permuted{{triangle.vertices[order[0]],
                                                 triangle.vertices[order[1]],
                                                 triangle.vertices[order[2]]}};
            const auto segment = triangles::DegenerateTriangleSegment(permuted);
            EXPECT_DOUBLE_EQ(segment.start.cord_x, expected.start.cord_x);
            EXPECT_DOUBLE_EQ(segment.start.cord_y, expected.start.cord_y);
            EXPECT_DOUBLE_EQ(segment.start.cord_z, expected.start.cord_z);
            EXPECT_DOUBLE_EQ(segment.end.cord_x, expected.end.cord_x);
            EXPECT_DOUBLE_EQ(segment.end.cord_y, expected.end.cord_y);
            EXPECT_DOUBLE_EQ(segment.end.cord_z, expected.end.cord_z);
        } while (std::next_permutation(order.begin(), order.end()));
    }
}

TEST(DegenerateTriangleSegmentTest, ChoosesOuterVerticesInsteadOfFirstTwo)
{
    const triangles::Triangle3 triangle{{triangles::Point3{2.0, 0.0, 0.0},
                                         triangles::Point3{0.0, 0.0, 0.0},
                                         triangles::Point3{5.0, 0.0, 0.0}}};
    ExpectDegenerateSegmentInEveryOrder(triangle, {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}});
}

TEST(DegenerateTriangleSegmentTest, HandlesRepeatedVertex)
{
    const triangles::Point3 first{-1.0, -2.0, -3.0};
    const triangles::Point3 last{3.0, 4.0, 5.0};
    ExpectDegenerateSegmentInEveryOrder({{last, last, first}}, {first, last});
}

TEST(DegenerateTriangleSegmentTest, ConvertsPointToZeroLengthSegment)
{
    const triangles::Point3 point{7.0, -3.0, 2.0};
    ExpectDegenerateSegmentInEveryOrder({{point, point, point}}, {point, point});
}

TEST(DegenerateTriangleSegmentTest, HandlesEveryCoordinateAxis)
{
    triangles::Triangle3 triangle{{triangles::Point3{0.0, 3.0, 4.0},
                                    triangles::Point3{-2.0, 3.0, 4.0},
                                    triangles::Point3{5.0, 3.0, 4.0}}};
    for (int rotation = 0; rotation < 3; ++rotation)
    {
        SCOPED_TRACE(rotation);
        ExpectDegenerateSegmentInEveryOrder(triangle, {triangle.vertices[1], triangle.vertices[2]});
        for (auto& point : triangle.vertices)
            point = {point.cord_y, point.cord_z, point.cord_x};
    }
}

TEST(DegenerateTriangleSegmentTest, PreservesWholeEndpointsOfObliqueLine)
{
    const triangles::Point3 first{-4.0, 8.0, -12.0};
    const triangles::Point3 middle{1.0, -2.0, 3.0};
    const triangles::Point3 last{3.0, -6.0, 9.0};
    ExpectDegenerateSegmentInEveryOrder({{middle, last, first}}, {first, last});
}

TEST(DegenerateTriangleSegmentTest, DoesNotOverflowWhenEndpointDifferenceWouldOverflow)
{
    const double maximum = std::numeric_limits<double>::max();
    const triangles::Point3 first{-maximum, 0.0, 0.0};
    const triangles::Point3 last{maximum, 0.0, 0.0};
    ExpectDegenerateSegmentInEveryOrder({{first, {0.0, 0.0, 0.0}, last}}, {first, last});
}

TEST(DegenerateTriangleSegmentTest, PreservesDistinctSubnormalVertices)
{
    const double tiny = std::numeric_limits<double>::denorm_min();
    const triangles::Point3 first{0.0, 0.0, 0.0};
    const triangles::Point3 last{2.0 * tiny, 0.0, 0.0};
    // Compare exactly: tolerant floating comparisons could merge these points.
    const triangles::Triangle3 triangle{{triangles::Point3{tiny, 0.0, 0.0}, first, last}};
    const auto segment = triangles::DegenerateTriangleSegment(triangle);
    EXPECT_EQ(segment.start.cord_x, first.cord_x);
    EXPECT_EQ(segment.end.cord_x, last.cord_x);
}

TEST(DegenerateTriangleSegmentTest, RejectsRegularAndVeryThinTriangles)
{
    const double n = 1099511627776.0;
    const triangles::Triangle3 regular{{triangles::Point3{0.0, 0.0, 0.0},
                                        triangles::Point3{1.0, 0.0, 0.0},
                                        triangles::Point3{0.0, 1.0, 0.0}}};
    const triangles::Triangle3 thin{{triangles::Point3{0.0, 0.0, 0.0},
                                     triangles::Point3{n + 1.0, n, 0.0},
                                     triangles::Point3{n, n - 1.0, 0.0}}};
    EXPECT_THROW(triangles::DegenerateTriangleSegment(regular), std::invalid_argument);
    EXPECT_THROW(triangles::DegenerateTriangleSegment(thin), std::invalid_argument);
}

TEST(DegenerateTriangleSegmentTest, RejectsNonFiniteCoordinates)
{
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity()})
    {
        const triangles::Point3 point{invalid, 0.0, 0.0};
        const triangles::Triangle3 triangle{{point, point, point}};
        EXPECT_THROW(triangles::DegenerateTriangleSegment(triangle), std::invalid_argument);
    }
}

TEST(DegenerateTriangleSegmentTest, PreservesClassificationRangeRestriction)
{
    const double tiny = std::scalbn(1.0, -257);
    const triangles::Triangle3 triangle{{triangles::Point3{0.0, 0.0, 0.0},
                                         triangles::Point3{1.0, tiny, 0.0},
                                         triangles::Point3{2.0, 2.0 * tiny, 0.0}}};
    EXPECT_THROW(triangles::DegenerateTriangleSegment(triangle), std::range_error);
}

namespace
{
    void ExpectPointOnSegmentInBothDirections(const triangles::Point3& point,
                                             const triangles::Segment3& segment,
                                             bool expected)
    {
        EXPECT_EQ(triangles::PointOnSegment(point, segment), expected);
        EXPECT_EQ(triangles::PointOnSegment(point, {segment.end, segment.start}), expected);
    }
}

TEST(PointOnSegment3DTest, AcceptsInteriorPointOfObliqueSegment)
{
    ExpectPointOnSegmentInBothDirections({1.0, -2.0, 3.0},
                                         {{-4.0, 8.0, -12.0}, {3.0, -6.0, 9.0}}, true);
}

TEST(PointOnSegment3DTest, IncludesBothEndpoints)
{
    const triangles::Segment3 segment{{-2.0, 4.0, 7.0}, {6.0, -4.0, 3.0}};
    ExpectPointOnSegmentInBothDirections(segment.start, segment, true);
    ExpectPointOnSegmentInBothDirections(segment.end, segment, true);
}

TEST(PointOnSegment3DTest, RejectsCollinearPointsBeyondEitherEndpoint)
{
    const triangles::Segment3 segment{{1.0, 2.0, 3.0}, {3.0, 6.0, 9.0}};
    ExpectPointOnSegmentInBothDirections({0.0, 0.0, 0.0}, segment, false);
    ExpectPointOnSegmentInBothDirections({4.0, 8.0, 12.0}, segment, false);
}

TEST(PointOnSegment3DTest, BoundingBoxMembershipAloneIsInsufficient)
{
    triangles::Point3 point{1.0, 1.0, 0.5};
    const triangles::Segment3 segment{{0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}};
    for (int rotation = 0; rotation < 3; ++rotation)
    {
        SCOPED_TRACE(rotation);
        ExpectPointOnSegmentInBothDirections(point, segment, false);
        point = {point.cord_y, point.cord_z, point.cord_x};
    }
}

TEST(PointOnSegment3DTest, HandlesEveryCoordinateAxisAndFixedCoordinates)
{
    triangles::Segment3 segment{{-2.0, 3.0, 4.0}, {5.0, 3.0, 4.0}};
    triangles::Point3 on{0.0, 3.0, 4.0};
    triangles::Point3 off{0.0, 3.0, 5.0};
    for (int rotation = 0; rotation < 3; ++rotation)
    {
        SCOPED_TRACE(rotation);
        ExpectPointOnSegmentInBothDirections(on, segment, true);
        ExpectPointOnSegmentInBothDirections(off, segment, false);
        on = {on.cord_y, on.cord_z, on.cord_x};
        off = {off.cord_y, off.cord_z, off.cord_x};
        segment.start = {segment.start.cord_y, segment.start.cord_z, segment.start.cord_x};
        segment.end = {segment.end.cord_y, segment.end.cord_z, segment.end.cord_x};
    }
}

TEST(PointOnSegment3DTest, ZeroLengthSegmentContainsOnlyItsOwnPoint)
{
    const triangles::Point3 point{0.0, 0.0, 0.0};
    const triangles::Segment3 segment{point, point};
    ExpectPointOnSegmentInBothDirections(point, segment, true);
    const double tiny = std::scalbn(1.0, -100);
    for (const triangles::Point3& other : {triangles::Point3{tiny, 0.0, 0.0},
                                          triangles::Point3{0.0, tiny, 0.0},
                                          triangles::Point3{0.0, 0.0, tiny}})
        ExpectPointOnSegmentInBothDirections(other, segment, false);
}

TEST(PointOnSegment3DTest, RejectsNearCollinearityBeyondOrdinaryDeterminantPrecision)
{
    for (const int exponent : {27, 40, 52})
    {
        const double n = std::scalbn(1.0, exponent);
        triangles::Point3 point{n, n - 1.0, 0.0};
        triangles::Segment3 segment{{0.0, 0.0, 0.0}, {n + 1.0, n, 0.0}};
        for (int rotation = 0; rotation < 3; ++rotation)
        {
            SCOPED_TRACE(::testing::Message() << "exponent: " << exponent << ", rotation: " << rotation);
            ExpectPointOnSegmentInBothDirections(point, segment, false);
            point = {point.cord_y, point.cord_z, point.cord_x};
            segment.end = {segment.end.cord_y, segment.end.cord_z, segment.end.cord_x};
        }
    }
}

TEST(PointOnSegment3DTest, AvoidsOverflowForExtremeOppositeEndpoints)
{
    const double maximum = std::numeric_limits<double>::max();
    const triangles::Segment3 segment{{-maximum, 0.0, 0.0}, {maximum, 0.0, 0.0}};
    ExpectPointOnSegmentInBothDirections({0.0, 0.0, 0.0}, segment, true);
    ExpectPointOnSegmentInBothDirections(segment.start, segment, true);
    ExpectPointOnSegmentInBothDirections(segment.end, segment, true);
}

TEST(PointOnSegment3DTest, HandlesUniformlyTinyAndHugeScenes)
{
    for (const int exponent : {-900, 900})
    {
        SCOPED_TRACE(exponent);
        const double scale = std::scalbn(1.0, exponent);
        const triangles::Segment3 segment{{0.0, 0.0, 0.0}, {2.0 * scale, 2.0 * scale, 2.0 * scale}};
        ExpectPointOnSegmentInBothDirections({scale, scale, scale}, segment, true);
        ExpectPointOnSegmentInBothDirections({3.0 * scale, 3.0 * scale, 3.0 * scale}, segment, false);
        ExpectPointOnSegmentInBothDirections({scale, scale, 1.5 * scale}, segment, false);
    }
}

TEST(PointOnSegment3DTest, SupportsExponentSpreadBoundaryWithoutEpsilon)
{
    const double tiny = std::scalbn(1.0, -256);
    const triangles::Segment3 segment{{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}};
    ExpectPointOnSegmentInBothDirections({tiny, tiny, 0.0}, segment, true);
    ExpectPointOnSegmentInBothDirections({-tiny, -tiny, 0.0}, segment, false);
    ExpectPointOnSegmentInBothDirections({tiny, 2.0 * tiny, 0.0}, segment, false);
}

TEST(PointOnSegment3DTest, RejectsNonFiniteCoordinatesInEveryInputPoint)
{
    const std::array<triangles::Point3, 3> original{
        triangles::Point3{0.0, 0.0, 0.0}, triangles::Point3{2.0, 2.0, 2.0}, triangles::Point3{1.0, 1.0, 1.0}
    };
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity(),
                                -std::numeric_limits<double>::infinity()})
    {
        for (std::size_t index = 0; index < original.size(); ++index)
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                auto points = original;
                auto& point = points[index];
                const std::array<double*, 3> coordinates{&point.cord_x, &point.cord_y, &point.cord_z};
                *coordinates[axis] = invalid;
                const triangles::Segment3 segment{points[0], points[1]};
                EXPECT_THROW(triangles::PointOnSegment(points[2], segment), std::invalid_argument);
            }
        }
    }
}

TEST(PointOnSegment3DTest, PreservesClassificationRangeRestriction)
{
    const triangles::Point3 point{0.0, std::scalbn(1.0, -257), 0.0};
    const triangles::Segment3 segment{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    EXPECT_THROW(triangles::PointOnSegment(point, segment), std::range_error);
}
