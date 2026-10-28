#include "geometry.hpp"

#include <gtest/gtest.h>

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
