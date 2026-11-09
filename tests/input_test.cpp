#include "input.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

TEST(InputTests, AcceptsSmallestCount)
{
    std::istringstream input("1");
    EXPECT_EQ(triangles::ReadTriangleCount(input), std::size_t{1});
}

TEST(InputTests, AcceptsTypicalCount)
{
    std::istringstream input("34");
    EXPECT_EQ(triangles::ReadTriangleCount(input), std::size_t{34});
}

TEST(InputTests, AcceptsLargestCount)
{
    std::istringstream input("999999");
    EXPECT_EQ(triangles::ReadTriangleCount(input), std::size_t{999999});
}

TEST(InputTests, AcceptsLeadingWhitespace)
{
    std::istringstream input(" \n\t 2");
    EXPECT_EQ(triangles::ReadTriangleCount(input), std::size_t{2});
}

TEST(InputTests, RejectsZero)
{
    std::istringstream input("0");
    EXPECT_THROW(triangles::ReadTriangleCount(input), std::invalid_argument);
}

TEST(InputTests, RejectsNegativeCount)
{
    std::istringstream input("-1");
    EXPECT_THROW(triangles::ReadTriangleCount(input), std::invalid_argument);
}

TEST(InputTests, RejectsUpperBoundary)
{
    std::istringstream input("1000000");
    EXPECT_THROW(triangles::ReadTriangleCount(input), std::invalid_argument);
}

TEST(InputTests, RejectsEmptyInput)
{
    std::istringstream input("");
    EXPECT_THROW(triangles::ReadTriangleCount(input), std::invalid_argument);
}

TEST(InputTests, RejectsWhitespaceOnly)
{
    std::istringstream input(" \t\n ");
    EXPECT_THROW(triangles::ReadTriangleCount(input), std::invalid_argument);
}

TEST(InputTests, RejectsNonNumericToken)
{
    std::istringstream input("abc");
    EXPECT_THROW(triangles::ReadTriangleCount(input), std::invalid_argument);
}

TEST(InputTests, RejectsNumericPrefixWithSuffix)
{
    std::istringstream input("3abc");
    EXPECT_THROW(triangles::ReadTriangleCount(input), std::invalid_argument);
}

TEST(InputTests, RejectsFractionalCount)
{
    std::istringstream input("2.5");
    EXPECT_THROW(triangles::ReadTriangleCount(input), std::invalid_argument);
}

TEST(InputTests, RejectsOverflow)
{
    std::istringstream input(std::string(100, '9'));
    EXPECT_THROW(triangles::ReadTriangleCount(input), std::invalid_argument);
}

TEST(InputTests, PreservesFollowingCoordinates)
{
    std::istringstream input("2 -1.5 0 3.25");
    ASSERT_EQ(triangles::ReadTriangleCount(input), std::size_t{2});

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    ASSERT_TRUE(input >> x >> y >> z);
    EXPECT_DOUBLE_EQ(x, -1.5);
    EXPECT_DOUBLE_EQ(y, 0.0);
    EXPECT_DOUBLE_EQ(z, 3.25);
}

namespace
{
    void ExpectTriangleCoordinates(const triangles::Triangle3& actual, const triangles::Triangle3& expected)
    {
        for (std::size_t vertex = 0; vertex < expected.vertices.size(); ++vertex)
        {
            SCOPED_TRACE(vertex);
            EXPECT_DOUBLE_EQ(actual.vertices[vertex].cord_x, expected.vertices[vertex].cord_x);
            EXPECT_DOUBLE_EQ(actual.vertices[vertex].cord_y, expected.vertices[vertex].cord_y);
            EXPECT_DOUBLE_EQ(actual.vertices[vertex].cord_z, expected.vertices[vertex].cord_z);
        }
    }

    void ExpectInvalidPointTokenInEveryCoordinate(const std::string& token)
    {
        for (std::size_t position = 0; position < 3; ++position)
        {
            SCOPED_TRACE(::testing::Message() << "token=" << token << ", position=" << position);
            std::array<std::string, 3> coordinates{"1", "2", "3"};
            coordinates[position] = token;
            std::istringstream input(coordinates[0] + " " + coordinates[1] + " " + coordinates[2]);
            EXPECT_THROW(triangles::ReadPoint(input), std::invalid_argument);
        }
    }
}

TEST(ReadPointTest, AcceptsIntegersAndNegativeFractions)
{
    std::istringstream input("1 -2.5 3");
    const triangles::Point3 point = triangles::ReadPoint(input);
    EXPECT_DOUBLE_EQ(point.cord_x, 1.0);
    EXPECT_DOUBLE_EQ(point.cord_y, -2.5);
    EXPECT_DOUBLE_EQ(point.cord_z, 3.0);
}

TEST(ReadPointTest, AcceptsScientificNotation)
{
    std::istringstream input("1e2 -2.5E-2 3e+2");
    const triangles::Point3 point = triangles::ReadPoint(input);
    EXPECT_DOUBLE_EQ(point.cord_x, 100.0);
    EXPECT_DOUBLE_EQ(point.cord_y, -0.025);
    EXPECT_DOUBLE_EQ(point.cord_z, 300.0);
}

TEST(ReadPointTest, AcceptsLeadingPlusAndFractionWithoutIntegerPart)
{
    std::istringstream input("+1.5 +2E+2 +.5");
    const triangles::Point3 point = triangles::ReadPoint(input);
    EXPECT_DOUBLE_EQ(point.cord_x, 1.5);
    EXPECT_DOUBLE_EQ(point.cord_y, 200.0);
    EXPECT_DOUBLE_EQ(point.cord_z, 0.5);
}

TEST(ReadPointTest, AcceptsWhitespaceAcrossLines)
{
    std::istringstream input(" \t1\n-2\r\n3\t");
    const triangles::Point3 point = triangles::ReadPoint(input);
    EXPECT_DOUBLE_EQ(point.cord_x, 1.0);
    EXPECT_DOUBLE_EQ(point.cord_y, -2.0);
    EXPECT_DOUBLE_EQ(point.cord_z, 3.0);
}

TEST(ReadPointTest, PreservesFollowingDataAndReadsConsecutivePoints)
{
    std::istringstream input("1 2 3 4 5 6 42");
    const triangles::Point3 first = triangles::ReadPoint(input);
    const triangles::Point3 second = triangles::ReadPoint(input);
    EXPECT_DOUBLE_EQ(first.cord_x, 1.0);
    EXPECT_DOUBLE_EQ(first.cord_y, 2.0);
    EXPECT_DOUBLE_EQ(first.cord_z, 3.0);
    EXPECT_DOUBLE_EQ(second.cord_x, 4.0);
    EXPECT_DOUBLE_EQ(second.cord_y, 5.0);
    EXPECT_DOUBLE_EQ(second.cord_z, 6.0);
    int remaining = 0;
    ASSERT_TRUE(input >> remaining);
    EXPECT_EQ(remaining, 42);
}

TEST(ReadPointTest, ReadsCoordinatesAfterTriangleCount)
{
    std::istringstream input("1 -1.5 0 3.25");
    ASSERT_EQ(triangles::ReadTriangleCount(input), 1u);
    const triangles::Point3 point = triangles::ReadPoint(input);
    EXPECT_DOUBLE_EQ(point.cord_x, -1.5);
    EXPECT_DOUBLE_EQ(point.cord_y, 0.0);
    EXPECT_DOUBLE_EQ(point.cord_z, 3.25);
}

TEST(ReadPointTest, AcceptsExtremeFiniteAndSubnormalCoordinates)
{
    std::istringstream input("1.7976931348623157e308 -1.7976931348623157e308 4.9406564584124654e-324");
    const triangles::Point3 point = triangles::ReadPoint(input);
    EXPECT_EQ(point.cord_x, std::numeric_limits<double>::max());
    EXPECT_EQ(point.cord_y, -std::numeric_limits<double>::max());
    EXPECT_EQ(point.cord_z, std::numeric_limits<double>::denorm_min());
}

TEST(ReadPointTest, PreservesSignedZero)
{
    std::istringstream input("-0 +0 0");
    const triangles::Point3 point = triangles::ReadPoint(input);
    EXPECT_EQ(point.cord_x, 0.0);
    EXPECT_TRUE(std::signbit(point.cord_x));
    EXPECT_FALSE(std::signbit(point.cord_y));
    EXPECT_FALSE(std::signbit(point.cord_z));
}

TEST(ReadPointTest, RejectsMissingCoordinates)
{
    for (const std::string text : {"", " \t\n", "1", "1 2"})
    {
        SCOPED_TRACE(text);
        std::istringstream input(text);
        EXPECT_THROW(triangles::ReadPoint(input), std::invalid_argument);
    }
}

TEST(ReadPointTest, RejectsInvalidOrPartiallyParsedTokensInEveryCoordinate)
{
    for (const std::string token : {"abc", "3junk", "1e", "0x1p0", "--1", "++1", "+-1", "+", "1,5", "."})
        ExpectInvalidPointTokenInEveryCoordinate(token);
}

TEST(ReadPointTest, RejectsNonFiniteCoordinates)
{
    for (const std::string token : {"nan", "NaN", "nan(payload)", "-nan", "+nan", "inf", "-inf", "+inf", "infinity"})
        ExpectInvalidPointTokenInEveryCoordinate(token);
}

TEST(ReadPointTest, RejectsOverflowAndUnderflowInEveryCoordinate)
{
    for (const std::string token : {"1e309", "-1e309", "1e-9999", "-1e-9999"})
        ExpectInvalidPointTokenInEveryCoordinate(token);
}

TEST(ReadPointTest, RejectsAnAlreadyFailedStream)
{
    std::istringstream input("1 2 3");
    input.setstate(std::ios::failbit);
    EXPECT_THROW(triangles::ReadPoint(input), std::invalid_argument);
}

TEST(ReadTriangleTest, PreservesVertexAndCoordinateOrderAcrossWhitespace)
{
    std::istringstream input(" \n1 -2.5 3e2\t4.25 +5 -6\r\n-7 8.5 9.75");
    const triangles::Triangle3 expected{{triangles::Point3{1.0, -2.5, 300.0},
                                         triangles::Point3{4.25, 5.0, -6.0},
                                         triangles::Point3{-7.0, 8.5, 9.75}}};
    ExpectTriangleCoordinates(triangles::ReadTriangle(input), expected);
}

TEST(ReadTriangleTest, ReadsConsecutiveTrianglesAndPreservesFollowingData)
{
    std::istringstream input("1 2 3 4 5 6 7 8 9 -1 -2 -3 -4 -5 -6 -7 -8 -9 42");
    const triangles::Triangle3 expected_first{{triangles::Point3{1.0, 2.0, 3.0},
                                               triangles::Point3{4.0, 5.0, 6.0},
                                               triangles::Point3{7.0, 8.0, 9.0}}};
    const triangles::Triangle3 expected_second{{triangles::Point3{-1.0, -2.0, -3.0},
                                                triangles::Point3{-4.0, -5.0, -6.0},
                                                triangles::Point3{-7.0, -8.0, -9.0}}};
    ExpectTriangleCoordinates(triangles::ReadTriangle(input), expected_first);
    ExpectTriangleCoordinates(triangles::ReadTriangle(input), expected_second);
    int remaining = 0;
    ASSERT_TRUE(input >> remaining);
    EXPECT_EQ(remaining, 42);
}

TEST(ReadTriangleTest, ReadsTriangleAfterCount)
{
    std::istringstream input("1 0 0 0 1 0 0 0 1 0");
    ASSERT_EQ(triangles::ReadTriangleCount(input), 1u);
    const triangles::Triangle3 expected{{triangles::Point3{0.0, 0.0, 0.0},
                                         triangles::Point3{1.0, 0.0, 0.0},
                                         triangles::Point3{0.0, 1.0, 0.0}}};
    ExpectTriangleCoordinates(triangles::ReadTriangle(input), expected);
}

TEST(ReadTriangleTest, RejectsEveryIncompleteCoordinateCount)
{
    std::string text;
    for (std::size_t available = 0; available < 9; ++available)
    {
        SCOPED_TRACE(available);
        std::istringstream input(text);
        EXPECT_THROW(triangles::ReadTriangle(input), std::invalid_argument);
        text += "1 ";
    }
}

TEST(ReadTriangleTest, PropagatesInvalidCoordinateErrorsFromEveryVertex)
{
    for (const std::string token : {"abc", "3junk", "nan", "inf", "1e309", "1e-9999"})
    {
        for (std::size_t position = 0; position < 9; ++position)
        {
            SCOPED_TRACE(::testing::Message() << "token=" << token << ", position=" << position);
            std::string text;
            for (std::size_t coordinate = 0; coordinate < 9; ++coordinate)
            {
                text += coordinate == position ? token : "1";
                text += ' ';
            }
            std::istringstream input(text);
            EXPECT_THROW(triangles::ReadTriangle(input), std::invalid_argument);
        }
    }
}

TEST(ReadTriangleTest, AcceptsPointAndSegmentDegeneracies)
{
    std::istringstream point_input("1 2 3 1 2 3 1 2 3");
    const triangles::Point3 point{1.0, 2.0, 3.0};
    ExpectTriangleCoordinates(triangles::ReadTriangle(point_input), {{point, point, point}});

    std::istringstream segment_input("0 0 0 2 4 6 1 2 3");
    const triangles::Triangle3 segment{{triangles::Point3{0.0, 0.0, 0.0},
                                        triangles::Point3{2.0, 4.0, 6.0},
                                        triangles::Point3{1.0, 2.0, 3.0}}};
    ExpectTriangleCoordinates(triangles::ReadTriangle(segment_input), segment);
}

TEST(ReadSceneTest, ReadsOneCompleteTriangle)
{
    std::istringstream input("1 1 -2.5 3e2 4.25 +5 -6 -7 8.5 9.75");
    const auto scene = triangles::ReadScene(input);
    ASSERT_EQ(scene.size(), 1u);
    const triangles::Triangle3 expected{{triangles::Point3{1.0, -2.5, 300.0},
                                         triangles::Point3{4.25, 5.0, -6.0},
                                         triangles::Point3{-7.0, 8.5, 9.75}}};
    ExpectTriangleCoordinates(scene[0], expected);
}

TEST(ReadSceneTest, PreservesTriangleOrderDegeneraciesAndFollowingData)
{
    std::istringstream input("3\n0 0 0 2 0 0 0 2 0\n4 5 6 4 5 6 4 5 6\n"
                             "0 0 0 2 4 6 1 2 3\n42");
    const auto scene = triangles::ReadScene(input);
    ASSERT_EQ(scene.size(), 3u);
    ExpectTriangleCoordinates(scene[0], {{triangles::Point3{0.0, 0.0, 0.0},
                                          triangles::Point3{2.0, 0.0, 0.0},
                                          triangles::Point3{0.0, 2.0, 0.0}}});
    const triangles::Point3 point{4.0, 5.0, 6.0};
    ExpectTriangleCoordinates(scene[1], {{point, point, point}});
    ExpectTriangleCoordinates(scene[2], {{triangles::Point3{0.0, 0.0, 0.0},
                                          triangles::Point3{2.0, 4.0, 6.0},
                                          triangles::Point3{1.0, 2.0, 3.0}}});
    int remaining = 0;
    ASSERT_TRUE(input >> remaining);
    EXPECT_EQ(remaining, 42);
}

TEST(ReadSceneTest, RejectsMissingInvalidAndOutOfRangeCounts)
{
    for (const std::string text : {"", " \t\n", "abc", "0", "-1", "1.5", "3junk", "1000000",
                                   "999999999999999999999999999999999999999999999999"})
    {
        SCOPED_TRACE(text);
        std::istringstream input(text);
        EXPECT_THROW(triangles::ReadScene(input), std::invalid_argument);
    }
}

TEST(ReadSceneTest, RejectsTruncatedSceneAfterOneCompleteTriangle)
{
    std::string text = "2 0 0 0 1 0 0 0 1 0 ";
    for (std::size_t available = 0; available < 9; ++available)
    {
        SCOPED_TRACE(available);
        std::istringstream input(text);
        EXPECT_THROW(triangles::ReadScene(input), std::invalid_argument);
        text += "1 ";
    }
}

TEST(ReadSceneTest, PropagatesCoordinateErrorsFromLaterTriangles)
{
    for (const std::string token : {"abc", "3junk", "nan", "inf", "1e309"})
    {
        for (std::size_t position = 0; position < 9; ++position)
        {
            SCOPED_TRACE(::testing::Message() << "token=" << token << ", position=" << position);
            std::string text = "2 0 0 0 1 0 0 0 1 0 ";
            for (std::size_t coordinate = 0; coordinate < 9; ++coordinate)
            {
                text += coordinate == position ? token : "1";
                text += ' ';
            }
            std::istringstream input(text);
            EXPECT_THROW(triangles::ReadScene(input), std::invalid_argument);
        }
    }
}

TEST(ReadSceneTest, ReadsConsecutiveScenesFromOneStream)
{
    std::istringstream input("1 0 0 0 1 0 0 0 1 0\n1 10 0 0 11 0 0 10 1 0");
    const auto first = triangles::ReadScene(input);
    const auto second = triangles::ReadScene(input);
    ASSERT_EQ(first.size(), 1u);
    ASSERT_EQ(second.size(), 1u);
    ExpectTriangleCoordinates(first[0], {{triangles::Point3{0.0, 0.0, 0.0},
                                         triangles::Point3{1.0, 0.0, 0.0},
                                         triangles::Point3{0.0, 1.0, 0.0}}});
    ExpectTriangleCoordinates(second[0], {{triangles::Point3{10.0, 0.0, 0.0},
                                          triangles::Point3{11.0, 0.0, 0.0},
                                          triangles::Point3{10.0, 1.0, 0.0}}});
}

TEST(ReadSceneTest, RetainsAllTrianglesWhileVectorGrows)
{
    std::ostringstream text;
    text << "65\n";
    double x = 0.0;
    for (std::size_t i = 0; i < 65; ++i)
    {
        text << x << " 0 0 " << x + 1.0 << " 0 0 " << x << " 1 0\n";
        x += 3.0;
    }
    std::istringstream input(text.str());
    const auto scene = triangles::ReadScene(input);
    ASSERT_EQ(scene.size(), 65u);
    x = 0.0;
    for (std::size_t i = 0; i < scene.size(); ++i)
    {
        SCOPED_TRACE(i);
        ExpectTriangleCoordinates(scene[i], {{triangles::Point3{x, 0.0, 0.0},
                                             triangles::Point3{x + 1.0, 0.0, 0.0},
                                             triangles::Point3{x, 1.0, 0.0}}});
        x += 3.0;
    }
}
