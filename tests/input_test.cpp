#include "input.hpp"

#include <gtest/gtest.h>

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
