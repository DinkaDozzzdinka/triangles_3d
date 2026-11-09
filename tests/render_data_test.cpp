#include "render_data.hpp"
#include "input.hpp"
#include "bvh.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace triangles
{
    namespace
    {
        const std::array<float, 3> Red{1.0f, 0.0f, 0.0f};
        const std::array<float, 3> Blue{0.0f, 0.0f, 1.0f};

        Triangle3 ExampleTriangle()
        {
            return {{Point3{-1.0, 2.0, 3.0}, Point3{4.0, -5.0, 6.0}, Point3{7.0, 8.0, -9.0}}};
        }

        void ExpectRenderVertex(const RenderVertex& actual, const Point3& position,
                                const std::array<float, 3>& color)
        {
            EXPECT_EQ(actual.position.cord_x, position.cord_x);
            EXPECT_EQ(actual.position.cord_y, position.cord_y);
            EXPECT_EQ(actual.position.cord_z, position.cord_z);
            EXPECT_EQ(actual.color, color);
        }
    }

    TEST(BuildRenderVerticesTest, EmptySceneAndMaskReturnNoVertices)
    {
        EXPECT_TRUE(BuildRenderVertices({}, {}).empty());
    }

    TEST(BuildRenderVerticesTest, RejectsMismatchedMaskSizes)
    {
        const std::vector<Triangle3> scene(2, ExampleTriangle());
        EXPECT_THROW(BuildRenderVertices(scene, {}), std::invalid_argument);
        EXPECT_THROW(BuildRenderVertices(scene, {true}), std::invalid_argument);
        EXPECT_THROW(BuildRenderVertices(scene, {false, true, false}), std::invalid_argument);
        EXPECT_THROW(BuildRenderVertices({}, {true}), std::invalid_argument);
    }

    TEST(BuildRenderVerticesTest, NonIntersectingTriangleHasThreeBlueVertices)
    {
        const Triangle3 triangle = ExampleTriangle();
        const auto result = BuildRenderVertices({triangle}, {false});
        ASSERT_EQ(result.size(), 3u);
        for (std::size_t vertex = 0; vertex < 3; ++vertex)
        {
            SCOPED_TRACE(vertex);
            ExpectRenderVertex(result[vertex], triangle.vertices[vertex], Blue);
        }
    }

    TEST(BuildRenderVerticesTest, IntersectingTriangleHasThreeRedVertices)
    {
        const Triangle3 triangle = ExampleTriangle();
        const auto result = BuildRenderVertices({triangle}, {true});
        ASSERT_EQ(result.size(), 3u);
        for (std::size_t vertex = 0; vertex < 3; ++vertex)
        {
            SCOPED_TRACE(vertex);
            ExpectRenderVertex(result[vertex], triangle.vertices[vertex], Red);
        }
    }

    TEST(BuildRenderVerticesTest, MixedMaskPreservesEveryTriangleAndVertexInOrder)
    {
        const std::vector<Triangle3> scene{
            ExampleTriangle(),
            {{Point3{10.0, 11.0, 12.0}, Point3{13.0, 14.0, 15.0}, Point3{16.0, 17.0, 18.0}}},
            {{Point3{20.0, 21.0, 22.0}, Point3{23.0, 24.0, 25.0}, Point3{26.0, 27.0, 28.0}}}
        };
        const auto result = BuildRenderVertices(scene, {true, false, true});
        ASSERT_EQ(result.size(), 9u);
        ExpectRenderVertex(result[0], {-1.0, 2.0, 3.0}, Red);
        ExpectRenderVertex(result[1], {4.0, -5.0, 6.0}, Red);
        ExpectRenderVertex(result[2], {7.0, 8.0, -9.0}, Red);
        ExpectRenderVertex(result[3], {10.0, 11.0, 12.0}, Blue);
        ExpectRenderVertex(result[4], {13.0, 14.0, 15.0}, Blue);
        ExpectRenderVertex(result[5], {16.0, 17.0, 18.0}, Blue);
        ExpectRenderVertex(result[6], {20.0, 21.0, 22.0}, Red);
        ExpectRenderVertex(result[7], {23.0, 24.0, 25.0}, Red);
        ExpectRenderVertex(result[8], {26.0, 27.0, 28.0}, Red);
    }

    TEST(BuildRenderVerticesTest, KeepsRepeatedVerticesInPointAndSegmentDegeneracies)
    {
        const Point3 point{4.0, 5.0, 6.0};
        const Triangle3 segment{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 4.0, 6.0}, Point3{1.0, 2.0, 3.0}}};
        const auto result = BuildRenderVertices({{{point, point, point}}, segment}, {true, false});
        ASSERT_EQ(result.size(), 6u);
        ExpectRenderVertex(result[0], point, Red);
        ExpectRenderVertex(result[1], point, Red);
        ExpectRenderVertex(result[2], point, Red);
        ExpectRenderVertex(result[3], {0.0, 0.0, 0.0}, Blue);
        ExpectRenderVertex(result[4], {2.0, 4.0, 6.0}, Blue);
        ExpectRenderVertex(result[5], {1.0, 2.0, 3.0}, Blue);
    }

    TEST(BuildRenderVerticesTest, PreservesDoubleCoordinatesWithoutNarrowing)
    {
        const double maximum = std::numeric_limits<double>::max();
        const double tiny = std::numeric_limits<double>::denorm_min();
        const Triangle3 triangle{{Point3{maximum, -maximum, tiny},
                                  Point3{1.0000000000000002, 2.0, 3.0},
                                  Point3{-tiny, 4.0, 5.0}}};
        const auto result = BuildRenderVertices({triangle}, {false});
        ASSERT_EQ(result.size(), 3u);
        for (std::size_t vertex = 0; vertex < 3; ++vertex)
        {
            SCOPED_TRACE(vertex);
            ExpectRenderVertex(result[vertex], triangle.vertices[vertex], Blue);
        }
    }

    TEST(BuildRenderVerticesTest, SceneLoadingAndBvhProduceExpectedDemoColors)
    {
        std::istringstream input("3\n0 0 0 2 0 0 0 2 0\n"
                                 "0.5 0.5 -1 0.5 0.5 1 1.5 0.5 0\n"
                                 "10 10 0 11 10 0 10 11 0\n");
        const auto scene = ReadScene(input);
        BvhSearchStats stats;
        const auto mask = FindIntersectionsBvh(scene, stats);
        ASSERT_EQ(mask, (std::vector<bool>{true, true, false}));
        const auto result = BuildRenderVertices(scene, mask);
        ASSERT_EQ(result.size(), 9u);
        for (std::size_t vertex = 0; vertex < 6; ++vertex)
            EXPECT_EQ(result[vertex].color, Red);
        for (std::size_t vertex = 6; vertex < 9; ++vertex)
            EXPECT_EQ(result[vertex].color, Blue);
    }
}
