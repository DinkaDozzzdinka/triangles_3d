#include "bvh.hpp"
#include "intersection.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

namespace triangles
{
    namespace
    {
        void ExpectBvhBounds(const Aabb3& actual, const Aabb3& expected)
        {
            EXPECT_EQ(actual.minimum.cord_x, expected.minimum.cord_x);
            EXPECT_EQ(actual.minimum.cord_y, expected.minimum.cord_y);
            EXPECT_EQ(actual.minimum.cord_z, expected.minimum.cord_z);
            EXPECT_EQ(actual.maximum.cord_x, expected.maximum.cord_x);
            EXPECT_EQ(actual.maximum.cord_y, expected.maximum.cord_y);
            EXPECT_EQ(actual.maximum.cord_z, expected.maximum.cord_z);
        }

        void ExpectAllBvhIndexesExactlyOnce(const Bvh& bvh, std::size_t count)
        {
            ASSERT_EQ(bvh.indexes.size(), count);
            auto sorted = bvh.indexes;
            std::sort(sorted.begin(), sorted.end());
            for (std::size_t i = 0; i < count; ++i)
                EXPECT_EQ(sorted[i], i);
        }

        void ExpectRootLeaf(const Bvh& bvh, std::size_t count)
        {
            ASSERT_EQ(bvh.nodes.size(), 1u);
            const BvhNode& root = bvh.nodes[0];
            EXPECT_TRUE(root.IsLeaf());
            EXPECT_EQ(root.left, NoNode);
            EXPECT_EQ(root.right, NoNode);
            EXPECT_EQ(root.begin, 0u);
            EXPECT_EQ(root.count, count);
        }
    }

    TEST(BuildBvhTest, EmptySceneHasNoRootOrIndexes)
    {
        const Bvh bvh = BuildBvh({});
        EXPECT_TRUE(bvh.nodes.empty());
        EXPECT_TRUE(bvh.indexes.empty());
    }

    TEST(BuildBvhTest, SingleTriangleHasLeafAndExactBoundsAwayFromOrigin)
    {
        const Triangle3 triangle{{Point3{10.0, 20.0, 30.0}, Point3{14.0, 20.0, 32.0}, Point3{10.0, 25.0, 31.0}}};
        const Bvh bvh = BuildBvh({triangle});
        ExpectRootLeaf(bvh, 1);
        ExpectAllBvhIndexesExactlyOnce(bvh, 1);
        ASSERT_FALSE(bvh.nodes.empty());
        ExpectBvhBounds(bvh.nodes[0].bounds, {{10.0, 20.0, 30.0}, {14.0, 25.0, 32.0}});
    }

    TEST(BuildBvhTest, SeveralTrianglesHaveEveryIndexAndWholeGroupBounds)
    {
        const std::vector<Triangle3> scene{
            {{Point3{10.0, 20.0, 30.0}, Point3{12.0, 20.0, 30.0}, Point3{10.0, 22.0, 30.0}}},
            {{Point3{-8.0, 5.0, 9.0}, Point3{-6.0, 7.0, 9.0}, Point3{-7.0, 6.0, 12.0}}},
            {{Point3{40.0, -10.0, 60.0}, Point3{43.0, -10.0, 60.0}, Point3{40.0, -6.0, 65.0}}}
        };
        const Bvh bvh = BuildBvh(scene);
        ExpectRootLeaf(bvh, scene.size());
        ExpectAllBvhIndexesExactlyOnce(bvh, scene.size());
        ASSERT_FALSE(bvh.nodes.empty());
        ExpectBvhBounds(bvh.nodes[0].bounds, {{-8.0, -10.0, 9.0}, {43.0, 22.0, 65.0}});
    }

    TEST(BuildBvhTest, LastTriangleContributesToAllExtrema)
    {
        const Triangle3 first{{Point3{10.0, 20.0, 30.0}, Point3{12.0, 20.0, 30.0}, Point3{10.0, 22.0, 30.0}}};
        const Triangle3 last{{Point3{-5.0, -6.0, -7.0}, Point3{50.0, -6.0, 60.0}, Point3{-5.0, 70.0, 80.0}}};
        const Bvh bvh = BuildBvh({first, last});
        ExpectRootLeaf(bvh, 2);
        ASSERT_FALSE(bvh.nodes.empty());
        ExpectBvhBounds(bvh.nodes[0].bounds, {{-5.0, -6.0, -7.0}, {50.0, 70.0, 80.0}});
    }

    TEST(BuildBvhTest, NegativeSceneBoundsDoNotExpandToOrigin)
    {
        const std::vector<Triangle3> scene{
            {{Point3{-20.0, -30.0, -40.0}, Point3{-15.0, -30.0, -40.0}, Point3{-20.0, -25.0, -35.0}}},
            {{Point3{-12.0, -18.0, -22.0}, Point3{-10.0, -18.0, -22.0}, Point3{-12.0, -16.0, -20.0}}}
        };
        const Bvh bvh = BuildBvh(scene);
        ASSERT_FALSE(bvh.nodes.empty());
        ExpectBvhBounds(bvh.nodes[0].bounds, {{-20.0, -30.0, -40.0}, {-10.0, -16.0, -20.0}});
    }

    TEST(BuildBvhTest, SupportsPointAndSegmentTriangles)
    {
        const Point3 point{4.0, 5.0, 6.0};
        const Triangle3 collapsed{{point, point, point}};
        const Triangle3 segment{{Point3{1.0, 2.0, 3.0}, Point3{7.0, 8.0, 9.0}, Point3{3.0, 4.0, 5.0}}};
        const Bvh one = BuildBvh({collapsed});
        ExpectRootLeaf(one, 1);
        ASSERT_FALSE(one.nodes.empty());
        ExpectBvhBounds(one.nodes[0].bounds, {point, point});
        const Bvh two = BuildBvh({collapsed, segment});
        ExpectRootLeaf(two, 2);
        ExpectAllBvhIndexesExactlyOnce(two, 2);
        ASSERT_FALSE(two.nodes.empty());
        ExpectBvhBounds(two.nodes[0].bounds, {{1.0, 2.0, 3.0}, {7.0, 8.0, 9.0}});
    }

    TEST(BuildNodeTest, UsesOnlyRequestedRangeOfReorderedIndexesAndReturnsAppendedNode)
    {
        const Point3 excludedFirst{-1000.0, -1000.0, -1000.0};
        const Point3 excludedLast{1000.0, 1000.0, 1000.0};
        const std::vector<Triangle3> scene{
            {{Point3{10.0, 20.0, 30.0}, Point3{12.0, 20.0, 30.0}, Point3{10.0, 22.0, 30.0}}},
            {{excludedFirst, excludedFirst, excludedFirst}},
            {{Point3{-5.0, -6.0, -7.0}, Point3{-3.0, -6.0, -7.0}, Point3{-5.0, -4.0, -5.0}}},
            {{excludedLast, excludedLast, excludedLast}}
        };
        Bvh bvh{};
        bvh.indexes = {1, 2, 0, 3};
        const std::size_t first = BuildNode(bvh, scene, 0, 1);
        const std::size_t second = BuildNode(bvh, scene, 1, 2);
        ASSERT_EQ(bvh.nodes.size(), 2u);
        ASSERT_EQ(first, 0u);
        ASSERT_EQ(second, 1u);
        ExpectBvhBounds(bvh.nodes[first].bounds, {excludedFirst, excludedFirst});
        const BvhNode& node = bvh.nodes[second];
        EXPECT_TRUE(node.IsLeaf());
        EXPECT_EQ(node.begin, 1u);
        EXPECT_EQ(node.count, 2u);
        EXPECT_EQ(node.left, NoNode);
        EXPECT_EQ(node.right, NoNode);
        ExpectBvhBounds(node.bounds, {{-5.0, -6.0, -7.0}, {12.0, 22.0, 30.0}});
        EXPECT_EQ(bvh.indexes, (std::vector<std::size_t>{1, 2, 0, 3}));
    }

    TEST(SplitAxisTest, SelectsEachLongestAxis)
    {
        EXPECT_EQ(SplitAxis({{0.0, 0.0, 0.0}, {10.0, 2.0, 1.0}}), Axis::X);
        EXPECT_EQ(SplitAxis({{0.0, 0.0, 0.0}, {3.0, 8.0, 2.0}}), Axis::Y);
        EXPECT_EQ(SplitAxis({{0.0, 0.0, 0.0}, {1.0, 2.0, 9.0}}), Axis::Z);
    }

    TEST(SplitAxisTest, ResolvesTiesInXYZOrder)
    {
        EXPECT_EQ(SplitAxis({{0.0, 0.0, 0.0}, {4.0, 4.0, 1.0}}), Axis::X);
        EXPECT_EQ(SplitAxis({{0.0, 0.0, 0.0}, {4.0, 1.0, 4.0}}), Axis::X);
        EXPECT_EQ(SplitAxis({{0.0, 0.0, 0.0}, {1.0, 4.0, 4.0}}), Axis::Y);
        EXPECT_EQ(SplitAxis({{0.0, 0.0, 0.0}, {4.0, 4.0, 4.0}}), Axis::X);
    }

    TEST(SplitAxisTest, UsesExtentRatherThanAbsoluteCoordinate)
    {
        EXPECT_EQ(SplitAxis({{100.0, 20.0, 30.0}, {102.0, 28.0, 33.0}}), Axis::Y);
        EXPECT_EQ(SplitAxis({{-10.0, -3.0, -20.0}, {-8.0, -2.0, -11.0}}), Axis::Z);
    }

    TEST(SplitAxisTest, HandlesFlatLineAndPointBoxes)
    {
        EXPECT_EQ(SplitAxis({{0.0, 0.0, 3.0}, {2.0, 7.0, 3.0}}), Axis::Y);
        EXPECT_EQ(SplitAxis({{2.0, 3.0, 0.0}, {2.0, 3.0, 9.0}}), Axis::Z);
        EXPECT_EQ(SplitAxis({{2.0, 3.0, 4.0}, {2.0, 3.0, 4.0}}), Axis::X);
    }

    TEST(SplitAxisTest, DistinguishesExtremeExtentsWithoutOverflow)
    {
        const double maximum = std::numeric_limits<double>::max();
        EXPECT_EQ(SplitAxis({{-0.75 * maximum, -maximum, -0.5 * maximum},
                             {0.75 * maximum, maximum, 0.5 * maximum}}), Axis::Y);
        EXPECT_EQ(SplitAxis({{-0.75 * maximum, -0.5 * maximum, -maximum},
                             {0.75 * maximum, 0.5 * maximum, maximum}}), Axis::Z);
    }

    TEST(SplitAxisTest, UniformScalingPreservesChoice)
    {
        for (const int exponent : {-900, 900})
        {
            const double scale = std::scalbn(1.0, exponent);
            EXPECT_EQ(SplitAxis({{0.0, 0.0, 0.0}, {scale, 4.0 * scale, 2.0 * scale}}), Axis::Y);
        }
    }

    TEST(BoxCenterCoordinateTest, ReturnsSelectedCoordinate)
    {
        const Aabb3 box{{2.0, 4.0, 10.0}, {6.0, 12.0, 14.0}};
        EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::X), 4.0);
        EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::Y), 8.0);
        EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::Z), 12.0);
    }

    TEST(BoxCenterCoordinateTest, HandlesNegativeAndOppositeSignBounds)
    {
        const Aabb3 box{{-10.0, -4.0, -6.0}, {-2.0, 8.0, 2.0}};
        EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::X), -6.0);
        EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::Y), 2.0);
        EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::Z), -2.0);
    }

    TEST(BoxCenterCoordinateTest, PointBoxKeepsItsCoordinates)
    {
        const Point3 point{1.25, -3.5, 7.0};
        const Aabb3 box{point, point};
        EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::X), point.cord_x);
        EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::Y), point.cord_y);
        EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::Z), point.cord_z);
    }

    TEST(BoxCenterCoordinateTest, LargeSameSignBoundsHaveFiniteCenters)
    {
        const double maximum = std::numeric_limits<double>::max();
        const Aabb3 positive{{maximum / 2.0, maximum / 2.0, maximum / 2.0},
                             {maximum, maximum, maximum}};
        const Aabb3 negative{{-maximum, -maximum, -maximum},
                             {-maximum / 2.0, -maximum / 2.0, -maximum / 2.0}};
        for (const Axis axis : {Axis::X, Axis::Y, Axis::Z})
        {
            const double positiveCenter = BoxCenterCoordinate(positive, axis);
            const double negativeCenter = BoxCenterCoordinate(negative, axis);
            EXPECT_TRUE(std::isfinite(positiveCenter));
            EXPECT_TRUE(std::isfinite(negativeCenter));
            EXPECT_DOUBLE_EQ(positiveCenter, 0.75 * maximum);
            EXPECT_DOUBLE_EQ(negativeCenter, -0.75 * maximum);
        }
    }

    TEST(BoxCenterCoordinateTest, HandlesUniformlyTinyAndHugeBoxes)
    {
        for (const int exponent : {-900, 900})
        {
            const double scale = std::scalbn(1.0, exponent);
            const Aabb3 box{{2.0 * scale, 4.0 * scale, 10.0 * scale},
                            {6.0 * scale, 12.0 * scale, 14.0 * scale}};
            EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::X), 4.0 * scale);
            EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::Y), 8.0 * scale);
            EXPECT_DOUBLE_EQ(BoxCenterCoordinate(box, Axis::Z), 12.0 * scale);
        }
    }

    namespace
    {
        std::vector<std::size_t> SortedIndexRange(const Bvh& bvh, std::size_t begin, std::size_t count)
        {
            std::vector<std::size_t> result(bvh.indexes.begin() + begin,
                                             bvh.indexes.begin() + begin + count);
            std::sort(result.begin(), result.end());
            return result;
        }
    }

    TEST(CenterLessTest, UsesSelectedAxis)
    {
        const std::vector<Triangle3> scene{
            {{Point3{0.0, 9.0, 1.0}, Point3{4.0, 1.0, 3.0}, Point3{2.0, 6.0, 2.0}}},
            {{Point3{6.0, 0.0, 0.0}, Point3{8.0, 2.0, 2.0}, Point3{7.0, 0.0, 1.0}}}
        };
        const CenterLess x{scene, Axis::X}, y{scene, Axis::Y}, z{scene, Axis::Z};
        EXPECT_TRUE(x(0, 1));
        EXPECT_FALSE(x(1, 0));
        EXPECT_FALSE(y(0, 1));
        EXPECT_TRUE(y(1, 0));
        EXPECT_FALSE(z(0, 1));
        EXPECT_TRUE(z(1, 0));
    }

    TEST(CenterLessTest, EqualCentersUseIndexAndNeverPrecedeThemselves)
    {
        const Point3 point{0.0, 0.5, 0.0};
        const std::vector<Triangle3> scene{
            {{Point3{-2.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}}},
            {{Point3{-1.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 1.0, 0.0}}},
            {{point, point, point}}
        };
        for (const Axis axis : {Axis::X, Axis::Y, Axis::Z})
        {
            const CenterLess compare{scene, axis};
            EXPECT_TRUE(compare(0, 1));
            EXPECT_TRUE(compare(1, 2));
            EXPECT_TRUE(compare(0, 2));
            EXPECT_FALSE(compare(1, 0));
            for (std::size_t i = 0; i < scene.size(); ++i)
                EXPECT_FALSE(compare(i, i));
        }
    }

    TEST(CenterLessTest, UsesBoundsCenterRatherThanFirstVertexOrTriangleCentroid)
    {
        const std::vector<Triangle3> scene{
            {{Point3{100.0, 0.0, 0.0}, Point3{0.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}},
            {{Point3{60.0, 0.0, 0.0}, Point3{61.0, 0.0, 0.0}, Point3{61.0, 2.0, 0.0}}},
            {{Point3{0.0, 0.0, 0.0}, Point3{100.0, 0.0, 0.0}, Point3{100.0, 2.0, 0.0}}}
        };
        const CenterLess compare{scene, Axis::X};
        EXPECT_TRUE(compare(0, 1));
        EXPECT_TRUE(compare(2, 1));
    }

    TEST(CenterLessTest, PreservesOrderForLargeFiniteCenters)
    {
        const double maximum = std::numeric_limits<double>::max();
        const Point3 high{maximum, 0.0, 0.0}, low{0.75 * maximum, 0.0, 0.0};
        const std::vector<Triangle3> scene{{{high, high, high}}, {{low, low, low}}};
        const CenterLess compare{scene, Axis::X};
        EXPECT_TRUE(compare(1, 0));
        EXPECT_FALSE(compare(0, 1));
    }

    TEST(BvhPartitionTest, SeparatesSpatialHalvesAlongEveryAxis)
    {
        const double centers[]{100.0, 0.0, 101.0, 1.0, 102.0, 2.0, 103.0, 3.0};
        for (const Axis axis : {Axis::X, Axis::Y, Axis::Z})
        {
            std::vector<Triangle3> scene;
            for (const double center : centers)
            {
                Point3 point{};
                if (axis == Axis::X) point.cord_x = center;
                else if (axis == Axis::Y) point.cord_y = center;
                else point.cord_z = center;
                scene.push_back({{point, point, point}});
            }
            const Bvh bvh = BuildBvh(scene);
            ExpectAllBvhIndexesExactlyOnce(bvh, scene.size());
            ASSERT_EQ(bvh.indexes.size(), 8u);
            EXPECT_EQ(SortedIndexRange(bvh, 0, 4), (std::vector<std::size_t>{1, 3, 5, 7}));
            EXPECT_EQ(SortedIndexRange(bvh, 4, 4), (std::vector<std::size_t>{0, 2, 4, 6}));
        }
    }

    TEST(BvhPartitionTest, OddNonzeroRangeLeavesOutsideIndexesUntouched)
    {
        const double centers[]{-1000.0, -900.0, -800.0, 800.0, 900.0, 1000.0,
                               50.0, 10.0, 40.0, 20.0, 30.0, -700.0, 700.0, 1500.0};
        std::vector<Triangle3> scene;
        for (const double center : centers)
        {
            const Point3 point{center, 0.0, 0.0};
            scene.push_back({{point, point, point}});
        }
        Bvh bvh{};
        bvh.indexes = {0, 4, 1, 5, 2, 3, 8, 6, 10, 7, 9, 13, 11, 12};
        const auto before = bvh.indexes;
        const std::size_t node = BuildNode(bvh, scene, 6, 5);
        ASSERT_LT(node, bvh.nodes.size());
        ASSERT_EQ(bvh.indexes.size(), before.size());
        ExpectBvhBounds(bvh.nodes[node].bounds, {{10.0, 0.0, 0.0}, {50.0, 0.0, 0.0}});
        EXPECT_EQ(SortedIndexRange(bvh, 6, 2), (std::vector<std::size_t>{7, 9}));
        EXPECT_EQ(SortedIndexRange(bvh, 8, 3), (std::vector<std::size_t>{6, 8, 10}));
        for (std::size_t i = 0; i < before.size(); ++i)
        {
            if (i < 6 || i >= 11)
            {
                EXPECT_EQ(bvh.indexes[i], before[i]);
            }
        }
    }

    TEST(BvhPartitionTest, SmallGroupPreservesIndexOrder)
    {
        std::vector<Triangle3> scene;
        for (const double center : {0.0, 1.0, 2.0, 3.0})
        {
            const Point3 point{center, 0.0, 0.0};
            scene.push_back({{point, point, point}});
        }
        Bvh bvh{};
        bvh.indexes = {3, 2, 1, 0};
        BuildNode(bvh, scene, 0, 4);
        EXPECT_EQ(bvh.indexes, (std::vector<std::size_t>{3, 2, 1, 0}));
    }

    TEST(BvhPartitionTest, EqualCentersPartitionByOriginalIndex)
    {
        const Point3 point{4.0, 5.0, 6.0};
        const Triangle3 triangle{{point, point, point}};
        const std::vector<Triangle3> scene(9, triangle);
        const Bvh bvh = BuildBvh(scene);
        ExpectAllBvhIndexesExactlyOnce(bvh, scene.size());
        ASSERT_EQ(bvh.indexes.size(), 9u);
        EXPECT_EQ(SortedIndexRange(bvh, 0, 4), (std::vector<std::size_t>{0, 1, 2, 3}));
        EXPECT_EQ(SortedIndexRange(bvh, 4, 5), (std::vector<std::size_t>{4, 5, 6, 7, 8}));
        ASSERT_FALSE(bvh.nodes.empty());
        ExpectBvhBounds(bvh.nodes[0].bounds, {point, point});
    }

    namespace
    {
        void ExpectValidBvhSubtree(const Bvh& bvh, const std::vector<Triangle3>& scene,
                                  std::size_t id, std::vector<bool>& visited,
                                  std::vector<std::size_t>& coverage)
        {
            ASSERT_LT(id, bvh.nodes.size());
            ASSERT_FALSE(visited[id]) << "Repeated node or cycle: " << id;
            visited[id] = true;
            const BvhNode& node = bvh.nodes[id];
            if (node.IsLeaf())
            {
                ASSERT_GE(node.count, 1u);
                ASSERT_LE(node.count, 4u);
                EXPECT_EQ(node.left, NoNode);
                EXPECT_EQ(node.right, NoNode);
                ASSERT_LE(node.begin, bvh.indexes.size());
                ASSERT_LE(node.count, bvh.indexes.size() - node.begin);
                ASSERT_LT(bvh.indexes[node.begin], scene.size());
                Aabb3 expected = Bounds(scene[bvh.indexes[node.begin]]);
                for (std::size_t p = node.begin; p < node.begin + node.count; ++p)
                {
                    ASSERT_LT(bvh.indexes[p], scene.size());
                    ++coverage[p];
                    expected = MergeBounds(expected, Bounds(scene[bvh.indexes[p]]));
                }
                ExpectBvhBounds(node.bounds, expected);
                return;
            }

            EXPECT_EQ(node.count, 0u);
            ASSERT_LT(node.left, bvh.nodes.size());
            ASSERT_LT(node.right, bvh.nodes.size());
            ASSERT_NE(node.left, node.right);
            ExpectValidBvhSubtree(bvh, scene, node.left, visited, coverage);
            ExpectValidBvhSubtree(bvh, scene, node.right, visited, coverage);
            ExpectBvhBounds(node.bounds, MergeBounds(bvh.nodes[node.left].bounds,
                                                    bvh.nodes[node.right].bounds));
        }

        void ExpectValidBvh(const Bvh& bvh, const std::vector<Triangle3>& scene)
        {
            if (scene.empty())
            {
                EXPECT_TRUE(bvh.nodes.empty());
                EXPECT_TRUE(bvh.indexes.empty());
                return;
            }
            ASSERT_FALSE(bvh.nodes.empty());
            ASSERT_EQ(bvh.indexes.size(), scene.size());
            ExpectAllBvhIndexesExactlyOnce(bvh, scene.size());
            std::vector<bool> visited(bvh.nodes.size(), false);
            std::vector<std::size_t> coverage(bvh.indexes.size(), 0);
            ExpectValidBvhSubtree(bvh, scene, 0, visited, coverage);
            for (std::size_t i = 0; i < visited.size(); ++i)
            {
                EXPECT_TRUE(visited[i]) << "Unreachable node: " << i;
            }
            for (std::size_t p = 0; p < coverage.size(); ++p)
            {
                EXPECT_EQ(coverage[p], 1u) << "Leaf coverage at index position: " << p;
            }
        }

        std::vector<Triangle3> MakeSeparatedBvhScene(std::size_t count)
        {
            std::vector<Triangle3> scene;
            double x = 100.0;
            for (std::size_t i = 0; i < count; ++i)
            {
                scene.push_back({{Point3{x, 0.0, 0.0}, Point3{x + 1.0, 0.0, 0.0}, Point3{x, 1.0, 0.0}}});
                x += 3.0;
            }
            return scene;
        }
    }

    TEST(BvhRecursionTest, FiveTrianglesProduceTwoAndThreeElementLeaves)
    {
        const auto scene = MakeSeparatedBvhScene(5);
        const Bvh bvh = BuildBvh(scene);
        ExpectValidBvh(bvh, scene);
        ASSERT_EQ(bvh.nodes.size(), 3u);
        const BvhNode& root = bvh.nodes[0];
        ASSERT_FALSE(root.IsLeaf());
        ASSERT_LT(root.left, bvh.nodes.size());
        ASSERT_LT(root.right, bvh.nodes.size());
        EXPECT_EQ(root.count, 0u);
        EXPECT_EQ(bvh.nodes[root.left].begin, 0u);
        EXPECT_EQ(bvh.nodes[root.left].count, 2u);
        EXPECT_EQ(bvh.nodes[root.right].begin, 2u);
        EXPECT_EQ(bvh.nodes[root.right].count, 3u);
    }

    TEST(BvhRecursionTest, NineTrianglesProduceMultipleLevels)
    {
        const auto scene = MakeSeparatedBvhScene(9);
        const Bvh bvh = BuildBvh(scene);
        ExpectValidBvh(bvh, scene);
        ASSERT_EQ(bvh.nodes.size(), 5u);
        const BvhNode& root = bvh.nodes[0];
        ASSERT_FALSE(root.IsLeaf());
        ASSERT_LT(root.left, bvh.nodes.size());
        ASSERT_LT(root.right, bvh.nodes.size());
        EXPECT_TRUE(bvh.nodes[root.left].IsLeaf());
        EXPECT_FALSE(bvh.nodes[root.right].IsLeaf());
        std::vector<std::size_t> sizes;
        for (const BvhNode& node : bvh.nodes)
        {
            if (node.IsLeaf())
            {
                sizes.push_back(node.count);
            }
        }
        std::sort(sizes.begin(), sizes.end());
        EXPECT_EQ(sizes, (std::vector<std::size_t>{2, 3, 4}));
    }

    TEST(BvhRecursionTest, RespectsLeafLimitAtFourAndEightTriangles)
    {
        const auto four = MakeSeparatedBvhScene(4);
        const Bvh one = BuildBvh(four);
        ExpectValidBvh(one, four);
        ExpectRootLeaf(one, 4);
        const auto eight = MakeSeparatedBvhScene(8);
        const Bvh three = BuildBvh(eight);
        ExpectValidBvh(three, eight);
        ASSERT_EQ(three.nodes.size(), 3u);
        const BvhNode& root = three.nodes[0];
        ASSERT_FALSE(root.IsLeaf());
        ASSERT_LT(root.left, three.nodes.size());
        ASSERT_LT(root.right, three.nodes.size());
        EXPECT_EQ(three.nodes[root.left].count, 4u);
        EXPECT_EQ(three.nodes[root.right].count, 4u);
    }

    TEST(BvhRecursionTest, NodeIndexesRemainValidWhileVectorGrows)
    {
        const auto scene = MakeSeparatedBvhScene(513);
        const Bvh bvh = BuildBvh(scene);
        ExpectValidBvh(bvh, scene);
        ASSERT_GT(bvh.nodes.size(), 128u);
        ExpectBvhBounds(bvh.nodes[0].bounds, {{100.0, 0.0, 0.0}, {1637.0, 1.0, 0.0}});
    }

    TEST(BvhRecursionTest, CoincidentCentersStillTerminateAndCoverEveryTriangle)
    {
        const Triangle3 triangle{{Point3{-1.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const std::vector<Triangle3> scene(65, triangle);
        const Bvh bvh = BuildBvh(scene);
        ExpectValidBvh(bvh, scene);
        ASSERT_GT(bvh.nodes.size(), 1u);
        EXPECT_FALSE(bvh.nodes[0].IsLeaf());
    }

    TEST(BvhRecursionTest, SupportsMixedPointSegmentAndRegularTriangles)
    {
        std::vector<Triangle3> scene;
        double x = 10.0;
        for (std::size_t i = 0; i < 11; ++i)
        {
            const Point3 a{x, 0.0, 0.0}, b{x + 1.0, 0.0, 0.0}, c{x, 1.0, 0.0};
            if (i % 3 == 0)
            {
                scene.push_back({{a, a, a}});
            }
            else if (i % 3 == 1)
            {
                scene.push_back({{a, b, a}});
            }
            else
            {
                scene.push_back({{a, b, c}});
            }
            x += 3.0;
        }
        const Bvh bvh = BuildBvh(scene);
        ExpectValidBvh(bvh, scene);
    }

    TEST(BvhRecursionTest, ReversingScenePreservesWholeBoundsAndValidCoverage)
    {
        auto scene = MakeSeparatedBvhScene(37);
        const Bvh first = BuildBvh(scene);
        ExpectValidBvh(first, scene);
        std::reverse(scene.begin(), scene.end());
        const Bvh second = BuildBvh(scene);
        ExpectValidBvh(second, scene);
        ASSERT_FALSE(first.nodes.empty());
        ASSERT_FALSE(second.nodes.empty());
        ExpectBvhBounds(second.nodes[0].bounds, first.nodes[0].bounds);
    }

    namespace
    {
        std::vector<Triangle3> MakeInterleavedCandidateScene()
        {
            std::vector<Triangle3> scene;
            for (const double x : {100.0, 0.0, 101.0, 1.0, 102.0, 2.0, 103.0, 3.0})
            {
                const Point3 point{x, 0.0, 0.0};
                scene.push_back({{point, point, point}});
            }
            return scene;
        }

        void ExpectCandidateIndexes(std::vector<std::size_t> actual,
                                    std::vector<std::size_t> expected)
        {
            std::sort(actual.begin(), actual.end());
            std::sort(expected.begin(), expected.end());
            EXPECT_EQ(actual, expected);
        }
    }

    TEST(FindBvhCandidatesTest, EmptyTreeReturnsNoCandidates)
    {
        const Bvh bvh = BuildBvh({});
        EXPECT_TRUE(FindBvhCandidates(bvh, {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}}).empty());
    }

    TEST(FindBvhCandidatesTest, QueryOutsideRootReturnsNoCandidates)
    {
        const Bvh bvh = BuildBvh(MakeSeparatedBvhScene(8));
        EXPECT_TRUE(FindBvhCandidates(bvh, {{10000.0, 0.0, 0.0}, {10001.0, 1.0, 1.0}}).empty());
    }

    TEST(FindBvhCandidatesTest, QueryInsideRootGapCanMissBothChildren)
    {
        const Bvh bvh = BuildBvh(MakeSeparatedBvhScene(8));
        const Aabb3 query{{110.5, 0.0, 0.0}, {111.5, 1.0, 0.0}};
        ASSERT_FALSE(bvh.nodes.empty());
        ASSERT_TRUE(BoundsOverlap(query, bvh.nodes[0].bounds));
        EXPECT_TRUE(FindBvhCandidates(bvh, query).empty());
    }

    TEST(FindBvhCandidatesTest, WholeSceneQueryReturnsEachIndexExactlyOnce)
    {
        const auto scene = MakeSeparatedBvhScene(37);
        const Bvh bvh = BuildBvh(scene);
        std::vector<std::size_t> expected;
        for (std::size_t i = 0; i < scene.size(); ++i)
            expected.push_back(i);
        ExpectCandidateIndexes(FindBvhCandidates(bvh, {{-1000.0, -1000.0, -1000.0},
                                                       {1000.0, 1000.0, 1000.0}}), expected);
    }

    TEST(FindBvhCandidatesTest, RejectsFarClusterAndUsesOriginalTriangleIndexes)
    {
        const Bvh bvh = BuildBvh(MakeInterleavedCandidateScene());
        ExpectCandidateIndexes(FindBvhCandidates(bvh, {{-1.0, -1.0, -1.0}, {4.0, 1.0, 1.0}}),
                               {1, 3, 5, 7});
        ExpectCandidateIndexes(FindBvhCandidates(bvh, {{99.0, -1.0, -1.0}, {104.0, 1.0, 1.0}}),
                               {0, 2, 4, 6});
    }

    TEST(FindBvhCandidatesTest, IncludesFaceEdgeVertexAndPointContacts)
    {
        std::vector<Triangle3> scene;
        for (const double value : {0.0, 1.0, 2.0, 3.0, 100.0, 101.0, 102.0, 103.0})
        {
            const Point3 point{value, value, value};
            scene.push_back({{point, point, point}});
        }
        const Bvh bvh = BuildBvh(scene);
        const Aabb3 queries[]{
            {{3.0, 1.0, 1.0}, {4.0, 2.0, 2.0}},
            {{3.0, 3.0, 1.0}, {4.0, 4.0, 2.0}},
            {{3.0, 3.0, 3.0}, {4.0, 4.0, 4.0}},
            {{3.0, 3.0, 3.0}, {3.0, 3.0, 3.0}}
        };
        for (const Aabb3& query : queries)
            ExpectCandidateIndexes(FindBvhCandidates(bvh, query), {0, 1, 2, 3});
    }

    TEST(FindBvhCandidatesTest, PointQueryMayReturnExtraTrianglesFromTheSameLeaf)
    {
        const Bvh bvh = BuildBvh(MakeInterleavedCandidateScene());
        const Point3 point{1.0, 0.0, 0.0};
        ExpectCandidateIndexes(FindBvhCandidates(bvh, {point, point}), {1, 3, 5, 7});
    }

    TEST(FindBvhCandidatesTest, OverlappingLeavesStillReturnEachIndexOnlyOnce)
    {
        const Point3 point{5.0, 6.0, 7.0};
        const Triangle3 triangle{{point, point, point}};
        const std::vector<Triangle3> scene(17, triangle);
        const Bvh bvh = BuildBvh(scene);
        std::vector<std::size_t> expected;
        for (std::size_t i = 0; i < scene.size(); ++i)
            expected.push_back(i);
        ExpectCandidateIndexes(FindBvhCandidates(bvh, {point, point}), expected);
    }

    TEST(FindIntersectionsBvhTest, EmptySceneResetsStatistics)
    {
        BvhSearchStats stats{123, 456};
        EXPECT_TRUE(FindIntersectionsBvh({}, stats).empty());
        EXPECT_EQ(stats.candidate_pairs, 0u);
        EXPECT_EQ(stats.exact_checks, 0u);
    }

    TEST(FindIntersectionsBvhTest, SingleTriangleDoesNotIntersectItself)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        BvhSearchStats stats{123, 456};
        EXPECT_EQ(FindIntersectionsBvh({triangle}, stats), (std::vector<bool>{false}));
        EXPECT_EQ(stats.candidate_pairs, 0u);
        EXPECT_EQ(stats.exact_checks, 0u);
    }

    TEST(FindIntersectionsBvhTest, FiltersDisjointBoxesWithinOneLeaf)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const Point3 far{10.0, 10.0, 10.0};
        const std::vector<Triangle3> scene{triangle, {{far, far, far}}, triangle};
        BvhSearchStats stats;
        EXPECT_EQ(FindIntersectionsBvh(scene, stats), (std::vector<bool>{true, false, true}));
        EXPECT_EQ(stats.candidate_pairs, 3u);
        EXPECT_EQ(stats.exact_checks, 1u);
    }

    TEST(FindIntersectionsBvhTest, OverlappingBoxesDoNotGuaranteeTriangleIntersection)
    {
        const std::vector<Triangle3> scene{
            {{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}},
            {{Point3{2.0, 2.0, 0.0}, Point3{2.0, 1.5, 0.0}, Point3{1.5, 2.0, 0.0}}}
        };
        BvhSearchStats stats;
        EXPECT_EQ(FindIntersectionsBvh(scene, stats), (std::vector<bool>{false, false}));
        EXPECT_EQ(stats.candidate_pairs, 1u);
        EXPECT_EQ(stats.exact_checks, 1u);
    }

    TEST(FindIntersectionsBvhTest, ContinuesAfterFirstHitAndMarksBothMembers)
    {
        const std::vector<Triangle3> scene{
            {{Point3{0.0, 0.0, 0.0}, Point3{4.0, 0.0, 0.0}, Point3{0.0, 4.0, 0.0}}},
            {{Point3{0.5, 0.5, 0.0}, Point3{0.5, 0.5, 0.0}, Point3{0.5, 0.5, 0.0}}},
            {{Point3{2.0, 0.5, 0.0}, Point3{2.0, 0.5, 0.0}, Point3{2.0, 0.5, 0.0}}}
        };
        BvhSearchStats stats;
        EXPECT_EQ(FindIntersectionsBvh(scene, stats), (std::vector<bool>{true, true, true}));
        EXPECT_EQ(stats.candidate_pairs, 3u);
        EXPECT_EQ(stats.exact_checks, 2u);
    }

    TEST(FindIntersectionsBvhTest, InterleavedClustersKeepOriginalMaskIndexes)
    {
        auto scene = MakeInterleavedCandidateScene();
        scene.push_back(scene[3]);
        scene.push_back(scene[4]);
        BvhSearchStats stats;
        EXPECT_EQ(FindIntersectionsBvh(scene, stats),
                  (std::vector<bool>{false, false, false, true, true, false, false, false, true, true}));
        EXPECT_EQ(stats.exact_checks, 2u);
    }

    TEST(FindIntersectionsBvhTest, IncludesSegmentCrossingAndVertexContact)
    {
        const std::vector<Triangle3> scene{
            {{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}},
            {{Point3{0.5, 0.5, -1.0}, Point3{0.5, 0.5, 1.0}, Point3{0.5, 0.5, 0.0}}},
            {{Point3{2.0, 0.0, 0.0}, Point3{3.0, 0.0, 0.0}, Point3{2.0, -1.0, 0.0}}},
            {{Point3{20.0, 0.0, 0.0}, Point3{21.0, 0.0, 0.0}, Point3{20.0, 1.0, 0.0}}},
            {{Point3{40.0, 0.0, 0.0}, Point3{40.0, 0.0, 0.0}, Point3{40.0, 0.0, 0.0}}}
        };
        BvhSearchStats stats;
        EXPECT_EQ(FindIntersectionsBvh(scene, stats), (std::vector<bool>{true, true, true, false, false}));
        EXPECT_EQ(FindIntersectionsBvh(scene, stats), FindIntersectionsBruteForce(scene));
    }

    TEST(FindIntersectionsBvhTest, DenseSceneChecksEveryUnorderedPairExactlyOnce)
    {
        const Triangle3 triangle{{Point3{0.0, 0.0, 0.0}, Point3{2.0, 0.0, 0.0}, Point3{0.0, 2.0, 0.0}}};
        const std::vector<Triangle3> scene(17, triangle);
        const std::size_t pair_count = scene.size() * (scene.size() - 1) / 2;
        BvhSearchStats stats;
        EXPECT_EQ(FindIntersectionsBvh(scene, stats), std::vector<bool>(scene.size(), true));
        EXPECT_EQ(stats.candidate_pairs, pair_count);
        EXPECT_EQ(stats.exact_checks, pair_count);
    }

    TEST(FindIntersectionsBvhTest, SeparatedScenePrunesPairsBeforeExactChecks)
    {
        const auto scene = MakeSeparatedBvhScene(65);
        BvhSearchStats stats;
        EXPECT_EQ(FindIntersectionsBvh(scene, stats), std::vector<bool>(scene.size(), false));
        EXPECT_EQ(stats.exact_checks, 0u);
        EXPECT_LT(stats.candidate_pairs, scene.size() * (scene.size() - 1) / 2);
    }

    TEST(FindIntersectionsBvhTest, ReusingStatisticsDoesNotAccumulateEarlierCalls)
    {
        const Point3 point{1.0, 2.0, 3.0};
        const Triangle3 triangle{{point, point, point}};
        BvhSearchStats stats;
        FindIntersectionsBvh(std::vector<Triangle3>(8, triangle), stats);
        ASSERT_EQ(stats.exact_checks, 28u);
        EXPECT_EQ(FindIntersectionsBvh({triangle, triangle}, stats), (std::vector<bool>{true, true}));
        EXPECT_EQ(stats.candidate_pairs, 1u);
        EXPECT_EQ(stats.exact_checks, 1u);
        EXPECT_TRUE(FindIntersectionsBvh({}, stats).empty());
        EXPECT_EQ(stats.candidate_pairs, 0u);
        EXPECT_EQ(stats.exact_checks, 0u);
    }

    TEST(FindIntersectionsBvhTest, DeterministicMixedScenesMatchBruteForceAfterShuffling)
    {
        std::mt19937 generator(20261104);
        std::uniform_int_distribution<int> coordinate(-5, 5);
        for (std::size_t sample = 0; sample < 30; ++sample)
        {
            SCOPED_TRACE(sample);
            std::vector<Triangle3> scene;
            for (std::size_t i = 0; i < 19; ++i)
            {
                Triangle3 triangle{};
                for (Point3& point : triangle.vertices)
                {
                    point.cord_x = coordinate(generator);
                    point.cord_y = coordinate(generator);
                    point.cord_z = sample % 2 == 0 ? 0 : coordinate(generator);
                }
                if (i % 5 == 0)
                    triangle.vertices[2] = triangle.vertices[1];
                if (i % 7 == 0)
                    triangle.vertices[1] = triangle.vertices[2] = triangle.vertices[0];
                scene.push_back(triangle);
            }
            for (std::size_t order = 0; order < 2; ++order)
            {
                SCOPED_TRACE(order);
                BvhSearchStats stats;
                EXPECT_EQ(FindIntersectionsBvh(scene, stats), FindIntersectionsBruteForce(scene));

                std::size_t overlapping_box_pairs = 0;
                for (std::size_t i = 0; i < scene.size(); ++i)
                {
                    for (std::size_t j = i + 1; j < scene.size(); ++j)
                    {
                        if (BoundsOverlap(Bounds(scene[i]), Bounds(scene[j])))
                            ++overlapping_box_pairs;
                    }
                }
                EXPECT_EQ(stats.exact_checks, overlapping_box_pairs);
                EXPECT_LE(stats.exact_checks, stats.candidate_pairs);
                EXPECT_LE(stats.candidate_pairs, scene.size() * (scene.size() - 1) / 2);
                std::shuffle(scene.begin(), scene.end(), generator);
            }
        }
    }

    TEST(CollectCandidatesTest, AppendsOnlyRequestedSubtreeToExistingResult)
    {
        const Bvh bvh = BuildBvh(MakeInterleavedCandidateScene());
        ASSERT_FALSE(bvh.nodes.empty());
        const BvhNode& root = bvh.nodes[0];
        ASSERT_FALSE(root.IsLeaf());
        ASSERT_LT(root.right, bvh.nodes.size());
        std::vector<std::size_t> result{123456};
        CollectCandidates(bvh, {{-10.0, -1.0, -1.0}, {200.0, 1.0, 1.0}}, root.right, result);
        ASSERT_EQ(result.size(), 5u);
        EXPECT_EQ(result.front(), 123456u);
        ExpectCandidateIndexes(std::vector<std::size_t>(result.begin() + 1, result.end()), {0, 2, 4, 6});
    }
}
