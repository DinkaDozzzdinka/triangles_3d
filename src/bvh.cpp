#include "bvh.hpp"
#include "intersection.hpp"
#include <algorithm>



namespace triangles
{
    
    bool BvhNode::IsLeaf() const
    {
        return count > 0;
    }
    Bvh BuildBvh(const std::vector<Triangle3>& triangle)
    {
        Bvh bvh{};
        if (triangle.empty())
        return bvh;
        for (std::size_t i = 0; i < triangle.size(); ++i)
        {
            bvh.indexes.push_back(i);
        }
        BuildNode (bvh, triangle, 0, triangle.size());
        return bvh;
    }
    
    std::size_t BuildNode(Bvh& bvh, const std::vector<Triangle3>& triangles, std::size_t begin, std::size_t count)
    {
        Aabb3 box_current = Bounds(triangles[bvh.indexes[begin]]);
        for (std:: size_t i = begin + 1; i < begin + count; ++i)
        {
            Aabb3 box_i = Bounds(triangles[bvh.indexes[i]]);
            box_current = MergeBounds(box_current, box_i);
        }

        BvhNode leaf{box_current, NoNode, NoNode, begin, count};
        bvh.nodes.push_back(leaf);
        std:: size_t index_current_node = bvh.nodes.size() - 1;
        if (count > 4)
        {
            const Axis current_axis = SplitAxis(box_current);
            const CenterLess centerless{triangles, current_axis};
            const std::size_t middle_position = begin + count / 2;
            std::nth_element(bvh.indexes.begin() + begin, bvh.indexes.begin() + middle_position, bvh.indexes.begin() + begin + count, centerless);
            std:: size_t left_count  = count / 2;
            std:: size_t right_count = count - left_count;
            std:: size_t left = BuildNode(bvh, triangles, begin, left_count);
            std:: size_t right = BuildNode(bvh, triangles, begin + left_count, right_count);
            bvh.nodes[index_current_node].left = left;
            bvh.nodes[index_current_node].right = right;
            bvh.nodes[index_current_node].count = 0;

        }
        return index_current_node;
    }

    Axis SplitAxis(const Aabb3& box)
    {
        if (std:: max({0.5*box.maximum.cord_x - 0.5*box.minimum.cord_x, 0.5*box.maximum.cord_y - 0.5*box.minimum.cord_y, 0.5*box.maximum.cord_z - 0.5*box.minimum.cord_z}) == 0.5*box.maximum.cord_x - 0.5*box.minimum.cord_x)
            return Axis:: X;
        else if (std:: max({0.5*box.maximum.cord_x - 0.5*box.minimum.cord_x, 0.5*box.maximum.cord_y - 0.5*box.minimum.cord_y, 0.5*box.maximum.cord_z - 0.5*box.minimum.cord_z}) == 0.5*box.maximum.cord_y - 0.5*box.minimum.cord_y)
                return Axis:: Y;
        else return Axis:: Z;
            
    }

    double BoxCenterCoordinate(const Aabb3& box, Axis axis)
    {
        if (axis == Axis::X)
            return 0.5*box.maximum.cord_x + 0.5*box.minimum.cord_x;
        else if (axis == Axis::Y)
                return 0.5*box.maximum.cord_y + 0.5*box.minimum.cord_y;
        else return 0.5*box.maximum.cord_z + 0.5*box.minimum.cord_z;
    }   

    bool CenterLess::operator()(std::size_t first, std::size_t second) const
    {
        const Aabb3 box_first = Bounds(triangles[first]);
        const Aabb3 box_second = Bounds(triangles[second]);
        const double center_first = BoxCenterCoordinate(box_first, axis);
        const double center_second = BoxCenterCoordinate(box_second, axis);
        if (center_first < center_second)
            return true;
        if (center_first == center_second && first < second)
            return true;
        return false;
    }

    std::vector<std::size_t> FindBvhCandidates(const Bvh& bvh, const Aabb3& query_box)
    {
        std::vector<std::size_t> result;
        if (bvh.nodes.empty())
            return result;
        CollectCandidates(bvh, query_box, 0, result);
        return result;
    }

    std::vector<bool> FindIntersectionsBvh(const std::vector<Triangle3>& triangles, BvhSearchStats& stats)
    {
        stats = {};
        std::vector<bool> flags(triangles.size(), false);
        if (triangles.empty())
            return flags;

        const Bvh bvh = BuildBvh(triangles);
        std::vector<Aabb3> boxes;
        boxes.reserve(triangles.size());
        for (const Triangle3& triangle : triangles)
            boxes.push_back(Bounds(triangle));

        for (std::size_t i = 0; i < triangles.size(); ++i)
        {
            const auto candidates = FindBvhCandidates(bvh, boxes[i]);
            for (const std::size_t j : candidates)
            {
                if (j <= i)
                    continue;

                ++stats.candidate_pairs;
                if (!BoundsOverlap(boxes[i], boxes[j]))
                    continue;

                ++stats.exact_checks;
                if (TrianglesIntersect(triangles[i], triangles[j]))
                {
                    flags[i] = true;
                    flags[j] = true;
                }
            }
        }
        return flags;
    }

    void CollectCandidates(const Bvh& bvh, const Aabb3& query_box, std::size_t node_index, std::vector<std::size_t>& result)
    {
        const BvhNode& node = bvh.nodes[node_index];
        if (!BoundsOverlap(query_box, node.bounds))
            return;

        if (node.IsLeaf())
        {
            for (std::size_t i = node.begin; i < node.begin + node.count; ++i)
                result.push_back(bvh.indexes[i]);
            return;
        }

        CollectCandidates(bvh, query_box, node.left, result);
        CollectCandidates(bvh, query_box, node.right, result);
    }
}
