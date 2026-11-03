#pragma once

#include "geometry.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

namespace triangles
{
    inline constexpr std::size_t NoNode = std::numeric_limits<std::size_t>::max();

    struct BvhNode
    {
        Aabb3 bounds{};
        std::size_t left = NoNode;
        std::size_t right = NoNode;
        std::size_t begin = 0;
        std::size_t count = 0;

        bool IsLeaf() const;
    };

    struct Bvh
    {
        std::vector<BvhNode> nodes;
        std::vector<std::size_t> indexes;
    };

    struct BvhSearchStats
    {
        std::size_t candidate_pairs = 0;
        std::size_t exact_checks = 0;
    };

    struct CenterLess
    {
        const std::vector<Triangle3>& triangles;
        Axis axis;

        bool operator()(std::size_t first, std::size_t second) const;
    };

    Bvh BuildBvh(const std::vector<Triangle3>& triangles);
    std::size_t BuildNode(Bvh& bvh, const std::vector<Triangle3>& triangles, std::size_t begin, std::size_t count);
    Axis SplitAxis(const Aabb3& box);
    double BoxCenterCoordinate(const Aabb3& box, Axis axis);
    std::vector<std::size_t> FindBvhCandidates(const Bvh& bvh, const Aabb3& query_box);
    void CollectCandidates( const Bvh& bvh, const Aabb3& query_box, std::size_t node_index, std::vector<std::size_t>& result);
    std::vector<bool> FindIntersectionsBvh(const std::vector<Triangle3>& triangles, BvhSearchStats& stats);
}
