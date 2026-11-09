#pragma once

#include <vector>
#include <array>
#include "geometry.hpp"

namespace triangles 
{
    struct RenderVertex
    {
        Point3 position{};
        std::array<float, 3> color{};
    };

    std::vector<RenderVertex> BuildRenderVertices(const std::vector<Triangle3>& scene,const std::vector<bool>& mask);
}
