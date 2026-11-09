#include "bvh.hpp"
#include "input.hpp"

#include <cstddef>
#include <exception>
#include <iostream>

int main()
{
    try
    {
        const auto scene = triangles::ReadScene(std::cin);
        triangles::BvhSearchStats stats{};
        const auto mask_intersection = triangles::FindIntersectionsBvh(scene, stats);

        for (std::size_t i = 0; i < mask_intersection.size(); ++i)
        {
            if (i > 0)
                std::cout << ' ';
            std::cout << mask_intersection[i];
        }
        std::cout << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
