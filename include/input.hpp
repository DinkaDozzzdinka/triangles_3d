#pragma once

#include <cstddef>
#include <iosfwd>
#include <vector>

#include "geometry.hpp"


namespace triangles 
{
    std:: size_t ReadTriangleCount (std:: istream& input);
    Point3 ReadPoint(std::istream& input);
    Triangle3 ReadTriangle(std::istream& input);
    std::vector<Triangle3> ReadScene(std::istream& input);
}