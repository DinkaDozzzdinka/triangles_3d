#include <iostream>
#include "input.hpp"

int main()
{
    std:: size_t number_of_sets = triangles:: ReadTriangleCount(std:: cin);
    std:: cout << "Triangle3D started\n" << number_of_sets << "\n";
    return 0;
}