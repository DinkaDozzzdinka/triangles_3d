#include "input.hpp"

#include <charconv>
#include <cmath>
#include <istream>
#include <stdexcept>
#include <string>
#include <system_error>


namespace triangles
{
    namespace
    {
        double ReadCoordinate(std::istream& input)
        {
            std::string token;
            if (!(input >> token))
                throw std::invalid_argument("Missing point coordinate");

            const char* begin = token.data();
            const char* end = begin + token.size();
            if (token.front() == '+')
            {
                ++begin;
                if (begin == end || *begin == '-' || *begin == '+')
                    throw std::invalid_argument("Invalid point coordinate");
            }

            double coordinate = 0.0;
            const auto [ptr, ec] = std::from_chars(begin, end, coordinate, std::chars_format::general);
            if (ec != std::errc{} || ptr != end || !std::isfinite(coordinate))
                throw std::invalid_argument("Invalid or out-of-range point coordinate");
            return coordinate;
        }
    }

    std:: size_t ReadTriangleCount (std:: istream& input)
    {
        std:: string str_number_of_sets;
        if (!(input >> str_number_of_sets))
            throw std:: invalid_argument ("Error number of sets\n");
        std:: size_t number_of_sets = 0;
        auto [ptr, ec] = std:: from_chars(str_number_of_sets.data(), str_number_of_sets.data() + str_number_of_sets.size(), number_of_sets);
        if (ec != std:: errc{} || ptr != str_number_of_sets.data() + str_number_of_sets.size())
            throw std:: invalid_argument("Invalid recording number_of_sets\n");
    
        if (number_of_sets >= 1000000 || number_of_sets == 0)
            throw std:: invalid_argument ("Error: enter from edge of real value\n");
        return number_of_sets;
    }

    Point3 ReadPoint(std::istream& input)
    {
        Point3 point{};
        point.cord_x = ReadCoordinate(input);
        point.cord_y = ReadCoordinate(input);
        point.cord_z = ReadCoordinate(input);
        return point;
    }


    Triangle3 ReadTriangle(std::istream& input)
    {
        Triangle3 triangle{};
        triangle.vertices[0] = ReadPoint(input);
        triangle.vertices[1] = ReadPoint(input);
        triangle.vertices[2] = ReadPoint(input);
        return triangle;
    }

    std::vector<Triangle3> ReadScene(std::istream& input)
    {
        std::vector<Triangle3> triangles;
        std:: size_t N = ReadTriangleCount(input);
        for (std:: size_t i = 0; i < N; ++i)
        {
            triangles.push_back(ReadTriangle(input));
        }

        return triangles;
    }
}
