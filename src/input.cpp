#include "input.hpp"

#include <charconv>
#include <istream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace triangles
{
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

}
