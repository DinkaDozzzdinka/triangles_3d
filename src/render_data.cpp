#include "render_data.hpp"

#include <cstddef>
#include <stdexcept>

namespace triangles 
{
    std::vector<RenderVertex> BuildRenderVertices(const std::vector<Triangle3>& scene,const std::vector<bool>& mask)
    {
        if (scene.size() != mask.size())
            throw std:: invalid_argument("Error number of triangles does not match with mask of intersection");
        std:: vector<RenderVertex> render_vertex;
        if (scene.empty())
            return render_vertex;
        render_vertex.resize(3*scene.size());
        for (std:: size_t i = 0; i < scene.size(); ++i)
        {
            std::array<float, 3> color{0.0f, 0.0f, 1.0f};
            if (mask[i])
                color = {1.0f, 0.0f, 0.0f};

            for (std:: size_t j = 0; j < scene[i].vertices.size(); ++j)
            {
                render_vertex[3 * i + j] = {scene[i].vertices[j], color};
            }
        }
        return render_vertex;
    }
}
