#include "render_data.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace triangles 
{
    namespace
    {
        Vec3 ScaledDifference(const Point3& point, const Point3& origin, double factor)
        {
            return {factor * point.cord_x - factor * origin.cord_x,
                    factor * point.cord_y - factor * origin.cord_y,
                    factor * point.cord_z - factor * origin.cord_z};
        }

        Aabb3 RenderBounds(const std::vector<RenderVertex>& vertices)
        {
            Aabb3 bounds{vertices.front().position, vertices.front().position};
            for (const RenderVertex& vertex : vertices)
            {
                const Point3& point = vertex.position;
                if (!std::isfinite(point.cord_x) || !std::isfinite(point.cord_y)
                    || !std::isfinite(point.cord_z))
                    throw std::invalid_argument("View fitting requires finite coordinates");
                bounds.minimum.cord_x = std::min(bounds.minimum.cord_x, point.cord_x);
                bounds.minimum.cord_y = std::min(bounds.minimum.cord_y, point.cord_y);
                bounds.minimum.cord_z = std::min(bounds.minimum.cord_z, point.cord_z);
                bounds.maximum.cord_x = std::max(bounds.maximum.cord_x, point.cord_x);
                bounds.maximum.cord_y = std::max(bounds.maximum.cord_y, point.cord_y);
                bounds.maximum.cord_z = std::max(bounds.maximum.cord_z, point.cord_z);
            }
            return bounds;
        }
    }

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

    void FitRenderVerticesToView(std::vector<RenderVertex>& vertices)
    {
        if (vertices.empty())
            return;

        const Aabb3 bounds = RenderBounds(vertices);
        double factor = 0.5;
        Vec3 span = ScaledDifference(bounds.maximum, bounds.minimum, factor);
        double extent = std::max({span.cord_x, span.cord_y, span.cord_z});
        if (extent < std::numeric_limits<double>::min())
        {
            factor = 1.0;
            span = ScaledDifference(bounds.maximum, bounds.minimum, factor);
            extent = std::max({span.cord_x, span.cord_y, span.cord_z});
        }

        if (extent == 0.0)
        {
            for (RenderVertex& vertex : vertices)
                vertex.position = {};
            return;
        }

        const Vec3 midpoint{0.5 * (span.cord_x / extent),
                            0.5 * (span.cord_y / extent),
                            0.5 * (span.cord_z / extent)};
        for (RenderVertex& vertex : vertices)
        {
            const Vec3 relative = ScaledDifference(vertex.position, bounds.minimum, factor);
            vertex.position = {1.8 * (relative.cord_x / extent - midpoint.cord_x),
                               1.8 * (relative.cord_y / extent - midpoint.cord_y),
                               1.8 * (relative.cord_z / extent - midpoint.cord_z)};
        }
    }
}
