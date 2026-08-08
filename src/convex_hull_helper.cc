#include <vector>
#include <glm/glm.hpp>
#include "model_loader.h"

#include "quickhull.hpp"

namespace app {

using namespace glm;

std::vector<fvec3> compute_convex_hull(const Mesh& mesh) {
    using namespace quickhull;
 
    const auto& vertices = mesh.vertices;

    quickhull::QuickHull<float> qh;
    std::vector<Vector3<float>> pointCloud;

    pointCloud.reserve(vertices.size());
    for (const auto& vertex : vertices) {
        pointCloud.emplace_back(vertex.position.x, vertex.position.y, vertex.position.z);
    }
    auto hull = qh.getConvexHull(pointCloud, true, false);
    const auto& vertexBuffer = hull.getVertexBuffer();

    std::vector<fvec3> result;
    result.reserve(vertexBuffer.size());
    for (const auto& vertex : vertexBuffer) {
        result.emplace_back(vertex.x, vertex.y, vertex.z);
    }
    // std::cout << "Convex hull has " << result.size() << " vertices. Before: " << vertices.size() << std::endl;
    return result;
}

}  // namespace app