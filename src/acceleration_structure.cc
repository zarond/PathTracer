#include <iostream>

#include "acceleration_structure.h"
#include "additional_math.h"

namespace {
    using namespace app;
    using namespace fastgltf::math;

#define CULLING

    constexpr float kEpsilon = 1e-5f;

    inline barycentric_coords intersect_ray_triangle( // output vec is barycentric coords A, B, hit. (C = 1.0 - A - B)
        const ray ray_os, 
        const fvec3 p1, 
        const fvec3 p2, 
        const fvec3 p3) {
// MOLLER_TRUMBORE algorithm
        fvec3 p1p2 = p2 - p1;
        fvec3 p1p3 = p3 - p1;
        fvec3 pvec = cross(ray_os.direction, p1p3);
        float det = dot(p1p2, pvec);
#ifdef CULLING
        // If the determinant is negative, the triangle is back-facing.
        // If the determinant is close to 0, the ray misses the triangle.
        if (det < 0.0f) return { 0.0, 0.0, false }; // or epsilon?
#else
      // If det is close to 0, the ray and triangle are parallel.
        if (fabs(det) < kEpsilon) return { 0.0, 0.0, false };
#endif
        float invDet = 1 / det;

        fvec3 tvec = ray_os.origin - p1;
        float u = dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return { 0.0, 0.0, 0.0, false };

        fvec3 qvec = cross(tvec, p1p2);
        float v = dot(ray_os.direction, qvec) * invDet;
        if (v < 0 || u + v > 1) return { 0.0, 0.0, 0.0, false };

        float t = dot(p1p3, qvec) * invDet;

        return { 1.0f - u - v, u, t, true };
    }
}

namespace app {
    BBox::BBox() noexcept
        : min(std::numeric_limits<float>::max()),
          max(std::numeric_limits<float>::lowest()) {
    }
    constexpr BBox::BBox(fvec3 min_, fvec3 max_) noexcept : min(min_), max(max_) {}

    constexpr bool BBox::is_empty() const noexcept { return max[0] < min[0] || max[1] < min[1] || max[2] < min[2]; }
    void BBox::expand(const fvec3& ws_point) noexcept {
        min = fastgltf::math::min(min, ws_point);
        max = fastgltf::math::max(max, ws_point);
    }
    // Todo: optimize
    ray_bbox_hit_info BBox::ray_box_intersection(const ray& ray) const noexcept {
        auto ray_direction_inv = fvec3{
            1.0f / ray.direction[0],
            1.0f / ray.direction[1],
            1.0f / ray.direction[2]
        };
        float t_min = std::numeric_limits<float>::lowest();
        float t_max = std::numeric_limits<float>::max();
        for (int i = 0; i < 3; ++i) {
            float t0 = (min[i] - ray.origin[i]) * ray_direction_inv[i];
            float t1 = (max[i] - ray.origin[i]) * ray_direction_inv[i];
            if (t0 > t1) std::swap(t0, t1);
            t_min = std::max(t0, t_min);
            t_max = std::min(t1, t_max);
            if (t_max < t_min) return { false };
        }
        return { true, t_min, t_max };
    }

    BBox object_to_ws_bbox(const Object& obj, const Mesh& mesh)
    {
        BBox bbox;
        std::for_each(mesh.vertices.begin(), mesh.vertices.end(), 
            [&bbox, &obj](const vertex& vertex) {
                // Todo: optimize
                auto pos = fvec4(vertex.position[0], vertex.position[1], vertex.position[2], 1.0);
                bbox.expand(fvec3(obj.ModelMatrix * pos));
            }
        );
        return bbox;
    }

    NaiveAS::NaiveAS(const Model& model) : mesh_data_(&model.meshes_)
    {
        auto start = std::chrono::high_resolution_clock::now();
        object_data_.reserve(model.objects_.size());
        for (const auto& obj : model.objects_) {
            BBox bbox = object_to_ws_bbox(obj, model.meshes_[obj.meshIndex]);
            object_data_.emplace_back(bbox, obj.ModelMatrix, transpose(obj.NormalMatrix), obj.meshIndex);
        }
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
        std::cout << "NaiveAS constructed in " << diff.count() << " ms." << '\n';
    }

    ray_triangle_hit_info NaiveAS::intersect_ray(const ray& ray, bool any_hit) const
    {
        ray_triangle_hit_info hit{};
        // ToDo: optimize
        for (size_t i = 0; i < object_data_.size(); ++i) {
            const auto& obj = object_data_[i];
            ray_bbox_hit_info potential_obj_hit = obj.bbox.ray_box_intersection(ray);
            if (potential_obj_hit.forward_hit()) {
                // Perform detailed intersection test with the mesh
                ray_triangle_hit_info potential_mesh_hit = (any_hit)? 
                    mesh_ray_intersection<true>(ray, obj.ModelMatrix, obj.invModelMatrix, mesh_data_->at(obj.meshIndex)) :
                    mesh_ray_intersection<false>(ray, obj.ModelMatrix, obj.invModelMatrix, mesh_data_->at(obj.meshIndex));
                if (potential_mesh_hit.forward_hit() && (potential_mesh_hit.distance < hit.distance)) {
                    hit = potential_mesh_hit;
                    hit.objectIndex = i;
                    hit.meshIndex = obj.meshIndex;
                    if (any_hit) {
                        break;
                    }
                }
            }
        }
        return hit;
    }

    template<bool any_hit>
    ray_triangle_hit_info mesh_ray_intersection(
        const ray& ray_ws,
        const fmat4x4& ModelMatrix,
        const fmat4x4& invModelMatrix,
        const Mesh& mesh) noexcept
    {
        using namespace fastgltf::math;

        // Transform ray to object space
        auto ray_origin_os = fvec3(invModelMatrix * fvec4(ray_ws.origin[0], ray_ws.origin[1], ray_ws.origin[2], 1.0f));
        auto ray_direction_os = fvec3(invModelMatrix * fvec4(ray_ws.direction[0], ray_ws.direction[1], ray_ws.direction[2], 0.0f));
        
        ray os_ray{
            ray_origin_os,
            normalize(ray_direction_os)
        };

        ray_triangle_hit_info hit{};

        // Test all triangles
        for (std::uint32_t tri_idx = 0; tri_idx < mesh.indices.size(); tri_idx += 3) {
            fvec3 p1 = mesh.vertices[mesh.indices[tri_idx + 0]].position;
            fvec3 p2 = mesh.vertices[mesh.indices[tri_idx + 1]].position;
            fvec3 p3 = mesh.vertices[mesh.indices[tri_idx + 2]].position;
            barycentric_coords tri_hit = intersect_ray_triangle(os_ray, p1, p2, p3);
            if (!tri_hit.hit || tri_hit.t < 0.0f) {
                continue; // No hit or back from origin
            }
            if (tri_hit.t >= hit.b_coords.t) {
                continue; // Not the closest hit
            }
            hit = {
                .hit = true,
                .distance = 0.0f,
                .b_coords = tri_hit,
                .triangleIndex = tri_idx
            };
            if constexpr (any_hit) {
                break;
            }
        }
        if (!hit.hit) {
            return hit; // No hit
        }
        // Transform hit back to world space
        fvec3 best_hit_point = os_ray.origin + os_ray.direction * hit.b_coords.t;
        fvec3 hit_point_ws = fvec3(ModelMatrix * fvec4(best_hit_point[0], best_hit_point[1], best_hit_point[2], 1.0f));
        hit.distance = length(hit_point_ws - ray_ws.origin);
        return hit;
    }
}