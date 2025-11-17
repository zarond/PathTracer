#include <iostream>
#include <queue>
#include <algorithm>

#include "acceleration_structure.h"

namespace {
    using namespace app;
    using namespace glm;

#define CULLING

    constexpr float kEpsilon = 1e-5f;

    inline barycentric_coords intersect_ray_triangle( // output vec is barycentric coords A, B, hit. (C = 1.0 - A - B)
        const ray ray_os, 
        const fvec3 p1, 
        const fvec3 p2, 
        const fvec3 p3,
        const bool backface_culling = true) {
// MOLLER_TRUMBORE algorithm
        fvec3 p1p2 = p2 - p1;
        fvec3 p1p3 = p3 - p1;
        fvec3 pvec = cross(ray_os.direction, p1p3);
        float det = dot(p1p2, pvec);
        if (backface_culling) {
            // If the determinant is negative, the triangle is back-facing.
            if (det <= 0.0f) return { 0.0f, 0.0f, false, false }; // or epsilon?
        }
        // If det is close to 0, the ray and triangle are parallel.
        //if (fabs(det) < kEpsilon) return { 0.0f, 0.0f, false };

        float invDet = 1.0f / det;

        fvec3 tvec = ray_os.origin - p1;
        float u = dot(tvec, pvec) * invDet;
        if (u < 0.0f || u > 1.0f) return { 0.0f, 0.0f, 0.0f, false, false };

        fvec3 qvec = cross(tvec, p1p2);
        float v = dot(ray_os.direction, qvec) * invDet;
        if (v < 0.0f || u + v > 1.0f) return { 0.0f, 0.0f, 0.0f, false, false };

        float t = dot(p1p3, qvec) * invDet;

        return { 1.0f - u - v, u, t, true, (det < 0.0f) && !backface_culling };
    }
}

namespace app {
    BBox::BBox() noexcept
        : min(std::numeric_limits<float>::max()),
          max(std::numeric_limits<float>::lowest()) {
    }
    constexpr BBox::BBox(fvec3 min_, fvec3 max_) noexcept : min(min_), max(max_) {}

    bool BBox::is_empty() const noexcept { return max.x < min.x || max.y < min.y || max.z < min.z; }
    void BBox::expand(const fvec3& ws_point) noexcept {
        min = glm::min(min, ws_point);
        max = glm::max(max, ws_point);
    }
    // Todo: optimize
    ray_volume_hit_info BBox::ray_volume_intersection(const ray& ray) const noexcept {
        auto ray_direction_inv = 1.0f / ray.direction;
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
                auto pos = xyz1(vertex.position);
                bbox.expand(xyz(obj.ModelMatrix * pos));
            }
        );
        return bbox;
    }

    DOP::DOP() noexcept {
        min_max.fill(fvec2{ std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() });
    }

    bool DOP::is_empty() const noexcept { 
        return std::any_of(min_max.cbegin(), min_max.cend(), [](const auto& x) {return x.y < x.x; }); // max < min
    }
    void DOP::expand(const fvec3& ws_point) noexcept {
        for (int i = 0; i < 7; ++i)
        {
            float projection = dot(ws_point, axises[i]);
            auto& axis_min_max = min_max[i];
            axis_min_max.x = std::min(axis_min_max.x, projection);
            axis_min_max.y = std::max(axis_min_max.y, projection);
        }
    }

    ray_volume_hit_info DOP::ray_volume_intersection(const ray& ray) const noexcept {
        float t_min = std::numeric_limits<float>::lowest();
        float t_max = std::numeric_limits<float>::max();

        for (int i = 0; i < 7; ++i) {
            float o_projection = dot(ray.origin, axises[i]);
            float inv_d_projection = 1.0f / dot(ray.direction, axises[i]);
            const auto axis_min_max = min_max[i];
            float t0 = (axis_min_max.x - o_projection) * inv_d_projection;
            float t1 = (axis_min_max.y - o_projection) * inv_d_projection;
            if (t0 > t1) std::swap(t0, t1);
            t_min = std::max(t0, t_min);
            t_max = std::min(t1, t_max);
            if (t_max < t_min) return { false };
        }
        return { true, t_min, t_max };
    }

    DOP object_to_ws_dop(const Object& obj, const Mesh& mesh)
    {
        DOP dop;
        std::for_each(mesh.vertices.begin(), mesh.vertices.end(),
            [&dop, &obj](const vertex& vertex) {
                // Todo: optimize
                auto pos = xyz1(vertex.position);
                dop.expand(xyz(obj.ModelMatrix * pos));
            }
        );
        return dop;
    }

    NaiveAS::NaiveAS(const Model& model) {
        auto start = std::chrono::high_resolution_clock::now();
        object_data_.reserve(model.objects_.size());
        mesh_data_.reserve(model.meshes_.size());
        for (uint32_t i = 0; i < model.objects_.size(); ++i) {
            const auto& obj = model.objects_[i];
            const auto& mesh = model.meshes_[obj.meshIndex];
            DOP volume = object_to_ws_dop(obj, mesh);
            object_data_.emplace_back(
                volume, 
                obj.ModelMatrix, 
                transpose(obj.NormalMatrix), 
                i,
                obj.meshIndex,
                mesh.indices.size()
            );
        }
        // sort by complexity (number of triangles) to try easier objects first (starting from the back of array) in case of any_hit
        const auto complexity_cmp = [](const ObjectData& obj_a, const ObjectData& obj_b) {;
            return obj_a.complexity > obj_b.complexity; // more complex first
            };
        std::sort(object_data_.begin(), object_data_.end(), complexity_cmp);

        for (const auto& mesh : model.meshes_) {
            std::vector<fvec3> data;
            data.reserve(mesh.indices.size());
            std::for_each(mesh.indices.begin(), mesh.indices.end(),
                [&data, &mesh](std::uint32_t index) {
                    data.push_back(mesh.vertices[index].position);
                }
            );
            const auto& mat = model.materials_[mesh.materialIndex];
            bool doubleSided = mat.doubleSided || mat.hasVolume;
            mesh_data_.emplace_back(std::move(data), doubleSided);
        }
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
        volume_intersections.reserve(object_data_.size());
        std::cout << "NaiveAS constructed in " << diff.count() << " ms." << '\n';
    }

    std::vector<NaiveAS::volume_hit_and_obj_index> thread_local NaiveAS::volume_intersections;

    ray_triangle_hit_info NaiveAS::intersect_ray(const ray& ray, bool any_hit) const
    {
        ray_triangle_hit_info hit{};

        volume_intersections.clear();
        
        for (uint32_t i = 0; i < object_data_.size(); ++i) {
            const auto& obj = object_data_[i];
            ray_volume_hit_info potential_obj_hit = obj.volume.ray_volume_intersection(ray);
            volume_intersections.emplace_back(potential_obj_hit, i);
        }
        if (!any_hit) {
            // if closest_hit, make a min-heap to efficiently get closest volume hit
            std::make_heap(volume_intersections.begin(), volume_intersections.end());
        }
        while (!volume_intersections.empty()) {
            const auto& potential_obj_hit = (!any_hit) ? volume_intersections[0] : volume_intersections.back(); // heap top
            if (potential_obj_hit.hit_info.forward_hit() && potential_obj_hit.hit_info.forward_hit_distance() < hit.distance) {
                // Perform detailed intersection test with the mesh
                const auto& obj = object_data_[potential_obj_hit.object_index];
                ray_triangle_hit_info potential_mesh_hit = (any_hit)? 
                    mesh_ray_intersection<true>(ray, obj.ModelMatrix, obj.invModelMatrix, mesh_data_[obj.meshIndex]) :
                    mesh_ray_intersection<false>(ray, obj.ModelMatrix, obj.invModelMatrix, mesh_data_[obj.meshIndex]);
                if (potential_mesh_hit.forward_hit() && (potential_mesh_hit.distance < hit.distance)) {
                    hit = potential_mesh_hit;
                    hit.objectIndex = obj.objectIndex;
                    hit.meshIndex = obj.meshIndex;
                    if (any_hit) {
                        break;
                    }
                }
            } else if (!any_hit) {
                break;
            }
            if (!any_hit) {
                std::pop_heap(volume_intersections.begin(), volume_intersections.end());
            }
            volume_intersections.pop_back();
        }
        return hit;
    }

    template<bool any_hit>
    static ray_triangle_hit_info NaiveAS::mesh_ray_intersection(
        const ray& ray_ws,
        const fmat4x4& ModelMatrix,
        const fmat4x4& invModelMatrix,
        const MeshData& mesh) noexcept
    {
        // Transform ray to object space
        auto ray_origin_os = xyz(invModelMatrix * xyz1(ray_ws.origin));
        auto ray_direction_os = xyz(invModelMatrix * xyz0(ray_ws.direction));
        
        ray os_ray{
            ray_origin_os,
            normalize(ray_direction_os)
        };

        ray_triangle_hit_info hit{};

        // Test all triangles
        for (auto it = mesh.vertices.cbegin(); it != mesh.vertices.cend();) {
            fvec3 p1 = *it++;
            fvec3 p2 = *it++;
            fvec3 p3 = *it++;
            barycentric_coords tri_hit = intersect_ray_triangle(os_ray, p1, p2, p3, !mesh.doubleSided);
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
                .triangleIndex = static_cast<uint32_t>(std::distance(mesh.vertices.cbegin(), it) - 3)
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
        fvec3 hit_point_ws = xyz(ModelMatrix * xyz1(best_hit_point));
        hit.distance = length(hit_point_ws - ray_ws.origin);
        return hit;
    }
}