#pragma once

#include <limits>
#include <numeric>
#include <numbers>
#include <span>
#include <vector>

#include "ray_program.h"
#include "model_loader.h"

namespace app {

struct BBox {
    fvec3 min, max;

    BBox() noexcept;
    constexpr BBox(fvec3 min_, fvec3 max_) noexcept;
   
    bool is_empty() const noexcept;
    void expand(const fvec3& ws_point) noexcept;

    ray_volume_hit_info ray_volume_intersection(const ray& ray) const noexcept;
};

BBox object_to_ws_bbox(const Object& obj, const Mesh& mesh);

struct DOP {
    DOP() noexcept;

    bool is_empty() const noexcept;
    void expand(const fvec3& ws_point) noexcept;

    ray_volume_hit_info ray_volume_intersection(const ray& ray) const noexcept;

private:
    std::array<fvec2, 7> min_max;
    constexpr static auto inv_sqrt3 = std::numbers::inv_sqrt3_v<float>;
    static inline std::array<fvec3, 7> axises {
        fvec3{1.0f, 0.0f, 0.0f},
        fvec3{0.0f, 1.0f, 0.0f},
        fvec3{0.0f, 0.0f, 1.0f},
        fvec3{inv_sqrt3, inv_sqrt3, inv_sqrt3},
        fvec3{inv_sqrt3, inv_sqrt3, -inv_sqrt3},
        fvec3{inv_sqrt3, -inv_sqrt3, inv_sqrt3},
        fvec3{-inv_sqrt3, inv_sqrt3, inv_sqrt3},
    };
};

DOP object_to_ws_dop(const Object& obj, const Mesh& mesh);

class IAccelerationStructure {
public:
    virtual ~IAccelerationStructure() = default;
    virtual ray_triangle_hit_info intersect_ray(const ray& ray, bool any_hit = false) const = 0;
};

class NaiveAS : public IAccelerationStructure {
public:
    explicit NaiveAS(const Model& model);
    virtual ~NaiveAS() override = default;

    ray_triangle_hit_info intersect_ray(const ray& ray, bool any_hit = false) const;

private:
    struct ObjectData {
        DOP volume;
        fmat4x4 ModelMatrix;
        fmat4x4 invModelMatrix;
        uint32_t objectIndex;
        uint32_t meshIndex;
        uint32_t complexity;
    };

    struct MeshData {
        std::vector<fvec3> vertices; // tuples of 3 vertices positions that form triangles
        bool doubleSided = false;
    };

    std::vector<ObjectData> object_data_;
    std::vector<MeshData> mesh_data_;

    struct volume_hit_and_obj_index {
        ray_volume_hit_info hit_info;
        uint32_t object_index;
        bool operator < (const volume_hit_and_obj_index& other) const noexcept { // for min-heap
            return hit_info.forward_hit_distance() > other.hit_info.forward_hit_distance();
        }
    };

    static thread_local std::vector<volume_hit_and_obj_index> volume_intersections;

    template<bool any_hit = false>
    static ray_triangle_hit_info mesh_ray_intersection(
        const ray& ray_ws,
        const fmat4x4& ModelMatrix,
        const fmat4x4& invModelMatrix,
        const MeshData& mesh) noexcept;

};

}