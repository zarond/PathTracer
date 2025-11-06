#pragma once

#include <limits>
#include <numeric>
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

    ray_bbox_hit_info ray_box_intersection(const ray& ray) const noexcept;
};

BBox object_to_ws_bbox(const Object& obj, const Mesh& mesh);

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
        BBox bbox;
        fmat4x4 ModelMatrix;
        fmat4x4 invModelMatrix;
        uint32_t meshIndex;
    };

    using MeshData = std::vector<fvec3>; // tuples of 3 vertices positions that form triangles

    std::vector<ObjectData> object_data_;
    std::vector<MeshData> mesh_data_;

    template<bool any_hit = false>
    static ray_triangle_hit_info mesh_ray_intersection(
        const ray& ray_ws,
        const fmat4x4& ModelMatrix,
        const fmat4x4& invModelMatrix,
        const MeshData& mesh) noexcept;

};

}