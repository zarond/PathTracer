#pragma once

#include <limits>
#include <numeric>
#include <span>
#include <vector>

#include <fastgltf/types.hpp>

#include "additional_math.h"
#include "ray_generator.h"
#include "model_loader.h"

namespace app {

struct BBox {

    fvec3 min, max;

    BBox() noexcept;
    constexpr BBox(fvec3 min_, fvec3 max_) noexcept;
   
    constexpr bool is_empty() const noexcept;
    void expand(const fvec3& ws_point) noexcept;

    constexpr ray_bbox_hit_info ray_box_intersection(const ray& ray) const noexcept;
};

BBox object_to_ws_bbox(const Object& obj, const Mesh& mesh);

ray_triangle_hit_info mesh_ray_intersection(
    const ray& ray_ws,
    const fmat4x4& ModelMatrix,
    const fmat4x4& invModelMatrix,
    const Mesh& mesh) noexcept;

class AccelerationStructure {
public:
    virtual ~AccelerationStructure() = default;
};

class NaiveAS : public AccelerationStructure {
public:
    explicit NaiveAS(const Model& model);

    struct ObjectData {
        BBox bbox;
        fmat4x4 ModelMatrix;
        fmat4x4 invModelMatrix;
        size_t meshIndex;
    };

    ray_triangle_hit_info intersect_ray(const ray& ray, size_t& out_object_index) const;

//private:
    std::vector<ObjectData> object_data_;
    const std::vector<Mesh>* mesh_data_ = nullptr;
};

}