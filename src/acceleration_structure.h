#pragma once

#include <limits>
#include <numbers>
#include <numeric>
#include <span>
#include <variant>
#include <vector>

#include "model_loader.h"
#include "ray_program.h"

namespace app {

struct BBox {
    fvec3 min, max;

    BBox() noexcept;
    constexpr BBox(const fvec3& min_, const fvec3& max_) noexcept;

    bool is_empty() const noexcept;
    void expand(const fvec3& ws_point) noexcept;
    void expand(const BBox& bbox) noexcept;

    ray_volume_hit_info ray_volume_intersection(const ray& ray) const noexcept;
    ray_volume_hit_info ray_volume_intersection(const ray& ray, fvec3 ray_direction_inv) const noexcept;

    float surface_area() const noexcept;
};

BBox object_to_ws_bbox(const Object& obj, const Mesh& mesh);

struct DOP {
    DOP() noexcept;
    DOP(const fvec3& point) noexcept;

    bool is_empty() const noexcept;
    void expand(const fvec3& ws_point) noexcept;
    void expand(const DOP& dop) noexcept;
    BBox to_bbox();

    ray_volume_hit_info ray_volume_intersection(const ray& ray) const noexcept;
    ray_volume_hit_info ray_volume_intersection(const std::array<fvec2, 7>& projections) const noexcept;

  private:
    std::array<fvec2, 7> min_max;
    constexpr static auto inv_sqrt3 = std::numbers::inv_sqrt3_v<float>;

  public:
    static inline std::array<fvec3, 7> axises{
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

    virtual BBox get_scene_bounds() const = 0;
};

// common data for AS types
struct ObjectData {
    DOP volume;
    fmat4x4 ModelMatrix;
    fmat4x4 invModelMatrix;
    uint32_t objectIndex;
    uint32_t meshIndex;
    uint32_t complexity;
};

struct volume_hit_and_obj_index {
    ray_volume_hit_info hit_info;
    uint32_t object_index;
    bool operator<(const volume_hit_and_obj_index& other) const noexcept {  // for min-heap
        return hit_info.forward_hit_distance() > other.hit_info.forward_hit_distance();
    }
};

class NaiveAS : public IAccelerationStructure {
  public:
    explicit NaiveAS(const Model& model);
    virtual ~NaiveAS() override = default;

    ray_triangle_hit_info intersect_ray(const ray& ray, bool any_hit = false) const;

    BBox get_scene_bounds() const;

  private:
    struct MeshData {
        std::vector<fvec3> vertices;  // tuples of 3 vertices positions that form triangles
        bool doubleSided = false;
    };

    std::vector<ObjectData> object_data_;
    std::vector<MeshData> mesh_data_;

    static thread_local std::vector<volume_hit_and_obj_index> volume_intersections;

    template <bool any_hit = false>
    static ray_triangle_hit_info mesh_ray_intersection(
        const ray& ray_ws, const fmat4x4& ModelMatrix, const fmat4x4& invModelMatrix, const MeshData& mesh) noexcept;
};

class BVH_AS : public IAccelerationStructure {
  public:
    explicit BVH_AS(const Model& model, int max_triangles_per_leaf);
    virtual ~BVH_AS() override = default;

    ray_triangle_hit_info intersect_ray(const ray& ray, bool any_hit = false) const;

    BBox get_scene_bounds() const;

  private:
    struct MeshBVHNode {
        struct children {
            uint32_t left_child_index = -1;
            uint32_t right_child_index = -1;
        };
        struct triangle {
            fvec3 p1;
            fvec3 p2;
            fvec3 p3;
            uint32_t index;  // index of the first vertex of the triangle in vector of indices in original mesh
        };
        using triangles = std::span<triangle>;

        BBox volume;
        std::variant<children, triangles> payload;

        static BBox triangles_to_bbox(const std::span<MeshBVHNode::triangle> tris);
    };
    struct tree_info {
        int min_depth = std::numeric_limits<int>::max();
        int max_depth = 0;
        int mean_depth = 0;
        int min_tris_in_leaf = std::numeric_limits<int>::max();
        int max_tris_in_leaf = 0;
        int mean_tris_in_leaf = 0;
        int total_leaves = 0;
    };
    struct MeshBVHData {
        explicit MeshBVHData(const Mesh& mesh, bool double_sided, unsigned int max_triangles_per_leaf);
        std::vector<MeshBVHNode> nodes;
        bool doubleSided = false;
        unsigned int maxTrianglesPerLeaf = 8;

        std::vector<MeshBVHNode::triangle> data_storage;

        void parse(std::span<MeshBVHNode::triangle> triangles_span, uint32_t node_id = 0);
        std::span<MeshBVHNode::triangle>::iterator split_triangles(std::span<MeshBVHNode::triangle> triangles_span,
            const BBox& bbox);  // find best split estimate for bvh separation and partition data

        void collect_tree_info() const;
        void collect_tree_info_recursive(tree_info& info, uint32_t index, int depth) const;
    };

    std::vector<ObjectData> object_data_;
    std::vector<MeshBVHData> mesh_bvh_data_;

    static thread_local std::vector<volume_hit_and_obj_index> volume_intersections;
    static thread_local std::vector<uint32_t> bvh_stack;

    template <bool any_hit = false>
    static ray_triangle_hit_info mesh_ray_intersection(
        const ray& ray_ws, const fmat4x4& ModelMatrix, const fmat4x4& invModelMatrix, const MeshBVHData& mesh) noexcept;
};

}  // namespace app
