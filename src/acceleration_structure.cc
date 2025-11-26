#include <iostream>
#include <algorithm>
#include <execution>

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

    int get_longest_axis(const BBox& bbox) noexcept {
        fvec3 extents = bbox.max - bbox.min;
        if (extents.x >= extents.y && extents.x >= extents.z) {
            return 0;
        }
        else if (extents.y >= extents.x && extents.y >= extents.z) {
            return 1;
        }
        return 2;
    }
    float SurfaceAreaHeuristic(const BBox& bbox, int N) {
        if (bbox.is_empty()) return std::numeric_limits<float>::infinity();
        return bbox.surface_area() * N;
    }
    float SurfaceAreaHeuristic(const BBox& bbox_l, const BBox& bbox_r, int N_l, int N_r) {
        if (bbox_l.is_empty() || bbox_r.is_empty()) return std::numeric_limits<float>::infinity();
        return bbox_l.surface_area() * N_l + bbox_r.surface_area() * N_r;
    }
}

namespace app {
    BBox::BBox() noexcept
        : min(std::numeric_limits<float>::max()),
          max(std::numeric_limits<float>::lowest()) {
    }
    constexpr BBox::BBox(const fvec3& min_, const fvec3& max_) noexcept : min(min_), max(max_) {}

    bool BBox::is_empty() const noexcept { return max.x < min.x || max.y < min.y || max.z < min.z; }
    void BBox::expand(const fvec3& ws_point) noexcept {
        min = glm::min(min, ws_point);
        max = glm::max(max, ws_point);
    }
    void BBox::expand(const BBox& bbox) noexcept {
        min = glm::min(min, bbox.min);
        max = glm::max(max, bbox.max);
    }
    // mildly-optimised scalar version, without ray reuse
    ray_volume_hit_info BBox::ray_volume_intersection(const ray& ray) const noexcept {
        auto ray_direction_inv = 1.0f / ray.direction;

        float t_min = (min.x - ray.origin.x) * ray_direction_inv.x;
        float t_max = (max.x - ray.origin.x) * ray_direction_inv.x;
        if (t_max < t_min) std::swap(t_min, t_max);

        float t0 = (min.y - ray.origin.y) * ray_direction_inv.y;
        float t1 = (max.y - ray.origin.y) * ray_direction_inv.y;
        if (t0 > t1) std::swap(t0, t1);
        t_min = std::max(t0, t_min);
        t_max = std::min(t1, t_max);
        if (t_max < t_min) return { false };

        t0 = (min.z - ray.origin.z) * ray_direction_inv.z;
        t1 = (max.z - ray.origin.z) * ray_direction_inv.z;
        if (t0 > t1) std::swap(t0, t1);
        t_min = std::max(t0, t_min);
        t_max = std::min(t1, t_max);
        if (t_max < t_min) return { false };

        return { true, t_min, t_max };
    }
    // optimised simd version, with ray reuse
    ray_volume_hit_info BBox::ray_volume_intersection(const ray& ray, fvec3 ray_direction_inv) const noexcept {
        fvec3 t_min = (min - ray.origin) * ray_direction_inv;
        fvec3 t_max = (max - ray.origin) * ray_direction_inv;

        fvec3 t0 = glm::min(t_min, t_max);
        fvec3 t1 = glm::max(t_min, t_max);

        t0.x = (t0.x > t0.y) ? t0.x : t0.y;
        t0.x = (t0.x > t0.z) ? t0.x : t0.z;
        t1.x = (t1.x < t1.y) ? t1.x : t1.y;
        t1.x = (t1.x < t1.z) ? t1.x : t1.z;
        
        return { (t0.x <= t1.x), t0.x, t1.x };
    }

    float BBox::surface_area() const noexcept
    {
        auto dim = max - min;
        return dim.x * dim.y + dim.x * dim.z + dim.y * dim.z; // leave *2.0f
    }

    BBox object_to_ws_bbox(const Object& obj, const Mesh& mesh)
    {
        auto combine = [](BBox a, const BBox& b) {
            a.expand(b);
            return a;
        };
        auto make_bbox = [mat = obj.ModelMatrix](const vertex& v) {
            auto p = xyz(mat * xyz1(v.position));
            return BBox{ p, p };
        };
        return std::transform_reduce(std::execution::unseq,  mesh.vertices.begin(), mesh.vertices.end(), BBox{}, combine, make_bbox);
        // unfortunately no performance benefit using transform_reduce compared to simple for_each vertex expand() on my machine
        // std::execution::par_unseq is slower too
    }

    DOP::DOP() noexcept {
        min_max.fill(fvec2{ std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() });
    }
    DOP::DOP(const fvec3& point) noexcept {
        for (int i = 0; i < 7; ++i)
        {
            float projection = dot(point, axises[i]);
            min_max[i] = { projection , projection };
        }
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
    void DOP::expand(const DOP& dop) noexcept {
        for (int i = 0; i < 7; ++i)
        {
            auto& axis_min_max = min_max[i];
            axis_min_max.x = std::min(axis_min_max.x, dop.min_max[i].x);
            axis_min_max.y = std::max(axis_min_max.y, dop.min_max[i].y);
        }
    }
    BBox DOP::to_bbox() {
        return BBox{ fvec3{ min_max[0].x, min_max[1].x, min_max[2].x },
                     fvec3{ min_max[0].y, min_max[1].y, min_max[2].y } };
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
    // optimised for ray reuse
    ray_volume_hit_info DOP::ray_volume_intersection(const ray& ray, const std::array<fvec2, 7>& projections) const noexcept {
        float t_min = std::numeric_limits<float>::lowest();
        float t_max = std::numeric_limits<float>::max();

        for (int i = 0; i < 7; ++i) {
            const auto axis_min_max = min_max[i];
            float t0 = (axis_min_max.x - projections[i].x) * projections[i].y;
            float t1 = (axis_min_max.y - projections[i].x) * projections[i].y;
            if (t0 > t1) std::swap(t0, t1);
            t_min = std::max(t0, t_min);
            t_max = std::min(t1, t_max);
            if (t_max < t_min) return { false };
        }
        return { true, t_min, t_max };
    }

    DOP object_to_ws_dop(const Object& obj, const Mesh& mesh)
    {
        auto combine = [](DOP a, const DOP& b) {
            a.expand(b);
            return a;
            };
        auto make_dop = [mat = obj.ModelMatrix](const vertex& v) {
            auto p = xyz(mat * xyz1(v.position));
            return DOP{p};
            };
        return std::transform_reduce(std::execution::unseq, mesh.vertices.begin(), mesh.vertices.end(), DOP{}, combine, make_dop);
        // unfortunately no performance benefit using transform_reduce
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

    std::vector<volume_hit_and_obj_index> thread_local NaiveAS::volume_intersections;

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

    BBox NaiveAS::get_scene_bounds() const {
        DOP dop;
        for (const auto& obj : object_data_) {
            dop.expand(obj.volume);
        }
        return dop.to_bbox();
    }

    // NaiveAS

    BVH_AS::BVH_AS(const Model& model, int max_triangles_per_leaf) {
        auto start = std::chrono::high_resolution_clock::now();
        object_data_.reserve(model.objects_.size());
        mesh_bvh_data_.reserve(model.meshes_.size());
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
            const auto& mat = model.materials_[mesh.materialIndex];
            bool doubleSided = mat.doubleSided || mat.hasVolume;
            mesh_bvh_data_.emplace_back(mesh, doubleSided, max_triangles_per_leaf);
        }
        volume_intersections.reserve(object_data_.size());
        bvh_stack.reserve(64); // preallocate stack memory
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
        std::cout << "BVH_AS constructed in " << diff.count() << " ms." << '\n';

        for (const auto& mesh : mesh_bvh_data_) {
            mesh.collect_tree_info();
        }
    }

    BVH_AS::MeshBVHData::MeshBVHData(const Mesh& mesh, bool double_sided, int max_triangles_per_leaf) 
        : doubleSided(double_sided), maxTrianglesPerLeaf(max_triangles_per_leaf)
    {
        nodes.reserve(2 * mesh.indices.size() / (3 * maxTrianglesPerLeaf)); // upper bound

        data_storage.reserve(mesh.indices.size() / 3);
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            MeshBVHNode::triangle tri{
                mesh.vertices[mesh.indices[i]].position,
                mesh.vertices[mesh.indices[i + 1]].position,
                mesh.vertices[mesh.indices[i + 2]].position,
                static_cast<uint32_t>(i)
            };
            data_storage.push_back(tri);
        }

        parse(data_storage);
    }

    void BVH_AS::MeshBVHData::parse(std::span<MeshBVHNode::triangle> triangles_span, uint32_t node_id) {        
        BBox bbox = MeshBVHNode::triangles_to_bbox(triangles_span);

        MeshBVHNode new_node{
            .volume = bbox,
        };

        if (node_id == 0) {
            nodes.push_back(new_node);
        } else {
            nodes[node_id] = new_node;
        }
        
        if (triangles_span.size() <= maxTrianglesPerLeaf) {
            nodes[node_id].payload = triangles_span;
            return;
        }

        assert(!bbox.is_empty());

        auto central_it = split_triangles(triangles_span, bbox);

        if (central_it == triangles_span.end()) { // no good split found
            nodes[node_id].payload = triangles_span;
            return;
        }

        auto left_tris = std::span<MeshBVHNode::triangle>(triangles_span.begin(), central_it);
        auto right_tris = std::span<MeshBVHNode::triangle>(central_it, triangles_span.end());

        uint32_t left_child_id = static_cast<uint32_t>(nodes.size());
        uint32_t right_child_id = left_child_id + 1;
        nodes[node_id].payload = MeshBVHNode::children{ left_child_id , right_child_id };

        nodes.emplace_back();
        nodes.emplace_back();

        parse(left_tris, left_child_id);
        parse(right_tris, right_child_id);

        return;
    }

    std::span<BVH_AS::MeshBVHNode::triangle>::iterator BVH_AS::MeshBVHData::split_triangles(std::span<MeshBVHNode::triangle> triangles_span, const BBox& bbox) {
        float parent_weight = SurfaceAreaHeuristic(bbox, triangles_span.size());

        float best_cost = parent_weight;

        struct bins {
            BBox bbox;
            int n = 0;
        };

        constexpr int N_bins = 32;
        auto bbox_dim = (bbox.max - bbox.min);
        auto bbox_dim_scale = N_bins * 1.0f / bbox_dim;

        int best_axis = get_longest_axis(bbox);
        float best_center;

        for (int axis = 0; axis < 3; ++axis)
        {
            std::array<bins, N_bins> bins_;
            std::for_each(triangles_span.begin(), triangles_span.end(),
                [axis, bbox, bbox_dim_scale, &bins_](const MeshBVHNode::triangle& tri) {
                    float centroid = (tri.p1[axis] + tri.p2[axis] + tri.p3[axis]) / 3.0f;
                    int bin_id = (centroid - bbox.min[axis]) * bbox_dim_scale[axis];
                    bin_id = std::clamp(bin_id, 0, N_bins - 1);
                    ++bins_[bin_id].n;
                    bins_[bin_id].bbox.expand(tri.p1);
                    bins_[bin_id].bbox.expand(tri.p2);
                    bins_[bin_id].bbox.expand(tri.p3);
                });

            std::array<bins, N_bins - 1> left_sweep;
            std::array<bins, N_bins - 1> right_sweep;

            left_sweep[0] = bins_[0];
            for (int i = 1; i < N_bins - 1; ++i) {
                left_sweep[i] = left_sweep[i - 1];
                left_sweep[i].bbox.expand(bins_[i].bbox);
                left_sweep[i].n += bins_[i].n;
            }
            right_sweep[N_bins - 2] = bins_[N_bins - 1];
            for (int i = N_bins - 3; i >= 0; --i) {
                right_sweep[i] = right_sweep[i + 1];
                right_sweep[i].bbox.expand(bins_[i].bbox);
                right_sweep[i].n += bins_[i].n;
            }

            for (int i = 0; i < N_bins - 1; ++i) {
                float SAH = SurfaceAreaHeuristic(left_sweep[i].bbox, right_sweep[i].bbox, left_sweep[i].n, right_sweep[i].n);
                assert(SAH >= 0.0f && !isnan(SAH));
                if (SAH < best_cost) {
                    best_center = bbox.min[axis] + (i + 1) * bbox_dim[axis] * (1.0f / N_bins);
                    best_cost = SAH;
                    best_axis  = axis;
                }
            }
        }

        if (best_cost == parent_weight) { 
            best_axis =  get_longest_axis(bbox); 
            best_center = bbox.min[best_axis] + 0.5f * bbox_dim[best_axis];
        }

        auto central_it = std::partition(
            triangles_span.begin(),
            triangles_span.end(),
            [best_axis, bbox, best_center](const MeshBVHNode::triangle& tri) {
                float centroid = (tri.p1[best_axis] + tri.p2[best_axis] + tri.p3[best_axis]) / 3.0f;
                return centroid < best_center;
            }
        );

        if (best_cost != parent_weight) return central_it;

        if (triangles_span.size() <= maxTrianglesPerLeaf) {
            return triangles_span.end();
        }

        if (central_it == triangles_span.begin() || central_it == triangles_span.end() ||
            central_it == triangles_span.begin() + 1 || central_it + 1 == triangles_span.end()) {
            std::sort(
                triangles_span.begin(),
                triangles_span.end(),
                [best_axis](const MeshBVHNode::triangle& a, const MeshBVHNode::triangle& b) {
                    float centroid_a = (a.p1[best_axis] + a.p2[best_axis] + a.p3[best_axis]);
                    float centroid_b = (b.p1[best_axis] + b.p2[best_axis] + b.p3[best_axis]);
                    return centroid_a < centroid_b;
                }
            );
            size_t mid = triangles_span.size() / 2;
            central_it = triangles_span.begin() + mid;
        }
        return central_it;
    }

    void BVH_AS::MeshBVHData::collect_tree_info_recursive(tree_info& info, uint32_t index, int depth) const {
        const auto& node = nodes[index];
        if (std::holds_alternative<MeshBVHNode::children>(node.payload)) {
            MeshBVHNode::children ch = std::get<MeshBVHNode::children>(node.payload);
            collect_tree_info_recursive(info, ch.left_child_index, depth + 1);
            collect_tree_info_recursive(info, ch.right_child_index, depth + 1);
        }
        else if (std::holds_alternative<MeshBVHNode::triangles>(node.payload)) {
            MeshBVHNode::triangles tris = std::get<MeshBVHNode::triangles>(node.payload);

            info.min_depth = std::min(info.min_depth, depth);
            info.max_depth = std::max(info.max_depth, depth);
            info.mean_depth += depth;
            int num_tris = static_cast<int>(tris.size());
            info.min_tris_in_leaf = std::min(info.min_tris_in_leaf, num_tris);
            info.max_tris_in_leaf = std::max(info.max_tris_in_leaf, num_tris);
            info.mean_tris_in_leaf += num_tris;
            info.total_leaves += 1;
        }
    }

    void BVH_AS::MeshBVHData::collect_tree_info() const
    {
        tree_info info;
        collect_tree_info_recursive(info, 0, 0);
        float mean_depth = static_cast<float>(info.mean_depth) / info.total_leaves;
        float mean_tris = static_cast<float>(info.mean_tris_in_leaf) / info.total_leaves;
        std::cout<<"Mesh BVH info: \n"
                 <<" Depth: min="<<info.min_depth<<", max="<<info.max_depth<<", mean="<<mean_depth<<"\n"
                 <<" Tris per leaf: min="<<info.min_tris_in_leaf<<", max="<<info.max_tris_in_leaf<<", mean="<<mean_tris<<"\n"
                 <<" Total leaves: " << info.total_leaves << "\n"
                 <<" Total tris: " << data_storage.size() << "\n";
        std::cout << std::endl;
    }

    BBox BVH_AS::MeshBVHNode::triangles_to_bbox(const std::span<MeshBVHNode::triangle> tris)
    {
        auto combine = [](BBox a, const BBox& b) {
            a.expand(b);
            return a;
            };
        auto make_bbox = [](const MeshBVHNode::triangle& tris) {
            BBox bbox{ tris.p1, tris.p1 };
            bbox.expand(tris.p2);
            bbox.expand(tris.p3);
            return bbox;
        };
        return std::transform_reduce(std::execution::unseq, tris.begin(), tris.end(), BBox{}, combine, make_bbox);
        // unfortunately no performance benefit using transform_reduce
    }

    std::vector<volume_hit_and_obj_index> thread_local BVH_AS::volume_intersections;
    std::vector<uint32_t> thread_local BVH_AS::bvh_stack;
    
    ray_triangle_hit_info BVH_AS::intersect_ray(const ray& ray, bool any_hit) const {
        ray_triangle_hit_info hit{};

        volume_intersections.clear();

        std::array<fvec2, 7> ray_projections;
        for (int i = 0; i < 7; ++i) {
            // caching ray params for reuse in DOP intersections
            ray_projections[i].x = dot(ray.origin, DOP::axises[i]);
            ray_projections[i].y = 1.0f / dot(ray.direction, DOP::axises[i]);
        }

        for (uint32_t i = 0; i < object_data_.size(); ++i) {
            const auto& obj = object_data_[i];
            ray_volume_hit_info potential_obj_hit = obj.volume.ray_volume_intersection(ray, ray_projections);
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
                ray_triangle_hit_info potential_mesh_hit = (any_hit) ?
                    mesh_ray_intersection<true>(ray, obj.ModelMatrix, obj.invModelMatrix, mesh_bvh_data_[obj.meshIndex]) :
                    mesh_ray_intersection<false>(ray, obj.ModelMatrix, obj.invModelMatrix, mesh_bvh_data_[obj.meshIndex]);
                if (potential_mesh_hit.forward_hit() && (potential_mesh_hit.distance < hit.distance)) {
                    hit = potential_mesh_hit;
                    hit.objectIndex = obj.objectIndex;
                    hit.meshIndex = obj.meshIndex;
                    if (any_hit) {
                        break;
                    }
                }
            }
            else if (!any_hit) {
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
    static ray_triangle_hit_info BVH_AS::mesh_ray_intersection(
        const ray& ray_ws,
        const fmat4x4& ModelMatrix,
        const fmat4x4& invModelMatrix,
        const MeshBVHData& mesh) noexcept
    {
        // Transform ray to object space
        auto ray_origin_os = xyz(invModelMatrix * xyz1(ray_ws.origin));
        auto ray_direction_os = normalize(xyz(invModelMatrix * xyz0(ray_ws.direction)));
        fvec3 inv_dir = 1.0f / ray_direction_os;

        ray os_ray{
            ray_origin_os,
            ray_direction_os
        };

        ray_triangle_hit_info hit{};

        bvh_stack.clear();
        bvh_stack.push_back(0); // root node index
        while (!bvh_stack.empty())
        {
            uint32_t curr_index = bvh_stack.back();
            bvh_stack.pop_back();
            const auto& node = mesh.nodes[curr_index];
            if (std::holds_alternative<MeshBVHNode::triangles>(node.payload)) {
                MeshBVHNode::triangles tris = std::get<MeshBVHNode::triangles>(node.payload);
                // Test triangles
                for (auto it = tris.begin(); it != tris.end(); it++) {
                    barycentric_coords tri_hit = intersect_ray_triangle(os_ray, it->p1, it->p2, it->p3, !mesh.doubleSided);
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
                        .triangleIndex = it->index
                    };
                    if constexpr (any_hit) {
                        break;
                    }
                }
            } else {
                MeshBVHNode::children children = std::get<MeshBVHNode::children>(node.payload);
                const auto& l_node = mesh.nodes[children.left_child_index];
                const auto& r_node = mesh.nodes[children.right_child_index];
                assert(children.left_child_index != -1 && children.right_child_index != -1);

                ray_volume_hit_info volume_hit_l = l_node.volume.ray_volume_intersection(os_ray, inv_dir);
                ray_volume_hit_info volume_hit_r = r_node.volume.ray_volume_intersection(os_ray, inv_dir);

                auto distance_l = volume_hit_l.forward_hit_distance();
                auto distance_r = volume_hit_r.forward_hit_distance();

                if (distance_l < volume_hit_r.forward_hit_distance()) {
                    if (volume_hit_r.hit && hit.b_coords.t > distance_r) bvh_stack.push_back(children.right_child_index);
                    if (volume_hit_l.hit && hit.b_coords.t > distance_l) bvh_stack.push_back(children.left_child_index);
                } else {
                    if (volume_hit_l.hit && hit.b_coords.t > distance_l) bvh_stack.push_back(children.left_child_index);
                    if (volume_hit_r.hit && hit.b_coords.t > distance_r) bvh_stack.push_back(children.right_child_index);
                }
            }
            if constexpr (any_hit) {
                if (hit.hit) break;
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

    BBox BVH_AS::get_scene_bounds() const {
        DOP dop;
        for (const auto& obj : object_data_) {
            dop.expand(obj.volume);
        }
        return dop.to_bbox();
    }

    // BVH_AS
}