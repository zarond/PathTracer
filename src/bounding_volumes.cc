#include "bounding_volumes.h"

#include <limits>
#include <algorithm>

namespace app {

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
    if (t_max < t_min) return {false};

    t0 = (min.z - ray.origin.z) * ray_direction_inv.z;
    t1 = (max.z - ray.origin.z) * ray_direction_inv.z;
    if (t0 > t1) std::swap(t0, t1);
    t_min = std::max(t0, t_min);
    t_max = std::min(t1, t_max);
    if (t_max < t_min) return {false};

    return {true, t_min, t_max};
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

    return {(t0.x <= t1.x), t0.x, t1.x};
}

float BBox::surface_area() const noexcept {
    auto dim = max - min;
    return dim.x * dim.y + dim.x * dim.z + dim.y * dim.z;  // leave *2.0f
}

int BBox::get_longest_axis() const noexcept {
    fvec3 extents = max - min;
    if (extents.x >= extents.y && extents.x >= extents.z) {
        return 0;
    } else if (extents.y >= extents.x && extents.y >= extents.z) {
        return 1;
    }
    return 2;
}

DOP::DOP() noexcept { min_max.fill(fvec2{std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()}); }
DOP::DOP(const fvec3& point) noexcept {
    for (int i = 0; i < 7; ++i) {
        float projection = dot(point, axises[i]);
        min_max[i] = {projection, projection};
    }
}

bool DOP::is_empty() const noexcept {
    return std::any_of(min_max.cbegin(), min_max.cend(), [](const auto& x) { return x.y < x.x; });  // max < min
}
void DOP::expand(const fvec3& ws_point) noexcept {
    for (int i = 0; i < 7; ++i) {
        float projection = dot(ws_point, axises[i]);
        auto& axis_min_max = min_max[i];
        axis_min_max.x = std::min(axis_min_max.x, projection);
        axis_min_max.y = std::max(axis_min_max.y, projection);
    }
}
void DOP::expand(const DOP& dop) noexcept {
    for (int i = 0; i < 7; ++i) {
        auto& axis_min_max = min_max[i];
        axis_min_max.x = std::min(axis_min_max.x, dop.min_max[i].x);
        axis_min_max.y = std::max(axis_min_max.y, dop.min_max[i].y);
    }
}
BBox DOP::to_bbox() const noexcept {
    return BBox{fvec3{min_max[0].x, min_max[1].x, min_max[2].x}, fvec3{min_max[0].y, min_max[1].y, min_max[2].y}};
}

ray_volume_hit_info DOP::ray_volume_intersection(const ray& ray) const noexcept {
    float t_min = std::numeric_limits<float>::lowest();
    float t_max = std::numeric_limits<float>::max();

    for (int i = 0; i < 7; ++i) {
        float o_projection = dot(ray.origin, axises[i]);
        float inv_d_projection = 1.0f / dot(ray.direction, axises[i]);
        const auto& axis_min_max = min_max[i];
        float t0 = (axis_min_max.x - o_projection) * inv_d_projection;
        float t1 = (axis_min_max.y - o_projection) * inv_d_projection;
        if (t0 > t1) std::swap(t0, t1);
        t_min = std::max(t0, t_min);
        t_max = std::min(t1, t_max);
        if (t_max < t_min) return {false};
    }
    return {true, t_min, t_max};
}
// optimised for ray reuse
ray_volume_hit_info DOP::ray_volume_intersection(const std::array<fvec2, 7>& projections) const noexcept {
    float t_min = std::numeric_limits<float>::lowest();
    float t_max = std::numeric_limits<float>::max();

    for (int i = 0; i < 7; ++i) {
        const auto& axis_min_max = min_max[i];
        float t0 = (axis_min_max.x - projections[i].x) * projections[i].y;
        float t1 = (axis_min_max.y - projections[i].x) * projections[i].y;
        if (t0 > t1) std::swap(t0, t1);
        t_min = std::max(t0, t_min);
        t_max = std::min(t1, t_max);
        if (t_max < t_min) return {false};
    }
    return {true, t_min, t_max};
}

}  // namespace app
