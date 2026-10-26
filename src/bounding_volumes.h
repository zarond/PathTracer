#pragma once

#include <array>
#include <limits>
#include <glm/glm.hpp>
#include <numbers>

namespace app {

using glm::fmat4x4;
using glm::fvec2;
using glm::fvec3;
using glm::fvec4;

struct ray {
    fvec3 origin;
    fvec3 direction;  // should always be normalized
};

struct ray_volume_hit_info {
    bool hit = false;  // Todo: optimize layout, use infinity as no-hit indicator?
    float t0 = 0.0f;   // distance along the ray
    float t1 = 0.0f;   // distance along the ray
    constexpr bool forward_hit() const noexcept { return hit && (t1 > 0.0f); }
    constexpr float forward_hit_distance() const noexcept {
        return forward_hit() ? (t0 > 0.0f ? t0 : 0.0f) : std::numeric_limits<float>::infinity();
    }
};

struct BBox {
    fvec3 min, max;

    inline BBox() noexcept : min(std::numeric_limits<float>::max()), max(std::numeric_limits<float>::lowest()) {}
    constexpr BBox(const fvec3& min_, const fvec3& max_) noexcept : min(min_), max(max_) {}

    bool is_empty() const noexcept;
    void expand(const fvec3& ws_point) noexcept;
    void expand(const BBox& bbox) noexcept;

    ray_volume_hit_info ray_volume_intersection(const ray& ray) const noexcept;
    ray_volume_hit_info ray_volume_intersection(const ray& ray, fvec3 ray_direction_inv) const noexcept;

    float surface_area() const noexcept;
    int get_longest_axis() const noexcept;
};

struct DOP {
    DOP() noexcept;
    DOP(const fvec3& point) noexcept;

    bool is_empty() const noexcept;
    void expand(const fvec3& ws_point) noexcept;
    void expand(const DOP& dop) noexcept;
    BBox to_bbox() const noexcept;

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

}  // namespace app
