#pragma once

#include <type_traits>
#include <concepts>

#include <fastgltf/types.hpp>

namespace app {
    using namespace fastgltf::math;

template<typename T>
concept TriviallyCopyableStruct = std::is_trivially_copyable_v<T> && std::is_standard_layout_v<T>;

struct ray {
    fvec3 origin;
    fvec3 direction; // should always be normalized
};

struct barycentric_coords {
    float A = 0.0f;
    float B = 0.0f;
    // implicit C = 1.0 - A - B
    bool hit = false;

    float C() const noexcept {
        return 1.0f - A - B;
    }
};

struct ray_bbox_hit_info {
    bool hit = false; // Todo: optimize layout, use infinity as no-hit indicator?
    float t0 = 0.0f; // distance along the ray
    float t1 = 0.0f; // distance along the ray
    constexpr bool forward_hit() const noexcept {
        return hit && (t1 > 0.0f);
    }
    constexpr float forward_hit_distance() const noexcept {
        return forward_hit() ? (t0 > 0.0f ? t0 : t1) : std::numeric_limits<float>::infinity();
    }
};

struct ray_triangle_hit_info {
    bool hit = false;
    float distance = std::numeric_limits<float>::infinity(); // distance along the ray
    barycentric_coords b_coords = {};
    std::uint32_t triangleIndex = 0; // index of the first point of the triangle in vector of indices in mesh
    
    constexpr bool forward_hit() const noexcept {
        return hit && (distance > 0.0f);
    }
};

class ray_generator {};

class ray_caster : public ray_generator {};

class ray_ao : public ray_generator {};

class path_tracer : public ray_generator {};

}