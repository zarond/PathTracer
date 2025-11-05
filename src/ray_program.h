#pragma once

#include <type_traits>
#include <concepts>
#include <random>

#include "model_loader.h"
#include "cpu_framebuffer.h"

#include <glm/glm.hpp>

namespace app {
    using namespace glm;

template<typename T>
concept TriviallyCopyableStruct = std::is_trivially_copyable_v<T> && std::is_standard_layout_v<T>;

struct ray {
    fvec3 origin;
    fvec3 direction; // should always be normalized
};

struct ray_with_payload : ray {
    fvec4 payload = fvec4(1.0f); // accumulated color or other data
    std::uint8_t depth = 0; // current recursion depth left
    bool any_hit = false;
};

struct barycentric_coords {
    float A = 0.0f;
    float B = 0.0f;
    // implicit C = 1.0 - A - B
    float t = std::numeric_limits<float>::infinity();
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
    uint32_t objectIndex = 0; // index of the object hit
    uint32_t meshIndex = 0; // index of the mesh hit
    uint32_t triangleIndex = 0; // index of the first vertex of the triangle in vector of indices in mesh
    
    constexpr bool forward_hit() const noexcept {
        return hit && (distance > 0.0f);
    }
};

class IRayProgram {
public:
    virtual fvec4 on_hit(const ray_with_payload& r, const ray_triangle_hit_info& hitInfo, std::vector<ray_with_payload>& ray_collection) const = 0;
};

class RayCasterProgram : public IRayProgram {
public:
    RayCasterProgram(const Model& model, const CPUTexture<hdr_pixel>& env);

    virtual fvec4 on_hit(const ray_with_payload& r, const ray_triangle_hit_info& hitInfo, std::vector<ray_with_payload>& ray_collection) const override;
private:
    const Model& modelRef;
    const CPUTexture<hdr_pixel>& envmapRef;
};

class AOProgram : public IRayProgram {
public:
    AOProgram(const Model& model, const CPUTexture<hdr_pixel>& env, const unsigned int ao_samples);

    virtual fvec4 on_hit(const ray_with_payload& r, const ray_triangle_hit_info& hitInfo, std::vector<ray_with_payload>& ray_collection) const override;
private:
    const Model& modelRef;
    const CPUTexture<hdr_pixel>& envmapRef;
    const unsigned int aoSamples = 32;

    static thread_local std::minstd_rand gen;
    static thread_local std::uniform_real_distribution<float> dist;
};

}