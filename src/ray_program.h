#pragma once

#include <type_traits>
#include <concepts>
#include <random>

#include "model_loader.h"
#include "cpu_framebuffer.h"
#include "render_settings.h"

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
    fvec3 payload = fvec3(1.0f); // accumulated color
    std::uint8_t depth = 0; // current recursion depth left
    bool any_hit = false;
    // unsigned short sample_index = 0; // todo: add for quasi-random sampling
    // bool inside_volume = false; // todo: ior of current medium instead?
};

struct barycentric_coords {
    float A = 0.0f;
    float B = 0.0f;
    // implicit C = 1.0 - A - B
    float t = std::numeric_limits<float>::infinity();
    bool hit = false;
    bool backface = false;

    float C() const noexcept {
        return 1.0f - A - B;
    }
};

struct ray_volume_hit_info {
    bool hit = false; // Todo: optimize layout, use infinity as no-hit indicator?
    float t0 = 0.0f; // distance along the ray
    float t1 = 0.0f; // distance along the ray
    constexpr bool forward_hit() const noexcept {
        return hit && (t1 > 0.0f);
    }
    constexpr float forward_hit_distance() const noexcept {
        return forward_hit() ? (t0 > 0.0f ? t0 : 0.0f) : std::numeric_limits<float>::infinity();
    }
};

struct ray_triangle_hit_info : public barycentric_coords {
    uint32_t objectIndex = 0; // index of the object hit
    uint32_t meshIndex = 0; // index of the mesh hit, redundant info, but might save one indirect memory access
    uint32_t triangleIndex = 0; // index of the first vertex of the triangle in vector of indices in mesh
    
    constexpr bool forward_hit() const noexcept {
        return hit && (t > 0.0f);
    }
};

class IRayProgram {
public:
    virtual ~IRayProgram() = default;
    virtual fvec3 on_hit(const ray_with_payload& r, const ray_triangle_hit_info& hitInfo, std::vector<ray_with_payload>& ray_collection) const = 0;
};

class RayCasterProgram : public IRayProgram {
public:
    RayCasterProgram(const Model& model, const CPUTexture<hdr_pixel>& env, const RenderSettings& settings);
    virtual ~RayCasterProgram() = default;

    virtual fvec3 on_hit(const ray_with_payload& r, const ray_triangle_hit_info& hitInfo, std::vector<ray_with_payload>& ray_collection) const override;
private:
    const Model& modelRef;
    const CPUTexture<hdr_pixel>& envmapRef;
    float envmap_rot;
};

class AOProgram : public IRayProgram {
public:
    AOProgram(const Model& model, const RenderSettings& settings);
    virtual ~AOProgram() = default;

    virtual fvec3 on_hit(const ray_with_payload& r, const ray_triangle_hit_info& hitInfo, std::vector<ray_with_payload>& ray_collection) const override;
private:
    const Model& modelRef;
    const unsigned int aoSamples = 32;
    const float inv_aoSamples;

    static thread_local std::minstd_rand gen;
    static thread_local std::uniform_real_distribution<float> dist;
};

class PBRProgram : public IRayProgram {
public:
    PBRProgram(const Model& model, const CPUTexture<hdr_pixel>& env, const RenderSettings& settings);
    virtual ~PBRProgram() = default;

    virtual fvec3 on_hit(const ray_with_payload& r, const ray_triangle_hit_info& hitInfo, std::vector<ray_with_payload>& ray_collection) const override;
private:
    const Model& modelRef;
    const CPUTexture<hdr_pixel>& envmapRef;
    float envmap_rot;

    static thread_local std::minstd_rand gen;
    static thread_local std::uniform_real_distribution<float> dist;
    // Todo: add quasi-random mode
};

}