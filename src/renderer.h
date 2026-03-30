#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <atomic>
#include <functional>

#include "acceleration_structure.h"
#include "arguments.h"
#include "cpu_framebuffer.h"
#include "ray_program.h"
#include "render_settings.h"

namespace app {

template <typename T>
struct SamplesAccumulator {
    T mean;
    T M2;         // sum of squares of differences from the current mean
    float count;  // logically int, should be fine for count < 2^24 = 16 million samples

    SamplesAccumulator() : mean(0), M2(0), count(0) {}

    // Welford's Online Algorithm
    void add_sample(const T& sample) noexcept {
        count += 1.0f;
        T delta = sample - mean;
        mean += delta / count;
        T delta2 = sample - mean;
        M2 += delta * delta2;
    }
    T get_mean() const { return mean; }
    T get_variance() const { return (count > 1.0f) ? M2 / (count - 1.0f) : T(0); }
    T get_stddev() const { return sqrt(get_variance()); }
    T get_stddev(T variance) const { return sqrt(variance); }
};

class IRenderer {
  public:
    virtual ~IRenderer() = default;

    enum RenderingState { 
        Idle, 
        ReadyToStart,
        Rendering, 
        Cancelling
    };

    virtual void update_camera_transform_state(
        fvec3 position, fvec3 direction, fvec3 up, fastgltf::Camera::Perspective perspectiveParams) = 0;

    virtual void load_scene(const Model& model, const CPUTexture<hdr_pixel>& envmap) = 0;  // should be in constructor?
    virtual void load_envmap(const CPUTexture<hdr_pixel>& envmap) = 0;
    virtual void load_model(const Model& model) = 0;
    virtual void reload_ray_program() = 0;
    virtual void reload_acceleration_structure() = 0;

    virtual void render_frame(CPUFrameBuffer& framebuffer, bool continuous, bool iterative, int iteration_count) = 0;
    virtual void set_render_settings(const RenderSettings& settings) = 0;
    virtual RenderSettings get_render_settings() const = 0;
    virtual BBox get_scene_bound() const = 0;

    virtual float get_progress() const = 0;
    virtual void cancel_rendering() = 0;
    virtual RenderingState get_rendering_state() const = 0;
    virtual void set_render_starting_state() = 0;
};

class Renderer : public IRenderer {
  public:
    Renderer() = default;
    ~Renderer() = default;
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    using IRenderer::RenderingState;

    void update_camera_transform_state(
        fvec3 position, fvec3 direction, fvec3 up, fastgltf::Camera::Perspective perspectiveParams);

    void load_scene(const Model& model, const CPUTexture<hdr_pixel>& envmap);  // should be in constructor?
    void load_envmap(const CPUTexture<hdr_pixel>& envmap);
    void load_model(const Model& model);
    void reload_ray_program();
    void reload_acceleration_structure();

    void render_frame(CPUFrameBuffer& framebuffer, bool continuous, bool iterative, int iteration_count);
    void set_render_settings(const RenderSettings& settings);
    RenderSettings get_render_settings() const;
    BBox get_scene_bound() const;

    float get_progress() const;
    void cancel_rendering();
    RenderingState get_rendering_state() const;
    void set_render_starting_state();

  protected:
    std::unique_ptr<IAccelerationStructure> accelStruct;
    std::unique_ptr<IRayProgram> rayProgram;
    const Model* modelRef = nullptr;
    const CPUTexture<hdr_pixel>* envmapRef = nullptr;
    RenderSettings renderSettings_;

    fmat4x4 viewMatrix_ = fmat4x4(1.0f);
    fmat4x4 projectionMatrix_ = fmat4x4(1.0f);
    fmat4x4 NDC2WorldMatrix_ = fmat4x4(1.0f);
    fvec3 origin_ = fvec3{0.0f};

    float progress_ = 0.0f;
    std::atomic<RenderingState> render_state_ = Idle;

  private:
    ray_with_payload generate_camera_ray(
        int x, int y, float inv_width, float inv_height, int sampleIndex = 0, fvec2 jitter = {0.0f, 0.0f}) const noexcept;
    void generate_subsample_positions();

    std::vector<fvec2> subsamplesPositions;

    size_t lastModelFenceValue = 0; // ??
    size_t lastEnvmapFenceValue = 0;
};

}  // namespace app
