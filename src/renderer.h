#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>

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
    void add_sample(const T& sample) {
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

class Renderer {
  public:
    Renderer() = default;
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    void update_camera_transform_state(
        fvec3 position, fvec3 direction, fvec3 up, fastgltf::Camera::Perspective perspectiveParams);

    void load_scene(const Model& model, const CPUTexture<hdr_pixel>& envmap);  // should be in constructor?
    void render_frame(CPUFrameBuffer& framebuffer);
    void set_render_settings(const RenderSettings& settings);
    RenderSettings get_render_settings() const;
    BBox get_scene_bound() const;

    ~Renderer() = default;

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

  private:
    ray_with_payload generate_camera_ray(int x, int y, float inv_width, float inv_height, int sampleIndex = 0) const;
    void generate_subsample_positions();

    std::vector<fvec2> subsamplesPositions;
};

}  // namespace app
