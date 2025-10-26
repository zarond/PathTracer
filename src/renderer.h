#pragma once

#include <memory>

#include "ray_generator.h"
#include "acceleration_structure.h"
#include "cpu_framebuffer.h"

namespace app {

template<typename T>
struct SamplesAccumulator {
    T mean;
    T M2; // sum of squares of differences from the current mean
    float count; // logically int, should be fine for count < 2^24 = 16 million samples
    
    SamplesAccumulator() : mean(0), M2(0), count(0) {}

    // Welford's Online Algorithm
    void add_sample(const T& sample) {
        count += 1.0f;
        T delta = sample - mean;
        mean += delta / count;
        T delta2 = sample - mean;
        M2 += delta * delta2;
    }
    T get_mean() const {
        return mean;
    }
    T get_variance() const {
        return (count > 1.0f) ? M2 / (count - 1.0f) : T(0);
    }
    T get_stddev() const {
        return sqrt(get_variance());
    }
    T get_stddev(T variance) const {
        return sqrt(variance);
    }
};

struct RenderSettings {
    unsigned int samplesPerPixel = 1;
    unsigned int maxRayBounces = 1;
    unsigned int maxNewRaysPerBounce = 0; // Todo: unused for now
};

class Renderer {
public:
    Renderer() = default;
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    void update_camera_transform_state(
        fvec3 position,
        fvec3 direction,
        fvec3 up,
        fastgltf::Camera::Perspective perspectiveParams
    );

    void load_scene(const Model& model); // should be in constructor?
    void render_frame(CPUFrameBuffer& framebuffer);
    void set_render_settings(const RenderSettings& settings);
    RenderSettings get_render_settings() const;

    ~Renderer() = default;

protected:
    std::unique_ptr<IAccelerationStructure> accelStruct;
    const Model* modelRef = nullptr;
    RenderSettings renderSettings_;

    fmat4x4 viewMatrix_ = fmat4x4(1.0f);
    fmat4x4 projectionMatrix_ = fmat4x4(1.0f);
    fmat4x4 NDC2WorldMatrix_ = fmat4x4(1.0f);
    fvec3 origin_;

private:
    ray generate_camera_ray(int x, int y, int width, int height, int sampleIndex = 0) const;
    void generate_subsample_positions();

    std::vector<fvec2> subsamplesPositions;
};

}