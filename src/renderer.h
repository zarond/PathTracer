#pragma once

#include <memory>

#include "ray_generator.h"
#include "acceleration_structure.h"
#include "cpu_framebuffer.h"

namespace app {

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

    ~Renderer() = default;

protected:
    std::unique_ptr<IAccelerationStructure> accelStruct;
    const Model* modelRef = nullptr;
    int samplesPerPixel = 1;
    bool distributeSamples = false; // uniformly distribute samples across pixel

    fmat4x4 viewMatrix_ = fmat4x4(1.0f);
    fmat4x4 projectionMatrix_ = fmat4x4(1.0f);
    fmat4x4 NDC2WorldMatrix_ = fmat4x4(1.0f);
    fvec3 origin_;

public:
    ray generate_camera_ray(int x, int y, int width, int height, int sampleIndex = 0) const;
};

}