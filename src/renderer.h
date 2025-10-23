#pragma once

#include <memory>

#include "ray_generator.h"
#include "acceleration_structure.h"
#include "cpu_framebuffer.h"

namespace app {

class Renderer {
protected:
    std::unique_ptr<ray_generator> rayGen;
    //std::unique_ptr<AccelerationStructure> accelStruct; // Todo
    std::unique_ptr<NaiveAS> accelStruct; // Todo
    int samplesPerPixel = 1;
    bool distributeSamples = false; // uniformly distribute samples across pixel

public:
    virtual void load_scene(const Model& model) = 0;
    virtual void render_frame(CPUFrameBuffer& framebuffer) = 0;
    virtual void update_camera_transform_state(
        fvec3 position, 
        fvec3 direction, 
        fvec3 up,
        fastgltf::Camera::Perspective perspectiveParams
    ) = 0;

    virtual ~Renderer() = default;
};

class PerspectiveCameraRenderer : public Renderer {
private:
    fmat4x4 viewMatrix_ = fmat4x4(1.0f);
    fmat4x4 projectionMatrix_ = fmat4x4(1.0f);

    fmat4x4 NDC2WorldMatrix_ = fmat4x4(1.0f);

    fvec3 origin_;

public:
    virtual void load_scene(const Model& model);
    virtual void render_frame(CPUFrameBuffer& framebuffer);
    virtual void update_camera_transform_state(
        fvec3 position,
        fvec3 direction,
        fvec3 up,
        fastgltf::Camera::Perspective perspectiveParams
    );
};

}