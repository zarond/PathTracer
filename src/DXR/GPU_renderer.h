#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <fastgltf/types.hpp>

#include "../render_settings.h"
#include "../model_loader.h"

namespace app {

using namespace glm;

class GPU_renderer {
  public:
    GPU_renderer();
    ~GPU_renderer();

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

    enum RenderingState { Idle, ReadyToStart, Rendering, Cancelling };

    float get_progress() const;
    void cancel_rendering();
    RenderingState get_rendering_state() const;
    void set_render_starting_state();

  private:

    std::unique_ptr<GPU_model> gpu_model_;
    std::unique_ptr<RayProgram> ray_program_;

    // Camera state
    fmat4x4 viewMatrix_ = fmat4x4(1.0f);
    fmat4x4 projectionMatrix_ = fmat4x4(1.0f);
    fmat4x4 NDC2WorldMatrix_ = fmat4x4(1.0f);
    fvec3 origin_ = fvec3{0.0f};

    float progress_ = 0.0f;
    std::atomic<RenderingState> render_state_ = Idle;

    void release_gpu_resource();
};

}