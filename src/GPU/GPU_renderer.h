#pragma once

#include <atomic>
#include <fastgltf/types.hpp>
#include <glm/fwd.hpp>
#include <memory>
#include <vector>

#include "../acceleration_structure.h"
#include "../cpu_framebuffer.h"
#include "../model_loader.h"
#include "../render_settings.h"
#include "../renderer.h"
#include "GPU_model.h"
#include "DXR_pipeline.h"

namespace app {

using glm::fvec2;
using glm::fvec3;
using glm::fvec4;

enum class RenderPipelineMode : int {
    DXRPipeline = 0,
    RasterPipeline = 1,

    Count = 2,
};

class GPURenderer : public IRenderer {
  public:
    GPURenderer();
    ~GPURenderer() = default;
    GPURenderer(const GPURenderer&) = delete;
    GPURenderer& operator=(const GPURenderer&) = delete;

    void update_camera_transform_state(
        fvec3 position, fvec3 direction, fvec3 up, fastgltf::Camera::Perspective perspectiveParams);

    void load_scene(const Model& model, const CPUTexture<hdr_pixel>& envmap, size_t ModelFenceValue,
        size_t EnvmapFenceValue);  // should be in constructor?
    void load_envmap(const CPUTexture<hdr_pixel>& envmap, size_t EnvmapFenceValue);
    void load_model(const Model& model, size_t ModelFenceValue);
    void reload_ray_program();
    void reload_acceleration_structure();
    void reload_materials();

    void render_frame(CPUFrameBuffer& framebuffer, bool continuous, bool iterative, int iteration_count);
    void set_render_settings(const RenderSettings& settings);
    RenderSettings get_render_settings() const;
    BBox get_scene_bound() const;

    float get_progress() const;
    void cancel_rendering();
    RenderingState get_rendering_state() const;
    void set_render_starting_state();

    RenderPipelineMode get_active_pipeline_mode() const;
    void set_active_pipeline_mode(RenderPipelineMode mode);

  private:
    const Model* model_ref_ = nullptr;
    const CPUTexture<hdr_pixel>* envmap_ref_ = nullptr;
    std::unique_ptr<GPU_model> gpu_model_;
    std::unique_ptr<GPU_texture> gpu_envmap_;
    RenderSettings render_settings_;

    fmat4x4 viewMatrix_ = fmat4x4(1.0f);
    fmat4x4 projectionMatrix_ = fmat4x4(1.0f);
    fmat4x4 NDC2WorldMatrix_ = fmat4x4(1.0f);  // without translation component
    fvec3 origin_ = fvec3{0.0f};

    float progress_ = 0.0f;
    std::atomic<RenderingState> render_state_ = RenderingState::Idle;

    std::shared_ptr<IRender_pipeline> pipeline_;                // currently chosen render pipeline
    std::vector<std::shared_ptr<IRender_pipeline>> pipelines_;  // initialize all render pipelines
    RenderPipelineMode active_pipeline_mode_;

    size_t last_model_fence_value_ = 0;
    size_t last_envmap_fence_value_ = 0;

    unsigned int frameID_ = 0;
};

}  // namespace app