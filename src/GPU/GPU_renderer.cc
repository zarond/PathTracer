#define NOMINMAX
#include "GPU_renderer.h"

#include <directx/d3d12.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <fastgltf/types.hpp>
#include <glm/ext.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>

#include "../d3d_context.h"
#include "Raster_pipeline.h"

namespace app {

using namespace glm;

GPURenderer::GPURenderer() {
    pipelines_.resize((int)RenderPipelineMode::Count);

    pipelines_[(int)RenderPipelineMode::RasterPipeline] = std::make_shared<Raster_pipeline>();
    pipeline_ = pipelines_[(int)RenderPipelineMode::RasterPipeline];
    active_pipeline_mode_ = RenderPipelineMode::RasterPipeline;

    D3DContext& d3d_ctx = D3DContext::Get();
    if (d3d_ctx.hardware_ray_tracing_support) {
        pipelines_[(int)RenderPipelineMode::DXRPipeline] = std::make_shared<DXR_pipeline>();
        pipeline_ = pipelines_[(int)RenderPipelineMode::DXRPipeline];
        active_pipeline_mode_ = RenderPipelineMode::DXRPipeline;
    }
}

void GPURenderer::update_camera_transform_state(
    fvec3 position, fvec3 direction, fvec3 up, fastgltf::Camera::Perspective perspectiveParams) {
    origin_ = position;
    viewMatrix_ = glm::lookAt(position, position + direction, up);
    projectionMatrix_ = glm::perspectiveRH(
        perspectiveParams.yfov, 
        perspectiveParams.aspectRatio.value_or(1.77777777777777777f),
        perspectiveParams.znear, 
        perspectiveParams.zfar.value_or(1000.f));
    auto viewMatrixNoTranslation = viewMatrix_;
    viewMatrixNoTranslation[3] = fvec4(0.0f, 0.0f, 0.0f, 1.0f);  // Remove translation from view matrix for direction calculation
    NDC2WorldMatrix_ = glm::inverse(projectionMatrix_ * viewMatrixNoTranslation);
}

void GPURenderer::load_scene(
    const Model& model, const CPUTexture<hdr_pixel>& envmap, size_t ModelFenceValue, size_t EnvmapFenceValue) {
    if (ModelFenceValue == last_model_fence_value_ && EnvmapFenceValue == last_envmap_fence_value_) return;

    D3DContext& d3d_ctx = D3DContext::Get();
    d3d_ctx.WaitForPendingCopy();

    model_ref_ = &model;
    envmap_ref_ = &envmap;
    reload_acceleration_structure();
    reload_ray_program();

    last_model_fence_value_ = ModelFenceValue;
    last_envmap_fence_value_ = EnvmapFenceValue;
}

void GPURenderer::load_envmap(const CPUTexture<hdr_pixel>& envmap, size_t EnvmapFenceValue) {
    if (EnvmapFenceValue == last_envmap_fence_value_) return;
    envmap_ref_ = &envmap;
    reload_ray_program();
    last_envmap_fence_value_ = EnvmapFenceValue;
}

void GPURenderer::load_model(const Model& model, size_t ModelFenceValue) {
    if (ModelFenceValue == last_model_fence_value_) return;
    model_ref_ = &model;
    reload_acceleration_structure();
    last_model_fence_value_ = ModelFenceValue;
}

void GPURenderer::reload_ray_program() {
    if (envmap_ref_ == nullptr) return;
    gpu_envmap_ = std::make_unique<GPU_texture>(*envmap_ref_);

    OnEnvmapLoad();
}
void GPURenderer::reload_acceleration_structure() {
    if (model_ref_ == nullptr) return;
    auto start = std::chrono::high_resolution_clock::now();

    bool raytracing_support = D3DContext::Get().hardware_ray_tracing_support;
    gpu_model_ = std::make_unique<GPU_model>(*model_ref_, raytracing_support);

    OnModelLoad();

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "GPU DXR model Acceleration Structure created in " << diff.count() << " ms." << '\n';
}

void GPURenderer::reload_materials() {
    if (model_ref_ == nullptr || gpu_model_ == nullptr) return;
    D3DContext& d3d_ctx = D3DContext::Get();
    gpu_model_->update_materials_array_buffer(*model_ref_);
}

void GPURenderer::render_frame(CPUFrameBuffer& framebuffer, bool continuous, bool iterative, int iteration_count) {
    assert(model_ref_);
    if (model_ref_ == nullptr || envmap_ref_ == nullptr || gpu_model_ == nullptr) {
        throw std::runtime_error("One of components is nullptr in GPURenderer::render_frame()");
    }
    if (gpu_model_->isEmpty()) {
        std::cout << "Model has no vertices, skipping rendering" << '\n';
        render_state_ = RenderingState::Idle;
        return;
    }
    progress_ = 0.0f;
    render_state_ = RenderingState::Rendering;

    glm::fvec2 jitter = glm::fvec2{0.0f};
    float inverse_iteration_count = 1.0f;
    if (iterative) {
        static std::minstd_rand gen = std::minstd_rand(std::random_device{}());
        static std::uniform_real_distribution<float> dist{-0.5f, 0.5f};
        jitter = fvec2{dist(gen), dist(gen)};
        float n_sqrt = sqrtf(render_settings_.samplesPerPixel);
        jitter /= n_sqrt;
        inverse_iteration_count = 1.0f / static_cast<float>(iteration_count);
    }

    pipeline_->SetRenderingSettings(render_settings_, origin_, NDC2WorldMatrix_, viewMatrix_, projectionMatrix_, jitter, 
        ++frameID_, iteration_count, inverse_iteration_count);

    D3DContext& d3d_ctx = D3DContext::Get();
    d3d_ctx.InitDXRCommandList();

    ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()};
    d3d_ctx.m_DXRCommandList->SetDescriptorHeaps(1, desc_heap);

    pipeline_->DoRender(*gpu_model_, *gpu_envmap_, framebuffer);

    d3d_ctx.DispatchDXRCommandList();
    d3d_ctx.WaitForPendingDXR();

    if (render_state_ == RenderingState::Rendering) {
        progress_ = 1.0f;
    }
    if (continuous && (render_state_ != RenderingState::Cancelling)) {
        render_state_ = RenderingState::ReadyToStart;
    } else {
        render_state_ = RenderingState::Idle;
    }
}

void GPURenderer::set_render_settings(const RenderSettings& settings) {
    const auto currentSettings = render_settings_;
    render_settings_ = settings;

    if (currentSettings.programMode != settings.programMode) {
        reload_ray_program();
    }
}
RenderSettings GPURenderer::get_render_settings() const { return render_settings_; }

BBox GPURenderer::get_scene_bound() const {
    // Todo: compute scene bound from model
    // can't get from TLAS of GPU_model
    return BBox{};
}

float GPURenderer::get_progress() const { return progress_; }

void GPURenderer::cancel_rendering() {
    if (render_state_ == RenderingState::Rendering) {
        render_state_ = RenderingState::Cancelling;
    }
}

RenderingState GPURenderer::get_rendering_state() const { return render_state_; }

void GPURenderer::set_render_starting_state() {
    if (render_state_ == RenderingState::Idle) {
        render_state_ = RenderingState::ReadyToStart;
    }
}

RenderPipelineMode GPURenderer::get_active_pipeline_mode() const { return active_pipeline_mode_; }

void GPURenderer::set_active_pipeline_mode(RenderPipelineMode mode) {
    if ((int)mode < 0 || mode >= RenderPipelineMode::Count) {
        throw std::runtime_error("Invalid Pipeline Mode.");
    }
    if (!pipelines_[(int)mode]) {
        throw std::runtime_error("Pipeline not initialized");
    }
    pipeline_ = pipelines_[(int)mode];
}

void GPURenderer::OnEnvmapLoad() {
    // Keep state of pipelines in sync for simpler logic
    for (auto& pipeline : pipelines_) {
        if (pipeline) {
            pipeline->OnEnvmapLoad(*gpu_envmap_);
        }
    }
}
void GPURenderer::OnModelLoad() {
    // Keep state of pipelines in sync for simpler logic
    for (auto& pipeline : pipelines_) {
        if (pipeline) {
            pipeline->OnModelLoad(*gpu_model_);
        }
    }
}

}  // namespace app