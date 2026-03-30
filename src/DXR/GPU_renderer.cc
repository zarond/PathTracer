#include "GPU_renderer.h"

#include <glm/ext.hpp>
#include <glm/glm.hpp>
#include <iostream>

namespace app {

void GPURenderer::update_camera_transform_state(
    fvec3 position, fvec3 direction, fvec3 up, fastgltf::Camera::Perspective perspectiveParams) {
    origin_ = position;
    viewMatrix_ = glm::lookAt(position, position + direction, up);
    projectionMatrix_ = glm::perspectiveRH(perspectiveParams.yfov, perspectiveParams.aspectRatio.value_or(1.77777777777777777f),
        perspectiveParams.znear, perspectiveParams.zfar.value_or(1000.f));
    NDC2WorldMatrix_ = glm::inverse(projectionMatrix_ * viewMatrix_);
}

void GPURenderer::load_scene(const Model& model, const CPUTexture<hdr_pixel>& envmap) {
    // Todo: check against fence values ?
    D3DContext& d3d_ctx = D3DContext::Get();
    d3d_ctx.WaitForPendingCopy();

    modelRef = &model;
    envmapRef = &envmap;
    reload_acceleration_structure();
    reload_ray_program();
}

void GPURenderer::load_envmap(const CPUTexture<hdr_pixel>& envmap) {
    envmapRef = &envmap;
    reload_ray_program();
}

void GPURenderer::load_model(const Model& model) {
    modelRef = &model;
    reload_acceleration_structure();
    reload_ray_program();
}

void GPURenderer::reload_ray_program() {
    // Todo: switch shaders
    switch (renderSettings_.programMode) {
        case RayProgramMode::AmbientOcclusion:
            break;
        case RayProgramMode::PBR:
            break;
        case RayProgramMode::RayCaster:
        default:
            break;
    }
}
void GPURenderer::reload_acceleration_structure() {
    auto start = std::chrono::high_resolution_clock::now();

    gpu_model_ = std::make_unique<GPU_model>(*modelRef);

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "GPU DXR model Acceleration Structure created in " << diff.count() << " ms." << '\n';
}

void GPURenderer::render_frame(CPUFrameBuffer& framebuffer, bool continuous, bool iterative, int iteration_count) {
    // atomic<bool>& instead of bool continuous?
    assert(modelRef);
    if (modelRef == nullptr || envmapRef == nullptr || gpu_model_ == nullptr) {
        throw std::runtime_error("One of components is nullptr in GPURenderer::render_frame()");
    }
    progress_ = 0.0f;
    render_state_ = RenderingState::Rendering;

    glm::fvec2 jitter = glm::fvec2{0.0f};
    float inverse_iteration_count = 1.0f;
    if (iterative) {
        static std::minstd_rand gen = std::minstd_rand(std::random_device{}());
        static std::uniform_real_distribution<float> dist{-0.5f, 0.5f};
        jitter = fvec2{dist(gen), dist(gen)};
        float n_sqrt = sqrtf(renderSettings_.samplesPerPixel);
        jitter /= n_sqrt;
        inverse_iteration_count = 1.0f / static_cast<float>(iteration_count);
    }
    
    // Todo: render on GPU
    D3DContext& d3d_ctx = D3DContext::Get();
    d3d_ctx.InitDXRCommandList();

    ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.g_pd3dSrvDescHeap.Get()};
    d3d_ctx.g_pd3dDXRCommandList->SetDescriptorHeaps(1, desc_heap);

    framebuffer.transition_from_srv_to_uav();
    pipeline_.DoRaytracing(*gpu_model_, framebuffer, NDC2WorldMatrix_, xyz1(origin_));
    framebuffer.transition_from_uav_to_srv();

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

void GPURenderer::set_render_settings(const RenderSettings& settings) { renderSettings_ = settings; }
RenderSettings GPURenderer::get_render_settings() const { return renderSettings_; }

BBox GPURenderer::get_scene_bound() const {
    // Todo: compute scene bound from model
    return BBox{};
}

float GPURenderer::get_progress() const { return progress_; }

void GPURenderer::cancel_rendering() {
    if (render_state_ == Rendering) {
        render_state_ = Cancelling;
    }
}

GPURenderer::RenderingState GPURenderer::get_rendering_state() const { return render_state_; }

void GPURenderer::set_render_starting_state() {
    if (render_state_ == Idle) {
        render_state_ = ReadyToStart;
    }
}

}  // namespace app