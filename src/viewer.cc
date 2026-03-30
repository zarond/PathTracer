#include "viewer.h"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <variant>

#include "cpu_framebuffer.h"

#include <glm/ext.hpp>

#ifndef NO_WINDOWS
#include "DXR/GPU_renderer.h"
#endif

namespace app {

using namespace glm;

Viewer::Viewer(Model&& model, CPUTexture<hdr_pixel>&& environmentTexture, const RenderSettings& settings)
    : model_(std::move(model)), environmentTexture_(std::move(environmentTexture)) {
    if (model_.cameras_.size() > 0) {
        activeCameraIndex_ = 0;
    }
    framebuffer_ = CPUFrameBuffer(windowDimensions_.x, windowDimensions_.y);

    renderers_.push_back(std::make_shared<Renderer>());
    renderer_ = renderers_[0]; // chose CPU renderer by default

    ++lastModelFenceValue;
    ++lastEnvmapFenceValue;
    renderer_->set_render_settings(settings);
    renderer_->load_scene(model_, environmentTexture_, lastModelFenceValue, lastEnvmapFenceValue);

    snap_to_camera();
}

#ifndef NO_WINDOWS
void Viewer::InitGPURenderer() {
    if (renderers_.size() >= 2) {
        return;
    }
    renderers_.push_back(std::make_shared<GPURenderer>());
}

void Viewer::switch_to_gpu_renderer() {
    if (renderers_.size() < 2) {
        throw std::runtime_error("GPU renderer not initialized. Call InitGPURenderer() first.");
    }
    if (renderer_->get_rendering_state() == Renderer::RenderingState::Rendering) {
        throw std::runtime_error("Cannot switch GPU renderer while rendering is in progress");
    }
    auto settings = get_render_settings();
    renderer_ = renderers_[1]; // find by type instead?
    renderer_->set_render_settings(settings);
    renderer_->load_scene(model_, environmentTexture_, lastModelFenceValue, lastEnvmapFenceValue);
    snap_to_camera(false);
    GPU_renderer_active = true;
}

void Viewer::switch_to_cpu_renderer() {
    if (renderer_->get_rendering_state() == Renderer::RenderingState::Rendering) {
        throw std::runtime_error("Cannot switch GPU renderer while rendering is in progress");
    }
    auto settings = get_render_settings();
    renderer_ = renderers_[0];  // find by type instead?
    renderer_->set_render_settings(settings);
    renderer_->load_scene(model_, environmentTexture_, lastModelFenceValue, lastEnvmapFenceValue);
    snap_to_camera(false);
    GPU_renderer_active = false;
}
#endif

void Viewer::resize_window(const ivec2& newDimensions) {
    windowDimensions_ = newDimensions;
#ifndef NO_WINDOWS
    framebuffer_.release_gpu_resource();
#endif
    framebuffer_ = CPUFrameBuffer(windowDimensions_.x, windowDimensions_.y);
}

ivec2 Viewer::get_window_dimensions() const { return windowDimensions_; }

void Viewer::render() { 
    renderer_->render_frame(framebuffer_, continuous_rendering, iterative_rendering, iterations_counter);
    if (iterative_rendering) {
        ++iterations_counter;
    } else {
        reset_iteration_counter();
    }
}

void Viewer::cancel_rendering() {
    renderer_->cancel_rendering(); 
    cv_render.notify_one();
}

Renderer::RenderingState Viewer::get_rendering_state() const { return renderer_->get_rendering_state(); }

void Viewer::async_start_render() { 
    renderer_->set_render_starting_state();
    cv_render.notify_one();
}

void Viewer::clear_framebuffer_black() { framebuffer_.clear(); }

void Viewer::set_active_camera(std::optional<uint32_t> cameraIndex) {
    if (cameraIndex.has_value()) {
        if (*cameraIndex >= model_.cameras_.size()) {
            throw std::out_of_range("Camera index out of range in Viewer::set_active_camera");
        }
        activeCameraIndex_ = cameraIndex;
        snap_to_camera();
    } else {
        activeCameraIndex_ = cameraIndex;
    }
}

std::optional<uint32_t> Viewer::get_active_camera() const { return activeCameraIndex_; }

void Viewer::take_snapshot(const std::filesystem::path& filePath) const { framebuffer_.save_to_file(filePath, is_using_gpu_renderer()); }

bool Viewer::snap_to_camera(bool use_default) {
    bool success = false;

    if (activeCameraIndex_.has_value()) {
        const auto& camera = model_.cameras_[*activeCameraIndex_];
        std::visit(fastgltf::visitor{
                [&](const fastgltf::Camera::Perspective& perspective) {
                    position_ = xyz(camera.ModelMatrix[3]);
                    direction_ = -xyz(camera.ModelMatrix[2]);
                    up_ = xyz(camera.ModelMatrix[1]);
                    cam_params_ = perspective;

                    success = true;
                },
                [&](const fastgltf::Camera::Orthographic& orthographic) {
                    // Todo: implement orthographic camera snapping
                },
            },
        camera.camera_params);
    }
    cam_params_.aspectRatio = static_cast<float>(windowDimensions_.x) / windowDimensions_.y;  // adjust camera aspect ratio to screen
    if (!success && use_default) {
        set_up_default_camera_transforms();
    }
    renderer_->update_camera_transform_state(position_, direction_, up_, cam_params_);

    return success;
}

int Viewer::get_number_of_cameras() const { return static_cast<int>(model_.cameras_.size()); }

void Viewer::set_render_settings(const RenderSettings& settings) {
    const auto currentSettings = renderer_->get_render_settings();
    renderer_->set_render_settings(settings);
    if (currentSettings.accelStructType != settings.accelStructType ||
        currentSettings.maxTrianglesPerBVHLeaf != settings.maxTrianglesPerBVHLeaf) {
        renderer_->reload_acceleration_structure();
    }
    if (currentSettings.programMode != settings.programMode || 
        currentSettings.envmapRotation != settings.envmapRotation ||
        currentSettings.maxNewRaysPerBounce != settings.maxNewRaysPerBounce) {
        renderer_->reload_ray_program();
    }
}
RenderSettings Viewer::get_render_settings() const { return renderer_->get_render_settings(); }

float Viewer::get_render_progress() const { return renderer_->get_progress(); }

void Viewer::load_envmap(CPUTexture<hdr_pixel>&& environmentTexture) { 
    environmentTexture_ = std::move(environmentTexture);
    ++lastEnvmapFenceValue;
    renderer_->load_envmap(environmentTexture_, lastEnvmapFenceValue);
}

void Viewer::load_model(Model&& model) { 
    model_ = std::move(model);
    activeCameraIndex_ = std::nullopt;
    if (model_.cameras_.size() > 0) {
        activeCameraIndex_ = 0;
    }
    ++lastModelFenceValue;
    renderer_->load_model(model_, lastModelFenceValue);
}

const Model& Viewer::get_model() const { return model_; }

CPUFrameBuffer& Viewer::get_framebuffer() { return framebuffer_; }

void Viewer::reset_iteration_counter() { iterations_counter = 1; }
int Viewer::get_iteration_counter() const { return iterations_counter; }

void Viewer::set_up_default_camera_transforms() {
    direction_ = fvec3(0.0f, 0.0f, -1.0f);
    up_ = fvec3(0.0f, 1.0f, 0.0f);  // up view direction
    cam_params_ = {
        .aspectRatio = static_cast<float>(windowDimensions_.x) / windowDimensions_.y,
        .yfov = glm::radians(60.f),
        .zfar = 1000.0f,
        .znear = 0.1f};
    auto bounds = renderer_->get_scene_bound();
    bool not_empty = !bounds.is_empty();
    auto dims = (not_empty) ? (bounds.max - bounds.min) : fvec3{0.0f};
    auto max_dim = (not_empty) ? max(dims.x, max(dims.y, dims.z)) : 1.0f;
    auto center = (not_empty) ? bounds.min + dims * 0.5f : fvec3{0.0f};
    fvec3 offset = fvec3(0.0f, 0.0f, 1.0f) * max_dim * 1.5f;
    position_ = center + offset;
}

float& Viewer::get_yfov() { return cam_params_.yfov; }

fvec3 Viewer::get_euler_angles_camera() const { 
    glm::quat quat = glm::quatLookAt(direction_, up_);
    return glm::eulerAngles(quat);
}

bool Viewer::is_using_gpu_renderer() const { return GPU_renderer_active; }

}  // namespace app
