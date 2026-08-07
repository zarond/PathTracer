#include "viewer.h"

#include <fastgltf/util.hpp>
#include <glm/ext.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <variant>

#ifdef WINDOWS_SPECIFIC
#include "GPU/GPU_renderer.h"
#include "d3d_context.h"
#endif

namespace app {

using namespace glm;

Viewer::Viewer(Model&& model, CPUTexture<hdr_pixel>&& environmentTexture, const RenderSettings& settings)
    : model_(std::move(model)), environment_texture_(std::move(environmentTexture)) {
    if (model_.cameras.size() > 0) {
        active_camera_index_ = 0;
    }
    framebuffer_ = CPUFrameBuffer(window_dimensions_.x, window_dimensions_.y);

    renderers_.resize((int)RendererMode::Count);
    renderers_[(int)RendererMode::CPURenderer] = std::make_shared<Renderer>();
    renderer_ = renderers_[(int)RendererMode::CPURenderer];  // chose CPU renderer by default
    active_renderer_mode_ = RendererMode::CPURenderer;

    ++last_model_fence_value_;
    ++last_envmap_fence_value_;
    renderer_->set_render_settings(settings);
    renderer_->load_scene(model_, environment_texture_, last_model_fence_value_, last_envmap_fence_value_);

    materials_backups_ = model_.materials;

    snap_to_camera();
}

#ifdef WINDOWS_SPECIFIC
void Viewer::init_GPU_renderer() {
    if (renderers_[(int)RendererMode::GPURenderer]) {
        return;
    }
    renderers_[(int)RendererMode::GPURenderer] = std::make_shared<GPURenderer>();
}

void Viewer::switch_to_renderer(RendererMode mode) {
    if ((int)mode < 0 || mode >= RendererMode::Count) {
        throw std::runtime_error("Invalid Renderer Mode.");
    }
    if (!renderers_[(int)mode]) {
        throw std::runtime_error("Renderer not initialized. Try calling InitGPURenderer() first.");
    }
    if (renderer_->get_rendering_state() == RenderingState::Rendering) {
        throw std::runtime_error("Cannot switch renderer while rendering is in progress");
    }
    if (active_renderer_mode_ == mode) {
        return;
    }
    auto settings = get_render_settings();
    renderer_ = renderers_[(int)mode];
    renderer_->set_render_settings(settings);
    renderer_->load_scene(model_, environment_texture_, last_model_fence_value_, last_envmap_fence_value_);
    snap_to_camera(false);
    active_renderer_mode_ = mode;
}

RendererMode Viewer::get_renderer_mode() const { return active_renderer_mode_; }
#endif

void Viewer::resize_window(const ivec2& newDimensions, bool createGPUTex) {
    window_dimensions_ = newDimensions;
#ifdef WINDOWS_SPECIFIC
    framebuffer_.release_gpu_resource();
#endif
    framebuffer_ = CPUFrameBuffer(window_dimensions_.x, window_dimensions_.y);
#ifdef WINDOWS_SPECIFIC
    if (createGPUTex) {
        framebuffer_.create_texture_resource();
    }
#endif
}

ivec2 Viewer::get_window_dimensions() const { return window_dimensions_; }

void Viewer::render() {
    if (need_materials_update_) {
        renderer_->reload_materials();
        need_materials_update_ = false;
    }
    if (animation_playing_) {
        need_transforms_update_ = true;
        animation_time_ = std::chrono::high_resolution_clock::now() - start_timestamp_;
        model_.apply_animation(current_animation_index_, animation_time_.count(), animation_looping);
    }
    if (need_transforms_update_) {
        renderer_->update_transforms();
        need_transforms_update_ = false;

        if (active_camera_index_.has_value()) {
            snap_to_camera();
        }
    }
    bool camera_moved = camera_updated_in_last_frame_.load();
    bool use_iterative_rendering = !camera_moved && iterative_rendering;
    renderer_->render_frame(framebuffer_, continuous_rendering, use_iterative_rendering, iterations_counter_);
    if (use_iterative_rendering) {
        ++iterations_counter_;
    } else {
        reset_iteration_counter();
    }
    if (camera_moved) {
        camera_updated_in_last_frame_.store(false);
    }
}

void Viewer::cancel_rendering() {
    renderer_->cancel_rendering();
    cv_render_.notify_one();
}

RenderingState Viewer::get_rendering_state() const { return renderer_->get_rendering_state(); }

void Viewer::async_start_render() {
    renderer_->set_render_starting_state();
    cv_render_.notify_one();
}

void Viewer::clear_framebuffer_black() { framebuffer_.clear(); }

void Viewer::set_active_camera(std::optional<uint32_t> cameraIndex) {
    if (cameraIndex.has_value()) {
        if (*cameraIndex >= model_.cameras.size()) {
            throw std::out_of_range("Camera index out of range in Viewer::set_active_camera");
        }
        active_camera_index_ = cameraIndex;
        snap_to_camera();
    } else {
        active_camera_index_ = cameraIndex;
    }
}

std::optional<uint32_t> Viewer::get_active_camera() const { return active_camera_index_; }

void Viewer::take_snapshot(const std::filesystem::path& filePath) const {
    framebuffer_.save_to_file(filePath, is_using_gpu_renderer());
}

bool Viewer::snap_to_camera(bool use_default) {
    camera_updated_in_last_frame_.store(true);
    bool success = false;

    if (active_camera_index_.has_value()) {
        const auto& camera = model_.cameras[*active_camera_index_];
        std::visit(fastgltf::visitor{
                [&](const fastgltf::Camera::Perspective& perspective) {
                    position = xyz(camera.ModelMatrix[3]);
                    direction = -xyz(camera.ModelMatrix[2]);
                    up = xyz(camera.ModelMatrix[1]);
                    cam_params_ = perspective;

                    success = true;
                },
                [&](const fastgltf::Camera::Orthographic& orthographic) {
                    // Todo: implement orthographic camera snapping
                },
            },
        camera.camera_params);
    }
    cam_params_.aspectRatio = static_cast<float>(window_dimensions_.x) / window_dimensions_.y;  // adjust camera aspect ratio to screen
    if (!success && use_default) {
        set_up_default_camera_transforms();
    }
    renderer_->update_camera_transform_state(position, direction, up, cam_params_);

    return success;
}

int Viewer::get_number_of_cameras() const { return static_cast<int>(model_.cameras.size()); }

void Viewer::set_render_settings(const RenderSettings& settings) { renderer_->set_render_settings(settings); }
RenderSettings Viewer::get_render_settings() const { return renderer_->get_render_settings(); }

float Viewer::get_render_progress() const { return renderer_->get_progress(); }

void Viewer::load_envmap(CPUTexture<hdr_pixel>&& environmentTexture) {
    environment_texture_ = std::move(environmentTexture);
    ++last_envmap_fence_value_;
    renderer_->load_envmap(environment_texture_, last_envmap_fence_value_);
}

void Viewer::load_model(Model&& model) {
    model_ = std::move(model);
    active_camera_index_ = std::nullopt;
    if (model_.cameras.size() > 0) {
        active_camera_index_ = 0;
    }
    ++last_model_fence_value_;
    renderer_->load_model(model_, last_model_fence_value_);

    materials_backups_ = model_.materials;
}

const Model& Viewer::get_model() const { return model_; }
Model& Viewer::get_model() { return model_; }
const std::vector<Material>& Viewer::get_materials_backup() const { return materials_backups_; }
void Viewer::set_materials_updated() { need_materials_update_ = true; }

CPUFrameBuffer& Viewer::get_framebuffer() { return framebuffer_; }

void Viewer::reset_iteration_counter() { iterations_counter_ = 1; }
int Viewer::get_iteration_counter() const { return iterations_counter_; }

void Viewer::set_up_default_camera_transforms() {
    direction = fvec3(0.0f, 0.0f, -1.0f);
    up = fvec3(0.0f, 1.0f, 0.0f);  // up view direction
    cam_params_ = {
        .aspectRatio = static_cast<float>(window_dimensions_.x) / window_dimensions_.y,
        .yfov = glm::radians(60.f),
        .zfar = 1000.0f,
        .znear = 0.1f};
    auto bounds = renderer_->get_scene_bound();
    bool not_empty = !bounds.is_empty();
    auto dims = (not_empty) ? (bounds.max - bounds.min) : fvec3{0.0f};
    auto max_dim = (not_empty) ? max(dims.x, max(dims.y, dims.z)) : 1.0f;
    auto center = (not_empty) ? bounds.min + dims * 0.5f : fvec3{0.0f};
    fvec3 offset = fvec3(0.0f, 0.0f, 1.0f) * max_dim * 1.5f;
    position = center + offset;
}

fvec3 Viewer::right() const { return cross(direction, up); }

float& Viewer::get_yfov() { return cam_params_.yfov; }

fvec3 Viewer::get_euler_angles_camera() const {
    glm::quat quat = glm::quatLookAt(direction, up);
    const glm::quat q_shfl{quat.w, quat.y, quat.x, quat.z};  // To overcome GLM's gimbal lock issue
    const glm::fvec3 euler{
        glm::yaw(q_shfl),    // Pitch
        glm::pitch(q_shfl),  // Yaw
        glm::roll(q_shfl)    // Roll
    };
    return euler;
}

fvec2 Viewer::get_near_far_camera_values() const { 
    return fvec2(cam_params_.znear, cam_params_.zfar.value_or(std::numeric_limits<float>::infinity()));
}
void Viewer::set_near_far_camera_values(float near_, float far_) { 
    cam_params_.znear = near_; 
    cam_params_.zfar = far_;
}

bool Viewer::is_using_gpu_renderer() const { return (active_renderer_mode_ == RendererMode::GPURenderer); }

#ifdef WINDOWS_SPECIFIC
RenderPipelineMode Viewer::get_active_gpu_pipeline_mode() const { 
    const auto& gpu_renderer = renderers_[(int)RendererMode::GPURenderer];
    if (gpu_renderer != nullptr) {
        return static_cast<GPURenderer*>(gpu_renderer.get())->get_active_pipeline_mode();
    }
    return RenderPipelineMode::Count; 
}

void Viewer::switch_gpu_pipeline_mode(RenderPipelineMode mode) {
    const auto& gpu_renderer = renderers_[(int)RendererMode::GPURenderer];
    if (gpu_renderer == nullptr) return;
    if ((int)mode < 0 || mode >= RenderPipelineMode::Count) {
        return;
    }
    static_cast<GPURenderer*>(gpu_renderer.get())->set_active_pipeline_mode(mode);
}
#endif

void Viewer::wait_for_render_start(std::stop_token stop) {
    std::unique_lock<std::mutex> lock(mtx_render_);
    cv_render_.wait(lock, [&]() { return stop.stop_requested() || (get_rendering_state() == RenderingState::ReadyToStart); });
}

void Viewer::start_animation_playback() {
    if (model_.animations.empty()) {
        return;
    }

    // Ensure current_animation_index_ is within valid range
    if (current_animation_index_ < 0 || current_animation_index_ >= static_cast<int>(model_.animations.size())) {
        current_animation_index_ = 0;
    }

    animation_playing_ = true;
    start_timestamp_ = std::chrono::high_resolution_clock::now() -
                       std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(animation_time_);
}

void Viewer::stop_animation_playback() {
    animation_playing_ = false;
    animation_time_ = std::chrono::high_resolution_clock::now() - start_timestamp_;
}

void Viewer::choose_animation(uint32_t animationIndex) {
    if (animationIndex >= model_.animations.size()) {
        return;
    }

    current_animation_index_ = static_cast<int>(animationIndex);

    // Reset animation time if playback is running
    if (animation_playing_) {
        animation_time_ = std::chrono::duration<double>(0.0);
        start_timestamp_ = std::chrono::high_resolution_clock::now();
    }
}

void Viewer::rewind_animation() {
    animation_time_ = std::chrono::duration<double>(0.0);

    // If animation is playing, reset the start timestamp
    if (animation_playing_) {
        start_timestamp_ = std::chrono::high_resolution_clock::now();
    }
}

void Viewer::set_animation_looping(bool loop) {
    animation_looping = loop;
}

bool Viewer::get_animation_looping() const {
    return animation_looping;
}

void save_render_image_timed_action(const Viewer& viewer, const std::filesystem::path& image_path) {
    auto start = std::chrono::high_resolution_clock::now();
    viewer.take_snapshot(image_path);
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "output saved in " << diff.count() << " ms." << '\n';
}

}  // namespace app
