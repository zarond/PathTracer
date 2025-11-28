#include "viewer.h"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <variant>

#include "cpu_framebuffer.h"

namespace app {

using namespace glm;

Viewer::Viewer(Model&& model, CPUTexture<hdr_pixel>&& environmentTexture, const RenderSettings& settings)
    : model_(std::move(model)), environmentTexture_(std::move(environmentTexture)) {
    if (model_.cameras_.size() > 0) {
        activeCameraIndex_ = 0;
    }
    framebuffer_ = CPUFrameBuffer(windowDimensions_.x, windowDimensions_.y);

    renderer_.set_render_settings(settings);
    renderer_.load_scene(model_, environmentTexture_);

    bool res = snap_to_camera();
    if (res == false) {
        set_up_default_camera_transforms();
        renderer_.update_camera_transform_state(position_, direction_, up_, cam_params_);
    }
}

void Viewer::resize_window(const ivec2& newDimensions) {
    windowDimensions_ = newDimensions;
    framebuffer_ = CPUFrameBuffer(windowDimensions_.x, windowDimensions_.y);
}

ivec2 Viewer::get_window_dimensions() const { return windowDimensions_; }

void Viewer::render() { renderer_.render_frame(framebuffer_); }

void Viewer::set_active_camera(std::optional<uint32_t> cameraIndex) {
    if (cameraIndex.has_value()) {
        if (*cameraIndex >= model_.cameras_.size()) {
            throw std::out_of_range("Camera index out of range in Viewer::set_active_camera");
        }
        activeCameraIndex_ = cameraIndex;
        snap_to_camera();
    }
}

std::optional<uint32_t> Viewer::get_active_camera() const { return activeCameraIndex_; }

void Viewer::take_snapshot(const std::filesystem::path& filePath) const { framebuffer_.save_to_file(filePath); }

bool Viewer::snap_to_camera() {
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
    renderer_.update_camera_transform_state(position_, direction_, up_, cam_params_);

    return success;
}

void Viewer::set_render_settings(const RenderSettings& settings) {
    const auto currentSettings = renderer_.get_render_settings();
    renderer_.set_render_settings(settings);
    // Todo: reload scene only if settings have changed that require it
    if (currentSettings != settings) {
        renderer_.load_scene(model_, environmentTexture_);
    }
}
RenderSettings Viewer::get_render_settings() const { return renderer_.get_render_settings(); }

void Viewer::set_up_default_camera_transforms() {
    direction_ = fvec3(0.0f, 0.0f, -1.0f);
    up_ = fvec3(0.0f, 1.0f, 0.0f);  // up view direction
    cam_params_ = {
        .aspectRatio = static_cast<float>(windowDimensions_.x) / windowDimensions_.y,
        .yfov = glm::radians(60.f),
        .zfar = 1000.0f,
        .znear = 0.1f};
    auto bounds = renderer_.get_scene_bound();
    auto dims = (bounds.max - bounds.min);
    auto max_dim = max(dims.x, max(dims.y, dims.z));
    auto center = bounds.min + dims * 0.5f;
    fvec3 offset = fvec3(0.0f, 0.0f, 1.0f) * max_dim * 1.5f;
    position_ = center + offset;
}

}  // namespace app
