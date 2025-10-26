#include <stdexcept>
#include <variant>

#include "viewer.h"
#include "cpu_framebuffer.h"

namespace app {

using namespace fastgltf::math;

Viewer::Viewer(Model&& model, CPUTexture<hdr_pixel>&& environmentTexture):
    model_(std::move(model)),
    environmentTexture_(std::move(environmentTexture))
{
    if (model_.cameras_.size() > 0) {
        activeCameraIndex_ = 0;
    }
    framebuffer_ = CPUFrameBuffer(windowDimensions_[0], windowDimensions_[1]);

    renderer_.load_scene(model_);

    snap_to_camera();
}

void Viewer::resize_window(const ivec2& newDimensions)
{
    windowDimensions_ = newDimensions;
    framebuffer_ = CPUFrameBuffer(windowDimensions_[0], windowDimensions_[1]);
}

ivec2 Viewer::get_window_dimensions() const { return windowDimensions_; }

void Viewer::render()
{
    renderer_.render_frame(framebuffer_);
}

void Viewer::set_active_camera(std::optional<uint32_t> cameraIndex)
{
    if (cameraIndex.has_value()) {
        if (*cameraIndex >= model_.cameras_.size()) {
            throw std::out_of_range("Camera index out of range in Viewer::set_active_camera");
        }
    }
    activeCameraIndex_ = cameraIndex;
    snap_to_camera();
}

std::optional<uint32_t> Viewer::get_active_camera() const
{
    return activeCameraIndex_;
}

void Viewer::take_snapshot(const std::filesystem::path & filePath) const
{
    framebuffer_.save_to_file(filePath);
}

bool Viewer::snap_to_camera()
{
    if (!activeCameraIndex_.has_value()) return false; 

    const auto& camera = model_.cameras_[*activeCameraIndex_];

    bool success = false;

	std::visit(fastgltf::visitor{
		[&](const fastgltf::Camera::Perspective& perspective) {
            position_ = fvec3(camera.ModelMatrix[3].x(), camera.ModelMatrix[3].y(), camera.ModelMatrix[3].z());
            direction_ = fvec3(-camera.ModelMatrix[2].x(), -camera.ModelMatrix[2].y(), -camera.ModelMatrix[2].z());
            up_ = fvec3(camera.ModelMatrix[1].x(), camera.ModelMatrix[1].y(), camera.ModelMatrix[1].z());
            
            renderer_.update_camera_transform_state(
                position_,
                direction_,
                up_,
                perspective
            );
            
            success = true;
		},
		[&](const fastgltf::Camera::Orthographic& orthographic) {
            // Todo: implement orthographic camera snapping
		},
	}, 
    camera.camera_params);

	return success;

}

void Viewer::set_render_settings(const RenderSettings& settings) {
    renderer_.set_render_settings(settings);
}
RenderSettings Viewer::get_render_settings() const { return renderer_.get_render_settings(); }

}