#pragma once

#include <chrono>
#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <condition_variable>

#include "cpu_framebuffer.h"
#include "model_loader.h"
#include "renderer.h"

namespace app {

using namespace glm;

class Viewer {
  public:
    explicit Viewer(Model&& model, CPUTexture<hdr_pixel>&& environmentTexture, const RenderSettings& settings);

    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    void resize_window(const ivec2& newDimensions);
    ivec2 get_window_dimensions() const;

    void render();
    void async_start_render();
    void cancel_rendering();
    Renderer::RenderingState get_rendering_state() const;

    void clear_framebuffer_black();

    void set_active_camera(std::optional<uint32_t> cameraIndex);
    std::optional<uint32_t> get_active_camera() const;

    void take_snapshot(const std::filesystem::path& filePath) const;

    bool snap_to_camera(bool use_default = true);  // use_default forces to set default camera if no camera is present in model
    int get_number_of_cameras() const;

    void set_render_settings(const RenderSettings& settings);
    RenderSettings get_render_settings() const;

    float get_render_progress() const;
    CPUFrameBuffer& get_framebuffer();

    void load_envmap(CPUTexture<hdr_pixel>&& environmentTexture);
    void load_model(Model&& model);

    std::condition_variable cv_render;
    std::atomic<bool> continuous_rendering = false;

  private:
    Model model_;
    CPUTexture<hdr_pixel> environmentTexture_;

    std::optional<uint32_t> activeCameraIndex_ = std::nullopt;

    Renderer renderer_;
    CPUFrameBuffer framebuffer_;

    ivec2 windowDimensions_ = ivec2(800, 600);

    std::chrono::steady_clock::time_point lastFrame_;
    std::chrono::milliseconds deltaTime_;

    fvec3 accelerationVector_ = fvec3(0.0f);
    fvec3 velocity_ = fvec3(0.0f);
    fvec3 position_ = fvec3(0.0f);

    fvec3 direction_ = fvec3(0.0f, 0.0f, -1.0f);  // center view direction
    fvec3 up_ = fvec3(0.0f, 1.0f, 0.0f);          // up view direction
    fastgltf::Camera::Perspective cam_params_;
    dvec2 lastCursorPosition_ = dvec2(0.0f);

    float yaw_ = -90.0f;
    float pitch_ = 0.0f;
    bool firstMouse_ = true;

    void set_up_default_camera_transforms();
};

}  // namespace app
