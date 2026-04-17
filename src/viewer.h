#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fastgltf/types.hpp>
#include <filesystem>
#include <glm/fwd.hpp>
#include <memory>
#include <optional>
#include <vector>
#include <mutex>
#include <stop_token>

#include "cpu_framebuffer.h"
#include "model_loader.h"
#include "render_settings.h"
#include "renderer.h"

namespace app {

using glm::ivec2;
using glm::fvec2;
using glm::fvec3;
using glm::fvec4;
using glm::dvec2;

class Viewer {
  public:
    explicit Viewer(Model&& model, CPUTexture<hdr_pixel>&& environmentTexture, const RenderSettings& settings);

    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    void resize_window(const ivec2& newDimensions, bool createGPUTex = false);
    ivec2 get_window_dimensions() const;

    void render();
    void async_start_render();
    void cancel_rendering();
    RenderingState get_rendering_state() const;

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
    const Model& get_model() const;
    Model& get_model();
    const std::vector<Material>& get_materials_backup() const;
    void set_materials_updated();

    std::atomic<bool> continuous_rendering = false;
    std::atomic<bool> iterative_rendering = false;
    void reset_iteration_counter();
    int get_iteration_counter() const;

    fvec3 position_ = fvec3(0.0f);                // camera position
    fvec3 direction_ = fvec3(0.0f, 0.0f, -1.0f);  // center view direction
    fvec3 up_ = fvec3(0.0f, 1.0f, 0.0f);          // up view direction
    fvec3 right_() const;

    float& get_yfov();
    fvec3 get_euler_angles_camera() const;

    bool is_using_gpu_renderer() const;

#ifndef NO_WINDOWS
    void InitGPURenderer();
    void switch_to_renderer(RendererMode mode);
    RendererMode get_renderer_mode() const;
#endif

    void wait_for_render_start(std::stop_token stop);

  private:
    Model model_;
    CPUTexture<hdr_pixel> environmentTexture_;
    std::vector<Material> materials_backups_;
    bool need_materials_update_ = false;
    
    std::mutex mtx_render_;
    std::condition_variable cv_render_;

    size_t lastModelFenceValue = 0;
    size_t lastEnvmapFenceValue = 0;

    int iterations_counter = 1;

    std::optional<uint32_t> activeCameraIndex_ = std::nullopt;

    std::shared_ptr<IRenderer> renderer_; // currently chosen renderer
    std::vector<std::shared_ptr<IRenderer>> renderers_;  // initialize all renderers
    RendererMode activeRendererMode_;

    CPUFrameBuffer framebuffer_;

    ivec2 windowDimensions_ = ivec2(800, 600);

    std::chrono::steady_clock::time_point lastFrame_;
    std::chrono::milliseconds deltaTime_;

    fvec3 accelerationVector_ = fvec3(0.0f);
    fvec3 velocity_ = fvec3(0.0f);

    fastgltf::Camera::Perspective cam_params_;

    dvec2 lastCursorPosition_ = dvec2(0.0f);
    float yaw_ = -90.0f;
    float pitch_ = 0.0f;
    bool firstMouse_ = true;

    void set_up_default_camera_transforms();
};

void save_render_image_timed_action(const Viewer& viewer, const std::filesystem::path& image_path);

}  // namespace app
