#pragma once

#include <optional>
#include <memory>
#include <chrono>

#include "model_loader.h"
#include "cpu_framebuffer.h"
#include "renderer.h"

#include <glm/glm.hpp>

namespace app {

using namespace glm;

class Viewer {
public:
    explicit Viewer(Model&& model, CPUTexture<hdr_pixel> && environmentTexture, const RenderSettings& settings);

    Viewer(const Viewer&) = delete;
    Viewer& operator=(const Viewer&) = delete;

    void resize_window(const ivec2& newDimensions);
    ivec2 get_window_dimensions() const;

    void render();
    
    void set_active_camera(std::optional<uint32_t> cameraIndex);
    std::optional<uint32_t> get_active_camera() const;

    void take_snapshot(const std::filesystem::path& filePath) const;

    bool snap_to_camera();

    void set_render_settings(const RenderSettings& settings);
    RenderSettings get_render_settings() const;

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

    fvec3 direction_ = fvec3(0.0f, 0.0f, -1.0f); // center view direction
    fvec3 up_ = fvec3(0.0f, 1.0f, 0.0f); // up view direction
    dvec2 lastCursorPosition_ = dvec2(0.0f);

    float yaw_ = -90.0f;
    float pitch_ = 0.0f;
    bool firstMouse_ = true;
};

}