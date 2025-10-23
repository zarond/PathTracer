#pragma once

#include <optional>
#include <memory>
#include <chrono>

#include "model_loader.h"
#include "cpu_framebuffer.h"
#include "renderer.h"

#include <fastgltf/core.hpp>
#include <fastgltf/types.hpp>
#include <fastgltf/tools.hpp>

namespace app {

class Viewer {
public:
    explicit Viewer(Model&& model, CPUTexture&& environmentTexture);

    void resize_window(const ivec2& newDimensions);
    ivec2 get_window_dimensions() const;

    void render();
    
    void set_active_camera(std::optional<size_t> cameraIndex);
    std::optional<size_t> get_active_camera() const;

    void take_snapshot(const std::filesystem::path& filePath) const;

    bool snap_to_camera();

private:
    Model model_;
    CPUTexture environmentTexture_;
    
    std::optional<size_t> activeCameraIndex_ = std::nullopt;

    std::unique_ptr<Renderer> renderer_;
    CPUFrameBuffer framebuffer_;

    ivec2 windowDimensions_ = ivec2(800, 600);

    std::chrono::steady_clock::time_point lastFrame_;
    std::chrono::milliseconds deltaTime_;

    fvec3 accelerationVector_ = fvec3(0.0f);
    fvec3 velocity_ = fvec3(0.0f);
    fvec3 position_ = fvec3(0.0f, 0.0f, 0.0f);

    dvec2 lastCursorPosition_ = dvec2(0.0f);
    fvec3 direction_ = fvec3(0.0f, 0.0f, -1.0f); // center view direction
    fvec3 up_ = fvec3(0.0f, 1.0f, 0.0f); // up view direction

    float yaw_ = -90.0f;
    float pitch_ = 0.0f;
    bool firstMouse_ = true;
};

}