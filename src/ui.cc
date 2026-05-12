#include "ui.h"

#include <glm/ext.hpp>
#include <iostream>
#include <string>

#include "brdf.h"
#include "cpu_framebuffer.h"
#include "d3d_context.h"

#include "backends/imgui_impl_dx12.h"
#include "backends/imgui_impl_win32.h"
#include "imgui.h"

// Forward declare message handler from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace {
using namespace app;
namespace fs = std::filesystem;

bool useTextureCheckbox(const char* text, int& texture, int original_texture_value) {
    bool use_texture = (texture >= 0);
    bool changed = ImGui::Checkbox(text, &use_texture);
    if (changed) {
        if (use_texture) {
            texture = original_texture_value;
            if (original_texture_value < 0) {
                changed = false;
            }
        } else {
            texture = -1;
        }
    }
    return changed;
}

}  // namespace

namespace app {

bool DXWindow::Init() {
    // Window class
    WNDCLASSEXW wcex{};
    wcex.cbSize = sizeof(wcex);
    // wcex.style = CS_CLASSDC;
    wcex.style = CS_OWNDC;
    wcex.lpfnWndProc = &WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = GetModuleHandleW(nullptr);
    wcex.hIcon = LoadIconW(nullptr, MAKEINTRESOURCEW(IDI_APPLICATION));
    wcex.hCursor = LoadCursorW(nullptr, MAKEINTRESOURCEW(IDC_ARROW));
    wcex.hbrBackground = nullptr;
    wcex.lpszMenuName = nullptr;
    wcex.lpszClassName = L"PathTracer";
    wcex.hIconSm = LoadIconW(nullptr, MAKEINTRESOURCEW(IDI_APPLICATION));
    wndClass = RegisterClassExW(&wcex);
    if (wndClass == 0) return false;
    // create window
    hwnd = ::CreateWindowExW(WS_EX_OVERLAPPEDWINDOW | WS_EX_APPWINDOW, (LPCWSTR)wndClass, L"PathTracer project by @zarond",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE, 100, 100, 1280, 720, nullptr, nullptr, wcex.hInstance, nullptr);
    if (hwnd == nullptr) return false;
    return true;
}
void DXWindow::ShutDown() {
    if (hwnd) {
        DestroyWindow(hwnd);
        hwnd = nullptr;
    }
    if (wndClass) {
        UnregisterClassW((LPCWSTR)wndClass, GetModuleHandleW(nullptr));
        wndClass = 0;
    }
}
void DXWindow::Update() {
    MSG msg;
    while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
        ::TranslateMessage(&msg);
        ::DispatchMessage(&msg);
    }
}
void DXWindow::SetFullscreen(bool enabled, HWND hWnd) {
    // Update window style
    DWORD style = WS_OVERLAPPEDWINDOW | WS_VISIBLE;
    DWORD ex_style = WS_EX_OVERLAPPEDWINDOW | WS_EX_APPWINDOW;
    if (enabled) {
        style = WS_POPUP | WS_VISIBLE;
        ex_style = WS_EX_APPWINDOW;
    }

    ::SetWindowLongW(hWnd, GWL_STYLE, style);
    ::SetWindowLongW(hWnd, GWL_EXSTYLE, ex_style);

    // Adjust window size
    if (enabled) {
        HMONITOR monitor = ::MonitorFromWindow(hWnd, MONITOR_DEFAULTTONEAREST);
        MONITORINFO monitorInfo{};
        monitorInfo.cbSize = sizeof(monitorInfo);
        if (GetMonitorInfoW(monitor, &monitorInfo)) {
            SetWindowPos(hWnd, nullptr, monitorInfo.rcMonitor.left, monitorInfo.rcMonitor.top,
                monitorInfo.rcMonitor.right - monitorInfo.rcMonitor.left,
                monitorInfo.rcMonitor.bottom - monitorInfo.rcMonitor.top, SWP_NOZORDER);
        }
    } else {
        ShowWindow(hWnd, SW_MAXIMIZE);
    }

    is_fullscreen = enabled;
}

// Win32 message handler
// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of
// the mouse data.
// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your
// copy of the keyboard data. Generally you may always pass all inputs to dear imgui, and hide them from your application based on
// those two flags.
LRESULT WINAPI DXWindow::WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam)) return true;

    DXWindow& dx_window = DXWindow::Get();

    switch (msg) {
        case WM_KEYDOWN:
            if (wParam == VK_F11) {
                dx_window.SetFullscreen(!dx_window.is_fullscreen, hWnd);
            }
            break;
        case WM_SIZE: 
            {
                D3DContext& d3d_ctx = D3DContext::Get();
                if (d3d_ctx.m_d3dDevice != nullptr && wParam != SIZE_MINIMIZED) {
                    d3d_ctx.ResizeSwapchain(LOWORD(lParam), HIWORD(lParam));
                }
            }
            return 0;
        case WM_SYSCOMMAND:
            if ((wParam & 0xfff0) == SC_KEYMENU)  // Disable ALT application menu
                return 0;
            break;
        case WM_CLOSE:
            dx_window.close_window = true;
            return 0;
        case WM_DESTROY:
            ::PostQuitMessage(0);
            dx_window.close_window = true;
            return 0;
    }
    return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}

std::string OpenFileDialog() {     // todo: modernize with IFileDialog
    const int max_path = 1024;     // MAX_PATH;
    char file_name[max_path] = "";  // todo: long file paths

    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = DXWindow::Get().hwnd;
    ofn.lpstrFilter =
        "All Files\0*.*\0"
        "Gltf Files (*.gltf;*.glb)\0*.gltf;*.glb\0"
        "HDR Files (*.hdr)\0*.hdr\0\0";
    ofn.lpstrFile = file_name;
    ofn.nMaxFile = max_path;
    ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;

    if (GetOpenFileNameA(&ofn)) return file_name;

    return "";
}

std::string SaveFileDialog() {     // todo: modernize with IFileDialog
    const int max_path = 1024;     // MAX_PATH;
    char file_name[max_path] = "";  // todo: long file paths

    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = DXWindow::Get().hwnd;
    ofn.lpstrFilter =
        "All Files\0*.*\0"
        "PNG File (*.png)\0*.png\0"
        "HDR File (*.hdr)\0*.hdr\0\0";
    ofn.lpstrFile = file_name;
    ofn.nMaxFile = max_path;
    ofn.lpstrDefExt = "png";
    ofn.Flags = OFN_EXPLORER | OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;

    if (GetSaveFileNameA(&ofn)) return file_name;

    return "";
}

bool SliderUInt(
    const char* label, unsigned int* v, unsigned int v_min, unsigned int v_max, const char* format, ImGuiSliderFlags flags) {
    return ImGui::SliderScalar(label, ImGuiDataType_U32, v, &v_min, &v_max, format, flags);
}

bool InputUInt(const char* label, unsigned int* v, unsigned int step, unsigned int step_fast, ImGuiInputTextFlags flags) {
    const char* format = (flags & ImGuiInputTextFlags_CharsHexadecimal) ? "%08X" : "%d";
    return ImGui::InputScalar(label, ImGuiDataType_U32, (void*)v, (void*)(step > 0 ? &step : NULL),
        (void*)(step_fast > 0 ? &step_fast : NULL), format, flags);
}

void HelpTooltip(const char* msg) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) ImGui::SetTooltip("%s", msg);
}

void SetupImGuiStyle() {
    float main_scale = ImGui_ImplWin32_GetDpiScaleForMonitor(::MonitorFromPoint(POINT{0, 0}, MONITOR_DEFAULTTOPRIMARY));
    ImGui::StyleColorsDark();

    // Setup scaling
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);  // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing
                                      // this requires resetting Style + calling this again)
    style.FontScaleDpi = main_scale;  // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We
                                      // leave both here for documentation purpose)
    // Convert colors to be used in sRGB render target
    auto& colors = style.Colors;
    for (auto& col : colors) {
        col.x = srgb_to_linear(col.x);
        col.y = srgb_to_linear(col.y);
        col.z = srgb_to_linear(col.z);
    }
}

static void MaterialsSettingsUI(Viewer& viewer) {
    auto& model = viewer.get_model();
    const int materials_size = model.materials.size();
    static int current_material_index = 0;
    if (current_material_index >= materials_size) {
        current_material_index = 0;
    }
    bool settings_changed = false;
    imgui_combo("Choose Material:", model.materials_names, current_material_index);
    Material& current_material = model.materials[current_material_index];
    const Material& original_material = viewer.get_materials_backup()[current_material_index];

    settings_changed |=
        useTextureCheckbox("Use Albedo Texture", current_material.baseColorTextureIndex, original_material.baseColorTextureIndex);
    settings_changed |=
        useTextureCheckbox("Use Metallic-Roughness Texture", current_material.metallicRoughnessTextureIndex, original_material.metallicRoughnessTextureIndex);
    settings_changed |= 
        useTextureCheckbox("Use Normal Texture", current_material.normalTextureIndex, original_material.normalTextureIndex);
    settings_changed |=
        useTextureCheckbox("Use Transmission Texture", current_material.transmissionTextureIndex, original_material.transmissionTextureIndex);
    settings_changed |= 
        useTextureCheckbox("Use Emissive Texture", current_material.emissiveTextureIndex, original_material.emissiveTextureIndex);

    settings_changed |=
        ImGui::ColorEdit4("BaseColor F.", reinterpret_cast<float*>(&current_material.baseColorFactor), ImGuiColorEditFlags_Float);
    settings_changed |=
        ImGui::ColorEdit3("Emissive F.", reinterpret_cast<float*>(&current_material.emissiveFactor), ImGuiColorEditFlags_Float);
    settings_changed |= 
        ImGui::ColorEdit3("Attenuation F.", reinterpret_cast<float*>(&current_material.attenuationFactor), ImGuiColorEditFlags_Float);
    settings_changed |=
        ImGui::SliderFloat("Metallic F.", &current_material.metallicFactor, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_ClampOnInput);
    settings_changed |=
        ImGui::SliderFloat("Roughness F.", &current_material.roughnessFactor, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_ClampOnInput);
    settings_changed |= ImGui::SliderFloat("IOR", &current_material.ior, 0.001f, 5.0f, "%.3f", ImGuiSliderFlags_ClampOnInput);
    if (ImGui::Button("Reset Dielectric F0 from IOR")) {
        current_material.dielectric_f0 = f0_dielectric(current_material.ior);
        settings_changed = true;
    }
    settings_changed |=
        ImGui::SliderFloat("Dielectric F0", &current_material.dielectric_f0, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_ClampOnInput);
    settings_changed |= 
        ImGui::SliderFloat("Transmission F.", &current_material.transmisionFactor, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_ClampOnInput);
    settings_changed |= ImGui::SliderFloat("Emissive Strength F.", &current_material.emissiveStrength, 0.0f, 10.0f, "%.3f");
    settings_changed |= ImGui::Checkbox("Double Sided", &current_material.doubleSided);
    settings_changed |= ImGui::Checkbox("Has Volume", &current_material.hasVolume);
    settings_changed |= ImGui::Checkbox("Alpha Blending", &current_material.alphaBlending);
    settings_changed |= ImGui::SliderFloat("Alpha Cutoff F.", &current_material.alphaCutoff, -1.0f, 1.0f, "%.3f");
    if (ImGui::Button("Reset to original material")) {
        current_material = original_material;
        settings_changed = true;
    }
    if (settings_changed) {
        viewer.set_materials_updated();
    }
}

void RenderedImageUI(Viewer& viewer, const bool hardware_raytracing_support) {
    // Show window with image
    ImGuiIO& io = ImGui::GetIO();
    const ImGuiWindowFlags flags =  ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                    ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBackground |
                                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
                                    ImGuiWindowFlags_NoBringToFrontOnFocus;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    auto WorkSize = viewport->WorkSize;

    static float zoom_scale = 0.0f;
    static ImVec2 offset = {-WorkSize.x * 0.5f, -WorkSize.y * 0.5f};

    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);

    auto dims = viewer.get_window_dimensions();
    auto& framebuffer = viewer.get_framebuffer();

    if (!viewer.is_using_gpu_renderer()) {
        framebuffer.transition_from_srv_to_copy();
        framebuffer.upload_to_gpu();
        framebuffer.transition_from_copy_to_srv();
    }

    ImGui::Begin("Rendering Image", nullptr, flags);
    const float scroll_speed = 0.05f;
    float scale = std::exp(zoom_scale * scroll_speed);
    if (ImGui::IsWindowHovered()) {
        if (io.MouseWheel != 0.0f) {
            zoom_scale += io.MouseWheel;
            zoom_scale = std::max(0.0f, zoom_scale);
            scale = std::exp(zoom_scale * scroll_speed);
        }
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            ImVec2 drag_delta = io.MouseDelta;
            offset.x += drag_delta.x / scale;
            offset.y += drag_delta.y / scale;
        }
    }
    const auto texture_srv_gpu_handle = framebuffer.srv_gpu_handle;
    ImGui::SetCursorPos(ImVec2(scale * offset.x + WorkSize.x * 0.5f, scale * offset.y + WorkSize.y * 0.5f));
    // todo: hardware_ray_tracing_support check is a temporary fix for problem with integrated GPU and ImGui
    if (framebuffer.nearest_filtering && hardware_raytracing_support) {
        auto& platform_io = ImGui::GetPlatformIO();
        ImGui::GetWindowDrawList()->AddCallback(platform_io.DrawCallback_SetSamplerNearest);  // Set custom sampler
        ImGui::Image((ImTextureID)texture_srv_gpu_handle.ptr, ImVec2(scale * (float)dims.x, scale * (float)dims.y));
        ImGui::GetWindowDrawList()->AddCallback(platform_io.DrawCallback_ResetRenderState);  // Restore sampler
    } else {
        ImGui::Image((ImTextureID)texture_srv_gpu_handle.ptr, ImVec2(scale * (float)dims.x, scale * (float)dims.y));
    }
    ImGui::Text("size = %d x %d", dims.x, dims.y);

    ImGui::Text("zoom = %.2f", scale);
    ImGui::End();
    ImGui::PopStyleVar();
}

static void RenderingProgressUI(Viewer& viewer) {
    ImGuiIO& io = ImGui::GetIO();

    const auto rendering_state = viewer.get_rendering_state();
    float progress = viewer.get_render_progress();

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::Text("Render progress %.1f percent.", 100.f * progress);
    ImGui::ProgressBar(progress);
    if (rendering_state == RenderingState::Idle) {
        if (ImGui::Button("Render")) {
            viewer.async_start_render();
        }
    } else if (rendering_state == RenderingState::Rendering) {
        if (ImGui::Button("Stop Rendering")) {
            viewer.cancel_rendering();
        }
    } else {
        ImGui::Button("Stop Rendering");  // disabled button to prevent UI jumping
    }
    {
        bool continuous_rendering = viewer.continuous_rendering.load();
        if (ImGui::Checkbox("Continuous Rendering", &continuous_rendering)) {
            viewer.continuous_rendering = continuous_rendering;
        }
    }
    {
        bool iterative_rendering = viewer.iterative_rendering.load();
        if (ImGui::Checkbox("Iterative Rendering", &iterative_rendering)) {
            viewer.iterative_rendering = iterative_rendering;
            if (!iterative_rendering) {
                viewer.reset_iteration_counter();
            }
        }
        if (iterative_rendering) {
            ImGui::Text("Iterations: %d", viewer.get_iteration_counter());
        }
    }
    {
        ImGui::Checkbox("Nearest Filtering", &viewer.get_framebuffer().nearest_filtering);
    }
}

static void CameraUI(Viewer& viewer) {
    ImGuiIO& io = ImGui::GetIO();
    if (ImGui::TreeNode("Camera")) {
        int cameras_N = viewer.get_number_of_cameras();
        auto active_camera = viewer.get_active_camera();
        bool use_camera = active_camera.has_value();
        bool settings_changed = false;
        if (cameras_N != 0) {
            settings_changed = ImGui::Checkbox("Use Camera from Gltf file", &use_camera);
        }
        if (use_camera) {
            int new_active_camera_index = active_camera.has_value() ? static_cast<int>(active_camera.value()) : 0;
            bool camera_changed = ImGui::SliderInt("Active Camera", &new_active_camera_index, 0, cameras_N - 1, nullptr,
                ImGuiSliderFlags_ClampOnInput | ImGuiSliderFlags_NoInput);
            if (camera_changed || settings_changed) {
                viewer.set_active_camera(static_cast<uint32_t>(new_active_camera_index));
            }
        } else if (settings_changed) {
            viewer.set_active_camera(std::nullopt);
        }
        if (!use_camera) {
            auto near_far_values = viewer.get_near_far_camera_values();
            bool near_far_changed = false;
            near_far_changed |=
                ImGui::SliderFloat("Camera Z Near", &near_far_values.x, 0.0f, 1000.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
            near_far_changed |=
                ImGui::SliderFloat("Camera Z Far", &near_far_values.y, 0.0f, 1000.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
            if (near_far_changed) {
                viewer.set_near_far_camera_values(near_far_values.x, near_far_values.y);
            }
            // manual camera controls
            auto& yfov = viewer.get_yfov();
            auto euler_angles = glm::degrees(viewer.get_euler_angles_camera());
            static float camera_speed = 1.0f;
            static float camera_rotation_speed = 10.0f;
            bool transform_changed = false;
            transform_changed |= near_far_changed;
            transform_changed |=
                ImGui::DragFloat3("Camera Position", &viewer.position.x, camera_speed * 0.001f, 0.0f, 0.0f, "%.4f");
            transform_changed |=
                ImGui::DragFloat3("Camera Euler", &euler_angles.x, 0.1f, -180.0f, 180.0f, nullptr, ImGuiSliderFlags_WrapAround);
            transform_changed |=
                ImGui::SliderFloat("Camera Y fov", &yfov, 0.01f, 3.1415f, nullptr, ImGuiSliderFlags_ClampOnInput);
            ImGui::SliderFloat("Camera Speed", &camera_speed, 0.01f, 20.0f);
            ImGui::SliderFloat("Camera Rotation Speed", &camera_rotation_speed, 0.01f, 50.0f);
            {
                static bool fps_free_camera = false;
                ImGui::Checkbox("Use First Person free camera", &fps_free_camera);
                HelpTooltip("Use WASD and QE to move, Right Mouse button to rotate");
                if (fps_free_camera) {
                    float dT = io.DeltaTime;
                    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
                        transform_changed |= true;
                        ImVec2 drag_delta = io.MouseDelta;
                        euler_angles.x -= drag_delta.y * camera_rotation_speed * dT;
                        euler_angles.y -= drag_delta.x * camera_rotation_speed * dT;
                        euler_angles.x = glm::clamp(euler_angles.x, -85.0f, 85.0f);
                        euler_angles.z = 0.0;
                    }
                    if (ImGui::IsKeyDown(ImGuiKey_W)) {
                        viewer.position += viewer.direction * camera_speed * dT;
                        transform_changed |= true;
                    }
                    if (ImGui::IsKeyDown(ImGuiKey_A)) {
                        viewer.position -= viewer.right() * camera_speed * dT;
                        transform_changed |= true;
                    }
                    if (ImGui::IsKeyDown(ImGuiKey_S)) {
                        viewer.position -= viewer.direction * camera_speed * dT;
                        transform_changed |= true;
                    }
                    if (ImGui::IsKeyDown(ImGuiKey_D)) {
                        viewer.position += viewer.right() * camera_speed * dT;
                        transform_changed |= true;
                    }
                    if (ImGui::IsKeyDown(ImGuiKey_E)) {
                        viewer.position += viewer.up * camera_speed * dT;
                        transform_changed |= true;
                    }
                    if (ImGui::IsKeyDown(ImGuiKey_Q)) {
                        viewer.position -= viewer.up * camera_speed * dT;
                        transform_changed |= true;
                    }
                }
            }
            if (transform_changed) {
                euler_angles = glm::radians(euler_angles);
                auto quat = glm::quat(euler_angles);
                viewer.direction = quat * fvec3(0.0f, 0.0f, -1.0f);
                viewer.up = quat * fvec3(0.0f, 1.0f, 0.0f);
                viewer.snap_to_camera(false);
            }
        }
        ImGui::TreePop();
    }
}

static void RenderSettingsUI(Viewer& viewer, ConsoleArgs& console_arguments, std::vector<PendingDelete>& deferredDeletes,
    const bool hardware_raytracing_support) {
    if (hardware_raytracing_support) {
        static RendererMode renderer_mode = viewer.get_renderer_mode();
        bool renderer_changed = imgui_combo("Choose Renderer:", std::array{"CPU renderer", "GPU renderer"}, renderer_mode);
        if (renderer_changed) {
            viewer.switch_to_renderer(renderer_mode);
        }
    } else {
        ImGui::Text("GPU does not support raytracing, DXR rendering mode is unavailable");
    }
    if (!viewer.is_using_gpu_renderer()) {
        if (ImGui::Button("Clear image with black")) {
            viewer.clear_framebuffer_black();
        }
    }
    if (ImGui::Button("Save image")) {
        fs::path filepath = SaveFileDialog();
        if (!filepath.empty()) {
            save_render_image_timed_action(viewer, filepath);
            console_arguments.outputPath = filepath;
        }
    }
    if (ImGui::Button("Load Model/Envmap")) {
        fs::path filepath = OpenFileDialog();
        std::cout << "Selected file: " << filepath << '\n';
        if (filepath.extension() == ".hdr") {
            std::cout << "Loading new environment map file \n";
            CPUTexture<hdr_pixel> new_environment_texture = CPUTexture<hdr_pixel>(filepath);
            viewer.load_envmap(std::move(new_environment_texture));
            console_arguments.useDefaultEnv = false;
            console_arguments.environmentPath = filepath;
        } else if (filepath.extension() == ".gltf" || filepath.extension() == ".glb") {
            std::cout << "Loading Gltf model file \n";
            ModelLoader loader{};
            bool success = loader.load_from_file(filepath);
            if (success) {
                Model new_model = loader.construct_model();
                viewer.load_model(std::move(new_model));
                viewer.snap_to_camera();
                console_arguments.modelPath = filepath;
            } else {
                std::cout << "Failed to load model from " << filepath << '\n';
            }
        } else if (filepath.empty()) {
            std::cout << "No file selected.\n";
        } else {
            std::cout << "Unsupported file format: " << filepath.extension() << "\n";
        }
    }
    bool use_def_envmap_check_changed = ImGui::Checkbox("Use default Envmap", &console_arguments.useDefaultEnv);
    if (console_arguments.useDefaultEnv) {
        bool def_envmap_type_changed =
            imgui_combo("Default Envmap:", std::array{"Black", "White"}, console_arguments.defaultEnv);
        if (def_envmap_type_changed || use_def_envmap_check_changed) {
            CPUTexture<hdr_pixel> new_environment_texture = (console_arguments.defaultEnv == DefaultEnvironment::White)
                                                                ? CPUTexture<hdr_pixel>::create_white_texture()
                                                                : CPUTexture<hdr_pixel>::create_black_texture();
            viewer.load_envmap(std::move(new_environment_texture));
        }
    } else if (use_def_envmap_check_changed) {
        if (console_arguments.environmentPath.empty()) {
            console_arguments.useDefaultEnv = true;
        } else {
            // switched from using default envmap to custom, load from path
            CPUTexture<hdr_pixel> new_environment_texture = CPUTexture<hdr_pixel>(console_arguments.environmentPath);
            viewer.load_envmap(std::move(new_environment_texture));
        }
    }
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
    {
        static bool size_changed = false;
        size_changed |= InputUInt("Width", &console_arguments.windowWidth);
        size_changed |= InputUInt("Height", &console_arguments.windowHeight);

        static bool settings_changed = false;
        settings_changed |= SliderUInt("samples per pixel", &console_arguments.samplesPerPixel, 1, 128);
        HelpTooltip("For GPU raytracing best to use Iterative Rendering and set Samples per pixel = 1 ");
        settings_changed |= SliderUInt("max ray bounces", &console_arguments.maxRayBounces, 0, 10);
        settings_changed |= SliderUInt("max new rays per bounce", &console_arguments.maxNewRaysPerBounce, 0, 32);
        HelpTooltip("For AmbientOcclusion mode only, set >= 1");
        if (!viewer.is_using_gpu_renderer()) {
            settings_changed |= SliderUInt("max triangles per BVH leaf", &console_arguments.maxTrianglesPerBVHLeaf, 1, 32);
        }
        settings_changed |= ImGui::DragInt("environment rotation", &console_arguments.envmapRotation, 1.0f, 0, 360);
        HelpTooltip("environment rotation in degrees around UP axis.");
        settings_changed |=
            imgui_combo("Ray Program Mode:", std::array{"RayCaster", "AmbientOcclusion", "PBR"}, console_arguments.programMode);
        if (!viewer.is_using_gpu_renderer()) {
            settings_changed |=
                imgui_combo("Acceleration Struct Type:", std::array{"Naive", "BVH"}, console_arguments.accelStructType);
        }

        if (settings_changed || size_changed) {
            ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.0f, 0.6f, 0.6f));
            if (ImGui::Button("Update render settings")) {
                if (settings_changed) {
                    auto new_render_settings = RenderSettings{console_arguments};
                    viewer.set_render_settings(new_render_settings);
                    settings_changed = false;
                }
                if (size_changed) {
                    auto& framebuffer = viewer.get_framebuffer();
                    deferredDeletes.emplace_back(framebuffer.get_gpu_resource());
                    deferredDeletes.emplace_back(framebuffer.get_gpu_upload_resource());
                    viewer.resize_window(ivec2(console_arguments.windowWidth, console_arguments.windowHeight), true);
                    viewer.snap_to_camera();
                    viewer.clear_framebuffer_black();
                    size_changed = false;
                }
            }
            ImGui::PopStyleColor();
        }
    }
    ImGui::PopItemWidth();
}

void OptionsWindowUI(Viewer& viewer, ConsoleArgs& console_arguments, std::vector<PendingDelete>& deferredDeletes,
    const bool hardware_raytracing_support) {
    ImGui::Begin("Options", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    RenderingProgressUI(viewer);

    ImGui::Separator();

    CameraUI(viewer);

    ImGui::Separator();
    static bool show_material_settings = false;
    if (ImGui::TreeNode("Materials")) {
        if (ImGui::Button("Edit Materials")) {
            show_material_settings = true;
        }
        ImGui::TreePop();
    }
    ImGui::Separator();
    if (viewer.get_rendering_state() == RenderingState::Idle) {
        RenderSettingsUI(viewer, console_arguments, deferredDeletes, hardware_raytracing_support);
    } else {
        ImGui::Text("Stop rendering process to access rendering options");
    }
    ImGui::End();
    // Show materials settings window
    if (show_material_settings) {
        ImGui::Begin("Materials Options", &show_material_settings, ImGuiWindowFlags_AlwaysAutoResize);
        MaterialsSettingsUI(viewer);
        ImGui::End();
    }
}

}  // namespace app
