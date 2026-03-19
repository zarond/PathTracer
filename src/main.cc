#include <chrono>
#include <iostream>

#ifndef NO_WINDOWS
#include <thread>
#include <stop_token>
#include <atomic>
#include <mutex>
#include <glm/ext.hpp>
#endif

#include "arguments.h"
#include "cpu_framebuffer.h"
#include "model_loader.h"
#include "viewer.h"

#ifndef NO_WINDOWS
#define NOMINMAX
#include <imgui.h>
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_dx12.h"

#include "d3d_debug_layer.h"
#include "d3d_context.h"

#include "ui.h"

#include "DXR/GPU_model.h"
#include "DXR/GPU_pipeline.h"

#endif

int main(int argc, char* argv[]) {
    using namespace app;

    ConsoleArgs console_arguments = parse_args(argc, argv, fs::current_path());
    if (console_arguments.exitImmediately) {
        return 1;
    }

    auto start = std::chrono::high_resolution_clock::now();
    Model model;
    {
        ModelLoader loader{};
        bool success = loader.loadFromFile(console_arguments.modelPath);
        if (success ) {
            model = loader.constructModel();
        } else {
            std::cerr << "Failed to load model from " << console_arguments.modelPath << '\n';
            if (console_arguments.noGui) {
                return 1;
            }
        }
    }
    CPUTexture<hdr_pixel> environment_texture;
    if (console_arguments.useDefaultEnv) {
        environment_texture = (console_arguments.defaultEnv == DefaultEnvironment::White)
                                  ? CPUTexture<hdr_pixel>::create_white_texture()
                                  : CPUTexture<hdr_pixel>::create_black_texture();
    } else {
        environment_texture = CPUTexture<hdr_pixel>(console_arguments.environmentPath);
    }

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "loaded in " << diff.count() << " ms." << '\n';

    auto render_settings = RenderSettings{console_arguments};

    // Create viewer
    Viewer viewer(std::move(model), std::move(environment_texture), render_settings);
    viewer.resize_window(ivec2(console_arguments.windowWidth, console_arguments.windowHeight));
    viewer.snap_to_camera();

    auto render_lambda = [&viewer]() {
        auto start = std::chrono::high_resolution_clock::now();
        viewer.render();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
        std::cout << "rendered in " << diff.count() << " ms." << '\n';
    };

    auto save_render_image_lambda = [&viewer](fs::path image_path) {
        auto start = std::chrono::high_resolution_clock::now();
        viewer.take_snapshot(image_path);
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
        std::cout << "output saved in " << diff.count() << " ms." << '\n';
    };

    if (console_arguments.noGui) {
        render_lambda();
        save_render_image_lambda(console_arguments.outputPath);
        return 0;
    }

#ifndef NO_WINDOWS
    //------------------------------------------------------------------
    // GUI version logic starts from here
    //------------------------------------------------------------------
    std::stop_source finish_worker_thread_source;
    std::stop_token finish_worker_thread_token = finish_worker_thread_source.get_token();
    auto render_worker_lambda = [&render_lambda, &viewer](std::stop_token stop) {
        std::mutex mtx_render;
        std::unique_lock<std::mutex> lock(mtx_render);
        while (!stop.stop_requested()) {
            if (viewer.get_rendering_state() == Renderer::ReadyToStart) {
                render_lambda();
                if (viewer.continuous_rendering) {
                    continue;
                }
            }
            viewer.cv_render.wait(lock, [&stop, &viewer]() 
                { return stop.stop_requested() || (viewer.get_rendering_state() == Renderer::ReadyToStart); });
        }
    };

    viewer.clear_framebuffer_black();
    viewer.async_start_render();
    std::thread worker_thread(render_worker_lambda, finish_worker_thread_token);

    DXDebugLayer::Get().Init();

    // Make process DPI aware and obtain main monitor scale
    ImGui_ImplWin32_EnableDpiAwareness();
    float main_scale = ImGui_ImplWin32_GetDpiScaleForMonitor(::MonitorFromPoint(POINT{0, 0}, MONITOR_DEFAULTTOPRIMARY));

    // Create application window
    DXWindow& dx_window = DXWindow::Get();
    dx_window.Init();
    assert(dx_window.hwnd != nullptr);

    // Initialize Direct3D
    D3DContext& d3d_ctx = D3DContext::Get();
    if (!d3d_ctx.CreateDeviceD3D(dx_window.hwnd)) {
        d3d_ctx.CleanupDeviceD3D();
        dx_window.ShutDown();
        return 1;
    }
    DXDebugLayer::Get().SetBreakOnSeverity(*d3d_ctx.g_pd3dDevice.Get());

    // Show the window
    ::ShowWindow(dx_window.hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(dx_window.hwnd);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = NULL;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup scaling
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(main_scale);  // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing
                                      // this requires resetting Style + calling this again)
    style.FontScaleDpi = main_scale;  // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We
                                      // leave both here for documentation purpose)

    // Setup Platform/Renderer backends
    ImGui_ImplWin32_Init(dx_window.hwnd);
    ImGui_ImplDX12_InitInfo init_info = {};
    init_info.Device = d3d_ctx.g_pd3dDevice.Get();
    init_info.CommandQueue = d3d_ctx.g_pd3dCommandQueue.Get();
    init_info.NumFramesInFlight = APP_NUM_FRAMES_IN_FLIGHT;
    init_info.RTVFormat = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    init_info.DSVFormat = DXGI_FORMAT_UNKNOWN;
    // Allocating SRV descriptors (for textures) is up to the application, so we provide callbacks.
    // (current version of the backend will only allocate one descriptor, future versions will need to allocate more)
    init_info.SrvDescriptorHeap = d3d_ctx.g_pd3dSrvDescHeap.Get();
    init_info.SrvDescriptorAllocFn = [](ImGui_ImplDX12_InitInfo*, D3D12_CPU_DESCRIPTOR_HANDLE* out_cpu_handle,
                                         D3D12_GPU_DESCRIPTOR_HANDLE* out_gpu_handle) {
        return D3DContext::Get().g_pd3dSrvDescHeapAlloc.Alloc(out_cpu_handle, out_gpu_handle);
    };
    init_info.SrvDescriptorFreeFn = [](ImGui_ImplDX12_InitInfo*, D3D12_CPU_DESCRIPTOR_HANDLE cpu_handle,
                                        D3D12_GPU_DESCRIPTOR_HANDLE gpu_handle) {
        return D3DContext::Get().g_pd3dSrvDescHeapAlloc.Free(cpu_handle, gpu_handle);
    };
    ImGui_ImplDX12_Init(&init_info);

    const float clear_color[] = {0.45f, 0.55f, 0.60f, 1.00f};

    std::unique_ptr<GPU_model> gpu_model;
    std::unique_ptr<GPU_pipeline> gpu_pipeline;
    {
        // Init time GPU instructions
        FrameContext* frameCtx = d3d_ctx.WaitForNextFrameContext();
        d3d_ctx.InitCommandList(*frameCtx->CommandAllocator.Get());
        
        gpu_model = std::make_unique<GPU_model>(viewer.get_model());

        gpu_pipeline = std::make_unique<GPU_pipeline>(viewer.get_NDC2WorldMatrix(), xyz1(viewer.position_));

        d3d_ctx.DispatchCommandList();
        d3d_ctx.WaitForPendingOperations();
    }


    // Main loop
    while (!dx_window.close_window) {
        // Poll and handle messages (inputs, window resize, etc.)
        dx_window.Update();
        if (dx_window.close_window) break;
        // Handle window screen locked / minimized or out of view
        if ((d3d_ctx.g_SwapChainOccluded && d3d_ctx.g_pSwapChain->Present(0, DXGI_PRESENT_TEST) == DXGI_STATUS_OCCLUDED) ||
            ::IsIconic(dx_window.hwnd)) {
            ::Sleep(10);
            continue;
        }
        d3d_ctx.g_SwapChainOccluded = false;

        // Start the Dear ImGui frame
        ImGui_ImplDX12_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // Start command list recording
        FrameContext* frameCtx = d3d_ctx.WaitForNextFrameContext();
        UINT backBufferIdx = d3d_ctx.g_pSwapChain->GetCurrentBackBufferIndex();
        d3d_ctx.InitCommandList(*frameCtx->CommandAllocator.Get());

        // Show window with image
        {
            const ImGuiWindowFlags flags =
                ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
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
            framebuffer.upload_to_gpu();

            ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.g_pd3dSrvDescHeap.Get()};
            d3d_ctx.g_pd3dCommandList->SetDescriptorHeaps(1, desc_heap);
            framebuffer.transition_from_srv_to_uav(); // remove, just testing now
            
            gpu_pipeline->DoRaytracing(*gpu_model, framebuffer, viewer.get_NDC2WorldMatrix(), xyz1(viewer.position_));
            
            framebuffer.transition_from_uav_to_srv(); //
            const auto& texture_srv_gpu_handle = framebuffer.srv_gpu_handle;
            ImGui::Begin("Rendering Image", nullptr, flags);
            const float scroll_speed = 0.05f;
            float scale = std::exp(zoom_scale * scroll_speed);
            if (ImGui::IsWindowHovered()) {
                if (io.MouseWheel != 0.0f){
                    zoom_scale += io.MouseWheel;
                    zoom_scale = max(0.0f, zoom_scale);
                    scale = std::exp(zoom_scale * scroll_speed);
                }
                if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                    ImVec2 drag_delta = io.MouseDelta;
                    offset.x += drag_delta.x / scale;
                    offset.y += drag_delta.y / scale; 
                }
            }
            ImGui::SetCursorPos(
                ImVec2(scale * offset.x + WorkSize.x * 0.5f, scale * offset.y + WorkSize.y * 0.5f));
            if (framebuffer.nearest_filtering) {
                ImGui::GetWindowDrawList()->AddCallback(ImDrawCallback_ImplDX12_SetSamplerNearest, nullptr); // Set custom sampler
                ImGui::Image((ImTextureID)texture_srv_gpu_handle.ptr, ImVec2(scale * (float)dims.x, scale * (float) dims.y));
                ImGui::GetWindowDrawList()->AddCallback(ImDrawCallback_ImplDX12_SetSamplerLinear, nullptr);  // Restore sampler
            } else {
                ImGui::Image((ImTextureID)texture_srv_gpu_handle.ptr, ImVec2(scale * (float)dims.x, scale * (float)dims.y));
            }
            ImGui::Text("size = %d x %d", dims.x, dims.y);

            ImGui::Text("zoom = %.2f", scale);
            ImGui::End();
            ImGui::PopStyleVar();
        }
        const auto rendering_state = viewer.get_rendering_state();
        // Show options window
        ImGui::Begin("Options", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        {
            float progress = viewer.get_render_progress();

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::Text("Render progress %.1f percent.", 100.f * progress);
            ImGui::ProgressBar(progress);
            if (rendering_state == Renderer::RenderingState::Idle) {
                if (ImGui::Button("Render")) {
                    viewer.async_start_render();
                }
            } else if (rendering_state == Renderer::RenderingState::Rendering) {
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
        ImGui::Separator();
        if (ImGui::TreeNode("Camera")) {
            int cameras_N = viewer.get_number_of_cameras();
            auto active_camera = viewer.get_active_camera();
            bool use_camera = active_camera.has_value();
            bool setings_changed = false;
            if (cameras_N != 0) {
                setings_changed = ImGui::Checkbox("Use Camera from Gltf file", &use_camera);
            }
            if (use_camera) {
                int new_acive_camera_index = active_camera.has_value() ? static_cast<int>(active_camera.value()) : 0;
                bool camera_changed = ImGui::SliderInt("Active Camera", &new_acive_camera_index, 0, cameras_N - 1, nullptr,
                    ImGuiSliderFlags_ClampOnInput | ImGuiSliderFlags_NoInput);
                if (camera_changed || setings_changed) {
                    viewer.set_active_camera(static_cast<uint32_t>(new_acive_camera_index));
                }
            } else if (setings_changed) {
                viewer.set_active_camera(std::nullopt);
            }
            if (!use_camera) {
                // manual camera controls
                auto& yfov = viewer.get_yfov();
                auto euler_angles = glm::degrees(viewer.get_euler_angles_camera()); 
                static float camera_speed = 1.0f;
                bool transform_changed = false;
                transform_changed |= ImGui::DragFloat3("Camera Position", &viewer.position_.x, camera_speed * 0.001f, 0.0f, 0.0f, "%.4f");
                transform_changed |= ImGui::DragFloat3("Camera Euler", &euler_angles.x, 0.1f, -180.0f, 180.0f, nullptr, ImGuiSliderFlags_WrapAround);
                transform_changed |= ImGui::SliderFloat("Camera Y fov", &yfov, 0.01f, 3.1415f, nullptr, ImGuiSliderFlags_ClampOnInput);
                ImGui::SliderFloat("Camera Speed", &camera_speed, 0.01f, 10.0f);
                if (transform_changed) {
                    euler_angles = glm::radians(euler_angles);
                    auto quat = glm::quat(euler_angles);
                    viewer.direction_ = quat * fvec3(0.0f, 0.0f, -1.0f);
                    viewer.up_ = quat * fvec3(0.0f, 1.0f, 0.0f);
                    viewer.snap_to_camera(false);
                }
            }
            ImGui::TreePop();
        }
        ImGui::Separator();
        if (rendering_state == Renderer::RenderingState::Idle)
        {
            if (ImGui::Button("Clear image with black")) {
                viewer.clear_framebuffer_black();
            }
            if (ImGui::Button("Save image")) {
                fs::path filepath = SaveFileDialog();
                if (!filepath.empty()) {
                    save_render_image_lambda(filepath);
                    console_arguments.outputPath = filepath;
                }
            }
            if (ImGui::Button("Load Model/Envmap")) {
                fs::path filepath = OpenFileDialog();
                std::cout << "Selected file: " << filepath << '\n';
                if (filepath.extension() == ".hdr") {
                    std::cout << "Loading new environment map file " << std::endl;
                    CPUTexture<hdr_pixel> new_environment_texture = CPUTexture<hdr_pixel>(filepath);
                    viewer.load_envmap(std::move(new_environment_texture));
                    console_arguments.useDefaultEnv = false;
                    console_arguments.environmentPath = filepath;
                } else if (filepath.extension() == ".gltf" || filepath.extension() == ".glb") {
                    std::cout << "Loading Gltf model file " << std::endl;
                    ModelLoader loader{};
                    bool success = loader.loadFromFile(filepath);
                    if (success) {
                        Model new_model = loader.constructModel();
                        viewer.load_model(std::move(new_model));
                        viewer.snap_to_camera();
                        console_arguments.modelPath = filepath;
                    } else {
                        std::cout << "Failed to load model from " << filepath << '\n';
                    }
                } else if (filepath.empty()) {
                    std::cout << "No file selected." << std::endl;
                } else {
                    std::cout << "Unsupported file format: " << filepath.extension() << std::endl;
                }
            }
            bool use_def_envmap_check_changed = ImGui::Checkbox("Use default Envmap", &console_arguments.useDefaultEnv);
            if (console_arguments.useDefaultEnv) {
                bool def_envmap_type_changed =
                    imgui_combo("Default Envmap:", std::array{"Black", "White"}, console_arguments.defaultEnv);
                if (def_envmap_type_changed || use_def_envmap_check_changed) {
                    CPUTexture<hdr_pixel> new_environment_texture = 
                        (console_arguments.defaultEnv == DefaultEnvironment::White)
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

                static bool setings_changed = false;
                setings_changed |= SliderUInt("samples per pixel", &console_arguments.samplesPerPixel, 1, 128);
                setings_changed |= SliderUInt("max ray bounces", &console_arguments.maxRayBounces, 0, 10);
                setings_changed |= SliderUInt("max new rays per bounce", &console_arguments.maxNewRaysPerBounce, 0, 32);
                HelpTooltip("For AmbientOcclusion mode only, set >= 1");
                setings_changed |= SliderUInt("max triangles per BVH leaf", &console_arguments.maxTrianglesPerBVHLeaf, 1, 32);
                setings_changed |= ImGui::DragInt(
                    "environment rotation", &console_arguments.envmapRotation, 1.0f, 0, 360);
                HelpTooltip("environment rotation in degrees around UP axis.");
                setings_changed |= imgui_combo(
                    "Ray Program Mode:", std::array{"RayCaster", "AmbientOcclusion", "PBR"}, console_arguments.programMode);
                setings_changed |= imgui_combo("Acceleration Struct Type:", std::array{"Naive", "BVH"}, console_arguments.accelStructType);

                if (setings_changed || size_changed) {
                    ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.0f, 0.6f, 0.6f));
                    if (ImGui::Button("Update render settings")) {
                        if (setings_changed) {
                            auto new_render_settings = RenderSettings{console_arguments};
                            viewer.set_render_settings(new_render_settings);
                            setings_changed = false;
                        }
                        if (size_changed) {
                            viewer.resize_window(ivec2(console_arguments.windowWidth, console_arguments.windowHeight));
                            viewer.snap_to_camera();
                            viewer.clear_framebuffer_black();
                            size_changed = false;
                        }
                    }
                    ImGui::PopStyleColor();
                }
            }
            ImGui::PopItemWidth();
        } else {
            ImGui::Text("Stop rendering process to access rendering options");
        }
        ImGui::End();
        // Rendering ImGui
        ImGui::Render();

        // Change resource state, begin frame
        D3D12_RESOURCE_BARRIER barrier = {};
        barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
        barrier.Transition.pResource = d3d_ctx.g_mainRenderTargetResource[backBufferIdx].Get();
        barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
        d3d_ctx.g_pd3dCommandList->ResourceBarrier(1, &barrier);

        d3d_ctx.g_pd3dCommandList->ClearRenderTargetView(
            d3d_ctx.g_mainRenderTargetDescriptor[backBufferIdx], clear_color, 0, nullptr);

        d3d_ctx.g_pd3dCommandList->OMSetRenderTargets(1, &d3d_ctx.g_mainRenderTargetDescriptor[backBufferIdx], false, nullptr);
        
        // Render Dear ImGui graphics
        ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.g_pd3dSrvDescHeap.Get()};
        d3d_ctx.g_pd3dCommandList->SetDescriptorHeaps(1, desc_heap);
        ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), d3d_ctx.g_pd3dCommandList.Get());
        
        // end frame, change resource state
        viewer.get_framebuffer().transition_from_srv_to_copy();
        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
        d3d_ctx.g_pd3dCommandList->ResourceBarrier(1, &barrier);

        d3d_ctx.DispatchCommandList();
        d3d_ctx.g_pd3dCommandQueue->Signal(d3d_ctx.g_fence.Get(), ++d3d_ctx.g_fenceLastSignaledValue);
        frameCtx->FenceValue = d3d_ctx.g_fenceLastSignaledValue;

        // Present
        HRESULT hr = d3d_ctx.g_pSwapChain->Present(1, 0);  // Present with vsync
        // HRESULT hr = g_pSwapChain->Present(0, g_SwapChainTearingSupport ? DXGI_PRESENT_ALLOW_TEARING : 0); // Present without
        // vsync
        d3d_ctx.g_SwapChainOccluded = (hr == DXGI_STATUS_OCCLUDED);
        d3d_ctx.g_frameIndex++;
    }

    d3d_ctx.WaitForPendingOperations();
    d3d_ctx.WaitForPending—opy();

    // Cleanup
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    d3d_ctx.CleanupDeviceD3D();
    dx_window.ShutDown();

    DXDebugLayer::Get().Shutdown();

    finish_worker_thread_source.request_stop();
    viewer.cancel_rendering();
    worker_thread.join();

#endif  // #ifndef NO_WINDOWS
    return 0;
}
