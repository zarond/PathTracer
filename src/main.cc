#include <chrono>
#include <iostream>
#include <thread>

#include "arguments.h"
#include "cpu_framebuffer.h"
#include "model_loader.h"
#include "viewer.h"

#define NOMINMAX

#include <imgui.h>
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_dx12.h"

#include "d3d_debug_layer.h"
#include "d3d_context.h"
#include "ui.h"

int main(int argc, char* argv[]) {
    using namespace app;
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
    init_info.RTVFormat = DXGI_FORMAT_R16G16B16A16_FLOAT;
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

    ConsoleArgs console_arguments = parse_args(argc, argv, fs::current_path());
    if (console_arguments.exitImmediately) {
        return 1;
    }

    auto start = std::chrono::high_resolution_clock::now();
    Model model;
    {
        ModelLoader loader{};
        auto success = loader.loadFromFile(console_arguments.modelPath);
        if (!success) {
            std::cerr << "Failed to load model from " << console_arguments.modelPath << '\n';
            return 1;
        }
        model = loader.constructModel();
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

    std::thread worker_thread([&viewer, console_arguments]() {
        auto start = std::chrono::high_resolution_clock::now();
        viewer.render();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
        std::cout << "rendered in " << diff.count() << " ms." << '\n';
    });

    const float clear_color[] = {0.45f, 0.55f, 0.60f, 1.00f};

    // Main loop
    while (!dx_window.close_window) {
        // Poll and handle messages (inputs, window resize, etc.)
        // See the WndProc() function below for our to dispatch events to the Win32 backend.
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

        // Show window with progress
        {
            float progress = viewer.get_render_progress();

            ImGui::Begin("Hello, world!");
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::Text("Render progress %.1f percent.", 100.f * progress);
            ImGui::ProgressBar(progress);
            if (ImGui::Button("Save image")) {
                start = std::chrono::high_resolution_clock::now();
                viewer.take_snapshot(console_arguments.outputPath);
                diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
                std::cout << "output saved in " << diff.count() << " ms." << '\n';
            }
            ImGui::End();
        }
        // Show window with image
        {
            auto dims = viewer.get_window_dimensions();
            viewer.get_framebuffer().upload_to_gpu();
            const auto& texture_srv_gpu_handle = viewer.get_framebuffer().srv_gpu_handle;
            ImGui::Begin("Rendering Image");
            ImGui::Text("size = %d x %d", dims.x, dims.y);
            ImGui::Image((ImTextureID)texture_srv_gpu_handle.ptr, ImVec2((float)dims.x, (float)dims.y));
            ImGui::End();
        }
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
        viewer.get_framebuffer().transition_back_for_copy();

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

    worker_thread.join();

    return 0;
}
