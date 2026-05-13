#include <cassert>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

#ifdef WINDOWS_SPECIFIC
#include <stop_token>
#include <thread>
#endif

#include "arguments.h"
#include "cpu_framebuffer.h"
#include "model_loader.h"
#include "render_settings.h"
#include "renderer.h"
#include "viewer.h"

#ifdef WINDOWS_SPECIFIC
#define NOMINMAX
#include <imgui.h>

#include "backends/imgui_impl_dx12.h"
#include "backends/imgui_impl_win32.h"

#include "d3d_context.h"
#include "d3d_debug_layer.h"

#include "ui.h"

#endif

int main(int argc, char* argv[]) {
    using namespace app;
    namespace fs = std::filesystem;

    ConsoleArgs console_arguments = parse_args(argc, argv, fs::current_path());
    if (console_arguments.exitImmediately) {
        return 1;
    }

    auto start = std::chrono::high_resolution_clock::now();
    Model model;
    {
        ModelLoader loader{};
        bool success = loader.load_from_file(console_arguments.modelPath);
        if (success) {
            model = loader.construct_model();
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
        static auto start = std::chrono::high_resolution_clock::now();
        const auto iteration_counter = viewer.get_iteration_counter();
        if (!viewer.iterative_rendering || iteration_counter == 1) {
            start = std::chrono::high_resolution_clock::now();
        }
        viewer.render();
        if (!viewer.iterative_rendering) {
            auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
            float time_ms = static_cast<float>(diff.count()) / 1000.0f;
            std::cout << "rendered in " << std::fixed << std::setprecision(2) << time_ms << " ms." << '\n';
        }
        if (viewer.iterative_rendering && viewer.continuous_rendering && (iteration_counter % 50 == 0)) {
            auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
            float time_ms = static_cast<float>(diff.count()) / 1000.0f;
            float avg_time_ms = static_cast<float>(diff.count()) / (iteration_counter * 1000.0f);
            std::cout << "rendered " << iteration_counter << " iteration in " << std::fixed << std::setprecision(2) << time_ms << " ms. ";
            std::cout << "Average " << std::fixed << std::setprecision(2) << avg_time_ms << " ms. per iteration. " << '\n';
        }
    };

    if (console_arguments.noGui) {
        render_lambda();
        save_render_image_timed_action(viewer, console_arguments.outputPath);
        return 0;
    }

#ifdef WINDOWS_SPECIFIC
    //------------------------------------------------------------------
    // GUI version logic starts from here
    //------------------------------------------------------------------
    std::stop_source finish_worker_thread_source;
    std::stop_token finish_worker_thread_token = finish_worker_thread_source.get_token();
    auto render_worker_lambda = [&render_lambda, &viewer](std::stop_token stop) {
        while (!stop.stop_requested()) {
            if (viewer.get_rendering_state() == RenderingState::ReadyToStart) {
                render_lambda();
                if (viewer.continuous_rendering) {
                    continue;
                }
            }
            viewer.wait_for_render_start(stop);
        }
    };

    viewer.clear_framebuffer_black();
    viewer.async_start_render();
    std::thread worker_thread(render_worker_lambda, finish_worker_thread_token);

    DXDebugLayer::Get().Init();

    // Make process DPI aware and obtain main monitor scale
    ImGui_ImplWin32_EnableDpiAwareness();

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
    DXDebugLayer::Get().SetBreakOnSeverity(*d3d_ctx.m_d3dDevice.Get());

    // Show the window
    ::ShowWindow(dx_window.hwnd, SW_SHOWDEFAULT);
    ::UpdateWindow(dx_window.hwnd);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

    SetupImGuiStyle();

    // Setup Platform/Renderer backends
    ImGui_ImplWin32_Init(dx_window.hwnd);
    ImGui_ImplDX12_InitInfo init_info = {};
    init_info.Device = d3d_ctx.m_d3dDevice.Get();
    init_info.CommandQueue = d3d_ctx.m_CommandQueue.Get();
    init_info.NumFramesInFlight = APP_NUM_FRAMES_IN_FLIGHT;
    init_info.RTVFormat = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    init_info.DSVFormat = DXGI_FORMAT_UNKNOWN;
    // Allocating SRV descriptors (for textures) is up to the application, so we provide callbacks.
    // (current version of the backend will only allocate one descriptor, future versions will need to allocate more)
    init_info.SrvDescriptorHeap = d3d_ctx.m_SrvDescHeap.Get();
    init_info.SrvDescriptorAllocFn = [](ImGui_ImplDX12_InitInfo*, D3D12_CPU_DESCRIPTOR_HANDLE* out_cpu_handle,
                                         D3D12_GPU_DESCRIPTOR_HANDLE* out_gpu_handle) {
        return D3DContext::Get().m_SrvDescHeapAlloc.Alloc(out_cpu_handle, out_gpu_handle);
    };
    init_info.SrvDescriptorFreeFn = [](ImGui_ImplDX12_InitInfo*, D3D12_CPU_DESCRIPTOR_HANDLE cpu_handle,
                                        D3D12_GPU_DESCRIPTOR_HANDLE gpu_handle) {
        return D3DContext::Get().m_SrvDescHeapAlloc.Free(cpu_handle, gpu_handle);
    };
    ImGui_ImplDX12_Init(&init_info);

    const float clear_color[] = {0.17f, 0.27f, 0.33f, 1.00f};

    // Init time GPU instructions
    {
        start = std::chrono::high_resolution_clock::now();

        d3d_ctx.InitDXRCommandList();

        viewer.init_GPU_renderer();

        d3d_ctx.DispatchDXRCommandList();
        d3d_ctx.WaitForPendingDXR();

        diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
        std::cout << "GPU renderer initialized in " << diff.count() << " ms." << '\n';
    }

    std::vector<PendingDelete> deferredDeletes;

    // Main loop
    while (!dx_window.close_window) {
        // Poll and handle messages (inputs, window resize, etc.)
        dx_window.Update();
        if (dx_window.close_window) break;
        // Handle window screen locked / minimized or out of view
        if ((d3d_ctx.m_SwapChainOccluded && d3d_ctx.m_SwapChain->Present(0, DXGI_PRESENT_TEST) == DXGI_STATUS_OCCLUDED) ||
            ::IsIconic(dx_window.hwnd)) {
            ::Sleep(10);
            continue;
        }
        d3d_ctx.m_SwapChainOccluded = false;

        // Start the Dear ImGui frame
        ImGui_ImplDX12_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // Start command list recording
        FrameContext* frameCtx = d3d_ctx.WaitForNextFrameContext();
        UINT backBufferIdx = d3d_ctx.m_SwapChain->GetCurrentBackBufferIndex();
        d3d_ctx.InitCommandList(*frameCtx->CommandAllocator.Get());

        // Custom UI
        RenderedImageUI(viewer, d3d_ctx.hardware_ray_tracing_support);
        OptionsWindowUI(viewer, console_arguments, deferredDeletes, d3d_ctx.hardware_ray_tracing_support);

        // Rendering ImGui
        ImGui::Render();

        // Change resource state, begin frame
        D3D12_RESOURCE_BARRIER barrier = {};
        barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
        barrier.Transition.pResource = d3d_ctx.m_mainRenderTargetResource[backBufferIdx].Get();
        barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;
        d3d_ctx.m_CommandList->ResourceBarrier(1, &barrier);

        d3d_ctx.m_CommandList->ClearRenderTargetView(
            d3d_ctx.m_mainRenderTargetDescriptor[backBufferIdx], clear_color, 0, nullptr);

        d3d_ctx.m_CommandList->OMSetRenderTargets(1, &d3d_ctx.m_mainRenderTargetDescriptor[backBufferIdx], false, nullptr);

        // Render Dear ImGui graphics
        ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()};
        d3d_ctx.m_CommandList->SetDescriptorHeaps(1, desc_heap);
        ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), d3d_ctx.m_CommandList.Get());

        // end frame, change resource state
        barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
        barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;
        d3d_ctx.m_CommandList->ResourceBarrier(1, &barrier);

        d3d_ctx.DispatchCommandList();
        d3d_ctx.m_CommandQueue->Signal(d3d_ctx.m_fence.Get(), ++d3d_ctx.m_fenceLastSignaledValue);
        frameCtx->FenceValue = d3d_ctx.m_fenceLastSignaledValue;

        // Present
        HRESULT hr = d3d_ctx.m_SwapChain->Present(1, 0);  // Present with vsync
        // HRESULT hr = m_SwapChain->Present(0, g_SwapChainTearingSupport ? DXGI_PRESENT_ALLOW_TEARING : 0); // Present without vsync
        d3d_ctx.m_SwapChainOccluded = (hr == DXGI_STATUS_OCCLUDED);
        d3d_ctx.m_frameIndex++;

        // Safe delete no longer used resources
        if (deferredDeletes.size() > 0) {
            d3d_ctx.WaitForPendingOperations();
            d3d_ctx.WaitForPendingCopy();
            auto& pendingDelete = deferredDeletes.front();
            pendingDelete.resource.Reset();
            deferredDeletes.erase(deferredDeletes.begin());
        }
    }
    finish_worker_thread_source.request_stop();
    viewer.cancel_rendering();
    worker_thread.join();

    d3d_ctx.WaitForPendingOperations();
    d3d_ctx.WaitForPendingCopy();
    d3d_ctx.WaitForPendingDXR();

    // Cleanup
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    d3d_ctx.CleanupDeviceD3D();
    dx_window.ShutDown();

    DXDebugLayer::Get().Shutdown(); // reports false positives because viewer destructor is not yet called
#endif  // #ifdef WINDOWS_SPECIFIC
    return 0;
}
