#include "d3d_context.h"
#include <iostream>
#include <cassert>

namespace app {

void ExampleDescriptorHeapAllocator::Create(ID3D12Device* device, ID3D12DescriptorHeap* heap) {
    assert(Heap == nullptr && FreeIndices.empty());
    Heap = heap;
    D3D12_DESCRIPTOR_HEAP_DESC desc = heap->GetDesc();
    HeapType = desc.Type;
    HeapStartCpu = Heap->GetCPUDescriptorHandleForHeapStart();
    HeapStartGpu = Heap->GetGPUDescriptorHandleForHeapStart();
    HeapHandleIncrement = device->GetDescriptorHandleIncrementSize(HeapType);
    FreeIndices.reserve((int)desc.NumDescriptors);
    for (int n = desc.NumDescriptors; n > 0; n--) FreeIndices.push_back(n - 1);
}
void ExampleDescriptorHeapAllocator::Destroy() {
    Heap = nullptr;
    FreeIndices.clear();
}
void ExampleDescriptorHeapAllocator::Alloc(
    D3D12_CPU_DESCRIPTOR_HANDLE* out_cpu_desc_handle, D3D12_GPU_DESCRIPTOR_HANDLE* out_gpu_desc_handle) {
    assert(FreeIndices.size() > 0);
    int idx = FreeIndices.back();
    FreeIndices.pop_back();
    out_cpu_desc_handle->ptr = HeapStartCpu.ptr + (idx * HeapHandleIncrement);
    out_gpu_desc_handle->ptr = HeapStartGpu.ptr + (idx * HeapHandleIncrement);
}
void ExampleDescriptorHeapAllocator::Free(
    D3D12_CPU_DESCRIPTOR_HANDLE out_cpu_desc_handle, D3D12_GPU_DESCRIPTOR_HANDLE out_gpu_desc_handle) {
    int cpu_idx = (int)((out_cpu_desc_handle.ptr - HeapStartCpu.ptr) / HeapHandleIncrement);
    int gpu_idx = (int)((out_gpu_desc_handle.ptr - HeapStartGpu.ptr) / HeapHandleIncrement);
    assert(cpu_idx == gpu_idx);
    FreeIndices.push_back(cpu_idx);
}

bool D3DContext::CreateDeviceD3D(HWND hWnd) {
    // Create DXGI Factory
    if (FAILED(CreateDXGIFactory2(0, IID_PPV_ARGS(&g_pdxgiFactory)))) return false;

    // Create device
    D3D_FEATURE_LEVEL featureLevel = D3D_FEATURE_LEVEL_12_0; // check first for 11

    ComPtr<IDXGIAdapter4> adapter;
    for (UINT i = 0; 
        g_pdxgiFactory->EnumAdapterByGpuPreference(i, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&adapter)) 
        != DXGI_ERROR_NOT_FOUND; ++i) {
        DXGI_ADAPTER_DESC3 desc = {};
        adapter->GetDesc3(&desc);
        if (desc.Flags & DXGI_ADAPTER_FLAG3_SOFTWARE) continue;
        if (SUCCEEDED(D3D12CreateDevice(adapter.Get(), featureLevel, IID_PPV_ARGS(&g_pd3dDevice)))) {
            break;
        } else {
            return false;
        }
    }

    // Create command queue
    {
        D3D12_COMMAND_QUEUE_DESC desc = {};
        desc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        desc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_HIGH;
        desc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        desc.NodeMask = 0;
        if (FAILED(g_pd3dDevice->CreateCommandQueue(&desc, IID_PPV_ARGS(&g_pd3dCommandQueue)))) return false;
    }
    // Create Copy queue
    {
        D3D12_COMMAND_QUEUE_DESC desc = {};
        desc.Type = D3D12_COMMAND_LIST_TYPE_COPY;
        desc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_HIGH;
        desc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        desc.NodeMask = 0;
        if (FAILED(g_pd3dDevice->CreateCommandQueue(&desc, IID_PPV_ARGS(&g_pd3dCopyQueue)))) return false;
        if (FAILED(g_pd3dDevice->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_COPY, IID_PPV_ARGS(&g_pd3dCopyAllocator))))
            return false;
        if (FAILED(g_pd3dDevice->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_COPY, g_pd3dCopyAllocator.Get(), nullptr, IID_PPV_ARGS(&g_pd3dCopyCommandList))) ||
            FAILED(g_pd3dCopyCommandList->Close()))
            return false;
        if (FAILED(g_pd3dDevice->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&copy_fence)))) return false;
        copy_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (copy_fenceEvent == nullptr) return false;
    }
    // Create DXR Queue, command list and allocator
    {
        D3D12_COMMAND_QUEUE_DESC desc = {};
        desc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        desc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_HIGH;
        desc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        desc.NodeMask = 0;
        if (FAILED(g_pd3dDevice->CreateCommandQueue(&desc, IID_PPV_ARGS(&g_pd3dDXRQueue)))) return false;
        if (FAILED(g_pd3dDevice->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&g_pd3dDXRAllocator))))
            return false;
        if (FAILED(g_pd3dDevice->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g_pd3dDXRAllocator.Get(),
                nullptr, IID_PPV_ARGS(&g_pd3dDXRCommandList))) || FAILED(g_pd3dDXRCommandList->Close()))
            return false;
        if (FAILED(g_pd3dDevice->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&dxr_fence)))) return false;
        dxr_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (dxr_fenceEvent == nullptr) return false;
    }

    // Create fence
    if (FAILED(g_pd3dDevice->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_fence)))) return false;

    g_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (g_fenceEvent == nullptr) return false;

    // Create command list
    for (UINT i = 0; i < APP_NUM_FRAMES_IN_FLIGHT; i++)
        if (FAILED(g_pd3dDevice->CreateCommandAllocator( D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&g_frameContext[i].CommandAllocator))))
            return false;
    if (FAILED(g_pd3dDevice->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g_frameContext[0].CommandAllocator.Get(),
            nullptr, IID_PPV_ARGS(&g_pd3dCommandList))) || FAILED(g_pd3dCommandList->Close()))
        return false;

    // Setup swap chain
    DXGI_SWAP_CHAIN_DESC1 sd = {};
    {
        sd.Width = 0;
        sd.Height = 0;
        sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        sd.Stereo = FALSE;
        sd.SampleDesc.Count = 1;
        sd.SampleDesc.Quality = 0;
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT; // or DXGI_USAGE_BACK_BUFFER | DXGI_USAGE_RENDER_TARGET_OUTPUT ?
        sd.BufferCount = APP_NUM_BACK_BUFFERS;
        sd.Scaling = DXGI_SCALING_STRETCH;
        sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        sd.AlphaMode = DXGI_ALPHA_MODE_UNSPECIFIED;
        // sd.AlphaMode = DXGI_ALPHA_MODE_IGNORE;
        sd.Flags = DXGI_SWAP_CHAIN_FLAG_FRAME_LATENCY_WAITABLE_OBJECT;
        // sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH | DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING; // ?
    }
    DXGI_SWAP_CHAIN_FULLSCREEN_DESC sfd = {};
    {
        sfd.Windowed = true;
    }

    {
        ComPtr<IDXGISwapChain1> swapChain1 = nullptr;
        if (FAILED(g_pdxgiFactory->CreateSwapChainForHwnd(g_pd3dCommandQueue.Get(), hWnd, &sd, &sfd, nullptr, &swapChain1)))
            return false;

        if (FAILED(swapChain1->QueryInterface(IID_PPV_ARGS(&g_pSwapChain)))) return false;
        /*
        BOOL allow_tearing = FALSE;
        g_pdxgiFactory->CheckFeatureSupport(DXGI_FEATURE_PRESENT_ALLOW_TEARING, &allow_tearing, sizeof(allow_tearing));
        g_SwapChainTearingSupport = (allow_tearing == TRUE);
        if (g_SwapChainTearingSupport) sd.Flags |= DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;
        if (g_SwapChainTearingSupport) g_pdxgiFactory->MakeWindowAssociation(hWnd, DXGI_MWA_NO_ALT_ENTER);
        */

        g_pSwapChain->SetMaximumFrameLatency(APP_NUM_BACK_BUFFERS);
        g_hSwapChainWaitableObject = g_pSwapChain->GetFrameLatencyWaitableObject();
    }

    // Create RTV Heap
    {
        D3D12_DESCRIPTOR_HEAP_DESC desc = {};
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        desc.NumDescriptors = APP_NUM_BACK_BUFFERS;
        desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
        desc.NodeMask = 0;
        if (FAILED(g_pd3dDevice->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&g_pd3dRtvDescHeap)))) return false;

        D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle = g_pd3dRtvDescHeap->GetCPUDescriptorHandleForHeapStart();
        SIZE_T rtvDescriptorSize = g_pd3dDevice->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
        for (UINT i = 0; i < APP_NUM_BACK_BUFFERS; i++) {
            g_mainRenderTargetDescriptor[i] = rtvHandle;
            rtvHandle.ptr += rtvDescriptorSize;
        }
    }
    // Create SRV Heap
    {
        D3D12_DESCRIPTOR_HEAP_DESC desc = {};
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        desc.NumDescriptors = APP_SRV_HEAP_SIZE;
        desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        if (FAILED(g_pd3dDevice->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&g_pd3dSrvDescHeap)))) return false;
        g_pd3dSrvDescHeapAlloc.Create(g_pd3dDevice.Get(), g_pd3dSrvDescHeap.Get());
    }

    hardware_ray_tracing_support = CheckRaytracingSupport();

    CreateRenderTarget();
    return true;
}

bool D3DContext::CheckRaytracingSupport() {
    D3D12_FEATURE_DATA_D3D12_OPTIONS5 options5 = {};
    if (FAILED(g_pd3dDevice->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5, &options5, sizeof(options5)))) return false;
    if (options5.RaytracingTier < D3D12_RAYTRACING_TIER_1_0) return false;
    return true;
}

void D3DContext::InitCommandList(ID3D12CommandAllocator& CommandAllocator) {
    CommandAllocator.Reset();
    g_pd3dCommandList->Reset(&CommandAllocator, nullptr);
}

void D3DContext::DispatchCommandList() {
    if (FAILED(g_pd3dCommandList->Close())) return;

    ID3D12CommandList* lists[] = {g_pd3dCommandList.Get()};
    g_pd3dCommandQueue->ExecuteCommandLists(1, lists);
}

void D3DContext::InitCopyCommandList() {
    g_pd3dCopyAllocator->Reset();
    g_pd3dCopyCommandList->Reset(g_pd3dCopyAllocator.Get(), nullptr);
}

void D3DContext::DispatchCopyCommandList() {
    if (FAILED(g_pd3dCopyCommandList->Close())) return;

    ID3D12CommandList* lists[] = {g_pd3dCopyCommandList.Get()};
    g_pd3dCopyQueue->ExecuteCommandLists(1, lists);
}

void D3DContext::InitDXRCommandList() {
    g_pd3dDXRAllocator->Reset();
    g_pd3dDXRCommandList->Reset(g_pd3dDXRAllocator.Get(), nullptr);
}

void D3DContext::DispatchDXRCommandList() {
    if (FAILED(g_pd3dDXRCommandList->Close())) return;

    ID3D12CommandList* lists[] = {g_pd3dDXRCommandList.Get()};
    g_pd3dDXRQueue->ExecuteCommandLists(1, lists);
}

void D3DContext::CleanupDeviceD3D() {
    CleanupRenderTarget();

    g_pd3dSrvDescHeapAlloc.Destroy();

    g_pd3dSrvDescHeap.Reset();
    g_pd3dRtvDescHeap.Reset();
    if (g_pSwapChain) {
        g_pSwapChain->SetFullscreenState(false, nullptr);
        g_pSwapChain.Reset();
    }
    if (g_hSwapChainWaitableObject != nullptr) {
        CloseHandle(g_hSwapChainWaitableObject);
    }
    g_pd3dCommandList.Reset();
    for (UINT i = 0; i < APP_NUM_FRAMES_IN_FLIGHT; i++) g_frameContext[i].CommandAllocator.Reset();
    g_fence.Reset();
    if (g_fenceEvent) {
        CloseHandle(g_fenceEvent);
        g_fenceEvent = nullptr;
    }
    copy_fence.Reset();
    if (copy_fenceEvent) {
        CloseHandle(copy_fenceEvent);
        copy_fenceEvent = nullptr;
    }
    g_pd3dDXRCommandList.Reset();
    g_pd3dDXRAllocator.Reset();
    g_pd3dDXRQueue.Reset();

    g_pd3dCopyCommandList.Reset();
    g_pd3dCopyAllocator.Reset();
    g_pd3dCopyQueue.Reset();

    g_pd3dCommandQueue.Reset();
    g_pd3dDevice.Reset();
    g_pdxgiFactory.Reset();
}

void D3DContext::WaitForPendingOperations() {
    if (FAILED(g_pd3dCommandQueue->Signal(g_fence.Get(), ++g_fenceLastSignaledValue))) std::exit(-1);

    if (g_fence->GetCompletedValue() < g_fenceLastSignaledValue) {
        if (FAILED(g_fence->SetEventOnCompletion(g_fenceLastSignaledValue, g_fenceEvent))) std::exit(-1);
        if (::WaitForSingleObject(g_fenceEvent, 20000) != WAIT_OBJECT_0) std::exit(-1);
    }
}
void D3DContext::WaitForPendingCopy() {
    if (FAILED(g_pd3dCopyQueue->Signal(copy_fence.Get(), ++copy_fenceLastSignaledValue))) std::exit(-1);

    if (copy_fence->GetCompletedValue() < copy_fenceLastSignaledValue) {
        if (FAILED(copy_fence->SetEventOnCompletion(copy_fenceLastSignaledValue, copy_fenceEvent))) std::exit(-1);
        if (::WaitForSingleObject(copy_fenceEvent, 20000) != WAIT_OBJECT_0) std::exit(-1);
    }
}

void D3DContext::WaitForPendingDXR() {
    if (FAILED(g_pd3dDXRQueue->Signal(dxr_fence.Get(), ++dxr_fenceLastSignaledValue))) std::exit(-1);

    if (dxr_fence->GetCompletedValue() < dxr_fenceLastSignaledValue) {
        if (FAILED(dxr_fence->SetEventOnCompletion(dxr_fenceLastSignaledValue, dxr_fenceEvent))) std::exit(-1);
        if (::WaitForSingleObject(dxr_fenceEvent, 20000) != WAIT_OBJECT_0) std::exit(-1);
    }
}

void D3DContext::CreateRenderTarget() {
    for (UINT i = 0; i < APP_NUM_BACK_BUFFERS; i++) {
        g_pSwapChain->GetBuffer(i, IID_PPV_ARGS(&g_mainRenderTargetResource[i]));
        
        D3D12_RENDER_TARGET_VIEW_DESC rtv{};
        rtv.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
        rtv.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
        rtv.Texture2D.MipSlice = 0;
        rtv.Texture2D.PlaneSlice = 0;
        g_pd3dDevice->CreateRenderTargetView(
            g_mainRenderTargetResource[i].Get(), &rtv, g_mainRenderTargetDescriptor[i]);
    }
}

void D3DContext::CleanupRenderTarget() {
    WaitForPendingOperations();
    for (UINT i = 0; i < APP_NUM_BACK_BUFFERS; i++) {
        g_mainRenderTargetResource[i].Reset();
    }
}

FrameContext* D3DContext::WaitForNextFrameContext() {
    FrameContext* frame_context = &g_frameContext[g_frameIndex % APP_NUM_FRAMES_IN_FLIGHT];
    if (g_fence->GetCompletedValue() < frame_context->FenceValue) {
        g_fence->SetEventOnCompletion(frame_context->FenceValue, g_fenceEvent);
        HANDLE waitableObjects[] = {g_hSwapChainWaitableObject, g_fenceEvent};
        ::WaitForMultipleObjects(2, waitableObjects, TRUE, INFINITE);
    } else
        ::WaitForSingleObject(g_hSwapChainWaitableObject, INFINITE);
    return frame_context;
}

FrameContext* D3DContext::GetCurrentFrameContext() {
    FrameContext* frame_context = &g_frameContext[g_frameIndex % APP_NUM_FRAMES_IN_FLIGHT];
    return frame_context;
}

void D3DContext::ResizeSwapchain(UINT Width, UINT Height) {
    CleanupRenderTarget();
    DXGI_SWAP_CHAIN_DESC1 desc = {};
    g_pSwapChain->GetDesc1(&desc);
    HRESULT result = g_pSwapChain->ResizeBuffers(APP_NUM_BACK_BUFFERS, Width, Height, desc.Format, desc.Flags);
    assert(SUCCEEDED(result) && "Failed to resize swapchain.");
    CreateRenderTarget();
}

}
