#include "d3d_context.h"

#include <cassert>
#include <iostream>

namespace app {

void DescriptorHeapAllocator::Create(ID3D12Device* device, ID3D12DescriptorHeap* heap) {
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
void DescriptorHeapAllocator::Destroy() {
    Heap = nullptr;
    FreeIndices.clear();
}
void DescriptorHeapAllocator::Alloc(
    D3D12_CPU_DESCRIPTOR_HANDLE* out_cpu_desc_handle, D3D12_GPU_DESCRIPTOR_HANDLE* out_gpu_desc_handle) {
    if (!out_cpu_desc_handle || !out_gpu_desc_handle) return;
    assert(FreeIndices.size() > 0);
    int idx = FreeIndices.back();
    FreeIndices.pop_back();
    out_cpu_desc_handle->ptr = HeapStartCpu.ptr + (idx * HeapHandleIncrement);
    out_gpu_desc_handle->ptr = HeapStartGpu.ptr + (idx * HeapHandleIncrement);
}
void DescriptorHeapAllocator::Alloc(D3D_Handle_Pair* out_desc_handle) { 
    Alloc(&out_desc_handle->cpuHandle, &out_desc_handle->gpuHandle);
}
void DescriptorHeapAllocator::Free(
    D3D12_CPU_DESCRIPTOR_HANDLE cpu_desc_handle, D3D12_GPU_DESCRIPTOR_HANDLE gpu_desc_handle) {
    int cpu_idx = (int)((cpu_desc_handle.ptr - HeapStartCpu.ptr) / HeapHandleIncrement);
    int gpu_idx = (int)((gpu_desc_handle.ptr - HeapStartGpu.ptr) / HeapHandleIncrement);
    assert(cpu_idx == gpu_idx);
    FreeIndices.push_back(cpu_idx);
}
void DescriptorHeapAllocator::Free(D3D_Handle_Pair desc_handle) {
    Free(desc_handle.cpuHandle, desc_handle.gpuHandle);
}
int DescriptorHeapAllocator::GetIndex(D3D12_GPU_DESCRIPTOR_HANDLE gpu_desc_handle) const {
    int gpu_idx = (int)((gpu_desc_handle.ptr - HeapStartGpu.ptr) / HeapHandleIncrement);
    return gpu_idx;
}

bool D3DContext::CreateDeviceD3D(HWND hWnd) {
    // Create DXGI Factory
    if (FAILED(CreateDXGIFactory2(0, IID_PPV_ARGS(&m_dxgiFactory)))) return false;

    // Create device
    D3D_FEATURE_LEVEL featureLevel = D3D_FEATURE_LEVEL_11_0;

    bool deviceCreated = false;
    ComPtr<IDXGIAdapter4> adapter;
    for (UINT i = 0; 
        m_dxgiFactory->EnumAdapterByGpuPreference(i, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&adapter)) 
        != DXGI_ERROR_NOT_FOUND; ++i) {
        DXGI_ADAPTER_DESC3 desc = {};
        adapter->GetDesc3(&desc);
        if (desc.Flags & DXGI_ADAPTER_FLAG3_SOFTWARE) continue;
        if (SUCCEEDED(D3D12CreateDevice(adapter.Get(), featureLevel, IID_PPV_ARGS(&m_d3dDevice)))) {
            deviceCreated = true;
            break;
        } else {
            adapter.Reset();
        }
    }
    if (!deviceCreated) return false;

    // Create command queue
    {
        D3D12_COMMAND_QUEUE_DESC desc = {};
        desc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        desc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_HIGH;
        desc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        desc.NodeMask = 0;
        if (FAILED(m_d3dDevice->CreateCommandQueue(&desc, IID_PPV_ARGS(&m_CommandQueue)))) return false;
    }
    // Create Copy queue
    {
        D3D12_COMMAND_QUEUE_DESC desc = {};
        desc.Type = D3D12_COMMAND_LIST_TYPE_COPY;
        desc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_HIGH;
        desc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        desc.NodeMask = 0;
        if (FAILED(m_d3dDevice->CreateCommandQueue(&desc, IID_PPV_ARGS(&m_CopyQueue)))) return false;
        if (FAILED(m_d3dDevice->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_COPY, IID_PPV_ARGS(&m_CopyAllocator))))
            return false;
        if (FAILED(m_d3dDevice->CreateCommandList(
                0, D3D12_COMMAND_LIST_TYPE_COPY, m_CopyAllocator.Get(), nullptr, IID_PPV_ARGS(&m_CopyCommandList))) ||
            FAILED(m_CopyCommandList->Close()))
            return false;
        if (FAILED(m_d3dDevice->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_copy_fence)))) return false;
        m_copy_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (m_copy_fenceEvent == nullptr) return false;
    }
    // Create DXR Queue, command list and allocator
    {
        D3D12_COMMAND_QUEUE_DESC desc = {};
        desc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
        desc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_HIGH;
        desc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        desc.NodeMask = 0;
        if (FAILED(m_d3dDevice->CreateCommandQueue(&desc, IID_PPV_ARGS(&m_DXRQueue)))) return false;
        if (FAILED(m_d3dDevice->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&m_DXRAllocator))))
            return false;
        if (FAILED(m_d3dDevice->CreateCommandList(
                0, D3D12_COMMAND_LIST_TYPE_DIRECT, m_DXRAllocator.Get(), nullptr, IID_PPV_ARGS(&m_DXRCommandList))) ||
            FAILED(m_DXRCommandList->Close()))
            return false;
        if (FAILED(m_d3dDevice->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_dxr_fence)))) return false;
        m_dxr_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (m_dxr_fenceEvent == nullptr) return false;
    }

    // Create fence
    if (FAILED(m_d3dDevice->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence)))) return false;

    m_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (m_fenceEvent == nullptr) return false;

    // Create command list
    for (UINT i = 0; i < APP_NUM_FRAMES_IN_FLIGHT; i++)
        if (FAILED(m_d3dDevice->CreateCommandAllocator(
                D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&m_frameContext[i].CommandAllocator))))
            return false;
    if (FAILED(m_d3dDevice->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, m_frameContext[0].CommandAllocator.Get(),
            nullptr, IID_PPV_ARGS(&m_CommandList))) ||
        FAILED(m_CommandList->Close()))
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
        sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;  // or DXGI_USAGE_BACK_BUFFER | DXGI_USAGE_RENDER_TARGET_OUTPUT ?
        sd.BufferCount = APP_NUM_BACK_BUFFERS;
        sd.Scaling = DXGI_SCALING_STRETCH;
        sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        sd.AlphaMode = DXGI_ALPHA_MODE_UNSPECIFIED;
        // sd.AlphaMode = DXGI_ALPHA_MODE_IGNORE;
        sd.Flags = DXGI_SWAP_CHAIN_FLAG_FRAME_LATENCY_WAITABLE_OBJECT;
        // sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH | DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING; // ?
    }
    DXGI_SWAP_CHAIN_FULLSCREEN_DESC sfd = {};
    sfd.Windowed = true;

    {
        ComPtr<IDXGISwapChain1> swapChain1 = nullptr;
        if (FAILED(m_dxgiFactory->CreateSwapChainForHwnd(m_CommandQueue.Get(), hWnd, &sd, &sfd, nullptr, &swapChain1)))
            return false;

        if (FAILED(swapChain1->QueryInterface(IID_PPV_ARGS(&m_SwapChain)))) return false;
        /*
        BOOL allow_tearing = FALSE;
        m_dxgiFactory->CheckFeatureSupport(DXGI_FEATURE_PRESENT_ALLOW_TEARING, &allow_tearing, sizeof(allow_tearing));
        g_SwapChainTearingSupport = (allow_tearing == TRUE);
        if (g_SwapChainTearingSupport) sd.Flags |= DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;
        if (g_SwapChainTearingSupport) m_dxgiFactory->MakeWindowAssociation(hWnd, DXGI_MWA_NO_ALT_ENTER);
        */

        m_SwapChain->SetMaximumFrameLatency(APP_NUM_BACK_BUFFERS);
        m_SwapChainWaitableObject = m_SwapChain->GetFrameLatencyWaitableObject();
    }

    // Create RTV Heap
    {
        D3D12_DESCRIPTOR_HEAP_DESC desc = {};
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        desc.NumDescriptors = APP_NUM_BACK_BUFFERS;
        desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
        desc.NodeMask = 0;
        if (FAILED(m_d3dDevice->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&m_RtvDescHeap)))) return false;

        D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle = m_RtvDescHeap->GetCPUDescriptorHandleForHeapStart();
        SIZE_T rtvDescriptorSize = m_d3dDevice->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
        for (UINT i = 0; i < APP_NUM_BACK_BUFFERS; i++) {
            m_mainRenderTargetDescriptor[i] = rtvHandle;
            rtvHandle.ptr += rtvDescriptorSize;
        }
    }
    // Create SRV Heap
    {
        D3D12_DESCRIPTOR_HEAP_DESC desc = {};
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        desc.NumDescriptors = APP_SRV_HEAP_SIZE;
        desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        if (FAILED(m_d3dDevice->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&m_SrvDescHeap)))) return false;
        m_SrvDescHeapAlloc.Create(m_d3dDevice.Get(), m_SrvDescHeap.Get());
    }
    // Create additional RTV Heap for custom rendering passes
    {
        D3D12_DESCRIPTOR_HEAP_DESC desc = {};
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        desc.NumDescriptors = 4;
        desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
        desc.NodeMask = 0;
        if (FAILED(m_d3dDevice->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&m_additional_RtvDescHeap)))) return false;
        m_RtvDescHeapAlloc.Create(m_d3dDevice.Get(), m_additional_RtvDescHeap.Get());
    }
    // Create additional DSV Heap for custom rendering passes
    {
        D3D12_DESCRIPTOR_HEAP_DESC desc = {};
        desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        desc.NumDescriptors = 4;
        desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
        desc.NodeMask = 0;
        if (FAILED(m_d3dDevice->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&m_additional_DsvDescHeap)))) return false;
        m_DsvDescHeapAlloc.Create(m_d3dDevice.Get(), m_additional_DsvDescHeap.Get());
    }

    hardware_ray_tracing_support = CheckRaytracingSupport();

    if (!hardware_ray_tracing_support) {
        std::cout << "WARNING: GPU does not support raytracing, DXR rendering mode is not available.\n";
    }

    CreateRenderTarget();
    return true;
}

bool D3DContext::CheckRaytracingSupport() const {
    D3D12_FEATURE_DATA_D3D12_OPTIONS5 options5 = {};
    if (FAILED(m_d3dDevice->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5, &options5, sizeof(options5)))) return false;
    if (options5.RaytracingTier < D3D12_RAYTRACING_TIER_1_0) return false;
    return true;
}

void D3DContext::InitCommandList(ID3D12CommandAllocator& CommandAllocator) {
    CommandAllocator.Reset();
    m_CommandList->Reset(&CommandAllocator, nullptr);
}

void D3DContext::DispatchCommandList() {
    if (FAILED(m_CommandList->Close())) return;

    ID3D12CommandList* lists[] = {m_CommandList.Get()};
    m_CommandQueue->ExecuteCommandLists(1, lists);
}

void D3DContext::InitCopyCommandList() {
    m_CopyAllocator->Reset();
    m_CopyCommandList->Reset(m_CopyAllocator.Get(), nullptr);
}

void D3DContext::DispatchCopyCommandList() {
    if (FAILED(m_CopyCommandList->Close())) return;

    ID3D12CommandList* lists[] = {m_CopyCommandList.Get()};
    m_CopyQueue->ExecuteCommandLists(1, lists);
}

void D3DContext::InitDXRCommandList() {
    m_DXRAllocator->Reset();
    m_DXRCommandList->Reset(m_DXRAllocator.Get(), nullptr);
}

void D3DContext::DispatchDXRCommandList() {
    if (FAILED(m_DXRCommandList->Close())) return;

    ID3D12CommandList* lists[] = {m_DXRCommandList.Get()};
    m_DXRQueue->ExecuteCommandLists(1, lists);
}

void D3DContext::CleanupDeviceD3D() {
    CleanupRenderTarget();

    m_SrvDescHeapAlloc.Destroy();
    m_RtvDescHeapAlloc.Destroy();
    m_DsvDescHeapAlloc.Destroy();

    m_SrvDescHeap.Reset();
    m_RtvDescHeap.Reset();
    m_additional_RtvDescHeap.Reset();
    m_additional_DsvDescHeap.Reset();
    if (m_SwapChain) {
        m_SwapChain->SetFullscreenState(false, nullptr);
        m_SwapChain.Reset();
    }
    if (m_SwapChainWaitableObject != nullptr) {
        CloseHandle(m_SwapChainWaitableObject);
    }
    m_CommandList.Reset();
    for (UINT i = 0; i < APP_NUM_FRAMES_IN_FLIGHT; i++) m_frameContext[i].CommandAllocator.Reset();
    m_fence.Reset();
    if (m_fenceEvent) {
        CloseHandle(m_fenceEvent);
        m_fenceEvent = nullptr;
    }
    m_copy_fence.Reset();
    if (m_copy_fenceEvent) {
        CloseHandle(m_copy_fenceEvent);
        m_copy_fenceEvent = nullptr;
    }
    m_dxr_fence.Reset();
    if (m_dxr_fenceEvent) {
        CloseHandle(m_dxr_fenceEvent);
        m_dxr_fenceEvent = nullptr;
    }
    m_DXRCommandList.Reset();
    m_DXRAllocator.Reset();
    m_DXRQueue.Reset();

    m_CopyCommandList.Reset();
    m_CopyAllocator.Reset();
    m_CopyQueue.Reset();

    m_CommandQueue.Reset();
    m_d3dDevice.Reset();
    m_dxgiFactory.Reset();
}

void D3DContext::WaitForPendingOperations() {
    if (FAILED(m_CommandQueue->Signal(m_fence.Get(), ++m_fenceLastSignaledValue))) std::exit(-1);

    if (m_fence->GetCompletedValue() < m_fenceLastSignaledValue) {
        if (FAILED(m_fence->SetEventOnCompletion(m_fenceLastSignaledValue, m_fenceEvent))) std::exit(-1);
        if (::WaitForSingleObject(m_fenceEvent, 20000) != WAIT_OBJECT_0) std::exit(-1);
    }
}
void D3DContext::WaitForPendingCopy() {
    if (FAILED(m_CopyQueue->Signal(m_copy_fence.Get(), ++m_copy_fenceLastSignaledValue))) std::exit(-1);

    if (m_copy_fence->GetCompletedValue() < m_copy_fenceLastSignaledValue) {
        if (FAILED(m_copy_fence->SetEventOnCompletion(m_copy_fenceLastSignaledValue, m_copy_fenceEvent))) std::exit(-1);
        if (::WaitForSingleObject(m_copy_fenceEvent, 20000) != WAIT_OBJECT_0) std::exit(-1);
    }
}

void D3DContext::WaitForPendingDXR() {
    if (FAILED(m_DXRQueue->Signal(m_dxr_fence.Get(), ++m_dxr_fenceLastSignaledValue))) std::exit(-1);

    if (m_dxr_fence->GetCompletedValue() < m_dxr_fenceLastSignaledValue) {
        if (FAILED(m_dxr_fence->SetEventOnCompletion(m_dxr_fenceLastSignaledValue, m_dxr_fenceEvent))) std::exit(-1);
        if (::WaitForSingleObject(m_dxr_fenceEvent, 20000) != WAIT_OBJECT_0) std::exit(-1);
    }
}

void D3DContext::CreateRenderTarget() {
    for (UINT i = 0; i < APP_NUM_BACK_BUFFERS; i++) {
        m_SwapChain->GetBuffer(i, IID_PPV_ARGS(&m_mainRenderTargetResource[i]));

        D3D12_RENDER_TARGET_VIEW_DESC rtv{};
        rtv.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
        rtv.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
        rtv.Texture2D.MipSlice = 0;
        rtv.Texture2D.PlaneSlice = 0;
        m_d3dDevice->CreateRenderTargetView(m_mainRenderTargetResource[i].Get(), &rtv, m_mainRenderTargetDescriptor[i]);
    }
}

void D3DContext::CleanupRenderTarget() {
    WaitForPendingOperations();
    for (UINT i = 0; i < APP_NUM_BACK_BUFFERS; i++) {
        m_mainRenderTargetResource[i].Reset();
    }
}

FrameContext* D3DContext::WaitForNextFrameContext() {
    FrameContext* frame_context = &m_frameContext[m_frameIndex % APP_NUM_FRAMES_IN_FLIGHT];
    if (m_fence->GetCompletedValue() < frame_context->FenceValue) {
        auto HR = m_fence->SetEventOnCompletion(frame_context->FenceValue, m_fenceEvent);
        if (FAILED(HR)) std::exit(-1);
        HANDLE waitableObjects[] = {m_SwapChainWaitableObject, m_fenceEvent};
        ::WaitForMultipleObjects(2, waitableObjects, TRUE, INFINITE);
    } else
        ::WaitForSingleObject(m_SwapChainWaitableObject, INFINITE);
    return frame_context;
}

FrameContext* D3DContext::GetCurrentFrameContext() {
    FrameContext* frame_context = &m_frameContext[m_frameIndex % APP_NUM_FRAMES_IN_FLIGHT];
    return frame_context;
}

void D3DContext::ResizeSwapchain(UINT Width, UINT Height) {
    CleanupRenderTarget();
    DXGI_SWAP_CHAIN_DESC1 desc = {};
    m_SwapChain->GetDesc1(&desc);
    HRESULT result = m_SwapChain->ResizeBuffers(APP_NUM_BACK_BUFFERS, Width, Height, desc.Format, desc.Flags);
    assert(SUCCEEDED(result) && "Failed to resize swapchain.");
    CreateRenderTarget();
}

}  // namespace app
