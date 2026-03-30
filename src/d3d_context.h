#pragma once

#include <d3d12.h>
#include <dxgi1_6.h>

#include <vector>

#define NOMINMAX
#include <windows.h>
#include <wrl.h>

namespace app {

using Microsoft::WRL::ComPtr;

// Config
static const int APP_NUM_FRAMES_IN_FLIGHT = 2;  // double buffering
static const int APP_NUM_BACK_BUFFERS = 2;
static const int APP_SRV_HEAP_SIZE = 64;

struct FrameContext {
    ComPtr<ID3D12CommandAllocator> CommandAllocator;
    UINT64 FenceValue;
};

// Simple free list based allocator
class ExampleDescriptorHeapAllocator {
  private:
    ID3D12DescriptorHeap* Heap = nullptr;
    D3D12_DESCRIPTOR_HEAP_TYPE HeapType = D3D12_DESCRIPTOR_HEAP_TYPE_NUM_TYPES;
    D3D12_CPU_DESCRIPTOR_HANDLE HeapStartCpu;
    D3D12_GPU_DESCRIPTOR_HANDLE HeapStartGpu;
    UINT HeapHandleIncrement;
    std::vector<int> FreeIndices;

  public:
    void Create(ID3D12Device* device, ID3D12DescriptorHeap* heap);
    void Destroy();
    void Alloc(D3D12_CPU_DESCRIPTOR_HANDLE* out_cpu_desc_handle, D3D12_GPU_DESCRIPTOR_HANDLE* out_gpu_desc_handle);
    void Free(D3D12_CPU_DESCRIPTOR_HANDLE out_cpu_desc_handle, D3D12_GPU_DESCRIPTOR_HANDLE out_gpu_desc_handle);
};

// Data
struct D3DContext {
    UINT g_frameIndex = 0;
    FrameContext g_frameContext[APP_NUM_FRAMES_IN_FLIGHT] = {};

    ComPtr<ID3D12Device5> g_pd3dDevice;

    ComPtr<ID3D12DescriptorHeap> g_pd3dRtvDescHeap;
    ComPtr<ID3D12DescriptorHeap> g_pd3dSrvDescHeap;
    ExampleDescriptorHeapAllocator g_pd3dSrvDescHeapAlloc;

    ComPtr<ID3D12CommandQueue> g_pd3dCommandQueue;
    ComPtr<ID3D12GraphicsCommandList4> g_pd3dCommandList;

    ComPtr<ID3D12CommandQueue> g_pd3dCopyQueue;
    ComPtr<ID3D12CommandAllocator> g_pd3dCopyAllocator;
    ComPtr<ID3D12GraphicsCommandList4> g_pd3dCopyCommandList;

    ComPtr<ID3D12CommandQueue> g_pd3dDXRQueue;
    ComPtr<ID3D12CommandAllocator> g_pd3dDXRAllocator;
    ComPtr<ID3D12GraphicsCommandList4> g_pd3dDXRCommandList;

    ComPtr<ID3D12Fence> copy_fence;
    HANDLE copy_fenceEvent = nullptr;
    UINT64 copy_fenceLastSignaledValue = 0;

    ComPtr<ID3D12Fence> g_fence;
    HANDLE g_fenceEvent = nullptr;
    UINT64 g_fenceLastSignaledValue = 0;

    ComPtr<ID3D12Fence> dxr_fence;
    HANDLE dxr_fenceEvent = nullptr;
    UINT64 dxr_fenceLastSignaledValue = 0;

    ComPtr<IDXGIFactory7> g_pdxgiFactory;
    ComPtr<IDXGISwapChain3> g_pSwapChain;
    // bool g_SwapChainTearingSupport = false;
    bool g_SwapChainOccluded = false;
    HANDLE g_hSwapChainWaitableObject = nullptr;

    bool hardware_ray_tracing_support = false;

    ComPtr<ID3D12Resource> g_mainRenderTargetResource[APP_NUM_BACK_BUFFERS] = {};
    D3D12_CPU_DESCRIPTOR_HANDLE g_mainRenderTargetDescriptor[APP_NUM_BACK_BUFFERS] = {};

    bool CreateDeviceD3D(HWND hWnd);
    void CleanupDeviceD3D();
    void WaitForPendingOperations();
    void WaitForPendingCopy();
    void WaitForPendingDXR();
    void InitCommandList(ID3D12CommandAllocator& CommandAllocator);
    void DispatchCommandList();
    void InitCopyCommandList();
    void DispatchCopyCommandList();
    void InitDXRCommandList();
    void DispatchDXRCommandList();
    void CreateRenderTarget();
    void CleanupRenderTarget();
    FrameContext* WaitForNextFrameContext();
    FrameContext* GetCurrentFrameContext();

    void ResizeSwapchain(UINT Width, UINT Height);

    // Singleton pattern
  public:
    D3DContext(const D3DContext&) = delete;
    D3DContext& operator=(const D3DContext&) = delete;

    inline static D3DContext& Get() {
        static D3DContext instance;
        return instance;
    }
  private:
    D3DContext() = default;

    bool CheckRaytracingSupport();
};

}
