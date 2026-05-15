#pragma once

#include <directx/d3d12.h>
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
static const int APP_SRV_HEAP_SIZE = 1024;  // maximum textures in whole pipeline, imgui + gltf scene

struct FrameContext {
    ComPtr<ID3D12CommandAllocator> CommandAllocator;
    UINT64 FenceValue;
};

struct D3D_Handle_Pair {
    D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle;
    D3D12_GPU_DESCRIPTOR_HANDLE gpuHandle;
};

// Simple free list based allocator
class DescriptorHeapAllocator {
  private:
    ID3D12DescriptorHeap* Heap = nullptr;
    D3D12_DESCRIPTOR_HEAP_TYPE HeapType = D3D12_DESCRIPTOR_HEAP_TYPE_NUM_TYPES;
    D3D12_CPU_DESCRIPTOR_HANDLE HeapStartCpu = {};
    D3D12_GPU_DESCRIPTOR_HANDLE HeapStartGpu = {};
    UINT HeapHandleIncrement = 0;
    std::vector<int> FreeIndices;

  public:
    void Create(ID3D12Device* device, ID3D12DescriptorHeap* heap);
    void Destroy();
    void Alloc(D3D12_CPU_DESCRIPTOR_HANDLE* out_cpu_desc_handle, D3D12_GPU_DESCRIPTOR_HANDLE* out_gpu_desc_handle);
    void Alloc(D3D_Handle_Pair* out_desc_handle);
    void Free(D3D12_CPU_DESCRIPTOR_HANDLE cpu_desc_handle, D3D12_GPU_DESCRIPTOR_HANDLE gpu_desc_handle);
    void Free(D3D_Handle_Pair desc_handle);
    int GetIndex(D3D12_GPU_DESCRIPTOR_HANDLE gpu_desc_handle) const;
};

// Data
struct D3DContext {
    UINT m_frameIndex = 0;
    FrameContext m_frameContext[APP_NUM_FRAMES_IN_FLIGHT] = {};

    ComPtr<ID3D12Device5> m_d3dDevice;

    ComPtr<ID3D12DescriptorHeap> m_RtvDescHeap;
    ComPtr<ID3D12DescriptorHeap> m_SrvDescHeap;
    DescriptorHeapAllocator m_SrvDescHeapAlloc;

    ComPtr<ID3D12DescriptorHeap> m_additional_RtvDescHeap;  // additional render targets
    DescriptorHeapAllocator m_RtvDescHeapAlloc;
    ComPtr<ID3D12DescriptorHeap> m_additional_DsvDescHeap;  // additional depth textures
    DescriptorHeapAllocator m_DsvDescHeapAlloc;

    ComPtr<ID3D12CommandQueue> m_CommandQueue;
    ComPtr<ID3D12GraphicsCommandList4> m_CommandList;

    ComPtr<ID3D12CommandQueue> m_CopyQueue;
    ComPtr<ID3D12CommandAllocator> m_CopyAllocator;
    ComPtr<ID3D12GraphicsCommandList4> m_CopyCommandList;

    ComPtr<ID3D12CommandQueue> m_DXRQueue;
    ComPtr<ID3D12CommandAllocator> m_DXRAllocator;
    ComPtr<ID3D12GraphicsCommandList4> m_DXRCommandList;

    ComPtr<ID3D12Fence> m_copy_fence;
    HANDLE m_copy_fenceEvent = nullptr;
    UINT64 m_copy_fenceLastSignaledValue = 0;

    ComPtr<ID3D12Fence> m_fence;
    HANDLE m_fenceEvent = nullptr;
    UINT64 m_fenceLastSignaledValue = 0;

    ComPtr<ID3D12Fence> m_dxr_fence;
    HANDLE m_dxr_fenceEvent = nullptr;
    UINT64 m_dxr_fenceLastSignaledValue = 0;

    ComPtr<IDXGIFactory7> m_dxgiFactory;
    ComPtr<IDXGISwapChain3> m_SwapChain;
    // bool m_SwapChainTearingSupport = false;
    bool m_SwapChainOccluded = false;
    HANDLE m_SwapChainWaitableObject = nullptr;

    bool hardware_ray_tracing_support = false;

    ComPtr<ID3D12Resource> m_mainRenderTargetResource[APP_NUM_BACK_BUFFERS] = {};
    D3D12_CPU_DESCRIPTOR_HANDLE m_mainRenderTargetDescriptor[APP_NUM_BACK_BUFFERS] = {};

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

    bool CheckRaytracingSupport() const;
};

struct PendingDelete {
    ComPtr<ID3D12Resource> resource;
    // UINT64 fenceValue;
};

}  // namespace app
