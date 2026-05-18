#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

class Mipmaps_helper {
  public:
    Mipmaps_helper();
    ~Mipmaps_helper();

    void CreateMips(GPU_texture& uav_texture);

  private:
    void CreateRootSignature();
    void CreatePipelineStateObject();
    void CreateDescriptorHeap();

    ComPtr<ID3D12RootSignature> m_rootSignature;
    ComPtr<ID3D12PipelineState> m_PipelineState;

    const wchar_t* c_cs_file_name = L"CS_SinglePassMips.dxil";

    ComPtr<ID3D12DescriptorHeap> m_SrvDescHeap;
    D3D12_CPU_DESCRIPTOR_HANDLE HeapStartCpu = {};
    D3D12_GPU_DESCRIPTOR_HANDLE HeapStartGpu = {};
    UINT HeapHandleIncrement = 0;

    constexpr static int MipsLimit = 16;

    void release_gpu_resources();
};

}  // namespace app