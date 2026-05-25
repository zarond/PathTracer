#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

class Kawase_blur_helper {
  public:
    Kawase_blur_helper();
    ~Kawase_blur_helper();

    void CreateBlurMips(GPU_texture& uav_texture);

    static GPU_texture GetBlankCompatibleUAVTex(GPU_texture& texture);

    constexpr static int BlurIterations = 10;

  private:
    void CreateRootSignature();
    void CreatePipelineStateObject();
    void CreateDescriptorHeap();

    ComPtr<ID3D12RootSignature> m_rootSignature;
    ComPtr<ID3D12PipelineState> m_DownsamplePipelineState;
    ComPtr<ID3D12PipelineState> m_UpsamplePipelineState;

    const wchar_t* c_cs_downsample_file_name = L"CS_KawaseBlurDownsample_13.dxil";
    const wchar_t* c_cs_upsample_file_name = L"CS_KawaseBlurUpsample.dxil";

    ComPtr<ID3D12DescriptorHeap> m_SrvDescHeap;
    D3D12_CPU_DESCRIPTOR_HANDLE HeapStartCpu = {};
    D3D12_GPU_DESCRIPTOR_HANDLE HeapStartGpu = {};
    UINT HeapHandleIncrement = 0;

    constexpr static int MipsLimit = 16;

    void release_gpu_resources();
};

}  // namespace app