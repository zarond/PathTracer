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
    Kawase_blur_helper(int max_concurrent_calls = 1, int mips_limit = 16, bool can_blur_cubemaps = false);
    ~Kawase_blur_helper();

    void CreateBlurMips(GPU_texture& uav_texture);
    void CreateUpsampleMips(GPU_texture& uav_texture);

    static GPU_texture GetBlankCompatibleUAVTex(GPU_texture& texture);

    constexpr static int BlurIterations = 10;

    static void Reload();

  private:
    static void CreateRootSignature();
    static void CreatePipelineStateObject();
    void CreateDescriptorHeap();

    static ComPtr<ID3D12RootSignature> m_rootSignature;
    static ComPtr<ID3D12PipelineState> m_DownsamplePipelineState;
    static ComPtr<ID3D12PipelineState> m_UpsamplePipelineState;

    static constexpr const wchar_t* c_cs_downsample_file_name = L"CS_KawaseBlurDownsample_13.dxil";
    static constexpr const wchar_t* c_cs_upsample_file_name = L"CS_KawaseBlurUpsample.dxil";

    ComPtr<ID3D12DescriptorHeap> m_SrvDescHeap;
    D3D12_CPU_DESCRIPTOR_HANDLE HeapStartCpu = {};
    D3D12_GPU_DESCRIPTOR_HANDLE HeapStartGpu = {};
    UINT HeapHandleIncrement = 0;

    int m_next_call_slot = 0;  // rotating index of the slot range used by the next blur call
    
    // Max number of CreateBlurMips calls that may be recorded into one command list per single helper instance
    int MaxConcurrentCalls = 8;
    int MipsLimit = 16;
    bool CanBlurCubemaps = false;

    void release_gpu_resources();
};

}  // namespace app