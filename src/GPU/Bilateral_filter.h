#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

class Bilateral_filter {
  public:
    Bilateral_filter();
    ~Bilateral_filter();

    void ApplyBlur(GPU_texture& output_uav_texture, GPU_texture& AO_texture, GPU_texture& G_buff_texture,
        GPU_texture& Depth_texture, float spatialSigma, float DepthSigma, float NormalSigma);

    float AODistance = 1.0f;

    static void Reload();

  private:
    static void CreatePipelineStateObject();
    static void CreateRootSignature();

    static ComPtr<ID3D12RootSignature> m_rootSignature;
    static ComPtr<ID3D12PipelineState> m_PipelineState;

    static constexpr const wchar_t* c_cs_file_name = L"CS_Bilateral_filter.dxil";

    void release_gpu_resources();
};

}  // namespace app