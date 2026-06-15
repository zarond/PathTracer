#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

class Silhouette_helper {
  public:
    Silhouette_helper();
    ~Silhouette_helper();

    void Apply(GPU_texture& uav_texture, GPU_texture& Depth_texture);

    static void Reload();

  private:
    static void CreateRootSignature();
    static void CreatePipelineStateObject();

    static ComPtr<ID3D12RootSignature> m_rootSignature;
    static ComPtr<ID3D12PipelineState> m_PipelineState;

    static constexpr const wchar_t* c_cs_file_name = L"CS_Silhouette.dxil";

    void release_gpu_resources();
};

}  // namespace app