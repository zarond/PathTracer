#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

class GTAO_helper {
  public:
    GTAO_helper();
    ~GTAO_helper();

    void CreateAO(GPU_texture& AO_uav_texture, GPU_texture& G_buff_texture, GPU_texture& Depth_texture,
        const fmat4x4& projectionToWorld, const fmat4x4& viewProjection, const fvec4& cameraPosition);

    float thinObjectFactor = 0.0f;

  private:
    void CreateRootSignature();
    void CreatePipelineStateObject();
    void CreateDescriptorHeap();

    ComPtr<ID3D12RootSignature> m_rootSignature;
    ComPtr<ID3D12PipelineState> m_PipelineState;

    const wchar_t* c_cs_file_name = L"CS_GTAO.dxil";

    void release_gpu_resources();
};

}  // namespace app