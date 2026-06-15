#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

class Reprojection_helper {
  public:
    Reprojection_helper();
    ~Reprojection_helper();

    void Reproject(GPU_texture& old_texture, GPU_texture& old_depth_texture, GPU_texture& new_texture,
        GPU_texture& new_depth_texture, const fmat4x4& Projection, 
        const fmat4x4& invViewProjection, const fmat4x4& ViewProjection_previous, const fmat4x4& Projection_previous, 
        float new_value_mix_factor, float depth_threshold, bool weak_depth_condition, GPU_texture* SSR_buffer);

    static bool DebugMode;

    static void Reload();

  private:
    static void CreateRootSignature();
    static void CreatePipelineStateObject();

    static ComPtr<ID3D12RootSignature> m_rootSignature;
    static ComPtr<ID3D12PipelineState> m_PipelineState;

    static constexpr const wchar_t* c_cs_file_name = L"CS_Reprojection.dxil";

    void release_gpu_resources();
};

}  // namespace app