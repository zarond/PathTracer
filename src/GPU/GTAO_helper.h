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

    void CreateAO(GPU_texture& AO_uav_texture, GPU_texture& G_buff_texture, GPU_texture& VelocityBuffer,
        GPU_texture& DepthUAVTexture, GPU_texture& DepthUAVTexture_previous, 
        const fmat4x4& Projection, const fmat4x4& invProjection,
        const fmat4x4& ViewProjection, unsigned int FrameID);

    void ResetFrameCounter();

    float AODistance = 1.0f;
    bool DenoiseEnabled = true;
    float DepthThreshold = 0.01f;
    float AOSpatialSigma = 1.0f;
    float AODepthSigma = 0.01;
    float AONormalSigma = 0.01;

    static void Reload();

  private:
    static void CreateRootSignature();
    static void CreatePipelineStateObject();
    void ResizeInnerResource(int new_width, int new_height);

    static ComPtr<ID3D12RootSignature> m_rootSignature;
    static ComPtr<ID3D12PipelineState> m_PipelineState;

    GPU_texture m_SpatialDenoised;
    GPU_texture m_SpatialDenoised_previous;
    UINT64 currentWidth = 0;
    UINT currentHeight = 0;
    int consecutive_frame_count = 0;

    fmat4x4 ViewProjection_previous{};
    fmat4x4 Projection_previous{};

    static constexpr const wchar_t* c_cs_file_name = L"CS_GTAO.dxil";

    void release_gpu_resources();
};

}  // namespace app