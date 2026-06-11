#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

class SSR_helper {
  public:
    SSR_helper();
    ~SSR_helper();

    void CalculateSSR(GPU_texture& SSR_uav_texture, GPU_texture& G_buff_texture, GPU_texture& DepthUAVTexture,
        GPU_texture& DepthUAVTexture_previous,
        GPU_texture& Frame_texture, const fmat4x4& Projection, const fmat4x4& invProjection, const fmat4x4& ViewProjection,
        unsigned int FrameID);

    void ResetFrameCounter();

    bool DenoiseEnabled = true;
    float DepthThreshold = 0.01f;
    float MaxRoughness = 0.8f;
    float SSR_GGXClamp = 0.0f;
    bool UsePrefiltering = true;
    float PrefilteringDistance = 0.1f;

  private:
    void CreateRootSignature();
    void CreatePipelineStateObject();
    void CreateDescriptorHeap();
    void ResizeInnerResource(int new_width, int new_height);

    ComPtr<ID3D12RootSignature> m_rootSignature;
    ComPtr<ID3D12PipelineState> m_PipelineState;

    GPU_texture m_SSR_texture_previous;
    GPU_texture m_SSR_reflection_depth;
    GPU_texture m_Frame_reprojected;
    UINT64 currentWidth = 0;
    UINT currentHeight = 0;

    int consecutive_frame_count = 0;
    fmat4x4 ViewProjection_previous{};
    fmat4x4 Projection_previous{};

    const wchar_t* c_cs_file_name = L"CS_SSR.dxil";

    void release_gpu_resources();
};

}  // namespace app