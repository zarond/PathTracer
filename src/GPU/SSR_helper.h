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

    void CalculateSSR(GPU_texture& SSR_uav_texture, GPU_texture& G_buff_texture,
        GPU_texture& VelocityBuffer, GPU_texture & DepthUAVTexture,
        GPU_texture& DepthUAVTexture_previous, GPU_texture& Frame_texture, const fmat4x4& Projection,
        const fmat4x4& invProjection, const fmat4x4& ViewProjection, const fmat4x4& ViewProjection_prev,
        unsigned int FrameID);

    void ResetFrameCounter();

    bool DenoiseEnabled = true;
    bool RayReuseEnabled = true;
    bool zeroAlphaMotionCleanup = true;
    float DepthThreshold = 0.01f;
    float MaxRoughness = 0.8f;
    float SSR_GGXClamp = 0.0f;
    bool UsePrefiltering = true;
    float PrefilteringDistance = 0.3f;
    bool ParallaxReprojection = true;

    static void Reload();

  private:
    static void CreateRootSignature();
    static void CreatePipelineStateObject();
    void ResizeInnerResource(int new_width, int new_height);

    static ComPtr<ID3D12RootSignature> m_rootSignature;
    static ComPtr<ID3D12PipelineState> m_TracePipelineState;
    static ComPtr<ID3D12PipelineState> m_ResolvePipelineState;

    GPU_texture m_SSR_texture_previous;
    GPU_texture m_SSR_buff;
    GPU_texture m_distance_texture;
    UINT64 currentWidth = 0;
    UINT currentHeight = 0;

    int consecutive_frame_count = 0;
    fmat4x4 Projection_previous{};

    static constexpr const wchar_t* c_cs_trace_file_name = L"CS_SSR_trace.dxil";
    static constexpr const wchar_t* c_cs_resolve_file_name = L"CS_SSR_resolve.dxil";

    void release_gpu_resources();
};

}  // namespace app