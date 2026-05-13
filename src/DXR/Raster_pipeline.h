#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <wrl.h>
#include <directx/d3dx12.h>

#include <cstdint>
#include <glm/fwd.hpp>

#include "../arguments.h"
#include "../cpu_framebuffer.h"
#include "../render_settings.h"
#include "GPU_model.h"
#include "GPU_pipeline.h"

namespace app {

using glm::fmat4x4;
using glm::fvec2;
using glm::fvec3;
using glm::fvec4;

using Microsoft::WRL::ComPtr;

struct RasterConstantBuffer {
    fmat4x4 projectionToWorld;
    fmat4x4 viewProjection;
    fvec4 cameraPosition;
    fvec2 subpixelOffset;
    unsigned int frameID;
    int iteration;
    float invIterationCount;
    int samplesPerPixel;
    float invSamplesPerPixel;
    int maxNewRaysPerBounce;
    float invMaxNewRaysPerBounce;
    int maxRayBounces;
    float envmapRotation;
};

struct RasterPerDrawData {
    fmat4x4 modelMatrix;
    fmat4x4 normalMatrix;
    int meshID;
    float padding[3];
};

class Raster_pipeline : public IRender_pipeline {
  public:
    Raster_pipeline();
    ~Raster_pipeline();

    void SetRenderingSettings(const RenderSettings& render_settings, fvec3 origin, const fmat4x4& NDC2WorldMatrix,
        const fmat4x4& ViewMatrix, const fmat4x4& ProjectionMatrix, fvec2 subpixelOffset, unsigned int frameID, int iteration,
        float invIterationCount) override;

    void DoRender(const GPU_model& gpu_model, const GPU_texture& envmap, const CPUFrameBuffer& framebuffer) override;

    void OnModelLoad(GPU_model& gpu_model) override;

    void OnEnvmapLoad(const GPU_texture& envmap) override;

  private:
    void CreateRootSignatures();
    void CreatePipelineStateObjects();
    void CreateConstantBuffers();

    void resize_render_targets(int new_width, int new_height);
    void copy_render_target_to_framebuffer(const CPUFrameBuffer& framebuffer);

    const wchar_t* c_vs_file_name = L"VS_Main.dxil";
    const wchar_t* c_ps_file_name = L"PS_Main.dxil";
    const wchar_t* c_vs_background_file_name = L"VS_Background.dxil";
    const wchar_t* c_ps_background_file_name = L"PS_Background.dxil";

    union AlignedSceneConstantBuffer {
        RasterConstantBuffer constants;
        uint8_t alignmentPadding[D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT];
    };
    AlignedSceneConstantBuffer* m_mappedConstantData;

    // Raytracing scene
    RasterConstantBuffer m_rasterCB;
    ComPtr<ID3D12Resource> m_perFrameConstants;

    // Pipeline state objects
    CD3DX12_VIEWPORT m_viewport;
    CD3DX12_RECT m_scissorRect;
    ComPtr<ID3D12RootSignature> m_rootSignature;
    ComPtr<ID3D12PipelineState> m_pipelineState;
    ComPtr<ID3D12PipelineState> m_backgroundPipelineState;

    // additional render targets
    GPU_texture m_renderTarget;
    GPU_texture m_depthTexture;

    UINT64 currentWidth = 0;
    UINT currentHeight = 0;

    RayProgramMode RaytracingMode = RayProgramMode::AmbientOcclusion;

    void release_gpu_resources();
};

}  // namespace app