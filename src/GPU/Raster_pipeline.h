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
#include "DXR_pipeline.h"
#include "Mipmaps_helper.h"

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
    int SpecularLutMips;
    float TexturesAOStrength;
    int RenderFrameMips;
    glm::ivec2 FrameSize;
    bool specular_aa_enabled;
    float specular_aa_variance;
    float specular_aa_threshold;
};

struct RasterPerDrawData {
    fmat4x4 modelMatrix;
    fmat4x4 normalMatrix;
    int meshID;
    float modelScale;
    float padding[2];
};

struct DrawableSortingInfo {
    const Object* object;
    float modelScale;
    float ZDistanceToCamera;
    bool alphaBlending;
    bool transmittance;
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

    void OnEnvmapLoad(GPU_texture& envmap) override;

  private:
    void CreateRootSignatures();
    void CreatePipelineStateObjects();
    void CreateConstantBuffers();
    void ComputeDFGLut();
    void ComputeEnvmapLut(const GPU_texture& envmap);

    static void ComputeMipMaps(GPU_texture& texture);

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
    ComPtr<ID3D12PipelineState> m_alphaBlendingPipelineState;
    ComPtr<ID3D12PipelineState> m_backgroundPipelineState;
    
    Mipmaps_helper m_mip_helper{};  // todo: replace with proper blur pass

    // additional render targets
    GPU_texture m_renderTarget;
    GPU_texture m_depthTexture;
    GPU_texture m_frameCopy;

    // additional texture resources
    GPU_texture DFG_lut;  // precomputed DFG LUT for split-sum approximation of specular IBL
    GPU_texture Diffuse_lut;
    GPU_texture Specular_lut;

    std::vector<DrawableSortingInfo> m_sortedDrawables;  // reusable vector for sorting drawables every frame
    void sort_objects_for_rendering(
        const GPU_model& gpu_model, int& num_opaque_objects, int& num_transmissive_objects, int& num_alpha_blended_objects);

    UINT64 currentWidth = 0;
    UINT currentHeight = 0;

    RayProgramMode RaytracingMode = RayProgramMode::AmbientOcclusion;

    void release_gpu_resources();
};

}  // namespace app