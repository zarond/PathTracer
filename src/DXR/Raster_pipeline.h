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

namespace app {

using glm::fmat4x4;
using glm::fvec2;
using glm::fvec3;
using glm::fvec4;

using Microsoft::WRL::ComPtr;

struct RasterConstantBuffer {
    fmat4x4 projectionToWorld;
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

class Raster_pipeline {
  public:
    Raster_pipeline();
    ~Raster_pipeline();

    void SetRenderingSettings(const RenderSettings& render_settings, fvec3 origin, const fmat4x4& NDC2WorldMatrix,
        fvec2 subpixelOffset, unsigned int frameID, int iteration, float invIterationCount);

    void DoRaytracing(const GPU_model& gpu_model, const GPU_texture& envmap, const CPUFrameBuffer& framebuffer);

    void OnModelLoad(GPU_model& gpu_model);

    void OnEnvmapLoad(const GPU_texture& envmap);

  private:
    void CreateRootSignatures();
    void CreatePipelineStateObjects();

    const wchar_t* c_dxilLibraryName = L"RasterizationShaders.dxil";  // DXIL library file name

    union AlignedSceneConstantBuffer {
        RasterConstantBuffer constants;
        uint8_t alignmentPadding[D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT];
    };
    AlignedSceneConstantBuffer* m_mappedConstantData;

    // Raytracing scene
    RasterConstantBuffer m_rayGenCB;
    ComPtr<ID3D12Resource> m_perFrameConstants;

    // Pipeline state objects
    CD3DX12_VIEWPORT m_viewport;
    CD3DX12_RECT m_scissorRect;
    ComPtr<ID3D12RootSignature> m_rootSignature;
   // ComPtr<ID3D12PipelineState> m_pipelineState;
    ComPtr<ID3D12StateObject> m_pipelineState;

    void release_gpu_resources();
};

}  // namespace app