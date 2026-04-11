#pragma once

#define NOMINMAX
#include <d3d12.h>
#include <wrl.h>
#include <cstdint>
#include <glm/fwd.hpp>

#include "../arguments.h"
#include "../cpu_framebuffer.h"
#include "GPU_model.h"
#include "helpers/d3dx12.h"

namespace app {

using glm::fvec2;
using glm::fvec3;
using glm::fvec4;
using glm::fmat4x4;

constexpr int PBR_DXR_RECURSION_DEPTH = 11;

using Microsoft::WRL::ComPtr;

struct RayGenConstantBuffer {
    fmat4x4 projectionToWorld;
    fvec4 cameraPosition;
    fvec2 subpixel_offset;
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

class GPU_pipeline {
  public:
    GPU_pipeline();
    ~GPU_pipeline();

    // Root signatures
    ComPtr<ID3D12RootSignature> m_raytracingGlobalRootSignature;
    ComPtr<ID3D12RootSignature> m_raytracingLocalRootSignature;

    // Create root signatures for the shaders.
    void CreateRootSignatures();

    void CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);

    void CreateRaytracingPipelines();

    void CreateRaytracingPipeline(D3D12_SHADER_BYTECODE libdxil,
        const wchar_t* c_anyHitShaderName, const wchar_t* c_closestHitShaderName,
        const wchar_t* c_missShaderName, ComPtr<ID3D12StateObject>& m_dxrStateObject, UINT maxRecursionDepth);

    void BuildAllShaderTables();

    void BuildShaderTables(
        const wchar_t* c_closestHitShaderName, const wchar_t* c_missShaderName, ComPtr<ID3D12StateObject>& m_dxrStateObject, 
        ComPtr<ID3D12Resource>& m_missShaderTable, ComPtr<ID3D12Resource>& m_hitGroupShaderTable);

    void CreateConstantBuffers();

    const wchar_t* c_dxilLibraryName = L"RaytracingShaders.dxil"; // DXIL library file name

    const wchar_t* c_hitGroupName = L"MyHitGroup";
    const wchar_t* c_raygenShaderName = L"RayGen";
    const wchar_t* c_anyHitRCShaderName = L"AnyHitRC";
    const wchar_t* c_closestHitAOShaderName = L"ClosestHitAO";
    const wchar_t* c_closestHitRCShaderName = L"ClosestHitRC";
    const wchar_t* c_closestHitPBRShaderName = L"ClosestHitPBR";
    const wchar_t* c_missAOShaderName = L"MissAO";
    const wchar_t* c_missEnvmapShaderName = L"MissEnvmap";

    union AlignedSceneConstantBuffer {
        RayGenConstantBuffer constants;
        uint8_t alignmentPadding[D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT];
    };
    AlignedSceneConstantBuffer* m_mappedConstantData;

    // Raytracing scene
    RayGenConstantBuffer m_rayGenCB;
    ComPtr<ID3D12Resource> m_perFrameConstants;

    // Shader table for RayGen (common)
    ComPtr<ID3D12Resource> m_rayGenShaderTable;
    
    // Shader tables RC
    ComPtr<ID3D12Resource> m_RC_missShaderTable;
    ComPtr<ID3D12Resource> m_RC_hitGroupShaderTable;

    // Shader tables AO
    ComPtr<ID3D12Resource> m_AO_missShaderTable;
    ComPtr<ID3D12Resource> m_AO_hitGroupShaderTable;

    // Shader tables PBR
    ComPtr<ID3D12Resource> m_PBR_missShaderTable;
    ComPtr<ID3D12Resource> m_PBR_hitGroupShaderTable;

    // Ray tracing pipeline states
    ComPtr<ID3D12StateObject> m_dxrStateObjectRayCaster;
    ComPtr<ID3D12StateObject> m_dxrStateObjectAmbientOcclusion;
    ComPtr<ID3D12StateObject> m_dxrStateObjectPBR;

    RayProgramMode RaytracingMode = RayProgramMode::AmbientOcclusion;

    // Ray tracing pipeline state properties, retaining the shader identifiers
    // to use in the Shader Binding Table
    ComPtr<ID3D12StateObjectProperties> m_rtStateObjectProps;

    void DoRaytracing(const GPU_model& gpu_model, const GPU_texture& envmap, const CPUFrameBuffer& framebuffer);

private:
    void release_gpu_resources();
};

}