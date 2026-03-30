#pragma once

#include "../cpu_framebuffer.h"
#include "GPU_model.h"

#include <d3d12.h>

#include "helpers/d3dx12.h"

#define NOMINMAX
//#include <windows.h>
#include <wrl.h>

namespace app {

using Microsoft::WRL::ComPtr;

struct RayGenConstantBuffer {
    fmat4x4 projectionToWorld;
    fvec4 cameraPosition;
    fvec2 subpixel_offset;
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

    //ComPtr<ID3D12RootSignature> CreateRayGenSignature();
    //ComPtr<ID3D12RootSignature> CreateMissSignature();
    //ComPtr<ID3D12RootSignature> CreateHitSignature();

    void CreateRaytracingPipeline();

    void BuildShaderTables();

    void CreateConstantBuffers();

    const wchar_t* c_dxilLibraryName = L"RaytracingSimpleShaders.dxil"; // DXIL library file name

    const wchar_t* c_hitGroupName = L"MyHitGroup";
    const wchar_t* c_raygenShaderName = L"RayGen";
    const wchar_t* c_closestHitShaderName = L"ClosestHit";
    const wchar_t* c_missShaderName = L"Miss";

    union AlignedSceneConstantBuffer {
        RayGenConstantBuffer constants;
        uint8_t alignmentPadding[D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT];
    };
    AlignedSceneConstantBuffer* m_mappedConstantData;

    // Raytracing scene
    RayGenConstantBuffer m_rayGenCB;
    ComPtr<ID3D12Resource> m_perFrameConstants;

    //ComPtr<IDxcBlob> m_rayGenLibrary;
    //ComPtr<IDxcBlob> m_hitLibrary;
    //ComPtr<IDxcBlob> m_missLibrary;

    //ComPtr<ID3D12RootSignature> m_rayGenSignature;
    //ComPtr<ID3D12RootSignature> m_hitSignature;
    //ComPtr<ID3D12RootSignature> m_missSignature;

    // Shader tables
    ComPtr<ID3D12Resource> m_missShaderTable;
    ComPtr<ID3D12Resource> m_hitGroupShaderTable;
    ComPtr<ID3D12Resource> m_rayGenShaderTable;

    // Ray tracing pipeline state
    ComPtr<ID3D12StateObject> m_dxrStateObject;
    // Ray tracing pipeline state properties, retaining the shader identifiers
    // to use in the Shader Binding Table
    ComPtr<ID3D12StateObjectProperties> m_rtStateObjectProps;

    void DoRaytracing(
        const GPU_model& gpu_model, const CPUFrameBuffer& framebuffer, const fmat4x4& NDC2WorldMatrix, const fvec4& origin);

private:
    void release_gpu_resources();
};

}