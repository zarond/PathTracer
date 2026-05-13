#include "GPU_pipeline.h"

#include <d3dcompiler.h>

#include <iostream>

#include "../d3d_context.h"
#include "helpers/DXSampleHelper.h"
#include "helpers/DirectXRaytracingHelper.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    OutputViewSlot = 0,
    AccelerationStructureSlot,
    SceneConstantSlot,
    IndexBufferSlot,
    VertexBufferSlot,
    IndicesOffsetBufferSlot,
    MaterialsBufferSlot,
    EnvmapTex,

    Count
};
}

namespace LocalRootSignatureParams {
enum Value : int {
    ViewportConstantSlot = 0,

    Count
};
}

namespace app {

using namespace glm;

void SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig) {
    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;
    ComPtr<ID3DBlob> blob;
    ComPtr<ID3DBlob> error;

    ThrowIfFailed(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &error),
        error ? static_cast<wchar_t*>(error->GetBufferPointer()) : nullptr);
    ThrowIfFailed(device->CreateRootSignature(1, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&(*rootSig))));
}

GPU_pipeline::GPU_pipeline() {
    CreateRootSignatures();
    CreateRaytracingPipelines();
    CreateConstantBuffers();
    BuildAllShaderTables();
}

GPU_pipeline::~GPU_pipeline() { release_gpu_resources(); }

void GPU_pipeline::CreateRootSignatures() {
    // Global Root Signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    {
        CD3DX12_DESCRIPTOR_RANGE ranges[6];                        // Perfomance TIP: Order from most frequent to least frequent.
        ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);     // 1 output texture
        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1);     // 2 static index buffer.
        ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 2);     // 3 static vertex buffer.
        ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 3);     // 4 static index offsets buffer.
        ranges[4].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 4);     // 5 materials buffer.
        ranges[5].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 1);  // Envmap texture.

        D3D12_STATIC_SAMPLER_DESC envmap_sampler = {};  // Envmap static sampler.
        envmap_sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
        envmap_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
        envmap_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
        envmap_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
        envmap_sampler.ShaderRegister = 0;  // s0
        envmap_sampler.RegisterSpace = 1;
        envmap_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

        D3D12_STATIC_SAMPLER_DESC default_sampler = envmap_sampler;  // Default static sampler.
        default_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
        default_sampler.ShaderRegister = 1;  // s1

        D3D12_STATIC_SAMPLER_DESC samplers[] = {envmap_sampler, default_sampler};

        CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
        rootParameters[GlobalRootSignatureParams::OutputViewSlot].InitAsDescriptorTable(1, &ranges[0]);
        rootParameters[GlobalRootSignatureParams::AccelerationStructureSlot].InitAsShaderResourceView(0);
        rootParameters[GlobalRootSignatureParams::SceneConstantSlot].InitAsConstantBufferView(0);
        rootParameters[GlobalRootSignatureParams::IndexBufferSlot].InitAsDescriptorTable(1, &ranges[1]);
        rootParameters[GlobalRootSignatureParams::VertexBufferSlot].InitAsDescriptorTable(1, &ranges[2]);
        rootParameters[GlobalRootSignatureParams::IndicesOffsetBufferSlot].InitAsDescriptorTable(1, &ranges[3]);
        rootParameters[GlobalRootSignatureParams::MaterialsBufferSlot].InitAsDescriptorTable(1, &ranges[4]);
        rootParameters[GlobalRootSignatureParams::EnvmapTex].InitAsDescriptorTable(1, &ranges[5]);

        auto flags = D3D12_ROOT_SIGNATURE_FLAG_CBV_SRV_UAV_HEAP_DIRECTLY_INDEXED;
        CD3DX12_ROOT_SIGNATURE_DESC globalRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters, 2, samplers, flags);
        SerializeAndCreateRaytracingRootSignature(globalRootSignatureDesc, &m_raytracingGlobalRootSignature);
    }

    // Local Root Signature
    // This is a root signature that enables a shader to have unique arguments that come from shader tables.
    {
        CD3DX12_ROOT_PARAMETER rootParameters[LocalRootSignatureParams::Count];
        rootParameters[LocalRootSignatureParams::ViewportConstantSlot].InitAsConstants(SizeOfInUint32(m_rayGenCB), 1);
        CD3DX12_ROOT_SIGNATURE_DESC localRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
        localRootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE;
        SerializeAndCreateRaytracingRootSignature(localRootSignatureDesc, &m_raytracingLocalRootSignature);
    }
}

// Local root signature and shader association
// This is a root signature that enables a shader to have unique arguments that come from shader tables.
void GPU_pipeline::CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline) {
    // Hit group and miss shaders in this sample are not using a local root signature and thus one is not associated with them.

    // Local root signature to be used in a ray gen shader.
    {
        auto localRootSignature = raytracingPipeline->CreateSubobject<CD3DX12_LOCAL_ROOT_SIGNATURE_SUBOBJECT>();
        localRootSignature->SetRootSignature(m_raytracingLocalRootSignature.Get());
        // Shader association
        auto rootSignatureAssociation = raytracingPipeline->CreateSubobject<CD3DX12_SUBOBJECT_TO_EXPORTS_ASSOCIATION_SUBOBJECT>();
        rootSignatureAssociation->SetSubobjectToAssociate(*localRootSignature);
        rootSignatureAssociation->AddExport(c_raygenShaderName);
    }
}

// Create a raytracing pipeline state object (RTPSO).
// An RTPSO represents a full set of shaders reachable by a DispatchRays() call,
// with all configuration options resolved, such as local signatures and other state.
void GPU_pipeline::CreateRaytracingPipelines() {
    // Create 7 subobjects that combine into a RTPSO:
    // Subobjects need to be associated with DXIL exports (i.e. shaders) either by way of default or explicit associations.
    // Default association applies to every exported shader entrypoint that doesn't have any of the same type of subobject associated with it.
    // This simple sample utilizes default shader association except for local root signature subobject
    // which has an explicit association specified purely for demonstration purposes.
    // 1 - DXIL library
    // 1 - Triangle hit group
    // 1 - Shader config
    // 2 - Local root signature and association
    // 1 - Global root signature
    // 1 - Pipeline config

    // DXIL library
    // This contains the shaders and their entrypoints for the state object.
    // Since shaders are not considered a subobject, they need to be passed in via DXIL library subobjects.
    ComPtr<ID3DBlob> shaderBlob;
    auto hr = D3DReadFileToBlob(c_dxilLibraryName, &shaderBlob);
    if (FAILED(hr)) {
        std::wcout << "Failed to read DXIL library: " << c_dxilLibraryName << std::endl;
        throw HrException(hr);
    }
    D3D12_SHADER_BYTECODE libdxil = {
        .pShaderBytecode = shaderBlob->GetBufferPointer(),
        .BytecodeLength = shaderBlob->GetBufferSize(),
    };

    CreateRaytracingPipeline(libdxil, c_anyHitRCShaderName, c_closestHitRCShaderName, c_missEnvmapShaderName,
        m_dxrStateObjectRayCaster, PBR_DXR_RECURSION_DEPTH);  // RayCaster
    CreateRaytracingPipeline(libdxil, nullptr, c_closestHitAOShaderName, c_missAOShaderName, 
        m_dxrStateObjectAmbientOcclusion, 2);  // AO
    CreateRaytracingPipeline(libdxil, c_anyHitRCShaderName, c_closestHitPBRShaderName, c_missEnvmapShaderName,
        m_dxrStateObjectPBR, PBR_DXR_RECURSION_DEPTH);  // PBR
}

void GPU_pipeline::CreateRaytracingPipeline(D3D12_SHADER_BYTECODE libdxil, const wchar_t* c_anyHitShaderName,
    const wchar_t* c_closestHitShaderName, const wchar_t* c_missShaderName, ComPtr<ID3D12StateObject>& m_dxrStateObject,
    UINT maxRecursionDepth) {
    CD3DX12_STATE_OBJECT_DESC raytracingPipeline{D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE};
    auto lib = raytracingPipeline.CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
    lib->SetDXILLibrary(&libdxil);
    // Define which shader exports to surface from the library.
    // If no shader exports are defined for a DXIL library subobject, all shaders will be surfaced.
    // In this sample, this could be omitted for convenience since the sample uses all shaders in the library.
    {
        lib->DefineExport(c_raygenShaderName);
        lib->DefineExport(c_closestHitShaderName);
        lib->DefineExport(c_missShaderName);
        if (c_anyHitShaderName) {
            lib->DefineExport(c_anyHitShaderName);
        }
    }

    // Triangle hit group
    // A hit group specifies closest hit, any hit and intersection shaders to be executed when a ray intersects the geometry's
    // triangle/AABB. In this sample, we only use triangle geometry with a closest hit shader, so others are not set.
    auto hitGroup = raytracingPipeline.CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
    hitGroup->SetClosestHitShaderImport(c_closestHitShaderName);
    if (c_anyHitShaderName) {
        hitGroup->SetAnyHitShaderImport(c_anyHitShaderName);
    }
    hitGroup->SetHitGroupExport(c_hitGroupName);
    hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_TRIANGLES);

    // Shader config
    // Defines the maximum sizes in bytes for the ray payload and attribute structure.
    auto shaderConfig = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_SHADER_CONFIG_SUBOBJECT>();
    UINT payloadSize = 6 * sizeof(float) + 2 * sizeof(int);  // float3 color + float3 absorption + int depth + int iteration
    UINT attributeSize = 2 * sizeof(float);                  // float2 barycentrics
    shaderConfig->Config(payloadSize, attributeSize);

    // Local root signature and shader association
    CreateLocalRootSignatureSubobjects(&raytracingPipeline);
    // This is a root signature that enables a shader to have unique arguments that come from shader tables.

    // Global root signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    auto globalRootSignature = raytracingPipeline.CreateSubobject<CD3DX12_GLOBAL_ROOT_SIGNATURE_SUBOBJECT>();
    globalRootSignature->SetRootSignature(m_raytracingGlobalRootSignature.Get());

    // Pipeline config
    // Defines the maximum TraceRay() recursion depth.
    auto pipelineConfig = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_PIPELINE_CONFIG_SUBOBJECT>();
    // PERFOMANCE TIP: Set max recursion depth as low as needed
    // as drivers may apply optimization strategies for low recursion depths.
    pipelineConfig->Config(maxRecursionDepth);

#if _DEBUG
    PrintStateObjectDesc(raytracingPipeline);
#endif

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    // Create the state object.
    ThrowIfFailed(device->CreateStateObject(raytracingPipeline, IID_PPV_ARGS(&m_dxrStateObject)),
        L"Couldn't create DirectX Raytracing state object.\n");
}

void GPU_pipeline::BuildAllShaderTables() {
    BuildShaderTables(c_closestHitRCShaderName, c_missEnvmapShaderName, m_dxrStateObjectRayCaster, m_RC_missShaderTable,
        m_RC_hitGroupShaderTable);      // RayCaster
    BuildShaderTables(c_closestHitAOShaderName, c_missAOShaderName, m_dxrStateObjectAmbientOcclusion, m_AO_missShaderTable,
        m_AO_hitGroupShaderTable);      // AO
    BuildShaderTables(c_closestHitAOShaderName, c_missEnvmapShaderName, m_dxrStateObjectPBR, m_PBR_missShaderTable,
        m_PBR_hitGroupShaderTable);     // PBR
}

void GPU_pipeline::BuildShaderTables(const wchar_t* c_closestHitShaderName, const wchar_t* c_missShaderName,
    ComPtr<ID3D12StateObject>& m_dxrStateObject, ComPtr<ID3D12Resource>& m_missShaderTable,
    ComPtr<ID3D12Resource>& m_hitGroupShaderTable) {
    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    void* rayGenShaderIdentifier;
    void* missShaderIdentifier;
    void* hitGroupShaderIdentifier;

    auto GetShaderIdentifiers = [&](auto* stateObjectProperties) {
        rayGenShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_raygenShaderName);
        missShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_missShaderName);
        hitGroupShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_hitGroupName);
    };

    // Get shader identifiers.
    UINT shaderIdentifierSize;
    {
        ComPtr<ID3D12StateObjectProperties> stateObjectProperties;
        ThrowIfFailed(m_dxrStateObject.As(&stateObjectProperties));
        GetShaderIdentifiers(stateObjectProperties.Get());
        shaderIdentifierSize = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;
    }

    // Ray gen shader table
    {
        struct RootArguments {
            RayGenConstantBuffer cb;
        } rootArguments;
        rootArguments.cb = m_rayGenCB;

        UINT numShaderRecords = 1;
        UINT shaderRecordSize = shaderIdentifierSize + sizeof(rootArguments);
        ShaderTable rayGenShaderTable(device.Get(), numShaderRecords, shaderRecordSize, L"RayGenShaderTable");
        rayGenShaderTable.push_back(
            ShaderRecord(rayGenShaderIdentifier, shaderIdentifierSize, &rootArguments, sizeof(rootArguments)));
        m_rayGenShaderTable = rayGenShaderTable.GetResource();
    }

    // Miss shader table
    {
        UINT numShaderRecords = 1;
        UINT shaderRecordSize = shaderIdentifierSize;
        ShaderTable missShaderTable(device.Get(), numShaderRecords, shaderRecordSize, L"MissShaderTable");
        missShaderTable.push_back(ShaderRecord(missShaderIdentifier, shaderIdentifierSize));
        m_missShaderTable = missShaderTable.GetResource();
    }

    // Hit group shader table
    {
        UINT numShaderRecords = 1;
        UINT shaderRecordSize = shaderIdentifierSize;
        ShaderTable hitGroupShaderTable(device.Get(), numShaderRecords, shaderRecordSize, L"HitGroupShaderTable");
        hitGroupShaderTable.push_back(ShaderRecord(hitGroupShaderIdentifier, shaderIdentifierSize));
        m_hitGroupShaderTable = hitGroupShaderTable.GetResource();
    }
}

void GPU_pipeline::release_gpu_resources() {
    m_raytracingGlobalRootSignature.Reset();
    m_raytracingLocalRootSignature.Reset();

    m_dxrStateObjectRayCaster.Reset();
    m_dxrStateObjectAmbientOcclusion.Reset();
    m_dxrStateObjectPBR.Reset();

    m_perFrameConstants.Reset();

    m_RC_missShaderTable.Reset();
    m_RC_hitGroupShaderTable.Reset();

    m_AO_missShaderTable.Reset();
    m_AO_hitGroupShaderTable.Reset();

    m_PBR_missShaderTable.Reset();
    m_PBR_hitGroupShaderTable.Reset();

    m_rayGenShaderTable.Reset();

    m_rtStateObjectProps.Reset();
}

void GPU_pipeline::CreateConstantBuffers() {
    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;
    // auto frameCount = m_deviceResources->GetBackBufferCount();

    // Create the constant buffer memory and map the CPU and GPU addresses
    const D3D12_HEAP_PROPERTIES uploadHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD);

    // Allocate one constant buffer per frame, since it gets updated every frame.
    size_t cbSize = /* frameCount **/ sizeof(AlignedSceneConstantBuffer);
    const D3D12_RESOURCE_DESC constantBufferDesc = CD3DX12_RESOURCE_DESC::Buffer(cbSize);

    ThrowIfFailed(device->CreateCommittedResource(&uploadHeapProperties, D3D12_HEAP_FLAG_NONE, &constantBufferDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&m_perFrameConstants)));

    // Map the constant buffer and cache its heap pointers.
    // We don't unmap this until the app closes. Keeping buffer mapped for the lifetime of the resource is okay.
    CD3DX12_RANGE readRange(0, 0);  // We do not intend to read from this resource on the CPU.
    ThrowIfFailed(m_perFrameConstants->Map(0, nullptr, reinterpret_cast<void**>(&m_mappedConstantData)));
}

void GPU_pipeline::SetRenderingSettings(const RenderSettings& render_settings, fvec3 origin, const fmat4x4& NDC2WorldMatrix,
    const fmat4x4& ViewMatrix, const fmat4x4& ProjectionMatrix, fvec2 subpixelOffset, unsigned int frameID, int iterationCount,
    float invIterationCount) {
    m_rayGenCB.cameraPosition = xyz1(origin);
    m_rayGenCB.projectionToWorld = NDC2WorldMatrix;
    m_rayGenCB.subpixelOffset = subpixelOffset;
    m_rayGenCB.frameID = frameID;
    m_rayGenCB.iteration = iterationCount;
    m_rayGenCB.invIterationCount = invIterationCount;
    m_rayGenCB.samplesPerPixel = render_settings.samplesPerPixel;
    m_rayGenCB.invSamplesPerPixel = 1.0f / static_cast<float>(render_settings.samplesPerPixel);
    m_rayGenCB.maxNewRaysPerBounce = render_settings.maxNewRaysPerBounce;
    m_rayGenCB.invMaxNewRaysPerBounce = 1.0f / render_settings.maxNewRaysPerBounce;
    m_rayGenCB.maxRayBounces = render_settings.maxRayBounces;
    m_rayGenCB.envmapRotation = render_settings.envmapRotation;
    RaytracingMode = render_settings.programMode;
}

void GPU_pipeline::DoRender(const GPU_model& gpu_model, const GPU_texture& envmap, const CPUFrameBuffer& framebuffer) {
    framebuffer.transition_from_srv_to_uav();

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    UINT m_width = framebuffer.width();
    UINT m_height = framebuffer.height();
    auto DispatchRays = [&](auto* commandList, auto* stateObject, D3D12_DISPATCH_RAYS_DESC* dispatchDesc,
        ComPtr<ID3D12Resource> hitGroupShaderTable, ComPtr<ID3D12Resource> missShaderTable) 
    {
        // Since each shader table has only one shader record, the stride is same as the size.
        dispatchDesc->HitGroupTable.StartAddress = hitGroupShaderTable->GetGPUVirtualAddress();
        dispatchDesc->HitGroupTable.SizeInBytes = hitGroupShaderTable->GetDesc().Width;
        dispatchDesc->HitGroupTable.StrideInBytes = dispatchDesc->HitGroupTable.SizeInBytes;
        dispatchDesc->MissShaderTable.StartAddress = missShaderTable->GetGPUVirtualAddress();
        dispatchDesc->MissShaderTable.SizeInBytes = missShaderTable->GetDesc().Width;
        dispatchDesc->MissShaderTable.StrideInBytes = dispatchDesc->MissShaderTable.SizeInBytes;
        dispatchDesc->RayGenerationShaderRecord.StartAddress = m_rayGenShaderTable->GetGPUVirtualAddress();
        dispatchDesc->RayGenerationShaderRecord.SizeInBytes = m_rayGenShaderTable->GetDesc().Width;
        dispatchDesc->Width = m_width;
        dispatchDesc->Height = m_height;
        dispatchDesc->Depth = 1;
        commandList->SetPipelineState1(stateObject);
        commandList->DispatchRays(dispatchDesc);
    };

    commandList->SetComputeRootSignature(m_raytracingGlobalRootSignature.Get());

    // Copy the updated scene constant buffer to GPU.
    memcpy(&m_mappedConstantData->constants, &m_rayGenCB, sizeof(m_rayGenCB));
    auto cbGpuAddress = m_perFrameConstants->GetGPUVirtualAddress();
    commandList->SetComputeRootConstantBufferView(GlobalRootSignatureParams::SceneConstantSlot, cbGpuAddress);

    // Set index and successive vertex buffer decriptor tables
    commandList->SetComputeRootDescriptorTable(
        GlobalRootSignatureParams::IndexBufferSlot, gpu_model.combined_mesh_indices.gpuDescriptorHandle);
    commandList->SetComputeRootDescriptorTable(
        GlobalRootSignatureParams::VertexBufferSlot, gpu_model.combined_mesh_vertices.gpuDescriptorHandle);
    commandList->SetComputeRootDescriptorTable(
        GlobalRootSignatureParams::IndicesOffsetBufferSlot, gpu_model.combined_mesh_offsets.gpuDescriptorHandle);
    commandList->SetComputeRootDescriptorTable(
        GlobalRootSignatureParams::MaterialsBufferSlot, gpu_model.materials_array.gpuDescriptorHandle);

    // Envmap texture and sampler
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::EnvmapTex, envmap.GetSRVHandle());

    // Bind the heaps, acceleration structure and dispatch rays.
    D3D12_DISPATCH_RAYS_DESC dispatchDesc = {};
    // commandList->SetDescriptorHeaps(1, m_descriptorHeap.GetAddressOf());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputViewSlot, framebuffer.uav_gpu_handle);
    commandList->SetComputeRootShaderResourceView(
        GlobalRootSignatureParams::AccelerationStructureSlot, gpu_model.GetGPUVirtualAddress());

    switch (RaytracingMode) {
        case RayProgramMode::AmbientOcclusion:
            DispatchRays(commandList.Get(), m_dxrStateObjectAmbientOcclusion.Get(), &dispatchDesc, m_AO_hitGroupShaderTable,
                m_AO_missShaderTable);
            break;
        case RayProgramMode::PBR:
            DispatchRays(
                commandList.Get(), m_dxrStateObjectPBR.Get(), &dispatchDesc, m_PBR_hitGroupShaderTable, m_PBR_missShaderTable);
            break;
        case RayProgramMode::RayCaster:
        default:
            DispatchRays(commandList.Get(), m_dxrStateObjectRayCaster.Get(), &dispatchDesc, m_RC_hitGroupShaderTable,
                m_RC_missShaderTable);
    }

    framebuffer.transition_from_uav_to_srv();
}

// no actions required for a model in this raytracing pipeline, because it doesn't use mips
void GPU_pipeline::OnModelLoad(GPU_model& gpu_model) {} 

// no actions required for envmap in this raytracing pipeline, because it doesn't use mips and is not a cubemap
void GPU_pipeline::OnEnvmapLoad(const GPU_texture& envmap) {} 

}  // namespace app