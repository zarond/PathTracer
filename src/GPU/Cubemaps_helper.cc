#include "Cubemaps_helper.h"

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    OutputViewSlot = 0,
    EnvmapTex,
    RootConstants,

    Count
};
}

namespace app {

struct LutCSInput {
    int currentMip;
    int numMips;
    float roughness;
};

EnvCube_helper::EnvCube_helper() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

EnvCube_helper::~EnvCube_helper() { 
    ReleaseTemporaryGPUResources();
    release_gpu_resources(); 
}

void EnvCube_helper::CreateDiffuseEnvmapCube(const GPU_texture& envmap) {
    Diffuse_lut.release_gpu_resource();
    TEXTURE_TRAITS flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV | TEXTURE_TRAITS::Cubemap;
    Diffuse_lut = GPU_texture{Diffuse_size, Diffuse_size, flags};

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()};
    commandList->SetDescriptorHeaps(1, desc_heap);

    commandList->SetPipelineState(m_DiffusePipelineState.Get());
    commandList->SetComputeRootSignature(m_rootSignature.Get());

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputViewSlot, Diffuse_lut.GetUAVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::EnvmapTex, envmap.GetSRVHandle());

    commandList->Dispatch(Diffuse_size / 8, Diffuse_size / 8, 6);
}

GPU_texture&& EnvCube_helper::GetDiffuseEnvmapCube() { return std::move(Diffuse_lut); }

GPU_texture EnvCube_helper::GetBlankSRVDiffuseTexture() {
    return GPU_texture{Diffuse_size, Diffuse_size, TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::Cubemap};
}
GPU_texture EnvCube_helper::GetBlankSRVSpecularTexture() {
    return
        GPU_texture{Specular_size, Specular_size, TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::Cubemap | TEXTURE_TRAITS::AllocateMips};
}

void EnvCube_helper::CreateSpecularEnvmapCube(const GPU_texture& envmap) {
    Specular_lut.release_gpu_resource();
    TEXTURE_TRAITS flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV | TEXTURE_TRAITS::Cubemap | TEXTURE_TRAITS::AllocateMips;
    Specular_lut = GPU_texture{Specular_size, Specular_size, flags};

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()};
    commandList->SetDescriptorHeaps(1, desc_heap);

    commandList->SetPipelineState(m_SpecularPipelineState.Get());
    commandList->SetComputeRootSignature(m_rootSignature.Get());

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::EnvmapTex, envmap.GetSRVHandle());

    m_mip_uav_handles.resize(SpecularMips);

    for (int mipLevel = 0; mipLevel < SpecularMips; ++mipLevel) {
        D3D_Handle_Pair handles{};
        d3d_ctx.m_SrvDescHeapAlloc.Alloc(&handles);

        Specular_lut.GetUAVHandleForMipLevel(mipLevel, handles.cpuHandle);

        m_mip_uav_handles[mipLevel] = handles;
    }

    for (int mipLevel = 0; mipLevel < SpecularMips; ++mipLevel) {
        auto dimension = GPU_texture::GetMipDimension(Specular_size, mipLevel);
        auto uav_gpu_handle = m_mip_uav_handles[mipLevel].gpuHandle;

        commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputViewSlot, uav_gpu_handle);

        float roughness = static_cast<float>(mipLevel) / static_cast<float>(SpecularMips - 1);
        LutCSInput input{mipLevel, SpecularMips, roughness};
        constexpr int inputSizeInInt = sizeof(LutCSInput) / 4;
        commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);
    
        commandList->Dispatch((dimension + 7) / 8, (dimension + 7) / 8, 6);
    }
}

void EnvCube_helper::ReleaseTemporaryGPUResources() {
    D3DContext& d3d_ctx = D3DContext::Get();
    for (const auto& handle : m_mip_uav_handles) {
        d3d_ctx.m_SrvDescHeapAlloc.Free(handle);
    }
    m_mip_uav_handles.clear();
}

GPU_texture&& EnvCube_helper::GetSpecularEnvmapCube() { return std::move(Specular_lut); }

void EnvCube_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 output texture
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 1);  // 1 input texture
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);     // 1 constant buffer.

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::OutputViewSlot].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::EnvmapTex].InitAsDescriptorTable(1, &ranges[1]);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(LutCSInput) / 4, 0);

    D3D12_STATIC_SAMPLER_DESC envmap_sampler = {};  // Envmap static sampler.
    envmap_sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    envmap_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    envmap_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    envmap_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    envmap_sampler.MaxLOD = D3D12_FLOAT32_MAX;
    envmap_sampler.ShaderRegister = 0;  // s0
    envmap_sampler.RegisterSpace = 1;
    envmap_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 1, &envmap_sampler, flags);

    SerializeAndCreateRootSignature(rootSignatureDesc, &m_rootSignature);
}

void EnvCube_helper::CreatePipelineStateObject() {
    auto [cs_diffuse_shaderBlob, cs_diffuse_bytecode] = LoadShader(c_cs_diffuse_file_name);
    auto [cs_specular_shaderBlob, cs_specular_bytecode] = LoadShader(c_cs_specular_file_name);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDesc = {};
    computePsoDesc.pRootSignature = m_rootSignature.Get();  // Must match shader layout
    computePsoDesc.CS = cs_diffuse_bytecode;
    computePsoDesc.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
    computePsoDesc.NodeMask = 1;

    ThrowIfFailed(device->CreateComputePipelineState(&computePsoDesc, IID_PPV_ARGS(&m_DiffusePipelineState)));

    computePsoDesc.CS = cs_specular_bytecode;
    ThrowIfFailed(device->CreateComputePipelineState(&computePsoDesc, IID_PPV_ARGS(&m_SpecularPipelineState)));
}

void EnvCube_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_DiffusePipelineState.Reset();
    m_SpecularPipelineState.Reset();
}

}  // namespace app