#include "Cubemaps_helper.h"

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    OutputViewSlot = 0,
    EnvmapTex,

    Count
};
}

namespace app {

EnvCube_helper::EnvCube_helper() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

EnvCube_helper::~EnvCube_helper() { release_gpu_resources(); }

void EnvCube_helper::CreateDiffuseEnvmapCube(const GPU_texture& envmap) {
    Diffuse_lut.release_gpu_resource();
    Diffuse_lut = GPU_texture{Diffuse_size, Diffuse_size, true, false, true, false, false, true};

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()};
    commandList->SetDescriptorHeaps(1, desc_heap);

    commandList->SetPipelineState(m_pipelineState.Get());
    commandList->SetComputeRootSignature(m_rootSignature.Get());

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputViewSlot, Diffuse_lut.GetUAVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::EnvmapTex, envmap.GetSRVHandle());

    commandList->Dispatch(Diffuse_size / 8, Diffuse_size / 8, 6);
}

GPU_texture&& EnvCube_helper::GetDiffuseEnvmapCube() { return std::move(Diffuse_lut); }

void EnvCube_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 output texture
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 1);  // 1 input texture

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::OutputViewSlot].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::EnvmapTex].InitAsDescriptorTable(1, &ranges[1]);

    D3D12_STATIC_SAMPLER_DESC envmap_sampler = {};  // Envmap static sampler.
    envmap_sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    envmap_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    envmap_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    envmap_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    envmap_sampler.ShaderRegister = 0;  // s0
    envmap_sampler.RegisterSpace = 1;
    envmap_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 1, &envmap_sampler, flags);

    SerializeAndCreateRaytracingRootSignature(rootSignatureDesc, &m_rootSignature);
}

void EnvCube_helper::CreatePipelineStateObject() {
    auto [cs_shaderBlob, cs_bytecode] = LoadShader(c_cs_file_name);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDesc = {};
    computePsoDesc.pRootSignature = m_rootSignature.Get();  // Must match shader layout
    computePsoDesc.CS = cs_bytecode;
    computePsoDesc.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
    computePsoDesc.NodeMask = 1;

    ThrowIfFailed(device->CreateComputePipelineState(&computePsoDesc, IID_PPV_ARGS(&m_pipelineState)));
}

void EnvCube_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_pipelineState.Reset();
}

}  // namespace app