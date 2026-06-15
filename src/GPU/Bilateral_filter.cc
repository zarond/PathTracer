#include "Bilateral_filter.h"

#include <glm/glm.hpp>

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    Gbuffer = 0,
    Depth,
    InputSRV,
    OutputUAV,
    RootConstants,

    Count
};
}

namespace app {

using namespace glm;

struct FilterCSInput {
    uvec2 FrameSize;
    fvec2 texel_size;
    float rcp_spatialDenom;
    float rcp_DepthRangeDenom;
    float rcp_NormalRangeDenom;
};

ComPtr<ID3D12RootSignature> Bilateral_filter::m_rootSignature{};
ComPtr<ID3D12PipelineState> Bilateral_filter::m_PipelineState{};

Bilateral_filter::Bilateral_filter() {
    if (!m_rootSignature || !m_PipelineState) {
        Reload();
    }
}

void Bilateral_filter::Reload() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

Bilateral_filter::~Bilateral_filter() { release_gpu_resources(); }

void Bilateral_filter::ApplyBlur(GPU_texture& output_uav_texture, GPU_texture& AO_texture, GPU_texture& G_buff_texture,
    GPU_texture& Depth_texture, float spatialSigma, float DepthSigma, float NormalSigma) {
    const auto& resource = AO_texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    commandList->SetComputeRootSignature(m_rootSignature.Get());
    commandList->SetPipelineState(m_PipelineState.Get());

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::Gbuffer, G_buff_texture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::Depth, Depth_texture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::InputSRV, AO_texture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputUAV, output_uav_texture.GetUAVHandle());

    fvec2 texel{1.0f / width, 1.0f / height};
    float spatialSigmaDenom = 2.0f * spatialSigma * spatialSigma;
    float DepthRangeDenom = 2.0f * DepthSigma * DepthSigma;
    float NormalRangeDenom = 2.0f * NormalSigma * NormalSigma;
    FilterCSInput input{{width, height}, texel, 1.0f / spatialSigmaDenom, 1.0f / DepthRangeDenom, 1.0f / NormalRangeDenom};
    constexpr int inputSizeInInt = sizeof(FilterCSInput) / 4;
    commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);

    int GroupsX = (width + 15) / 16;
    int GroupsY = (height + 15) / 16;

    commandList->Dispatch(GroupsX, GroupsY, 1);
}

void Bilateral_filter::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);  // 1 Gbuffer srv
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1);  // 1 Depth srv
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 2);  // 1 Input srv
    ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 output uav
    ranges[4].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0);  // 1 constant buffer.

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::Gbuffer].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::Depth].InitAsDescriptorTable(1, &ranges[1]);
    rootParameters[GlobalRootSignatureParams::InputSRV].InitAsDescriptorTable(1, &ranges[2]);
    rootParameters[GlobalRootSignatureParams::OutputUAV].InitAsDescriptorTable(1, &ranges[3]);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(FilterCSInput) / 4, 0);

    D3D12_STATIC_SAMPLER_DESC sampler = {};
    sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_POINT;
    sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.MaxLOD = 5.0f;
    sampler.ShaderRegister = 0;
    sampler.RegisterSpace = 0;
    sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 1, &sampler, flags);

    SerializeAndCreateRootSignature(rootSignatureDesc, &m_rootSignature);
}

void Bilateral_filter::CreatePipelineStateObject() {
    auto [cs_shaderBlob, cs_bytecode] = LoadShader(c_cs_file_name);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDesc = {};
    computePsoDesc.pRootSignature = m_rootSignature.Get();  // Must match shader layout
    computePsoDesc.CS = cs_bytecode;
    computePsoDesc.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
    computePsoDesc.NodeMask = 1;

    ThrowIfFailed(device->CreateComputePipelineState(&computePsoDesc, IID_PPV_ARGS(&m_PipelineState)));
}

void Bilateral_filter::release_gpu_resources() {
    m_rootSignature.Reset();
    m_PipelineState.Reset();
}

}  // namespace app