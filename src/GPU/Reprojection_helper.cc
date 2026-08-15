#include "Reprojection_helper.h"

#include <glm/glm.hpp>

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    OldTexture,
    OldDepth,
    NewTexture,
    NewDepth,
    DistanceTexture,
    VelocityBuffer,
    RootConstants,

    Count
};
}

namespace app {

using namespace glm;

struct ReprojectionCSInput {
    fmat4x4 Reprojection;
    fmat4x4 Projection_prev;
    float proj_22;
    float proj_23;
    uvec2 FrameSize;
    fvec2 texel_size;
    float depth_threshold;  // default: 0.001f
    float new_mix_factor;
    int weak_depth_condition;
    int use_reflection_distance;
    int DebugMode;
};

ComPtr<ID3D12RootSignature> Reprojection_helper::m_rootSignature{};
ComPtr<ID3D12PipelineState> Reprojection_helper::m_PipelineState{};

Reprojection_helper::Reprojection_helper() {
    if (!m_rootSignature || !m_PipelineState) {
        Reload();
    }
}

void Reprojection_helper::Reload() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

bool Reprojection_helper::DebugMode = false;

Reprojection_helper::~Reprojection_helper() { release_gpu_resources(); }

void Reprojection_helper::Reproject(GPU_texture& old_texture, GPU_texture& old_depth_texture, GPU_texture& new_texture,
    GPU_texture& new_depth_texture, GPU_texture& VelocityBuffer, const fmat4x4& Projection, const fmat4x4& invViewProjection,
    const fmat4x4& ViewProjection_previous, const fmat4x4& Projection_previous, 
    float new_value_mix_factor, float depth_threshold, bool weak_depth_condition,
    GPU_texture* DistanceTexture) 
{
    const auto& resource = new_texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    commandList->SetComputeRootSignature(m_rootSignature.Get());
    commandList->SetPipelineState(m_PipelineState.Get());

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OldTexture, old_texture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OldDepth, old_depth_texture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::NewTexture, new_texture.GetUAVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::NewDepth, new_depth_texture.GetSRVHandle());
    bool use_reflection_distance = false;
    if (DistanceTexture) {
        commandList->SetComputeRootDescriptorTable(
            GlobalRootSignatureParams::DistanceTexture, DistanceTexture->GetSRVHandle());
        use_reflection_distance = true;
    }
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::VelocityBuffer, VelocityBuffer.GetSRVHandle());

    fvec2 texel{1.0f / width, 1.0f / height};
    fmat4x4 Reprojection = ViewProjection_previous * invViewProjection;
    ReprojectionCSInput input{
        Reprojection, 
        Projection_previous, 
        Projection[2][2],
        Projection[3][2],
        {width, height}, 
        texel,
        depth_threshold,
        new_value_mix_factor,
        weak_depth_condition, 
        use_reflection_distance,
        DebugMode
    };
    constexpr int inputSizeInInt = sizeof(ReprojectionCSInput) / 4;
    commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);

    int GroupsX = (width + 15) / 16;
    int GroupsY = (height + 15) / 16;

    commandList->Dispatch(GroupsX, GroupsY, 1);
}

void Reprojection_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);  // 1 OldTexture srv
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1);  // 1 OldDepth srv
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 NewTexture uav
    ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 2);  // 1 NewDepth srv
    ranges[4].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 3);  // 1 DistanceTexture srv
    ranges[5].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 4);  // 1 VelocityBuffer srv
    ranges[6].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0);  // 1 constant buffer.

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::OldTexture].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::OldDepth].InitAsDescriptorTable(1, &ranges[1]);
    rootParameters[GlobalRootSignatureParams::NewTexture].InitAsDescriptorTable(1, &ranges[2]);
    rootParameters[GlobalRootSignatureParams::NewDepth].InitAsDescriptorTable(1, &ranges[3]);
    rootParameters[GlobalRootSignatureParams::DistanceTexture].InitAsDescriptorTable(1, &ranges[4]);
    rootParameters[GlobalRootSignatureParams::VelocityBuffer].InitAsDescriptorTable(1, &ranges[5]);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(ReprojectionCSInput) / 4, 0);

    D3D12_STATIC_SAMPLER_DESC point_sampler = {};
    point_sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_POINT;
    point_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    point_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    point_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    point_sampler.ShaderRegister = 0;
    point_sampler.RegisterSpace = 0;
    point_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_STATIC_SAMPLER_DESC default_sampler = {};  // Default static sampler.
    default_sampler.Filter = D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT;
    default_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    default_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    default_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    default_sampler.ShaderRegister = 1;  // s0
    default_sampler.RegisterSpace = 0;
    default_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_STATIC_SAMPLER_DESC samplers[] = {point_sampler, default_sampler};

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 2, samplers, flags);

    SerializeAndCreateRootSignature(rootSignatureDesc, &m_rootSignature);
}

void Reprojection_helper::CreatePipelineStateObject() {
    auto [cs_shaderBlob, cs_bytecode] = LoadShader(c_cs_file_name);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDesc = {};
    computePsoDesc.pRootSignature = m_rootSignature.Get();
    computePsoDesc.CS = cs_bytecode;
    computePsoDesc.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
    computePsoDesc.NodeMask = 1;

    ThrowIfFailed(device->CreateComputePipelineState(&computePsoDesc, IID_PPV_ARGS(&m_PipelineState)));
}

void Reprojection_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_PipelineState.Reset();
}

}  // namespace app