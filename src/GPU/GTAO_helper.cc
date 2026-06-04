#include "GTAO_helper.h"

#include <glm/glm.hpp>

#include "../d3d_context.h"
#include "Common_helpers.h"
#include "Mipmaps_helper.h"
#include "Bilateral_filter.h"
#include "Reprojection_helper.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    Gbuffer = 0,
    Depth, 
    OutputUAV,
    RootConstants,

    Count
};
}

namespace app {

using namespace glm;

struct GTAOCSInput {
    fmat4x4 Projection;
    float inv_proj_00;
    float inv_proj_11;
    uvec2 FrameSize;
    fvec2 texel_size;
    float AO_distance;
    unsigned int frameID;
};

GTAO_helper::GTAO_helper() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

GTAO_helper::~GTAO_helper() { release_gpu_resources(); }

void GTAO_helper::CreateAO(GPU_texture& AO_uav_texture, GPU_texture& G_buff_texture, 
    GPU_texture& DepthUAVTexture, GPU_texture& DepthUAVTexture_previous,
    const fmat4x4& Projection, const fmat4x4& invProjection, const fmat4x4& ViewProjection,
    unsigned int FrameID) 
{
    const auto& resource = AO_uav_texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;

    ResizeInnerResource(width, height);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    static Bilateral_filter bilateral_filter{};
    static Reprojection_helper reprojection_helper{};

    auto barrier_g_buff = CD3DX12_RESOURCE_BARRIER::Transition(G_buff_texture.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    auto barrier_depth_uav = CD3DX12_RESOURCE_BARRIER::Transition(DepthUAVTexture.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    auto barrier_depth_uav_previous = CD3DX12_RESOURCE_BARRIER::Transition(DepthUAVTexture_previous.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    D3D12_RESOURCE_BARRIER bariers_in[] = {barrier_g_buff, barrier_depth_uav, barrier_depth_uav_previous};
    commandList->ResourceBarrier(3, bariers_in);

    commandList->SetComputeRootSignature(m_rootSignature.Get());
    commandList->SetPipelineState(m_PipelineState.Get());

    int NumMips = GPU_texture::CalculateMipCount(width, height);

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::Gbuffer, G_buff_texture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::Depth, DepthUAVTexture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputUAV, AO_uav_texture.GetUAVHandle());

    fvec2 texel{1.0f / width, 1.0f / height};
    GTAOCSInput input{Projection, invProjection[0][0], invProjection[1][1], {width, height}, texel, AODistance,
        FrameID};
    constexpr int inputSizeInInt = sizeof(GTAOCSInput) / 4;
    commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);

    int GroupsX = (width + 15) / 16;
    int GroupsY = (height + 15) / 16;

    commandList->Dispatch(GroupsX, GroupsY, 1);

    if (DenoiseEnabled) {
        auto barrier_AO_tex = CD3DX12_RESOURCE_BARRIER::Transition(AO_uav_texture.get_gpu_resource().Get(),
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
        commandList->ResourceBarrier(1, &barrier_AO_tex);
        // Spatial denoiser
        bilateral_filter.ApplyBlur(
            m_SpatialDenoised, AO_uav_texture, G_buff_texture, DepthUAVTexture, AOSpatialSigma, AODepthSigma, AONormalSigma);
        // Temporal reprojection + denoiser
        if (consecutive_frame_count > 1) {
            reprojection_helper.Reproject(m_SpatialDenoised_previous, DepthUAVTexture_previous, m_SpatialDenoised,
                DepthUAVTexture, glm::inverse(ViewProjection), ViewProjection_previous, Projection_previous, 1.0f / 6.0f,
                DepthThreshold, false);
        }

        // Copy back to AO texture for output
        GPU_texture::copy_texture_mip0_only(AO_uav_texture, m_SpatialDenoised, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, commandList);
        barrier_AO_tex = CD3DX12_RESOURCE_BARRIER::Transition(AO_uav_texture.get_gpu_resource().Get(),
            D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        commandList->ResourceBarrier(1, &barrier_AO_tex);

        std::swap(m_SpatialDenoised, m_SpatialDenoised_previous);

        ViewProjection_previous = ViewProjection;
        Projection_previous = Projection;
        ++consecutive_frame_count;
    }

    barrier_g_buff = CD3DX12_RESOURCE_BARRIER::Transition(G_buff_texture.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
    barrier_depth_uav = CD3DX12_RESOURCE_BARRIER::Transition(DepthUAVTexture.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    barrier_depth_uav_previous = CD3DX12_RESOURCE_BARRIER::Transition(DepthUAVTexture_previous.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    D3D12_RESOURCE_BARRIER bariers_out[] = {barrier_g_buff, barrier_depth_uav, barrier_depth_uav_previous};
    commandList->ResourceBarrier(3, bariers_out);
}

void GTAO_helper::ResetFrameCounter() { consecutive_frame_count = 0; }

void GTAO_helper::ResizeInnerResource(int new_width, int new_height) {
    if (currentWidth == new_width && currentHeight == new_height) return;
    currentWidth = std::max(new_width, 0);
    currentHeight = std::max(new_height, 0);
    auto flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV;
    m_SpatialDenoised = GPU_texture{currentWidth, currentHeight, flags};
    m_SpatialDenoised_previous = GPU_texture{currentWidth, currentHeight, flags};
}

void GTAO_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);  // 1 Gbuffer srv
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1);  // 1 Depth srv
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 output uav
    ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0);  // 1 constant buffer.

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::Gbuffer].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::Depth].InitAsDescriptorTable(1, &ranges[1]);
    rootParameters[GlobalRootSignatureParams::OutputUAV].InitAsDescriptorTable(1, &ranges[2]);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(GTAOCSInput) / 4, 0);

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

void GTAO_helper::CreatePipelineStateObject() {
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


void GTAO_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_PipelineState.Reset();
}

}  // namespace app