#include "SSR_helper.h"

#include <glm/glm.hpp>

#include "../d3d_context.h"
#include "Common_helpers.h"
#include "Reprojection_helper.h"
#include "Kawase_blur_helper.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    Gbuffer = 0,
    Depth,
    Frame,
    SSR_buffer,
    OutputUAV,
    RootConstants,

    Count
};
}

namespace app {

using namespace glm;

struct SSRCSInput {
    fmat4x4 Projection;
    float inv_proj_00;
    float inv_proj_11;
    uvec2 FrameSize;
    fvec2 texel_size;
    float DepthThreshold;
    float MaxRoughness;
    float GGXBias;  // 1.0 is no bias, < 1.0 reduces tail of distribution
    int frameID;
    int MaxDepthMipLevel;
    float MaxFrameMipLevel;
    float PrefilterDistanceMult;
};

ComPtr<ID3D12RootSignature> SSR_helper::m_rootSignature{};
ComPtr<ID3D12PipelineState> SSR_helper::m_TracePipelineState{};
ComPtr<ID3D12PipelineState> SSR_helper::m_ResolvePipelineState{};

SSR_helper::SSR_helper() {
    if (!m_rootSignature || !m_TracePipelineState || !m_ResolvePipelineState) {
        Reload();
    }
}

void SSR_helper::Reload() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

SSR_helper::~SSR_helper() { release_gpu_resources(); }

void SSR_helper::CalculateSSR(GPU_texture& SSR_uav_texture, GPU_texture& G_buff_texture, 
    GPU_texture& VelocityBuffer, GPU_texture & DepthUAVTexture,
    GPU_texture& DepthUAVTexture_previous, GPU_texture& Frame_texture,
    const fmat4x4& Projection, const fmat4x4& invProjection, const fmat4x4& ViewProjection, 
    const fmat4x4& ViewProjection_prev, unsigned int FrameID)
{
    const auto& resource = SSR_uav_texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;
    const auto RenderFrameMips = std::min(GPU_texture::CalculateMipCount(width, height), static_cast<unsigned int>(Kawase_blur_helper::BlurIterations));

    ResizeInnerResource(width, height);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    static Reprojection_helper reprojection_helper{};
    static Kawase_blur_helper m_blur_helper{};

    std::swap(m_SSR_texture_previous, SSR_uav_texture); // at the top because SSR_uav_texture is used after outside of this call to function

    // Reproject previous Frame before doing SSR
    {
        reprojection_helper.Reproject(Frame_texture, DepthUAVTexture_previous, m_Frame_reprojected, DepthUAVTexture, 
            VelocityBuffer, Projection, glm::inverse(ViewProjection), ViewProjection_prev, 
            Projection_previous, 0.0f, 10000.0f, true, nullptr);
        const auto barrier = CD3DX12_RESOURCE_BARRIER::UAV(m_Frame_reprojected.get_gpu_resource().Get());
        commandList->ResourceBarrier(1, &barrier);

        if (UsePrefiltering) {
            m_blur_helper.CreateBlurMips(m_Frame_reprojected);

            ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()};  // todo: don't replace heap during blur pass?
            commandList->SetDescriptorHeaps(1, desc_heap);
        }
    }

    auto barrier_depth_uav = CD3DX12_RESOURCE_BARRIER::Transition(DepthUAVTexture.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    auto barrier_depth_uav_previous = CD3DX12_RESOURCE_BARRIER::Transition(DepthUAVTexture_previous.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    auto barrier_velocity = CD3DX12_RESOURCE_BARRIER::Transition(VelocityBuffer.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    D3D12_RESOURCE_BARRIER bariers_in[] = {barrier_depth_uav, barrier_depth_uav_previous, barrier_velocity};
    commandList->ResourceBarrier(3, bariers_in);

    commandList->SetComputeRootSignature(m_rootSignature.Get());
    commandList->SetPipelineState(m_TracePipelineState.Get());

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::Gbuffer, G_buff_texture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::Depth, DepthUAVTexture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::Frame, m_Frame_reprojected.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputUAV, m_SSR_buff.GetUAVHandle());

    int MaxDepthMipLevel = GPU_texture::CalculateMipCount(width, height) - 1;
    float MaxFrameMipLevel = UsePrefiltering? RenderFrameMips - 1.0f : 0.0f;
    float PrefilterDistanceMult = 1.0f / max(PrefilteringDistance, 0.001f);

    fvec2 texel{1.0f / width, 1.0f / height};
    SSRCSInput input{Projection, invProjection[0][0], invProjection[1][1], {width, height}, texel, 
        DepthThreshold, MaxRoughness, 1.0f - SSR_GGXClamp, FrameID, 
        MaxDepthMipLevel, MaxFrameMipLevel, PrefilterDistanceMult};
    constexpr int inputSizeInInt = sizeof(SSRCSInput) / 4;
    commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);

    int GroupsX = (width + 15) / 16;
    int GroupsY = (height + 15) / 16;

    commandList->Dispatch(GroupsX, GroupsY, 1);

    auto barrier_ssr_buff = CD3DX12_RESOURCE_BARRIER::Transition(m_SSR_buff.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    commandList->ResourceBarrier(1, &barrier_ssr_buff);

    commandList->SetPipelineState(m_ResolvePipelineState.Get());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::SSR_buffer, m_SSR_buff.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputUAV, SSR_uav_texture.GetUAVHandle());

    commandList->Dispatch(GroupsX, GroupsY, 1);

    // Temporal reprojection denoiser
    if (DenoiseEnabled && consecutive_frame_count > 1) {
        auto barrier_ssr_uav = CD3DX12_RESOURCE_BARRIER::UAV(SSR_uav_texture.get_gpu_resource().Get());
        commandList->ResourceBarrier(1, &barrier_ssr_uav);
        reprojection_helper.Reproject(m_SSR_texture_previous, DepthUAVTexture_previous, SSR_uav_texture, DepthUAVTexture,
            VelocityBuffer, Projection, glm::inverse(ViewProjection), ViewProjection_prev, Projection_previous, 
            1.0f / 16.0f, DepthThreshold, false, ParallaxReprojection ? &m_SSR_buff : nullptr);
    }

    barrier_ssr_buff = CD3DX12_RESOURCE_BARRIER::Transition(m_SSR_buff.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    commandList->ResourceBarrier(1, &barrier_ssr_buff);

    barrier_depth_uav = CD3DX12_RESOURCE_BARRIER::Transition(DepthUAVTexture.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    barrier_depth_uav_previous = CD3DX12_RESOURCE_BARRIER::Transition(DepthUAVTexture_previous.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    barrier_velocity = CD3DX12_RESOURCE_BARRIER::Transition(VelocityBuffer.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
    D3D12_RESOURCE_BARRIER bariers_out[] = {barrier_depth_uav, barrier_depth_uav_previous, barrier_velocity};
    commandList->ResourceBarrier(3, bariers_out);

    Projection_previous = Projection;
    ++consecutive_frame_count;
}

void SSR_helper::ResetFrameCounter() { consecutive_frame_count = 0; }

void SSR_helper::ResizeInnerResource(int new_width, int new_height) {
    if (currentWidth == new_width && currentHeight == new_height) return;
    currentWidth = std::max(new_width, 0);
    currentHeight = std::max(new_height, 0);
    auto flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV;
    m_SSR_texture_previous.release_gpu_resource();
    m_SSR_texture_previous = GPU_texture{currentWidth, currentHeight, flags};

    flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV | TEXTURE_TRAITS::AllocateMips;
    m_Frame_reprojected.release_gpu_resource();
    m_Frame_reprojected = GPU_texture{currentWidth, currentHeight, flags};

    flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV;
    m_SSR_buff.release_gpu_resource();
    m_SSR_buff = GPU_texture{currentWidth, currentHeight, flags};
}

void SSR_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);  // 1 Gbuffer srv
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1);  // 1 Depth srv
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 2);  // 1 Frame srv
    ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 3);  // 1 SSR_buffer srv
    ranges[4].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 output uav
    ranges[5].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0);  // 1 constant buffer.

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::Gbuffer].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::Depth].InitAsDescriptorTable(1, &ranges[1]);
    rootParameters[GlobalRootSignatureParams::Frame].InitAsDescriptorTable(1, &ranges[2]);
    rootParameters[GlobalRootSignatureParams::SSR_buffer].InitAsDescriptorTable(1, &ranges[3]);
    rootParameters[GlobalRootSignatureParams::OutputUAV].InitAsDescriptorTable(1, &ranges[4]);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(SSRCSInput) / 4, 0);

    D3D12_STATIC_SAMPLER_DESC point_sampler = {};
    point_sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_POINT;
    point_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    point_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    point_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    point_sampler.MaxLOD = D3D12_FLOAT32_MAX;
    point_sampler.ShaderRegister = 0;
    point_sampler.RegisterSpace = 0;
    point_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_STATIC_SAMPLER_DESC default_sampler = {};  // Default static sampler.
    default_sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    default_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    default_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    default_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    default_sampler.MaxLOD = static_cast<float>(Kawase_blur_helper::BlurIterations) - 1.0f;
    default_sampler.ShaderRegister = 1;  // s0
    default_sampler.RegisterSpace = 0;
    default_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_STATIC_SAMPLER_DESC samplers[] = {point_sampler, default_sampler};

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 2, samplers, flags);

    SerializeAndCreateRootSignature(rootSignatureDesc, &m_rootSignature);
}

void SSR_helper::CreatePipelineStateObject() {
    auto [cs_trace_shaderBlob, cs_trace_bytecode] = LoadShader(c_cs_trace_file_name);
    auto [cs_resolve_shaderBlob, cs_resolve_bytecode] = LoadShader(c_cs_resolve_file_name);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDesc = {};
    computePsoDesc.pRootSignature = m_rootSignature.Get();
    computePsoDesc.CS = cs_trace_bytecode;
    computePsoDesc.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
    computePsoDesc.NodeMask = 1;

    ThrowIfFailed(device->CreateComputePipelineState(&computePsoDesc, IID_PPV_ARGS(&m_TracePipelineState)));

    computePsoDesc.CS = cs_resolve_bytecode;
    ThrowIfFailed(device->CreateComputePipelineState(&computePsoDesc, IID_PPV_ARGS(&m_ResolvePipelineState)));
}

void SSR_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_TracePipelineState.Reset();
    m_ResolvePipelineState.Reset();
}

}  // namespace app