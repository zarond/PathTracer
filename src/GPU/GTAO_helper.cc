#include "GTAO_helper.h"

#include <glm/glm.hpp>

#include "../d3d_context.h"
#include "Common_helpers.h"
#include "Mipmaps_helper.h"

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
    fmat4x4 projectionToWorld;  // without translation component
    fmat4x4 viewProjection;
    fvec4 cameraPosition;
    ivec2 FrameSize;
    fvec2 texel_size;
    float thin_object_factor;
};

GTAO_helper::GTAO_helper() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

GTAO_helper::~GTAO_helper() { release_gpu_resources(); }

void GTAO_helper::CreateAO(GPU_texture& AO_uav_texture, GPU_texture& G_buff_texture, GPU_texture& Depth_texture,
    const fmat4x4& projectionToWorld, const fmat4x4& viewProjection, const fvec4& cameraPosition) {  // works on the input uav texture
    const auto& resource = AO_uav_texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;

    ResizeInnerResource(width, height);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    static Mipmaps_helper mip_helper{};
    mip_helper.Init();

    auto barrier_g_buff = CD3DX12_RESOURCE_BARRIER::Transition(G_buff_texture.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    auto barrier_depth = CD3DX12_RESOURCE_BARRIER::Transition(Depth_texture.get_gpu_resource().Get(), 
        D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    D3D12_RESOURCE_BARRIER bariers[] = {barrier_g_buff, barrier_depth};
    commandList->ResourceBarrier(2, bariers);

    GPU_texture::copy_texture_mip0_only(m_DepthUAVTexture, Depth_texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, d3d_ctx.m_DXRCommandList);

    mip_helper.CreateMips(m_DepthUAVTexture, true, true);

    ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()};  // todo: don't replace heap during mip map pass?
    commandList->SetDescriptorHeaps(1, desc_heap);

    auto barrier_depth_uav = CD3DX12_RESOURCE_BARRIER::Transition(m_DepthUAVTexture.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    commandList->ResourceBarrier(1, &barrier_depth_uav);

    commandList->SetComputeRootSignature(m_rootSignature.Get());
    commandList->SetPipelineState(m_PipelineState.Get());

    int NumMips = GPU_texture::CalculateMipCount(width, height);

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::Gbuffer, G_buff_texture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::Depth, m_DepthUAVTexture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputUAV, AO_uav_texture.GetUAVHandle());

    fvec2 texel{1.0f / width, 1.0f / height};
    GTAOCSInput input{projectionToWorld, viewProjection, cameraPosition, {width, height}, texel, thinObjectFactor};
    constexpr int inputSizeInInt = sizeof(GTAOCSInput) / 4;
    commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);

    int GroupsX = (width + 15) / 16;
    int GroupsY = (height + 15) / 16;

    commandList->Dispatch(GroupsX, GroupsY, 1);

    barrier_g_buff = CD3DX12_RESOURCE_BARRIER::Transition(G_buff_texture.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
    barrier_depth = CD3DX12_RESOURCE_BARRIER::Transition(Depth_texture.get_gpu_resource().Get(), 
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE);
    barrier_depth_uav = CD3DX12_RESOURCE_BARRIER::Transition(m_DepthUAVTexture.get_gpu_resource().Get(),
        D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    D3D12_RESOURCE_BARRIER bariers_out[] = {barrier_g_buff, barrier_depth, barrier_depth_uav};
    commandList->ResourceBarrier(3, bariers_out);
}

void GTAO_helper::ResizeInnerResource(int new_width, int new_height) {
    if (currentWidth == new_width && currentHeight == new_height) return;
    currentWidth = std::max(new_width, 0);
    currentHeight = std::max(new_height, 0);
    m_DepthUAVTexture.release_gpu_resource();
    auto flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV | TEXTURE_TRAITS::AllocateMips;
    m_DepthUAVTexture = GPU_texture{currentWidth, currentHeight, flags, DXGI_FORMAT_R32_FLOAT};
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
    sampler.Filter = D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT;
    sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
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
    computePsoDesc.pRootSignature = m_rootSignature.Get();  // Must match shader layout
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