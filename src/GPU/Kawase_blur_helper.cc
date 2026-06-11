#include "Kawase_blur_helper.h"

#include <glm/glm.hpp>

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    InputSRV = 0,
    OutputUAV,
    RootConstants,

    Count
};
}

namespace app {

struct KawaseCSInput {
    glm::fvec2 halftexel;
    int numMips;
};

ComPtr<ID3D12RootSignature> Kawase_blur_helper::m_rootSignature{};
ComPtr<ID3D12PipelineState> Kawase_blur_helper::m_DownsamplePipelineState{};
ComPtr<ID3D12PipelineState> Kawase_blur_helper::m_UpsamplePipelineState{};

Kawase_blur_helper::Kawase_blur_helper() {
    if (!m_rootSignature || !m_DownsamplePipelineState || !m_UpsamplePipelineState) {
        Reload();
    }
    CreateDescriptorHeap();
}

void Kawase_blur_helper::Reload() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

Kawase_blur_helper::~Kawase_blur_helper() { release_gpu_resources(); }

void Kawase_blur_helper::CreateBlurMips(GPU_texture& uav_texture) {  // works on the input uav texture
    const auto& resource = uav_texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    ID3D12DescriptorHeap* desc_heap[] = {m_SrvDescHeap.Get()};
    commandList->SetDescriptorHeaps(1, desc_heap);

    commandList->SetComputeRootSignature(m_rootSignature.Get());
    commandList->SetPipelineState(m_DownsamplePipelineState.Get());

    int NumMips = GPU_texture::CalculateMipCount(width, height);
    NumMips = std::min(NumMips, MipsLimit);

    CD3DX12_CPU_DESCRIPTOR_HANDLE cpuHandle(HeapStartCpu);
    CD3DX12_GPU_DESCRIPTOR_HANDLE gpuHandle(HeapStartGpu);

    auto iter_uav_cpuHandle = cpuHandle;
    auto iter_srv_cpuHandle = cpuHandle;
    iter_srv_cpuHandle.Offset(MipsLimit, HeapHandleIncrement);
    for (int mipLevel = 0; mipLevel < NumMips; ++mipLevel) {
        uav_texture.GetUAVHandleForMipLevel(mipLevel, iter_uav_cpuHandle);
        uav_texture.GetSRVHandleForMipLevel(mipLevel, iter_srv_cpuHandle);
        iter_uav_cpuHandle.Offset(1, HeapHandleIncrement);
        iter_srv_cpuHandle.Offset(1, HeapHandleIncrement);
    }

    auto iter_uav_gpuHandle = gpuHandle;
    auto iter_srv_gpuHandle = gpuHandle;
    iter_srv_gpuHandle.Offset(MipsLimit, HeapHandleIncrement);
    iter_uav_gpuHandle.Offset(1, HeapHandleIncrement); // start from uav for first mip
    for (int mipLevel = 0; mipLevel < std::min(BlurIterations, NumMips) - 1; ++mipLevel) {
        const auto mip_width = GPU_texture::GetMipDimension(width, mipLevel + 1);
        const auto mip_height = GPU_texture::GetMipDimension(height, mipLevel + 1);
        int GroupsX = (mip_width + 15) / 16;
        int GroupsY = (mip_height + 15) / 16;

        commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::InputSRV, iter_srv_gpuHandle);
        commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputUAV, iter_uav_gpuHandle);

        fvec2 halftexel{0.5f / mip_width, 0.5f / mip_height};
        KawaseCSInput input{halftexel, NumMips};
        constexpr int inputSizeInInt = sizeof(KawaseCSInput) / 4;
        commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);

        const auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
            uav_texture.get_gpu_resource().Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, 
            D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, mipLevel);
        commandList->ResourceBarrier(1, &barrier);

        commandList->Dispatch(GroupsX, GroupsY, 1);

        iter_uav_gpuHandle.Offset(1, HeapHandleIncrement);
        iter_srv_gpuHandle.Offset(1, HeapHandleIncrement);
    }
    
    for (int mipLevel = std::min(BlurIterations, NumMips) - 1; mipLevel < NumMips; ++mipLevel) {
        const auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(uav_texture.get_gpu_resource().Get(),
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, mipLevel);
        commandList->ResourceBarrier(1, &barrier);
    }

    const auto barrier =
        CD3DX12_RESOURCE_BARRIER::Transition(uav_texture.get_gpu_resource().Get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES);

    commandList->ResourceBarrier(1, &barrier);
}

GPU_texture Kawase_blur_helper::GetBlankCompatibleUAVTex(GPU_texture& texture) {
    const auto& resource = texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;
    TEXTURE_TRAITS flags = texture.texture_options | TEXTURE_TRAITS::AllocateMips | TEXTURE_TRAITS::UAV;
    return GPU_texture{width, height, flags};
}

void Kawase_blur_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);   // 1 input srv
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);   // 1 output uav
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0);      // 1 constant buffer.

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::InputSRV].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::OutputUAV].InitAsDescriptorTable(1, &ranges[1]);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(KawaseCSInput) / 4, 0);

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

void Kawase_blur_helper::CreatePipelineStateObject() {
    auto [cs_downsample_shaderBlob, cs_downsample_bytecode] = LoadShader(c_cs_downsample_file_name);
    auto [cs_upsample_shaderBlob, cs_upsample_bytecode] = LoadShader(c_cs_upsample_file_name);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDesc = {};
    computePsoDesc.pRootSignature = m_rootSignature.Get();  // Must match shader layout
    computePsoDesc.CS = cs_downsample_bytecode;
    computePsoDesc.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
    computePsoDesc.NodeMask = 1;

    ThrowIfFailed(device->CreateComputePipelineState(&computePsoDesc, IID_PPV_ARGS(&m_DownsamplePipelineState)));

    computePsoDesc.CS = cs_upsample_bytecode;
    ThrowIfFailed(device->CreateComputePipelineState(&computePsoDesc, IID_PPV_ARGS(&m_UpsamplePipelineState)));
}

void Kawase_blur_helper::CreateDescriptorHeap() {
    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_DESCRIPTOR_HEAP_DESC desc = {};
    desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    desc.NumDescriptors = 2 * MipsLimit;
    desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    ThrowIfFailed(device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&m_SrvDescHeap)));
    HeapStartCpu = m_SrvDescHeap->GetCPUDescriptorHandleForHeapStart();
    HeapStartGpu = m_SrvDescHeap->GetGPUDescriptorHandleForHeapStart();
    HeapHandleIncrement = device->GetDescriptorHandleIncrementSize(desc.Type);
}

void Kawase_blur_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_DownsamplePipelineState.Reset();
    m_UpsamplePipelineState.Reset();
    m_SrvDescHeap.Reset();
}

}  // namespace app