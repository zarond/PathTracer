#include "Mipmaps_helper.h"

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    OutputViewSlot = 0,
    GroupCounters,
    RootConstants,

    Count
};
}

namespace app {

struct MipCSInput {
    int numMips;
    int isSRGB;
    int isNormalMap;
    int startingMip;
};

Mipmaps_helper::Mipmaps_helper() {
    CreateRootSignature();
    CreatePipelineStateObject();
    CreateDescriptorHeap();
}

Mipmaps_helper::~Mipmaps_helper() {
    release_gpu_resources();
}

void Mipmaps_helper::CreateMips(GPU_texture& uav_texture) { // on input uav texture
    const auto& resource = uav_texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    ID3D12DescriptorHeap* desc_heap[] = {m_SrvDescHeap.Get()};
    commandList->SetDescriptorHeaps(1, desc_heap);

    commandList->SetPipelineState(m_PipelineState.Get());
    commandList->SetComputeRootSignature(m_rootSignature.Get());

    int NumMips = GPU_texture::CalculateMipCount(width, height);
    NumMips = std::min(NumMips, MipsLimit);

    CD3DX12_CPU_DESCRIPTOR_HANDLE cpuHandle(HeapStartCpu, HeapHandleIncrement);
    CD3DX12_GPU_DESCRIPTOR_HANDLE gpuHandle(HeapStartGpu, HeapHandleIncrement);

    for (int mipLevel = 0; mipLevel < NumMips; ++mipLevel) {
        uav_texture.GetUAVHandleForMipLevel(mipLevel, cpuHandle);
        cpuHandle.Offset(1, HeapHandleIncrement);
    }

    // UAV Barrier forces the GPU to fully finish writing the new mip before the next pass reads it
    D3D12_RESOURCE_BARRIER uavBarrier = {};
    uavBarrier.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    uavBarrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    uavBarrier.UAV.pResource = uav_texture.get_gpu_resource().Get();

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputViewSlot, gpuHandle);
    //commandList->SetComputeRootUnorderedAccessView(GlobalRootSignatureParams::GroupCounters, GroupCounters->GetGPUVirtualAddress());

    // todo: replace multipass with single pass
    for (int mipLevel = 0; mipLevel < NumMips; mipLevel += 5) {
        int next_mip_width = GPU_texture::GetMipDimension(width, mipLevel + 1);
        int next_mip_height = GPU_texture::GetMipDimension(width, mipLevel + 1);

        MipCSInput input{NumMips, 0, 0, mipLevel};
        constexpr int inputSizeInInt = sizeof(MipCSInput) / 4;
        commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);

        commandList->Dispatch((next_mip_width + 15) / 16, (next_mip_height + 15) / 16, 1);
        commandList->ResourceBarrier(1, &uavBarrier);
    }
}

void Mipmaps_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    //auto flags = D3D12_DESCRIPTOR_RANGE_FLAG_DESCRIPTORS_VOLATILE | D3D12_DESCRIPTOR_RANGE_FLAG_DATA_VOLATILE;
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 16, 0, 0);  // 1 all mips texture
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0, 1);   // 1 atomic counters buffer texture
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0);      // 1 constant buffer.

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::OutputViewSlot].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::GroupCounters].InitAsUnorderedAccessView(0, 1);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(MipCSInput) / 4, 0);

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 0, nullptr, flags);

    SerializeAndCreateRootSignature(rootSignatureDesc, &m_rootSignature);
}

void Mipmaps_helper::CreatePipelineStateObject() {
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

void Mipmaps_helper::CreateDescriptorHeap() {
    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_DESCRIPTOR_HEAP_DESC desc = {};
    desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    desc.NumDescriptors = MipsLimit;
    desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    ThrowIfFailed(device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&m_SrvDescHeap)));
    HeapStartCpu = m_SrvDescHeap->GetCPUDescriptorHandleForHeapStart();
    HeapStartGpu = m_SrvDescHeap->GetGPUDescriptorHandleForHeapStart();
    HeapHandleIncrement = device->GetDescriptorHandleIncrementSize(desc.Type);
}

void Mipmaps_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_PipelineState.Reset();
    m_SrvDescHeap.Reset();
}

}  // namespace app