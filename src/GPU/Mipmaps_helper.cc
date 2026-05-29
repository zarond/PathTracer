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
    int minFilter;
    int numGroups;
};

Mipmaps_helper::Mipmaps_helper() {
    CreateRootSignature();
    CreatePipelineStateObject();
    CreateDescriptorHeap();
}

Mipmaps_helper::~Mipmaps_helper() {
    release_gpu_resources();
}

void Mipmaps_helper::CreateMips(GPU_texture& uav_texture, bool only5Mips, bool minFilter) {  // works on the input uav texture
    const auto& resource = uav_texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;
    const bool isSRGB = (uav_texture.texture_options & TEXTURE_TRAITS::sRGB) != TEXTURE_TRAITS::None;
    const bool isNormalMap = (uav_texture.texture_options & TEXTURE_TRAITS::NormalMap) != TEXTURE_TRAITS::None;

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    ID3D12DescriptorHeap* desc_heap[] = {m_SrvDescHeap.Get()};
    commandList->SetDescriptorHeaps(1, desc_heap);

    commandList->SetPipelineState(m_PipelineState.Get());
    commandList->SetComputeRootSignature(m_rootSignature.Get());

    int NumMips = GPU_texture::CalculateMipCount(width, height);
    NumMips = only5Mips ? 6 : std::min(NumMips, MipsLimit);

    CD3DX12_CPU_DESCRIPTOR_HANDLE cpuHandle(HeapStartCpu);
    CD3DX12_GPU_DESCRIPTOR_HANDLE gpuHandle(HeapStartGpu);

    for (int mipLevel = 0; mipLevel < NumMips; ++mipLevel) {
        uav_texture.GetUAVHandleForMipLevel(mipLevel, cpuHandle);
        cpuHandle.Offset(1, HeapHandleIncrement);
    }

    int GroupsX = (GPU_texture::GetMipDimension(width, 1) + 15) / 16;
    int GroupsY = (GPU_texture::GetMipDimension(height, 1) + 15) / 16;
    int numGroups = GroupsX * GroupsY;

    commandList->SetComputeRootUnorderedAccessView(GlobalRootSignatureParams::GroupCounters, GroupCounters->GetGPUVirtualAddress());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputViewSlot, gpuHandle);
    MipCSInput input{NumMips, isSRGB, isNormalMap, minFilter, numGroups};
    constexpr int inputSizeInInt = sizeof(MipCSInput) / 4;
    commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);

    commandList->Dispatch(GroupsX, GroupsY, 1);
}

GPU_texture Mipmaps_helper::GetBlankCompatibleUAVTex(GPU_texture& texture) {
    const auto& resource = texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;
    TEXTURE_TRAITS flags = texture.texture_options | TEXTURE_TRAITS::AllocateMips | TEXTURE_TRAITS::UAV;
    return GPU_texture{width, height, flags};
}

void Mipmaps_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
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

void Mipmaps_helper::Init() { 
    if (!GroupCounters) {
        CreateCounterBuffer();
    }
}

void Mipmaps_helper::CreateCounterBuffer() {
    D3DContext& d3d_ctx = D3DContext::Get();
    const auto& device = d3d_ctx.m_d3dDevice;

    const UINT bufferSize = 4;

    D3D12_RESOURCE_DESC desc = {};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    desc.Alignment = 0;
    desc.Width = bufferSize;
    desc.Height = 1;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.Format = DXGI_FORMAT_UNKNOWN;
    desc.SampleDesc.Count = 1;
    desc.SampleDesc.Quality = 0;
    desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    desc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

    D3D12_HEAP_PROPERTIES heapProps = {};
    heapProps.Type = D3D12_HEAP_TYPE_DEFAULT;

    // Create the actual VRAM allocation
    HRESULT hr = device->CreateCommittedResource(
        &heapProps, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&GroupCounters));

    const D3D12_HEAP_PROPERTIES upload_props{
        .Type = D3D12_HEAP_TYPE_UPLOAD,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        .CreationNodeMask = 1,
        .VisibleNodeMask = 1,
    };

    const D3D12_RESOURCE_DESC upload_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = bufferSize,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE,
    };

    ThrowIfFailed(d3d_ctx.m_d3dDevice->CreateCommittedResource(&upload_props, D3D12_HEAP_FLAG_NONE, &upload_desc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&zero_uploadBuffer)));

    void* pDataBegin = nullptr;
    int zero = 0;
    ThrowIfFailed(zero_uploadBuffer->Map(0, nullptr, &pDataBegin));
    memcpy(pDataBegin, &zero, bufferSize);
    zero_uploadBuffer->Unmap(0, nullptr);

    d3d_ctx.m_DXRCommandList->CopyBufferRegion(GroupCounters.Get(), 0, zero_uploadBuffer.Get(), 0, bufferSize);
}

void Mipmaps_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_PipelineState.Reset();
    m_SrvDescHeap.Reset();
    GroupCounters.Reset();
}

}  // namespace app