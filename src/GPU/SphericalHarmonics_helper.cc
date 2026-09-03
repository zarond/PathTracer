#include "SphericalHarmonics_helper.h"

#include <glm/glm.hpp>

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    SrcTexture,
    TmpBuffer,
    GroupCounters,
    RootConstants,

    Count
};
}

namespace app {

using namespace glm;

struct SHCSInput {
    uvec2 FrameSize;
    fvec2 inv_FrameSize;
    uvec2 GroupsSize;
    uint numGroups;
};

ComPtr<ID3D12RootSignature> SphericalHarmonics_helper::m_rootSignature{};
ComPtr<ID3D12PipelineState> SphericalHarmonics_helper::m_PipelineState{};

SphericalHarmonics_helper::SphericalHarmonics_helper() {
    if (!m_rootSignature || !m_PipelineState) {
        Reload();
    }
}

void SphericalHarmonics_helper::Reload() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

SphericalHarmonics_helper::~SphericalHarmonics_helper() { release_gpu_resources(); }

void SphericalHarmonics_helper::Init() {
    if (!GroupCounters) {
        CreateCounterBuffer();
    }
}

void SphericalHarmonics_helper::resize_tmp_buffer(int new_width, int new_height) {
    if (tmpWidth == new_width && tmpHeight == new_height) return;
    tmpWidth = std::max(new_width, 0);
    tmpHeight = std::max(new_height, 0);

    int GroupsX = (tmpWidth + 15) / 16;
    int GroupsY = (tmpHeight + 15) / 16;
    int numGroups = GroupsX * GroupsY;

    tmp_buffer.Reset();

    const UINT bufferSize = 9 * numGroups * 4 * sizeof(float);

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

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    HRESULT hr = device->CreateCommittedResource(
        &heapProps, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&tmp_buffer));
}

void SphericalHarmonics_helper::Compute(const GPU_texture& envmap) {
    const auto& resource = envmap.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;

    resize_tmp_buffer(width, height);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()};
    commandList->SetDescriptorHeaps(1, desc_heap);

    commandList->SetComputeRootSignature(m_rootSignature.Get());
    commandList->SetPipelineState(m_PipelineState.Get());

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::SrcTexture, envmap.GetSRVHandle());
    commandList->SetComputeRootUnorderedAccessView(GlobalRootSignatureParams::TmpBuffer, tmp_buffer->GetGPUVirtualAddress());

    int GroupsX = (width + 15) / 16;
    int GroupsY = (height + 15) / 16;
    int numGroups = GroupsX * GroupsY;

    SHCSInput input{{width, height}, {1.0f / width, 1.0f / height}, {GroupsX, GroupsY}, static_cast<uint>(numGroups)};
    constexpr int inputSizeInInt = sizeof(SHCSInput) / 4;
    commandList->SetComputeRootUnorderedAccessView(GlobalRootSignatureParams::GroupCounters, GroupCounters->GetGPUVirtualAddress());
    commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);

    commandList->Dispatch(GroupsX, GroupsY, 1);

    const auto barrier = CD3DX12_RESOURCE_BARRIER::UAV(GroupCounters.Get());
    commandList->ResourceBarrier(1, &barrier);
}

void SphericalHarmonics_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);  // 1 SrcTexture srv
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 TmpBuffer uav
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0, 1);  // 1 GroupCounters buffer
    ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0);  // 1 constant buffer.

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::SrcTexture].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::TmpBuffer].InitAsUnorderedAccessView(0, 0);
    rootParameters[GlobalRootSignatureParams::GroupCounters].InitAsUnorderedAccessView(0, 1);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(SHCSInput) / 4, 0);

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 0, nullptr, flags);

    SerializeAndCreateRootSignature(rootSignatureDesc, &m_rootSignature);
}

void SphericalHarmonics_helper::CreatePipelineStateObject() {
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

void SphericalHarmonics_helper::CreateCounterBuffer() {
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

std::vector<fvec4> SphericalHarmonics_helper::download_result_from_gpu() const {
    std::vector<hdr_pixel> readback_data;
    readback_data.resize(9);

    if (!tmp_buffer) {
        return readback_data;
    }

    D3DContext& d3d_ctx = D3DContext::Get();

    const D3D12_HEAP_PROPERTIES heap_props{
        .Type = D3D12_HEAP_TYPE_READBACK,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
    };

    // To copy only first 9 float4 values
    UINT64 totalBytes = 9 * sizeof(fvec4);

    const D3D12_RESOURCE_DESC upload_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = totalBytes,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE,
    };

    ComPtr<ID3D12Resource> readbackBuffer;
    d3d_ctx.m_d3dDevice->CreateCommittedResource(
        &heap_props, D3D12_HEAP_FLAG_NONE, &upload_desc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&readbackBuffer));

    d3d_ctx.InitDXRCommandList();

    auto barrier_uav = CD3DX12_RESOURCE_BARRIER::Transition(tmp_buffer.Get(),
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE);
    d3d_ctx.m_DXRCommandList->ResourceBarrier(1, &barrier_uav);

    d3d_ctx.m_DXRCommandList->CopyBufferRegion(readbackBuffer.Get(), 0, tmp_buffer.Get(), 0, totalBytes);

    barrier_uav = CD3DX12_RESOURCE_BARRIER::Transition(
        tmp_buffer.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
    d3d_ctx.m_DXRCommandList->ResourceBarrier(1, &barrier_uav);

    d3d_ctx.DispatchDXRCommandList();
    d3d_ctx.WaitForPendingDXR();

    void* mappedData = nullptr;
    readbackBuffer->Map(0, nullptr, &mappedData);

    memcpy(readback_data.data(), (void*)((uintptr_t)mappedData), totalBytes);

    readbackBuffer->Unmap(0, nullptr);
    return readback_data;
}

void SphericalHarmonics_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_PipelineState.Reset();
    GroupCounters.Reset();
    zero_uploadBuffer.Reset();
    tmp_buffer.Reset();
}

}  // namespace app
