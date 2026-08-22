#include "Copy_helper.h"

#include <glm/glm.hpp>

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    SrcTexture,
    DstTexture,
    RootConstants,

    Count
};
}

namespace app {

using namespace glm;

struct CopyCSInput {
    uvec2 FrameSize;
};

ComPtr<ID3D12RootSignature> Copy_helper::m_rootSignature{};
ComPtr<ID3D12PipelineState> Copy_helper::m_PipelineState{};

Copy_helper::Copy_helper() {
    if (!m_rootSignature || !m_PipelineState) {
        Reload();
    }
}

void Copy_helper::Reload() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

Copy_helper::~Copy_helper() { release_gpu_resources(); }

void Copy_helper::Copy(GPU_texture& dst_texture, GPU_texture& src_texture) {
    const auto& resource = dst_texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;

    Copy(dst_texture.GetUAVHandle(), src_texture.GetSRVHandle(), width, height);
}

void Copy_helper::Copy(
    D3D12_GPU_DESCRIPTOR_HANDLE dst_uav_handle, D3D12_GPU_DESCRIPTOR_HANDLE src_srv_handle, int width, int height) {

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    commandList->SetComputeRootSignature(m_rootSignature.Get());
    commandList->SetPipelineState(m_PipelineState.Get());

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::SrcTexture, src_srv_handle);
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::DstTexture, dst_uav_handle);

    CopyCSInput input{{width, height}};
    constexpr int inputSizeInInt = sizeof(CopyCSInput) / 4;
    commandList->SetComputeRoot32BitConstants(GlobalRootSignatureParams::RootConstants, inputSizeInInt, &input, 0);

    int GroupsX = (width + 15) / 16;
    int GroupsY = (height + 15) / 16;

    commandList->Dispatch(GroupsX, GroupsY, 1);
}

void Copy_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);  // 1 SrcTexture srv
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 DstTexture uav
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0);  // 1 constant buffer.

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::SrcTexture].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::DstTexture].InitAsDescriptorTable(1, &ranges[1]);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(CopyCSInput) / 4, 0);

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 0, nullptr, flags);

    SerializeAndCreateRootSignature(rootSignatureDesc, &m_rootSignature);
}

void Copy_helper::CreatePipelineStateObject() {
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

void Copy_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_PipelineState.Reset();
}

}  // namespace app