#include "Silhouette_helper.h"

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    Depth,
    OutputUAV,

    Count
};
}

namespace app {

using namespace glm;

ComPtr<ID3D12RootSignature> Silhouette_helper::m_rootSignature{};
ComPtr<ID3D12PipelineState> Silhouette_helper::m_PipelineState{};

Silhouette_helper::Silhouette_helper() {
    if (!m_rootSignature || !m_PipelineState) {
        Reload();
    }
}

void Silhouette_helper::Reload() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

Silhouette_helper::~Silhouette_helper() { release_gpu_resources(); }

void Silhouette_helper::Apply(GPU_texture& uav_texture, GPU_texture& Depth_texture) {
    const auto& resource = uav_texture.get_gpu_resource();
    const auto description = resource->GetDesc();
    const auto width = description.Width;
    const auto height = description.Height;

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    commandList->SetComputeRootSignature(m_rootSignature.Get());
    commandList->SetPipelineState(m_PipelineState.Get());

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::Depth, Depth_texture.GetSRVHandle());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputUAV, uav_texture.GetUAVHandle());

    int GroupsX = (width + 15) / 16;
    int GroupsY = (height + 15) / 16;

    commandList->Dispatch(GroupsX, GroupsY, 1);
}

void Silhouette_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);  // 1 Depth srv
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 output uav

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::Depth].InitAsDescriptorTable(1, &ranges[0]);
    rootParameters[GlobalRootSignatureParams::OutputUAV].InitAsDescriptorTable(1, &ranges[1]);

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 0, nullptr, flags);

    SerializeAndCreateRootSignature(rootSignatureDesc, &m_rootSignature);
}

void Silhouette_helper::CreatePipelineStateObject() {
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

void Silhouette_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_PipelineState.Reset();
}

}  // namespace app