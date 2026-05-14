#include "DFG_Lut_helper.h"

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    OutputViewSlot = 0,

    Count
};
}

namespace app {

DFG_Lut_helper::DFG_Lut_helper() { 
    CreateRootSignature();
    CreatePipelineStateObject();
}

DFG_Lut_helper::~DFG_Lut_helper() { release_gpu_resources(); }

void DFG_Lut_helper::CreateDFG_Lut() {
    DFG_lut.release_gpu_resource();
    DFG_lut = GPU_texture{DFG_size, DFG_size, true, false, false, false, false, true};

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()};
    commandList->SetDescriptorHeaps(1, desc_heap);

    commandList->SetPipelineState(m_pipelineState.Get());
    commandList->SetComputeRootSignature(m_rootSignature.Get());

    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputViewSlot, DFG_lut.GetUAVHandle());

    commandList->Dispatch(DFG_size / 8, DFG_size / 8, 1);
}

GPU_texture&& DFG_Lut_helper::GetDFG_Lut() { return std::move(DFG_lut); }

void DFG_Lut_helper::CreateRootSignature() {
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 output texture

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::OutputViewSlot].InitAsDescriptorTable(1, &ranges[0]);

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 0, nullptr, flags);

    SerializeAndCreateRaytracingRootSignature(rootSignatureDesc, &m_rootSignature);
}

void DFG_Lut_helper::CreatePipelineStateObject() {
    auto [cs_shaderBlob, cs_bytecode] = LoadShader(c_cs_file_name);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_COMPUTE_PIPELINE_STATE_DESC computePsoDesc = {};
    computePsoDesc.pRootSignature = m_rootSignature.Get();  // Must match shader layout
    computePsoDesc.CS = cs_bytecode;
    computePsoDesc.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
    computePsoDesc.NodeMask = 1;

    ThrowIfFailed(device->CreateComputePipelineState(&computePsoDesc, IID_PPV_ARGS(&m_pipelineState)));
}

void DFG_Lut_helper::release_gpu_resources() {
    m_rootSignature.Reset();
    m_pipelineState.Reset();
}

}