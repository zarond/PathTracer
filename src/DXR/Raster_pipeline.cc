#include "Raster_pipeline.h"

#include <d3dcompiler.h>

#include <iostream>

#include "../d3d_context.h"
#include "helpers/DXSampleHelper.h"

namespace app {

using namespace glm;  

void SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig);

app::Raster_pipeline::Raster_pipeline() {
    CreateRootSignatures();
    CreatePipelineStateObjects();
}

Raster_pipeline::~Raster_pipeline() { release_gpu_resources(); }

void Raster_pipeline::release_gpu_resources() {

}

void Raster_pipeline::CreateRootSignatures() {
    // Create an empty root signature.
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;
    rootSignatureDesc.Init(0, nullptr, 0, nullptr, D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

    SerializeAndCreateRaytracingRootSignature(rootSignatureDesc, &m_rootSignature);
}

void Raster_pipeline::CreatePipelineStateObjects() {
    ComPtr<ID3DBlob> shaderBlob;
    auto hr = D3DReadFileToBlob(c_dxilLibraryName, &shaderBlob);
    if (FAILED(hr)) {
        std::wcout << "Failed to read DXIL library: " << c_dxilLibraryName << std::endl;
        throw HrException(hr);
    }
    D3D12_SHADER_BYTECODE libdxil = {
        .pShaderBytecode = shaderBlob->GetBufferPointer(),
        .BytecodeLength = shaderBlob->GetBufferSize(),
    };
    /*
    CD3DX12_STATE_OBJECT_DESC Pipeline{D3D12_STATE_OBJECT_TYPE_EXECUTABLE};
    auto lib = Pipeline.CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
    lib->SetDXILLibrary(&libdxil);
    //lib->DefineExport(L"VSMain");
    //lib->DefineExport(L"PSMain");

    // Optional flag to allow state object additions
    auto pConfig = Pipeline.CreateSubobject<CD3DX12_STATE_OBJECT_CONFIG_SUBOBJECT>();
    pConfig->SetFlags(D3D12_STATE_OBJECT_FLAG_ALLOW_STATE_OBJECT_ADDITIONS);

    // 1. Associate a Global Root Signature
    // This defines how resources are bound for all shaders in this state object
    auto globalRootSig = Pipeline.CreateSubobject<CD3DX12_GLOBAL_ROOT_SIGNATURE_SUBOBJECT>();
    globalRootSig->SetRootSignature(m_rootSignature.Get());

    // 2. Define the Input Layout
    // This maps your vertex buffer data to the VSMain input
    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}};


    auto inputLayout = Pipeline.CreateSubobject<CD3DX12_INPUT_LAYOUT_SUBOBJECT>();

    for (UINT i = 0; i < _countof(inputElementDescs); i++) {
        inputLayout->AddInputLayoutElementDesc(inputElementDescs[i]);
    }

    // 3. Define the Rasterizer State
    auto rasterizer = Pipeline.CreateSubobject<CD3DX12_RASTERIZER_SUBOBJECT>();
    rasterizer->SetCullMode(D3D12_CULL_MODE_BACK);

    // 4. Define the Blend State
    auto blend = Pipeline.CreateSubobject<CD3DX12_BLEND_SUBOBJECT>(CD3DX12_BLEND_DESC(D3D12_DEFAULT));

    // 5. Define the Depth Stencil State
    auto depthStencil = Pipeline.CreateSubobject<CD3DX12_DEPTH_STENCIL_SUBOBJECT>(CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT));

    // 6. Define Render Target Formats (Equivalent to RTVFormats in PSO) and Depth Stencil Format (Equivalent to DSVFormat in PSO)
    auto rtFormats = Pipeline.CreateSubobject<CD3DX12_RENDER_TARGET_FORMATS_SUBOBJECT>();
    rtFormats->SetNumRenderTargets(1);
    rtFormats->SetRenderTargetFormat(0, DXGI_FORMAT_R32G32B32A32_FLOAT);

    auto depthFormats = Pipeline.CreateSubobject<CD3DX12_DEPTH_STENCIL_FORMAT_SUBOBJECT>();
    depthFormats->SetDepthStencilFormat(DXGI_FORMAT_D32_FLOAT);

    // 7. Set the Primitive Topology Type
    auto topology = Pipeline.CreateSubobject<CD3DX12_PRIMITIVE_TOPOLOGY_SUBOBJECT>();
    topology->SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);

    // 8 . Create a Generic Program Subobject
    auto pGenericProgram = Pipeline.CreateSubobject<CD3DX12_GENERIC_PROGRAM_SUBOBJECT>();
    pGenericProgram->SetProgramName(L"GraphicsProgram");
    pGenericProgram->AddExport(L"VSMain");
    pGenericProgram->AddExport(L"PSMain");
    pGenericProgram->AddSubobject(*inputLayout);
    pGenericProgram->AddSubobject(*topology);
    pGenericProgram->AddSubobject(*rtFormats);
    pGenericProgram->AddSubobject(*depthFormats);

    // 9. Create the State Object
    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    hr = device->CreateStateObject(Pipeline, IID_PPV_ARGS(&m_pipelineState));
    if (FAILED(hr)) {
        std::wcout << "Failed to create state object." << std::endl;
        throw HrException(hr);
    }
    */

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}};

    // Describe and create the graphics pipeline state object (PSO).
    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = {inputElementDescs, _countof(inputElementDescs)};
    psoDesc.pRootSignature = m_rootSignature.Get();
    psoDesc.VS = libdxil;
    psoDesc.PS = libdxil;
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.DepthStencilState.DepthEnable = FALSE;
    psoDesc.DepthStencilState.StencilEnable = FALSE;
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R32G32B32A32_FLOAT;
    psoDesc.SampleDesc.Count = 1;
    //ThrowIfFailed(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_pipelineState)));

}

}