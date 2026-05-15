#include "Raster_pipeline.h"

#include <algorithm>
#include <span>

#include "../d3d_context.h"
#include "Common_helpers.h"
#include "helpers/DXSampleHelper.h"
#include "DFG_Lut_helper.h"
#include "Cubemaps_helper.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    SceneConstantSlot = 0,
    MaterialsBufferSlot,
    EnvmapTex,
    DFGTex,
    DiffuseLutTex,
    RootConstants,

    Count
};
}

namespace app {

using namespace glm;

Raster_pipeline::Raster_pipeline() {
    CreateRootSignatures();
    CreatePipelineStateObjects();
    CreateConstantBuffers();

    ComputeDFGLut();
}

void Raster_pipeline::OnModelLoad(GPU_model& gpu_model) {
    // todo: generate mips
}

void Raster_pipeline::OnEnvmapLoad(const GPU_texture& envmap) {
    // todo: generate cubemaps
    ComputeDiffuseLut(envmap);
}

Raster_pipeline::~Raster_pipeline() { release_gpu_resources(); }

void Raster_pipeline::release_gpu_resources() {
    m_rootSignature.Reset();

    m_pipelineState.Reset();
    m_alphaBlendingPipelineState.Reset();
    m_backgroundPipelineState.Reset();

    m_perFrameConstants.Reset();

}

void Raster_pipeline::SetRenderingSettings(const RenderSettings& render_settings, fvec3 origin, const fmat4x4& NDC2WorldMatrix,
    const fmat4x4& ViewMatrix, const fmat4x4& ProjectionMatrix, fvec2 subpixelOffset, unsigned int frameID, int iterationCount,
    float invIterationCount) {
    m_rasterCB.cameraPosition = xyz1(origin);
    m_rasterCB.projectionToWorld = NDC2WorldMatrix;
    m_rasterCB.viewProjection = ProjectionMatrix * ViewMatrix;
    m_rasterCB.subpixelOffset = subpixelOffset;
    m_rasterCB.frameID = frameID;
    m_rasterCB.iteration = iterationCount;
    m_rasterCB.invIterationCount = invIterationCount;
    m_rasterCB.samplesPerPixel = render_settings.samplesPerPixel;
    m_rasterCB.invSamplesPerPixel = 1.0f / static_cast<float>(render_settings.samplesPerPixel);
    m_rasterCB.maxNewRaysPerBounce = render_settings.maxNewRaysPerBounce;
    m_rasterCB.invMaxNewRaysPerBounce = 1.0f / render_settings.maxNewRaysPerBounce;
    m_rasterCB.maxRayBounces = render_settings.maxRayBounces;
    m_rasterCB.envmapRotation = render_settings.envmapRotation;
    RaytracingMode = render_settings.programMode;
}

void Raster_pipeline::CreateRootSignatures() {
    // Create an empty root signature.
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[6];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);     // 1 scene constants buffer.
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1);     // 2 materials buffer.
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 1);  // 3 Envmap texture.
    ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1, 1);  // 4 DFG texture.
    ranges[4].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 2, 1);  // 5 DiffuseLut texture.
    ranges[5].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1);     // 6 per draw constants buffer.

    D3D12_STATIC_SAMPLER_DESC default_sampler = {};  // Default static sampler.
    default_sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    default_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    default_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    default_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    default_sampler.ShaderRegister = 0;  // s0
    default_sampler.RegisterSpace = 1;
    default_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_STATIC_SAMPLER_DESC dfg_sampler = {};
    dfg_sampler.Filter = D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT;
    dfg_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    dfg_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    dfg_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    dfg_sampler.ShaderRegister = 1;  // s1
    dfg_sampler.RegisterSpace = 1;
    dfg_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_STATIC_SAMPLER_DESC samplers[] = {default_sampler, dfg_sampler};

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::SceneConstantSlot].InitAsConstantBufferView(0);
    rootParameters[GlobalRootSignatureParams::MaterialsBufferSlot].InitAsDescriptorTable(1, &ranges[1]);
    rootParameters[GlobalRootSignatureParams::EnvmapTex].InitAsDescriptorTable(1, &ranges[2]);
    rootParameters[GlobalRootSignatureParams::DFGTex].InitAsDescriptorTable(1, &ranges[3]);
    rootParameters[GlobalRootSignatureParams::DiffuseLutTex].InitAsDescriptorTable(1, &ranges[4]);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(RasterPerDrawData) / 4, 1);

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT 
        | D3D12_ROOT_SIGNATURE_FLAG_CBV_SRV_UAV_HEAP_DIRECTLY_INDEXED;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 2, samplers, flags);

    SerializeAndCreateRaytracingRootSignature(rootSignatureDesc, &m_rootSignature);
}

void Raster_pipeline::CreatePipelineStateObjects() {
    auto [vs_shaderBlob, vs_bytecode] = LoadShader(c_vs_file_name);
    auto [ps_shaderBlob, ps_bytecode] = LoadShader(c_ps_file_name);
    auto [vs_background_shaderBlob, vs_background_bytecode] = LoadShader(c_vs_background_file_name);
    auto [ps_background_shaderBlob, ps_background_bytecode] = LoadShader(c_ps_background_file_name);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"NORMAL", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 16, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TANGENT", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 16*2, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 16*3, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}};

    // Describe and create the graphics pipeline state object (PSO).
    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = {inputElementDescs, _countof(inputElementDescs)};
    psoDesc.pRootSignature = m_rootSignature.Get();
    psoDesc.VS = vs_bytecode;
    psoDesc.PS = ps_bytecode;
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.RasterizerState.FrontCounterClockwise = TRUE;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL ^ D3D12_COLOR_WRITE_ENABLE_ALPHA;
    psoDesc.DepthStencilState.DepthEnable = TRUE;
    psoDesc.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
    psoDesc.DepthStencilState.StencilEnable = FALSE;
    psoDesc.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R32G32B32A32_FLOAT;
    psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    psoDesc.SampleDesc.Count = 1;
    ThrowIfFailed(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_pipelineState)));

    // PSO for alpha blending
    psoDesc.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
    psoDesc.BlendState.RenderTarget[0].BlendEnable = TRUE;
    psoDesc.BlendState.RenderTarget[0].SrcBlend = D3D12_BLEND_SRC_ALPHA;
    psoDesc.BlendState.RenderTarget[0].DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
    psoDesc.BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
    psoDesc.BlendState.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ZERO;
    psoDesc.BlendState.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ONE;
    psoDesc.BlendState.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
    ThrowIfFailed(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_alphaBlendingPipelineState)));

    // PSO for background rendering
    psoDesc.InputLayout = {nullptr, 0};
    psoDesc.VS = vs_background_bytecode;
    psoDesc.PS = ps_background_bytecode;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL ^ D3D12_COLOR_WRITE_ENABLE_ALPHA;
    ThrowIfFailed(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_backgroundPipelineState)));
}

void Raster_pipeline::CreateConstantBuffers() {
    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    // Create the constant buffer memory and map the CPU and GPU addresses
    const D3D12_HEAP_PROPERTIES uploadHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD);

    // Allocate one constant buffer per frame, since it gets updated every frame.
    size_t cbSize = /* frameCount **/ sizeof(AlignedSceneConstantBuffer);
    const D3D12_RESOURCE_DESC constantBufferDesc = CD3DX12_RESOURCE_DESC::Buffer(cbSize);

    ThrowIfFailed(device->CreateCommittedResource(&uploadHeapProperties, D3D12_HEAP_FLAG_NONE, &constantBufferDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&m_perFrameConstants)));

    // Map the constant buffer and cache its heap pointers.
    // We don't unmap this until the app closes. Keeping buffer mapped for the lifetime of the resource is okay.
    CD3DX12_RANGE readRange(0, 0);  // We do not intend to read from this resource on the CPU.
    ThrowIfFailed(m_perFrameConstants->Map(0, nullptr, reinterpret_cast<void**>(&m_mappedConstantData)));
}

void Raster_pipeline::DoRender(const GPU_model& gpu_model, const GPU_texture& envmap, const CPUFrameBuffer& framebuffer) {
    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    UINT width = framebuffer.width();
    UINT height = framebuffer.height();

    resize_render_targets(width, height);

    m_viewport = CD3DX12_VIEWPORT{0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height)},
    m_scissorRect = CD3DX12_RECT{0, 0, static_cast<LONG>(width), static_cast<LONG>(height)},

    commandList->SetGraphicsRootSignature(m_rootSignature.Get());
    commandList->SetPipelineState(m_pipelineState.Get());
    commandList->RSSetViewports(1, &m_viewport);
    commandList->RSSetScissorRects(1, &m_scissorRect);

    // Copy the updated scene constant buffer to GPU.
    memcpy(&m_mappedConstantData->constants, &m_rasterCB, sizeof(m_rasterCB));
    auto cbGpuAddress = m_perFrameConstants->GetGPUVirtualAddress();
    commandList->SetGraphicsRootConstantBufferView(GlobalRootSignatureParams::SceneConstantSlot, cbGpuAddress);
    commandList->SetGraphicsRootDescriptorTable(
        GlobalRootSignatureParams::MaterialsBufferSlot, gpu_model.materials_array.gpuHandle);
    commandList->SetGraphicsRootDescriptorTable(GlobalRootSignatureParams::EnvmapTex, envmap.GetSRVHandle());
    commandList->SetGraphicsRootDescriptorTable(GlobalRootSignatureParams::DFGTex, DFG_lut.GetSRVHandle());
    commandList->SetGraphicsRootDescriptorTable(GlobalRootSignatureParams::DiffuseLutTex, Diffuse_lut.GetSRVHandle());

    const auto& combined_mesh = gpu_model.get_combined_mesh();

    UINT vertex_count = combined_mesh.get_vertex_count();
    UINT index_count = combined_mesh.get_index_count();

    D3D12_VERTEX_BUFFER_VIEW vertexBufferView;
    vertexBufferView.BufferLocation = combined_mesh.vertexBuffer->GetGPUVirtualAddress();
    vertexBufferView.StrideInBytes = sizeof(vertex);
    vertexBufferView.SizeInBytes = sizeof(vertex) * vertex_count;

    D3D12_INDEX_BUFFER_VIEW indexBufferView;
    indexBufferView.BufferLocation = combined_mesh.indexBuffer->GetGPUVirtualAddress();
    indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    indexBufferView.SizeInBytes = sizeof(uint32_t) * index_count;

    auto rtvHandle = m_renderTarget.GetRTVHandle();
    auto depthHandle = m_depthTexture.GetDSVHandle();

    const float clearColor[] = {0.0f, 0.0f, 0.0f, 1.0f};
    commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &depthHandle);
    commandList->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);
    commandList->ClearDepthStencilView(depthHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
    commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    commandList->IASetVertexBuffers(0, 1, &vertexBufferView);
    commandList->IASetIndexBuffer(&indexBufferView);

    int num_opaque_objects;
    int num_alpha_blended_objects;
    sort_objects_for_rendering(gpu_model, num_opaque_objects, num_alpha_blended_objects);
    std::span<const DrawableSortingInfo> opaque_objects{
        m_sortedDrawables.begin(), m_sortedDrawables.begin() + num_opaque_objects};
    std::span<const DrawableSortingInfo> blend_objects{
        m_sortedDrawables.begin() + num_opaque_objects, m_sortedDrawables.end()};

    // draw mesh lambda
    auto draw_object = [&](const DrawableSortingInfo element) {
        const auto& obj = *element.object;
        index_count = gpu_model.indicesSizes[obj.meshIndex];
        UINT baseIndexLocation = gpu_model.indicesOffsets[obj.meshIndex];
        RasterPerDrawData draw_data{obj.ModelMatrix, obj.NormalMatrix, obj.meshIndex};
        constexpr int drawDataSizeInInt = sizeof(RasterPerDrawData) / 4;
        commandList->SetGraphicsRoot32BitConstants(GlobalRootSignatureParams::RootConstants, drawDataSizeInInt, &draw_data, 0);
        commandList->DrawIndexedInstanced(index_count, 1, baseIndexLocation, 0, 0);
     };

    // Draw opaque objects
    for (const auto& element : opaque_objects) {
        draw_object(element);
    }
    // Draw background
    commandList->SetPipelineState(m_backgroundPipelineState.Get());
    commandList->DrawInstanced(3, 1, 0, 0);

    // Draw objects with alpha blending
    commandList->SetPipelineState(m_alphaBlendingPipelineState.Get());
    for (const auto& element : blend_objects) {
        draw_object(element);
    }

    copy_render_target_to_framebuffer(framebuffer);
}

void Raster_pipeline::copy_render_target_to_framebuffer(const CPUFrameBuffer& framebuffer) {
    D3DContext& d3d_ctx = D3DContext::Get();
    const auto& commandList = d3d_ctx.m_DXRCommandList;
    const auto& gpuDestTarget = framebuffer.get_gpu_resource();
    const auto& gpuRenderTarget = m_renderTarget.get_gpu_resource();

    D3D12_RESOURCE_BARRIER to_barriers[2] = {
        CD3DX12_RESOURCE_BARRIER::Transition(gpuRenderTarget.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_SOURCE),
        CD3DX12_RESOURCE_BARRIER::Transition(gpuDestTarget.Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COMMON),
    };
    commandList->ResourceBarrier(2, to_barriers);

    commandList->CopyResource(gpuDestTarget.Get(), gpuRenderTarget.Get());

    D3D12_RESOURCE_BARRIER from_barriers[2] = {
        CD3DX12_RESOURCE_BARRIER::Transition(gpuRenderTarget.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET),
        CD3DX12_RESOURCE_BARRIER::Transition(gpuDestTarget.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE),
    };
    commandList->ResourceBarrier(2, from_barriers);
}

void Raster_pipeline::resize_render_targets(int new_width, int new_height) {
    if (currentWidth == new_width && currentHeight == new_height) return;
    currentWidth = std::max(new_width, 0);
    currentHeight = std::max(new_height, 0);
    m_depthTexture.release_gpu_resource();
    TEXTURE_TRAITS flags = TEXTURE_TRAITS::Depth;
    m_depthTexture = GPU_texture{currentWidth, currentHeight, flags};

    m_renderTarget.release_gpu_resource();
    flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::RenderTarget;
    m_renderTarget = GPU_texture{currentWidth, currentHeight, flags};
}

void Raster_pipeline::sort_objects_for_rendering(const GPU_model& gpu_model, int& num_opaque_objects, int& num_alpha_blended_objects) {
    num_opaque_objects = 0;
    num_alpha_blended_objects = 0;
    m_sortedDrawables.clear();
    m_sortedDrawables.reserve(gpu_model.objects.size());
    const auto& materials = gpu_model.get_materials_cpu_array();
    fvec3 camera_forward = normalize(xyz(m_rasterCB.projectionToWorld * fvec4(0, 0, 0, 1)));
    for (const auto& obj : gpu_model.objects) {
        fvec3 v = xyz(obj.ModelMatrix[3]) - xyz(m_rasterCB.cameraPosition);
        const auto& mat = materials[obj.meshIndex];
        float ZDistanceToCamera = dot(camera_forward, v);
        bool alphaBlending = mat.alphaBlending;
        bool transmittance = mat.hasVolume;

        if (alphaBlending) {
            num_alpha_blended_objects++;
        } else {
            num_opaque_objects++;
        }

        m_sortedDrawables.emplace_back(&obj, ZDistanceToCamera, alphaBlending, transmittance);
    }
    std::sort(m_sortedDrawables.begin(), m_sortedDrawables.end(), [](const DrawableSortingInfo& a, const DrawableSortingInfo& b) {
        if (a.alphaBlending != b.alphaBlending) {
            return !a.alphaBlending;  // opaque first
        }
        if (!a.alphaBlending) {
            return a.ZDistanceToCamera < b.ZDistanceToCamera;  // front to back for opaque objects
        } else {
            return a.ZDistanceToCamera > b.ZDistanceToCamera;  // back to front for alpha blended objects
        }
    });
}

void Raster_pipeline::ComputeDFGLut() {
    DFG_Lut_helper DFG_compute_helper{};
    DFG_compute_helper.CreateDFG_Lut();
    DFG_lut = std::move(DFG_compute_helper.GetDFG_Lut());

    D3DContext& d3d_ctx = D3DContext::Get();
    d3d_ctx.DispatchDXRCommandList(); // WARNING: this assumes that command list is open and ready at this point
    d3d_ctx.WaitForPendingDXR();

    d3d_ctx.InitDXRCommandList();
}

void Raster_pipeline::ComputeDiffuseLut(const GPU_texture& envmap) {
    D3DContext& d3d_ctx = D3DContext::Get();
    d3d_ctx.InitDXRCommandList();
    
    EnvCube_helper EnvCube_helper{};
    EnvCube_helper.CreateDiffuseEnvmapCube(envmap);
    Diffuse_lut = std::move(EnvCube_helper.GetDiffuseEnvmapCube());

    d3d_ctx.DispatchDXRCommandList();  // WARNING: this assumes that command list is open and ready at this point
    d3d_ctx.WaitForPendingDXR();

    //d3d_ctx.InitDXRCommandList();
}

}