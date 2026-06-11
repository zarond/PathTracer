#include "Raster_pipeline.h"

#include <algorithm>
#include <span>
#include <chrono>
#include <iostream>

#include "../d3d_context.h"
#include "Common_helpers.h"
#include "helpers/DXSampleHelper.h"
#include "DFG_Lut_helper.h"
#include "Cubemaps_helper.h"
#include "Mipmaps_helper.h"
#include "Silhouette_helper.h"
#include "Reprojection_helper.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    SceneConstantSlot = 0,
    MaterialsBufferSlot,
    DFGTex,
    DiffuseLutTex,
    SpecularLutTex,
    FrameTex,
    AmbientOcclusionTex,
    SSRTex,
    MaterialIDTex,
    RootConstants,

    Count
};
}

namespace app {

using namespace glm;

ComPtr<ID3D12RootSignature> Raster_pipeline::m_rootSignature{};
ComPtr<ID3D12PipelineState> Raster_pipeline::m_pipelineState{};
ComPtr<ID3D12PipelineState> Raster_pipeline::m_alphaBlendingPipelineState{};
ComPtr<ID3D12PipelineState> Raster_pipeline::m_backgroundPipelineState{};
ComPtr<ID3D12PipelineState> Raster_pipeline::m_GbufferPipelineState{};

Raster_pipeline::Raster_pipeline() {
    if (!m_rootSignature || !m_pipelineState || !m_alphaBlendingPipelineState || !m_backgroundPipelineState ||
        !m_GbufferPipelineState) {
        Reload();
    }
    CreateConstantBuffers();

    ComputeDFGLut();
}

void Raster_pipeline::Reload() {
    CreateRootSignatures();
    CreatePipelineStateObjects();
}

void Raster_pipeline::OnModelLoad(GPU_model& gpu_model) {
    auto start = std::chrono::high_resolution_clock::now();

    auto& textures = gpu_model.get_textures();
    for (auto& texture : textures) {
        if ((texture.texture_options & TEXTURE_TRAITS::AllocateMips) != TEXTURE_TRAITS::None) {
            ComputeMipMaps(texture);
        }
    }

    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
    float time_ms = static_cast<float>(diff.count()) / 1000.0f;
    std::cout << "Model's texture's mips were computed in " << std::fixed << std::setprecision(2) << time_ms << " ms." << '\n';
}

void Raster_pipeline::OnEnvmapLoad(GPU_texture& envmap) {
    auto start = std::chrono::high_resolution_clock::now();
    ComputeMipMaps(envmap);
    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
    float time_ms = static_cast<float>(diff.count()) / 1000.0f;
    std::cout << "Envmap texture mips were computed in " << std::fixed << std::setprecision(2) << time_ms << " ms." << '\n';
    ComputeEnvmapLut(envmap);
}

Raster_pipeline::~Raster_pipeline() { release_gpu_resources(); }

void Raster_pipeline::release_gpu_resources() {
    m_rootSignature.Reset();

    m_pipelineState.Reset();
    m_alphaBlendingPipelineState.Reset();
    m_backgroundPipelineState.Reset();
    m_GbufferPipelineState.Reset();

    m_perFrameConstants.Reset();

}

void Raster_pipeline::SetRenderingSettings(const RenderSettings& render_settings, fvec3 origin, const fmat4x4& NDC2WorldMatrix,
    const fmat4x4& ViewMatrix, const fmat4x4& ProjectionMatrix, fvec2 subpixelOffset, unsigned int frameID, int iterationCount,
    float invIterationCount) {
    m_rasterCB.cameraPosition = xyz1(origin);
    m_rasterCB.projectionToWorld = NDC2WorldMatrix;
    m_rasterCB.viewProjection = ProjectionMatrix * ViewMatrix;
    m_rasterCB.viewMatrix = ViewMatrix;
    m_rasterCB.Projection = ProjectionMatrix;
    m_rasterCB.invProjection = inverse(ProjectionMatrix);
    m_rasterCB.envmap_rotation_cos = cos(render_settings.envmapRotation);
    m_rasterCB.envmap_rotation_sin = sin(render_settings.envmapRotation);
    m_rasterCB.subpixelOffset = subpixelOffset;
    m_rasterCB.frameID = frameID;
    m_rasterCB.albedoOnlyMode = (render_settings.programMode == RayProgramMode::RayCaster);
    m_rasterCB.SpecularLutMips = EnvCube_helper::SpecularMips;
    m_rasterCB.GTAOStrength = render_settings.GTAOStrength;
    m_rasterCB.TexturesAOStrength = render_settings.TexturesAOStrength;
    m_rasterCB.specular_aa_enabled = render_settings.specular_aa_enabled;
    m_rasterCB.specular_aa_variance = render_settings.specular_aa_variance;
    m_rasterCB.specular_aa_threshold = render_settings.specular_aa_threshold;
    m_GTAO_helper.AODistance = render_settings.AODistance;
    m_GTAO_helper.DenoiseEnabled = render_settings.AODenoiseEnabled;
    if (!m_GTAO_helper.DenoiseEnabled) {
        m_GTAO_helper.ResetFrameCounter();
    }
    m_GTAO_helper.DepthThreshold = render_settings.AOReprojectionDepthThreshold;
    m_GTAO_helper.AOSpatialSigma = render_settings.AOSpatialSigma;
    m_GTAO_helper.AODepthSigma = render_settings.AODepthSigma;
    m_GTAO_helper.AONormalSigma = render_settings.AONormalSigma;
    m_rasterCB.SSREnabled = render_settings.SSREnabled;
    DrawSSROnly = render_settings.DrawSSROnly;
    m_SSR_helper.DenoiseEnabled = render_settings.SSRDenoiseEnabled;
    m_SSR_helper.DepthThreshold = render_settings.SSRDepthThreshold;
    m_SSR_helper.MaxRoughness = render_settings.SSRMaxRoughness;
    m_SSR_helper.SSR_GGXClamp = render_settings.SSR_GGXClamp;
    m_SSR_helper.UsePrefiltering = render_settings.SSRUsePrefiltering;
    m_SSR_helper.PrefilteringDistance = render_settings.SSRPrefilteringDistance;
    Reprojection_helper::DebugMode = render_settings.ReprojectionDebugMode;
    RaytracingMode = render_settings.programMode;
}

void Raster_pipeline::CreateRootSignatures() {
    // Create an empty root signature.
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);     // 1 scene constants buffer.
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1);     // 2 materials buffer.
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 1);  // 3 DFG texture.
    ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1, 1);  // 4 DiffuseLut texture.
    ranges[4].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 2, 1);  // 5 SpecularLut texture.
    ranges[5].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 3, 1);  // 6 Frame texture.
    ranges[6].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 4, 1);  // 7 Ambient Occlusion texture.
    ranges[7].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 5, 1);  // 8 SSR texture.
    ranges[8].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 6, 1);  // 9 MaterialID texture
    ranges[9].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1);     // 10 per draw constants buffer.

    D3D12_STATIC_SAMPLER_DESC default_sampler = {};  // Default static sampler.
    default_sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    default_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    default_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    default_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    default_sampler.MaxLOD = D3D12_FLOAT32_MAX;
    default_sampler.ShaderRegister = 0;  // s0
    default_sampler.RegisterSpace = 1;
    default_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_STATIC_SAMPLER_DESC anisotropic_sampler = default_sampler;  // Anisotropic static sampler.
    anisotropic_sampler.MaxAnisotropy = 8;
    anisotropic_sampler.Filter = D3D12_FILTER_ANISOTROPIC;
    anisotropic_sampler.ShaderRegister = 1;

    D3D12_STATIC_SAMPLER_DESC dfg_sampler = {};
    dfg_sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    dfg_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    dfg_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    dfg_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    dfg_sampler.MaxLOD = D3D12_FLOAT32_MAX;
    dfg_sampler.ShaderRegister = 2;
    dfg_sampler.RegisterSpace = 1;
    dfg_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_STATIC_SAMPLER_DESC samplers[] = {default_sampler, anisotropic_sampler, dfg_sampler};

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::SceneConstantSlot].InitAsConstantBufferView(0);
    rootParameters[GlobalRootSignatureParams::MaterialsBufferSlot].InitAsDescriptorTable(1, &ranges[1]);
    rootParameters[GlobalRootSignatureParams::DFGTex].InitAsDescriptorTable(1, &ranges[2]);
    rootParameters[GlobalRootSignatureParams::DiffuseLutTex].InitAsDescriptorTable(1, &ranges[3]);
    rootParameters[GlobalRootSignatureParams::SpecularLutTex].InitAsDescriptorTable(1, &ranges[4]);
    rootParameters[GlobalRootSignatureParams::FrameTex].InitAsDescriptorTable(1, &ranges[5]);
    rootParameters[GlobalRootSignatureParams::AmbientOcclusionTex].InitAsDescriptorTable(1, &ranges[6]);
    rootParameters[GlobalRootSignatureParams::SSRTex].InitAsDescriptorTable(1, &ranges[7]);
    rootParameters[GlobalRootSignatureParams::MaterialIDTex].InitAsDescriptorTable(1, &ranges[8]);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(RasterPerDrawData) / 4, 1);

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT 
        | D3D12_ROOT_SIGNATURE_FLAG_CBV_SRV_UAV_HEAP_DIRECTLY_INDEXED;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 3, samplers, flags);

    SerializeAndCreateRootSignature(rootSignatureDesc, &m_rootSignature);
}

void Raster_pipeline::CreatePipelineStateObjects() {
    auto [vs_shaderBlob, vs_bytecode] = LoadShader(c_vs_file_name);
    auto [ps_shaderBlob, ps_bytecode] = LoadShader(c_ps_file_name);
    auto [vs_background_shaderBlob, vs_background_bytecode] = LoadShader(c_vs_background_file_name);
    auto [ps_background_shaderBlob, ps_background_bytecode] = LoadShader(c_ps_background_file_name);
    auto [vs_gbuff_shaderBlob, vs_gbuff_bytecode] = LoadShader(c_vs_gbuff_file_name);
    auto [ps_gbuff_shaderBlob, ps_gbuff_bytecode] = LoadShader(c_ps_gbuff_file_name);

    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"NORMAL", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 16, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TANGENT", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 16*2, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 16*3, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}};

    // Describe and create the graphics pipeline state object (PSO).
    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    D3D12_RENDER_TARGET_BLEND_DESC& rtBlend = psoDesc.BlendState.RenderTarget[0];
    psoDesc.InputLayout = {inputElementDescs, _countof(inputElementDescs)};
    psoDesc.pRootSignature = m_rootSignature.Get();
    psoDesc.VS = vs_bytecode;
    psoDesc.PS = ps_bytecode;
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.RasterizerState.FrontCounterClockwise = TRUE;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    rtBlend.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL ^ D3D12_COLOR_WRITE_ENABLE_ALPHA;
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

    // PSO for G-buffer rendering
    psoDesc.VS = vs_gbuff_bytecode;
    psoDesc.PS = ps_gbuff_bytecode;
    rtBlend.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    psoDesc.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
    psoDesc.NumRenderTargets = 2;  // second render target for material ID
    psoDesc.RTVFormats[1] = DXGI_FORMAT_R8_UINT;
    ThrowIfFailed(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_GbufferPipelineState)));

    // PSO for alpha blending
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[1] = DXGI_FORMAT_UNKNOWN;
    psoDesc.VS = vs_bytecode;
    psoDesc.PS = ps_bytecode;
    psoDesc.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;
    rtBlend.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL ^ D3D12_COLOR_WRITE_ENABLE_ALPHA;
    rtBlend.BlendEnable = TRUE;
    rtBlend.SrcBlend = D3D12_BLEND_SRC_ALPHA;
    rtBlend.DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
    rtBlend.BlendOp = D3D12_BLEND_OP_ADD;
    rtBlend.SrcBlendAlpha = D3D12_BLEND_ZERO;
    rtBlend.DestBlendAlpha = D3D12_BLEND_ONE;
    rtBlend.BlendOpAlpha = D3D12_BLEND_OP_ADD;
    ThrowIfFailed(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_alphaBlendingPipelineState)));

    // PSO for background rendering
    psoDesc.InputLayout = {nullptr, 0};
    psoDesc.VS = vs_background_bytecode;
    psoDesc.PS = ps_background_bytecode;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    rtBlend.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL ^ D3D12_COLOR_WRITE_ENABLE_ALPHA;
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

    m_rasterCB.RenderFrameMips = std::min(GPU_texture::CalculateMipCount(width, height), static_cast<unsigned int>(Kawase_blur_helper::BlurIterations));
    m_rasterCB.FrameSize = {width, height};

    resize_render_targets(width, height);

    m_viewport = CD3DX12_VIEWPORT{0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height)},
    m_scissorRect = CD3DX12_RECT{0, 0, static_cast<LONG>(width), static_cast<LONG>(height)},

    commandList->SetGraphicsRootSignature(m_rootSignature.Get());
    commandList->RSSetViewports(1, &m_viewport);
    commandList->RSSetScissorRects(1, &m_scissorRect);

    // Copy the updated scene constant buffer to GPU.
    memcpy(&m_mappedConstantData->constants, &m_rasterCB, sizeof(m_rasterCB));
    auto cbGpuAddress = m_perFrameConstants->GetGPUVirtualAddress();
    commandList->SetGraphicsRootConstantBufferView(GlobalRootSignatureParams::SceneConstantSlot, cbGpuAddress);
    commandList->SetGraphicsRootDescriptorTable(
        GlobalRootSignatureParams::MaterialsBufferSlot, gpu_model.materials_array.gpuHandle);
    commandList->SetGraphicsRootDescriptorTable(GlobalRootSignatureParams::DFGTex, DFG_lut.GetSRVHandle());
    commandList->SetGraphicsRootDescriptorTable(GlobalRootSignatureParams::DiffuseLutTex, Diffuse_lut.GetSRVHandle());
    commandList->SetGraphicsRootDescriptorTable(GlobalRootSignatureParams::SpecularLutTex, Specular_lut.GetSRVHandle());

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
    auto gbufferRTVHandle = m_gbuffer.GetRTVHandle();
    auto ID_RTVHandle = m_IDTexture.GetRTVHandle();
    D3D12_CPU_DESCRIPTOR_HANDLE GbuffRTVHandles[] = {gbufferRTVHandle, ID_RTVHandle};

    const float clearColor[] = {0.0f, 0.0f, 0.0f, 1.0f};
    commandList->OMSetRenderTargets(2, GbuffRTVHandles, FALSE, &depthHandle);
    commandList->ClearRenderTargetView(gbufferRTVHandle, clearColor, 0, nullptr);
    commandList->ClearRenderTargetView(ID_RTVHandle, clearColor, 0, nullptr);

    commandList->ClearDepthStencilView(depthHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
    commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    commandList->IASetVertexBuffers(0, 1, &vertexBufferView);
    commandList->IASetIndexBuffer(&indexBufferView);

    int num_opaque_objects;
    int num_transmissive_objects;
    int num_alpha_blended_objects;
    int offset = 0;
    sort_objects_for_rendering(gpu_model, num_opaque_objects, num_transmissive_objects, num_alpha_blended_objects);
    std::span<const DrawableSortingInfo> opaque_objects{
        m_sortedDrawables.begin(), m_sortedDrawables.begin() + num_opaque_objects};
    offset += num_opaque_objects;
    std::span<const DrawableSortingInfo> transmissive_objects{
        m_sortedDrawables.begin() + offset, m_sortedDrawables.begin() + offset + num_transmissive_objects};
    offset += num_transmissive_objects;
    std::span<const DrawableSortingInfo> blend_objects{m_sortedDrawables.begin() + offset, m_sortedDrawables.end()};

    // draw mesh lambda
    auto draw_object = [&](const DrawableSortingInfo element, bool UseAOTexture = false) {
        const auto& obj = *element.object;
        index_count = gpu_model.indicesSizes[obj.meshIndex];
        UINT baseIndexLocation = gpu_model.indicesOffsets[obj.meshIndex];
        RasterPerDrawData draw_data{obj.ModelMatrix, obj.NormalMatrix, obj.meshIndex, element.modelScale, UseAOTexture};
        constexpr int drawDataSizeInInt = sizeof(RasterPerDrawData) / 4;
        commandList->SetGraphicsRoot32BitConstants(GlobalRootSignatureParams::RootConstants, drawDataSizeInInt, &draw_data, 0);
        commandList->DrawIndexedInstanced(index_count, 1, baseIndexLocation, 0, 0);
     };

    bool DrawAOMode = (RaytracingMode == RayProgramMode::AmbientOcclusion);
    bool UseGTAO = (m_rasterCB.GTAOStrength != 0.0f) && (RaytracingMode != RayProgramMode::RayCaster) || DrawAOMode;

    bool UseSSR = m_rasterCB.SSREnabled;

    if (UseGTAO || UseSSR) {
        // Draw opaque objects into Gbuffer
        commandList->SetPipelineState(m_GbufferPipelineState.Get());
        for (const auto& element : opaque_objects) {
            draw_object(element);
        }

        if (!transmissive_objects.empty()) {
            GPU_texture::copy_texture(m_depthTexture_opaque_only, m_depthTexture, D3D12_RESOURCE_STATE_DEPTH_WRITE,
                D3D12_RESOURCE_STATE_DEPTH_WRITE, commandList);
        }

        for (const auto& element : transmissive_objects) {
            draw_object(element);
        }
        if (DrawAOMode) {  // draw all other object as well to match DXR AO mode
            for (const auto& element : blend_objects) {
                draw_object(element);
            }
        }
        // Copy depth buffer to a UAV texture and generate mips for hierarchical depth
        GPU_texture::copy_texture_mip0_only(m_DepthUAVTexture, m_depthTexture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
            D3D12_RESOURCE_STATE_DEPTH_WRITE, commandList);

        static Mipmaps_helper mip_helper{};
        mip_helper.Init();
        mip_helper.CreateMips(m_DepthUAVTexture, false, true);

        ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()};  // todo: don't replace heap during mip map pass?
        commandList->SetDescriptorHeaps(1, desc_heap);
    }

    if (UseGTAO) {
        m_GTAO_helper.CreateAO(m_AOTexture, m_gbuffer, m_DepthUAVTexture, m_DepthUAVTexture_previous, 
            m_rasterCB.Projection, m_rasterCB.invProjection, m_rasterCB.viewProjection, m_rasterCB.frameID);
    }

    if (UseSSR) {
        m_SSR_helper.CalculateSSR(m_SSR, m_gbuffer, m_DepthUAVTexture, m_DepthUAVTexture_previous, m_renderTarget_previous,
            m_rasterCB.Projection, m_rasterCB.invProjection, m_rasterCB.viewProjection, m_rasterCB.frameID);
    }

    if (UseGTAO || UseSSR) {
        // Swap depth UAV textures for next frame
        std::swap(m_DepthUAVTexture, m_DepthUAVTexture_previous);
    }

    if (DrawAOMode) {
        GPU_texture::copy_texture(m_renderTarget, m_AOTexture, D3D12_RESOURCE_STATE_RENDER_TARGET,
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS, commandList);
        copy_render_target_to_framebuffer(framebuffer);
        return;
    }

    commandList->SetGraphicsRootDescriptorTable(GlobalRootSignatureParams::AmbientOcclusionTex, m_AOTexture.GetSRVHandle());
    commandList->SetGraphicsRootDescriptorTable(GlobalRootSignatureParams::SSRTex, m_SSR.GetSRVHandle());
    commandList->SetGraphicsRootDescriptorTable(GlobalRootSignatureParams::MaterialIDTex, m_IDTexture.GetSRVHandle());

    if ((UseGTAO || UseSSR) && !transmissive_objects.empty()) {
        depthHandle = m_depthTexture_opaque_only.GetDSVHandle();
    }

    // Draw opaque objects
    commandList->SetPipelineState(m_pipelineState.Get());
    commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &depthHandle);
    commandList->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);
    for (const auto& element : opaque_objects) {
        draw_object(element, true);
    }
    // Draw background
    commandList->SetPipelineState(m_backgroundPipelineState.Get());
    commandList->DrawInstanced(3, 1, 0, 0);

    if (!transmissive_objects.empty()) {
        GPU_texture::copy_texture_mip0_only(m_frame_opaque_only, m_renderTarget,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_RENDER_TARGET, commandList);
    }

    if ((UseGTAO || UseSSR) && !transmissive_objects.empty()) {
        depthHandle = m_depthTexture.GetDSVHandle();
        commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &depthHandle);
    }

    // Draw transmissive objects with rendered image so far as background
    if (!transmissive_objects.empty()) {
        // set background alpha to zero (for refractions)
        static Silhouette_helper silhouette_helper{};
        silhouette_helper.Apply(m_frame_opaque_only, m_depthTexture_opaque_only);
        // Apply Kawase Blur
        m_blur_helper.CreateBlurMips(m_frame_opaque_only);

        ID3D12DescriptorHeap* desc_heap[] = {d3d_ctx.m_SrvDescHeap.Get()}; // todo: don't replace heap during blur pass?
        commandList->SetDescriptorHeaps(1, desc_heap);

        const auto barrier_uav_to_srv = CD3DX12_RESOURCE_BARRIER::Transition(m_frame_opaque_only.get_gpu_resource().Get(),
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        commandList->ResourceBarrier(1, &barrier_uav_to_srv);

        commandList->SetGraphicsRootDescriptorTable(GlobalRootSignatureParams::FrameTex, m_frame_opaque_only.GetSRVHandle());
        commandList->SetPipelineState(m_pipelineState.Get());
        for (const auto& element : transmissive_objects) {
            draw_object(element, true);
        }

        const auto barrier_srv_to_uav = CD3DX12_RESOURCE_BARRIER::Transition(m_frame_opaque_only.get_gpu_resource().Get(),
            D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
        commandList->ResourceBarrier(1, &barrier_srv_to_uav);
    }

    // Draw objects with alpha blending
    commandList->SetPipelineState(m_alphaBlendingPipelineState.Get());
    for (const auto& element : blend_objects) {
        draw_object(element);
    }

    if (DrawSSROnly) {
        copy_ssr_to_framebuffer(framebuffer);
    } else {
        copy_render_target_to_framebuffer(framebuffer);
    }

    std::swap(m_renderTarget, m_renderTarget_previous);
}

void Raster_pipeline::copy_render_target_to_framebuffer(const CPUFrameBuffer& framebuffer) {
    D3DContext& d3d_ctx = D3DContext::Get();
    const auto& commandList = d3d_ctx.m_DXRCommandList;
    const auto& gpuDestTarget = framebuffer.get_gpu_resource();
    const auto& gpuRenderTarget = m_renderTarget.get_gpu_resource();

    GPU_texture::copy_texture(gpuDestTarget, gpuRenderTarget, 
        D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET, commandList);
}

void Raster_pipeline::copy_ssr_to_framebuffer(const CPUFrameBuffer& framebuffer) {
    D3DContext& d3d_ctx = D3DContext::Get();
    const auto& commandList = d3d_ctx.m_DXRCommandList;
    const auto& gpuDestTarget = framebuffer.get_gpu_resource();
    const auto& gpuSrcTarget = m_SSR.get_gpu_resource();

    GPU_texture::copy_texture(gpuDestTarget, gpuSrcTarget, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, commandList);
}

void Raster_pipeline::resize_render_targets(int new_width, int new_height) {
    if (currentWidth == new_width && currentHeight == new_height) return;
    currentWidth = std::max(new_width, 0);
    currentHeight = std::max(new_height, 0);

    m_depthTexture.release_gpu_resource();
    TEXTURE_TRAITS flags = TEXTURE_TRAITS::DepthWithSRV;
    m_depthTexture = GPU_texture{currentWidth, currentHeight, flags};

    m_depthTexture_opaque_only.release_gpu_resource();
    flags = TEXTURE_TRAITS::DepthWithSRV;
    m_depthTexture_opaque_only = GPU_texture{currentWidth, currentHeight, flags};

    m_DepthUAVTexture.release_gpu_resource();
    m_DepthUAVTexture_previous.release_gpu_resource();
    flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV | TEXTURE_TRAITS::AllocateMips;
    m_DepthUAVTexture = GPU_texture{currentWidth, currentHeight, flags, DXGI_FORMAT_R32_FLOAT};
    m_DepthUAVTexture_previous = GPU_texture{currentWidth, currentHeight, flags, DXGI_FORMAT_R32_FLOAT};

    m_renderTarget.release_gpu_resource();
    flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::RenderTarget;
    m_renderTarget = GPU_texture{currentWidth, currentHeight, flags};

    m_renderTarget_previous.release_gpu_resource();
    flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::RenderTarget;
    m_renderTarget_previous = GPU_texture{currentWidth, currentHeight, flags};

    m_frame_opaque_only.release_gpu_resource();
    flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV | TEXTURE_TRAITS::AllocateMips;
    m_frame_opaque_only = GPU_texture{currentWidth, currentHeight, flags};

    m_gbuffer.release_gpu_resource();
    flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::RenderTarget;
    m_gbuffer = GPU_texture{currentWidth, currentHeight, flags};

    m_IDTexture.release_gpu_resource();
    flags = TEXTURE_TRAITS::RenderTarget;
    m_IDTexture = GPU_texture{currentWidth, currentHeight, flags, DXGI_FORMAT_R8_UINT};

    m_AOTexture.release_gpu_resource();
    flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV;
    m_AOTexture = GPU_texture{currentWidth, currentHeight, flags};

    m_SSR.release_gpu_resource();
    flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::UAV;
    m_SSR = GPU_texture{currentWidth, currentHeight, flags};
}

void Raster_pipeline::sort_objects_for_rendering(
    const GPU_model& gpu_model, int& num_opaque_objects, int& num_transmissive_objects, int& num_alpha_blended_objects) {
    num_opaque_objects = 0;
    num_transmissive_objects = 0;
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
        bool transmittance = mat.hasVolume || (mat.transmisionFactor != 0.0f);

        glm::fvec3 scale;
        scale.x = glm::length(glm::fvec3(obj.ModelMatrix[0]));
        scale.y = glm::length(glm::fvec3(obj.ModelMatrix[1]));
        scale.z = glm::length(glm::fvec3(obj.ModelMatrix[2]));
        float model_scale = max(max(scale.x,scale.y),scale.z);

        if (alphaBlending) {
            num_alpha_blended_objects++;
        } else if (transmittance) {
            num_transmissive_objects++;
        }
        else {
            num_opaque_objects++;
        }

        m_sortedDrawables.emplace_back(&obj, model_scale, ZDistanceToCamera, alphaBlending, transmittance);
    }
    std::sort(m_sortedDrawables.begin(), m_sortedDrawables.end(), [](const DrawableSortingInfo& a, const DrawableSortingInfo& b) {
        if (a.alphaBlending != b.alphaBlending) {
            return !a.alphaBlending;  // opaque first
        }
        if (a.transmittance != b.transmittance) {
            return !a.transmittance;  // without transmittance first
        }
        if (!a.alphaBlending) {
            return a.ZDistanceToCamera < b.ZDistanceToCamera;  // front to back for opaque objects
        } else {
            return a.ZDistanceToCamera > b.ZDistanceToCamera;  // back to front for alpha blended objects
        }
    });
}

void Raster_pipeline::ComputeDFGLut() {
    auto start = std::chrono::high_resolution_clock::now();
    D3DContext& d3d_ctx = D3DContext::Get();
    DFG_lut.release_gpu_resource();
    DFG_lut = DFG_Lut_helper::GetBlankSRVTexture();

    DFG_Lut_helper DFG_compute_helper{};
    DFG_compute_helper.CreateDFG_Lut();
    
    // copy texture from UAV to SRV-only texture
    GPU_texture DFG_lut_tmp = std::move(DFG_compute_helper.GetDFG_Lut());
    GPU_texture::copy_texture(DFG_lut, DFG_lut_tmp, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, d3d_ctx.m_DXRCommandList);

    d3d_ctx.DispatchDXRCommandList(); // WARNING: this assumes that command list is open and ready at this point
    d3d_ctx.WaitForPendingDXR();

    d3d_ctx.InitDXRCommandList();

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "DFG Lut computed in " << diff.count() << " ms." << '\n';
}

void Raster_pipeline::ComputeEnvmapLut(const GPU_texture& envmap) {
    auto start = std::chrono::high_resolution_clock::now();

    Diffuse_lut.release_gpu_resource();
    Specular_lut.release_gpu_resource();

    Diffuse_lut = EnvCube_helper::GetBlankSRVDiffuseTexture();
    Specular_lut = EnvCube_helper::GetBlankSRVSpecularTexture();

    D3DContext& d3d_ctx = D3DContext::Get();
    d3d_ctx.InitDXRCommandList();
    
    static EnvCube_helper EnvCube_helper{};
    EnvCube_helper.CreateDiffuseEnvmapCube(envmap);
    EnvCube_helper.CreateSpecularEnvmapCube(envmap);

    GPU_texture Diffuse_lut_tmp = std::move(EnvCube_helper.GetDiffuseEnvmapCube());
    GPU_texture Specular_lut_tmp = std::move(EnvCube_helper.GetSpecularEnvmapCube());
    // copy textures from UAV to SRV-only textures
    GPU_texture::copy_texture(Diffuse_lut, Diffuse_lut_tmp, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, d3d_ctx.m_DXRCommandList);
    GPU_texture::copy_texture(Specular_lut, Specular_lut_tmp, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, d3d_ctx.m_DXRCommandList);

    d3d_ctx.DispatchDXRCommandList();
    d3d_ctx.WaitForPendingDXR();

    EnvCube_helper.ReleaseTemporaryGPUResources();

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "Diffuse and Specular Lut computed in " << diff.count() << " ms." << '\n';
}

void Raster_pipeline::ComputeMipMaps(GPU_texture& texture) { 
    if (texture.mipLevels <= 1) {
        return;
    }
    GPU_texture uav_texture = Mipmaps_helper::GetBlankCompatibleUAVTex(texture);
    static Mipmaps_helper mip_helper{};

    D3DContext& d3d_ctx = D3DContext::Get();
    d3d_ctx.InitDXRCommandList();
    mip_helper.Init();

    GPU_texture::copy_texture_mip0_only(uav_texture, texture, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
        D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, d3d_ctx.m_DXRCommandList);

    mip_helper.CreateMips(uav_texture);
    
    GPU_texture::copy_texture(texture, uav_texture, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, d3d_ctx.m_DXRCommandList);
    
    d3d_ctx.DispatchDXRCommandList();
    d3d_ctx.WaitForPendingDXR();
}

}