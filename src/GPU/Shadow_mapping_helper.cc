#include "Shadow_mapping_helper.h"

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace GlobalRootSignatureParams {
enum Value : int {
    SceneConstantSlot = 0,
    MaterialsBufferSlot,
    RootConstants,

    Count
};
}

namespace RenderShaders {
enum Value : unsigned int {
    Shadow = 0,

    Count
};
}

namespace {

using namespace app;

int find_texture_id(const GPU_texture& texture) {
    D3DContext& d3d_ctx = D3DContext::Get();
    const auto& heap_alloc = d3d_ctx.m_SrvDescHeapAlloc;

    return heap_alloc.GetIndex(texture.GetSRVHandle());
}

void calculate_shadow_mapping_info(const GPU_Light& light, const BBox& sceneBBox, const GPU_texture& ShadowMap, UINT ShadowSize,
    float BiasMultiplier, ShadowMappingInfo& out_info, float& out_far_plane) 
{
    fastgltf::LightType type = static_cast<fastgltf::LightType>(light.direction_type.w);
    fmat4x4 viewMat{};
    fmat4x4 projectionMat{};

    auto scene_center = (sceneBBox.min + sceneBBox.max) * 0.5f;
    auto dim = sceneBBox.max - sceneBBox.min;
    auto radius = glm::length(dim * 0.5f);
    auto direction = xyz(-light.direction_type);
    auto up = (glm::abs(direction.y) < 0.999f) ? fvec3{0, 1, 0} : fvec3{1, 0, 0};
    float texel_size = 1.0f / static_cast<float>(ShadowSize);
    if (type == fastgltf::LightType::Directional) {
        auto origin = scene_center - direction * radius;
        viewMat = glm::lookAt(scene_center - direction * radius, scene_center, up);
        float near_plane = 0.0f;
        float far_plane = 2.0f * radius;
        projectionMat = glm::orthoRH_ZO(-radius, radius, -radius, radius, near_plane, far_plane);
        texel_size *= 2.0f * radius;
        out_info.origin = xyz1(origin);
        out_info.direction = xyz0(direction);
        out_far_plane = far_plane;
    } else if (type == fastgltf::LightType::Spot) {
        auto eye = xyz(light.position_range);
        viewMat = glm::lookAt(eye, eye + direction, up);
        auto range = light.position_range.w;
        float far_plane;
        if (range > 0) {
            far_plane = std::min(radius, range);
        } else {
            far_plane = 2.0f * radius;
        }
        float near_plane = std::max(0.001f * far_plane, 0.01f);
        constexpr auto angle_limit = 3.1415926f * 80.0f / 180.f; // 80 degrees in radians
        float yfov = 2.0f * glm::min(std::acos(light.inner_outerConeAngleCos.y), angle_limit);
        projectionMat = glm::perspectiveRH_ZO(yfov, 1.0f, near_plane, far_plane);
        texel_size *= 2.0f * std::tan(yfov * 0.5f);
        out_info.origin = xyz1(eye);
        out_info.direction = xyz0(direction);
        out_far_plane = far_plane;
    } else if (type == fastgltf::LightType::Point) {
        // TODO: think of the way to point light shadows
    }

    out_info.viewProjection = projectionMat * viewMat;
    out_info.shadowMapTextureIndex_type = glm::ivec4(find_texture_id(ShadowMap), type, 0, 0);
    out_info.texel_size_padding = fvec2{texel_size * BiasMultiplier, 0};
}

}

namespace app {

using namespace glm;

struct ShadowConstantBuffer {
    fmat4x4 viewProjection;
    fvec4 lightDirection;
    fvec4 lightPosition;
    int isPointLight;
};

struct ShadowPerDrawData {
    fmat3x4 modelMatrix;
    int meshID;
};

ComPtr<ID3D12RootSignature> Shadow_mapping_helper::m_rootSignature{};
PSO_helper Shadow_mapping_helper::m_PSO_helper{};

Shadow_mapping_helper::Shadow_mapping_helper() {
    if (!m_rootSignature || !m_PSO_helper.is_initialized()) {
        Reload();
    }
}

void Shadow_mapping_helper::Reload() {
    CreateRootSignature();
    CreatePipelineStateObject();
}

Shadow_mapping_helper::~Shadow_mapping_helper() { release_gpu_resources(); }

void Shadow_mapping_helper::resize_shadow_maps(int new_size, int new_lights_count) {
    if (ShadowSize == new_size && m_shadowMaps.size() == new_lights_count) return;
    ShadowSize = std::max(new_size, 0);

    m_shadowMaps.clear();
    m_shadowMaps.reserve(new_lights_count);
    m_shadowMappingInfo.clear();
    m_shadowMappingInfo.resize(new_lights_count);

    TEXTURE_TRAITS flags = TEXTURE_TRAITS::HDR | TEXTURE_TRAITS::RenderTarget;

    for (int i = 0; i < new_lights_count; ++i) {
        m_shadowMaps.emplace_back(ShadowSize, ShadowSize, flags, 
            DXGI_FORMAT_R32G32B32A32_FLOAT, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    }

    m_depthTexture.release_gpu_resource();
    flags = TEXTURE_TRAITS::Depth;
    m_depthTexture = GPU_texture{ShadowSize, ShadowSize, flags};

    prepare_shadow_mapping_info_array_buffer();
}

const std::vector<ShadowMappingInfo>& Shadow_mapping_helper::GetShadowMappingInfo() const { return m_shadowMappingInfo; }

D3D12_GPU_DESCRIPTOR_HANDLE Shadow_mapping_helper::GetShadowMappingInfoGPUHandle() const {
    return shadow_mapping_info_array.gpuHandle;
}

void Shadow_mapping_helper::prepare_shadow_mapping_info_array_buffer() {
    D3DContext& d3d_ctx = D3DContext::Get();
    const auto& heap_alloc = d3d_ctx.m_SrvDescHeapAlloc;

    shadowMappingInfoArray.Reset();
    shadow_uploadBuffer.Reset();

    uint32_t lightCount = max(m_shadowMappingInfo.size(), size_t(1));
    const UINT BufferSize = sizeof(ShadowMappingInfo) * lightCount;

    const D3D12_HEAP_PROPERTIES def_props{
        .Type = D3D12_HEAP_TYPE_DEFAULT,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        .CreationNodeMask = 1,
        .VisibleNodeMask = 1,
    };

    const D3D12_RESOURCE_DESC upload_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = BufferSize,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE,
    };

    ThrowIfFailed(d3d_ctx.m_d3dDevice->CreateCommittedResource(
        &def_props, D3D12_HEAP_FLAG_NONE, &upload_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&shadowMappingInfoArray)));
    CreateBufferSRV(d3d_ctx, shadowMappingInfoArray, lightCount, sizeof(ShadowMappingInfo), shadow_mapping_info_array);

    const D3D12_HEAP_PROPERTIES upload_props{
        .Type = D3D12_HEAP_TYPE_UPLOAD,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        .CreationNodeMask = 1,
        .VisibleNodeMask = 1,
    };

    ThrowIfFailed(d3d_ctx.m_d3dDevice->CreateCommittedResource(&upload_props, D3D12_HEAP_FLAG_NONE, &upload_desc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&shadow_uploadBuffer)));
}

void Shadow_mapping_helper::update_shadow_mapping_info_array_buffer() {
    uint32_t lightCount = m_shadowMappingInfo.size();

    if (lightCount == 0) return;

    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    const UINT BufferSize = sizeof(ShadowMappingInfo) * lightCount;

    void* pDataBegin;
    ThrowIfFailed(shadow_uploadBuffer->Map(0, nullptr, &pDataBegin));
    memcpy(pDataBegin, m_shadowMappingInfo.data(), BufferSize);
    shadow_uploadBuffer->Unmap(0, nullptr);

    commandList->CopyBufferRegion(shadowMappingInfoArray.Get(), 0, shadow_uploadBuffer.Get(), 0, BufferSize);
}

void Shadow_mapping_helper::CreateShadowMaps(const GPU_model& gpu_model, UINT size) {
    resize_shadow_maps(size, gpu_model.get_lights_count());

    int shadow_map_count = static_cast<int>(m_shadowMaps.size());
    int light_count = gpu_model.get_lights_count();
    const auto& lights = gpu_model.get_lights_cpu_array();

    for (int i=0; i < shadow_map_count && i < light_count; ++i) {
        const auto& Light = lights[i];
        const auto& ShadowMap = m_shadowMaps[i];
        
        auto& info = m_shadowMappingInfo[i];
        float far_plane = 0.0f;
        calculate_shadow_mapping_info(Light, gpu_model.scene_bounds, ShadowMap, ShadowSize, BiasMultiplier, info, far_plane);
        CreateShadowMap(gpu_model, info, ShadowMap, size, far_plane);
    }

    update_shadow_mapping_info_array_buffer();
}

void Shadow_mapping_helper::CreateShadowMap(const GPU_model& gpu_model,
    const ShadowMappingInfo& shadow, const GPU_texture& shadowMap, UINT size, float far_plane) {
    D3DContext& d3d_ctx = D3DContext::Get();
    auto commandList = d3d_ctx.m_DXRCommandList;

    auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
        shadowMap.get_gpu_resource().Get(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
    commandList->ResourceBarrier(1, &barrier);

    const auto m_viewport = CD3DX12_VIEWPORT{0.0f, 0.0f, static_cast<float>(ShadowSize), static_cast<float>(ShadowSize)};
    const auto m_scissorRect = CD3DX12_RECT{0, 0, static_cast<LONG>(ShadowSize), static_cast<LONG>(ShadowSize)};

    commandList->SetGraphicsRootSignature(m_rootSignature.Get());
    commandList->RSSetViewports(1, &m_viewport);
    commandList->RSSetScissorRects(1, &m_scissorRect);
 
    ShadowConstantBuffer shadow_scene_data{
        .viewProjection = shadow.viewProjection,
        .lightDirection = shadow.direction,
        .lightPosition = shadow.origin,
        .isPointLight = (shadow.shadowMapTextureIndex_type.y != int(fastgltf::LightType::Directional)),
    };
    constexpr int ShadowSceneDataSizeInInt = sizeof(ShadowConstantBuffer) / 4;
    commandList->SetGraphicsRoot32BitConstants(
        GlobalRootSignatureParams::SceneConstantSlot, ShadowSceneDataSizeInInt, &shadow_scene_data, 0);
    commandList->SetGraphicsRootDescriptorTable(
        GlobalRootSignatureParams::MaterialsBufferSlot, gpu_model.materials_array.gpuHandle);

    const auto& combined_mesh = gpu_model.get_combined_mesh();

    UINT vertex_count = combined_mesh.get_vertex_count();
    UINT index_count = combined_mesh.get_index_count();

    D3D12_VERTEX_BUFFER_VIEW vertexBufferView{};
    vertexBufferView.BufferLocation = combined_mesh.vertexBuffer->GetGPUVirtualAddress();
    vertexBufferView.StrideInBytes = sizeof(vertex);
    vertexBufferView.SizeInBytes = sizeof(vertex) * vertex_count;

    D3D12_INDEX_BUFFER_VIEW indexBufferView{};
    indexBufferView.BufferLocation = combined_mesh.indexBuffer->GetGPUVirtualAddress();
    indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    indexBufferView.SizeInBytes = sizeof(uint32_t) * index_count;

    auto rtvHandle = shadowMap.GetRTVHandle();
    auto depthHandle = m_depthTexture.GetDSVHandle();

    const float clearColor[] = {far_plane, far_plane * far_plane, 0.0f, 0.0f};

    commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &depthHandle);
    commandList->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);
    commandList->ClearDepthStencilView(depthHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
    commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    commandList->IASetVertexBuffers(0, 1, &vertexBufferView);
    commandList->IASetIndexBuffer(&indexBufferView);

    int num_opaque_objects;
    int num_transmissive_objects;
    int num_alpha_blended_objects;
    int offset = 0;
    fvec3 light_forward = -xyz(shadow_scene_data.lightDirection);
    sort_objects_for_rendering(
        m_sortedDrawables, 
        light_forward,
        xyz(shadow_scene_data.lightPosition),
        gpu_model, num_opaque_objects, num_transmissive_objects, num_alpha_blended_objects
    );
    std::span<const DrawableSortingInfo> opaque_objects{
        m_sortedDrawables.begin(), m_sortedDrawables.begin() + num_opaque_objects};
    offset += num_opaque_objects;
    std::span<const DrawableSortingInfo> transmissive_objects{
        m_sortedDrawables.begin() + offset, m_sortedDrawables.begin() + offset + num_transmissive_objects};
    offset += num_transmissive_objects;
    std::span<const DrawableSortingInfo> blend_objects{m_sortedDrawables.begin() + offset, m_sortedDrawables.end()};

    // draw mesh lambda
    auto draw_object = [&](const DrawableSortingInfo element, unsigned int original_PSO_index, bool UseAOTexture = false) {
        const auto& obj = *element.object;
        index_count = gpu_model.indicesSizes[obj.meshIndex];
        UINT baseIndexLocation = gpu_model.indicesOffsets[obj.meshIndex];
        ShadowPerDrawData draw_data{ obj.ModelMatrix, obj.meshIndex };
        constexpr int drawDataSizeInInt = sizeof(ShadowPerDrawData) / 4;
        commandList->SetGraphicsRoot32BitConstants(GlobalRootSignatureParams::RootConstants, drawDataSizeInInt, &draw_data, 0);
        bool useReverseWinding = (glm::determinant(glm::mat3(obj.ModelMatrix)) < 0.0f);
        PSO_variant_key key{
            .original_PSO_index = original_PSO_index,
            .backface_culling = element.backfaceCulling,
            .depth_clipping = true,
            .reverse_winding = useReverseWinding,
            .alpha_blending = element.alphaBlending,
        };
        m_PSO_helper.set_pipeline_state(commandList, key);
        commandList->DrawIndexedInstanced(index_count, 1, baseIndexLocation, 0, 0);
    };

    m_PSO_helper.invalidate_last_used_key();  // ensure PSO is set on first draw call

    // Draw opaque objects
    for (const auto& element : opaque_objects) {
        draw_object(element, RenderShaders::Shadow);
    }
    // Draw transmissive objects
    for (const auto& element : transmissive_objects) {
        draw_object(element, RenderShaders::Shadow);
    }

    barrier = CD3DX12_RESOURCE_BARRIER::Transition(
        shadowMap.get_gpu_resource().Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
    commandList->ResourceBarrier(1, &barrier);
}

void Shadow_mapping_helper::CreateRootSignature() {
    // Create an empty root signature.
    CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDesc;

    CD3DX12_DESCRIPTOR_RANGE ranges[GlobalRootSignatureParams::Count];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0);     // 1 scene constants buffer.
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0);     // 2 materials buffer.
    ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 1);     // 3 per draw constants buffer.

    D3D12_STATIC_SAMPLER_DESC default_sampler = {};  // Default static sampler.
    default_sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
    default_sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    default_sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    default_sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
    default_sampler.MaxLOD = D3D12_FLOAT32_MAX;
    default_sampler.ShaderRegister = 0;  // s0
    default_sampler.RegisterSpace = 0;
    default_sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    D3D12_STATIC_SAMPLER_DESC anisotropic_sampler = default_sampler;  // Anisotropic static sampler.
    anisotropic_sampler.MaxAnisotropy = 8;
    anisotropic_sampler.Filter = D3D12_FILTER_ANISOTROPIC;
    anisotropic_sampler.ShaderRegister = 1;

    D3D12_STATIC_SAMPLER_DESC samplers[] = {default_sampler, anisotropic_sampler};

    CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
    rootParameters[GlobalRootSignatureParams::SceneConstantSlot].InitAsConstants(sizeof(ShadowConstantBuffer) / 4, 0);
    rootParameters[GlobalRootSignatureParams::MaterialsBufferSlot].InitAsDescriptorTable(1, &ranges[1]);
    rootParameters[GlobalRootSignatureParams::RootConstants].InitAsConstants(sizeof(ShadowPerDrawData) / 4, 1);

    auto flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT |
                 D3D12_ROOT_SIGNATURE_FLAG_CBV_SRV_UAV_HEAP_DIRECTLY_INDEXED;
    rootSignatureDesc.Init(ARRAYSIZE(rootParameters), rootParameters, 2, samplers, flags);

    SerializeAndCreateRootSignature(rootSignatureDesc, &m_rootSignature);
}

void Shadow_mapping_helper::CreatePipelineStateObject() {
    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"NORMAL", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 16, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TANGENT", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 16 * 2, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 16 * 3, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0}};

    auto vs_file_names = std::array<const wchar_t*, RenderShaders::Count>{
        c_vs_file_name};
    auto ps_file_names = std::array<const wchar_t*, RenderShaders::Count>{
        c_ps_file_name};
    auto use_input_layout = std::array<bool, RenderShaders::Count>{true};
    auto rt_write_mask = std::array<UINT8, RenderShaders::Count>{
        D3D12_COLOR_WRITE_ENABLE_ALL
    };
    auto depth_write_mask = std::array<D3D12_DEPTH_WRITE_MASK, RenderShaders::Count>{
        D3D12_DEPTH_WRITE_MASK_ALL};
    auto rt_num = std::array<int, RenderShaders::Count>{1};

    bool success = m_PSO_helper.init(RenderShaders::Count, vs_file_names, ps_file_names, m_rootSignature, inputElementDescs,
        use_input_layout, rt_write_mask, depth_write_mask, rt_num, true);

    if (!success) {
        throw std::runtime_error("Failed to initialize PSO helper");
    }

    // Preallocate most used PSO variants
    PSO_variant_key key{
        .original_PSO_index = RenderShaders::Shadow,
        .backface_culling = true,
        .depth_clipping = true,
        .reverse_winding = false,
        .alpha_blending = false,
    };
    m_PSO_helper.preallocate_pipeline_state(key);
}

void Shadow_mapping_helper::release_gpu_resources() {
    D3DContext& d3d_ctx = D3DContext::Get();
    if (d3d_ctx.m_CommandQueue != nullptr) {
        d3d_ctx.WaitForPendingOperations();
    }

    m_rootSignature.Reset();
    m_PSO_helper.release_gpu_resources();
    m_depthTexture.release_gpu_resource();
    shadowMappingInfoArray.Reset();
    shadow_uploadBuffer.Reset();

    d3d_ctx.m_SrvDescHeapAlloc.Free(shadow_mapping_info_array.cpuHandle, shadow_mapping_info_array.gpuHandle);
}

}  // namespace app