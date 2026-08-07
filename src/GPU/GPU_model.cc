#include "GPU_model.h"
#include "GPU_model.h"

#include "../d3d_context.h"

#include <bit>
#include <directx/d3dx12.h>

namespace {

using namespace app;
// Create SRV description for a buffer.
D3D12_SHADER_RESOURCE_VIEW_DESC CreateSRVDescription(UINT numElements, UINT elementSize) {
    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Buffer.NumElements = numElements;
    if (elementSize == 0) {
        srvDesc.Format = DXGI_FORMAT_R32_TYPELESS;
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_RAW;
        srvDesc.Buffer.StructureByteStride = 0;
    } else {
        srvDesc.Format = DXGI_FORMAT_UNKNOWN;
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_NONE;
        srvDesc.Buffer.StructureByteStride = elementSize;
    }
    return srvDesc;
}

void CreateBufferSRV(D3DContext& d3d_ctx, const ComPtr<ID3D12Resource>& buffer, UINT numElements, UINT elementSize,
    D3D_Handle_Pair& handles) {
    d3d_ctx.m_SrvDescHeapAlloc.Alloc(&handles.cpuHandle, &handles.gpuHandle);
    const auto srvDesc = CreateSRVDescription(numElements, elementSize);
    d3d_ctx.m_d3dDevice->CreateShaderResourceView(buffer.Get(), &srvDesc, handles.cpuHandle);
}

bool isTrue(TEXTURE_TRAITS a) { return a != TEXTURE_TRAITS::None; }
bool isHDR(TEXTURE_TRAITS a) { return (a & TEXTURE_TRAITS::HDR) != TEXTURE_TRAITS::None; }
bool isSRGB(TEXTURE_TRAITS a) { return (a & TEXTURE_TRAITS::sRGB) != TEXTURE_TRAITS::None; }
bool isCubemap(TEXTURE_TRAITS a) { return (a & TEXTURE_TRAITS::Cubemap) != TEXTURE_TRAITS::None; }
bool isRenderTarget(TEXTURE_TRAITS a) { return (a & TEXTURE_TRAITS::RenderTarget) != TEXTURE_TRAITS::None; }
bool isDepth(TEXTURE_TRAITS a) { return (a & (TEXTURE_TRAITS::Depth | TEXTURE_TRAITS::DepthWithSRV)) != TEXTURE_TRAITS::None; }
bool isDepthWithSRV(TEXTURE_TRAITS a) { return (a & TEXTURE_TRAITS::DepthWithSRV) != TEXTURE_TRAITS::None; }
bool isUAV(TEXTURE_TRAITS a) { return (a & TEXTURE_TRAITS::UAV) != TEXTURE_TRAITS::None; }
bool AllocateMips(TEXTURE_TRAITS a) { return (a & TEXTURE_TRAITS::AllocateMips) != TEXTURE_TRAITS::None; }

}  // namespace

namespace app {

using namespace glm;

inline void ThrowIfFailed(HRESULT hr) {
    if (FAILED(hr)) {
        throw std::exception();
    }
}

GPU_mesh::GPU_mesh(const Mesh& cpu_mesh, bool positions_only) : positionsOnly(positions_only) {
    D3DContext& d3d_ctx = D3DContext::Get();

    d3d_ctx.InitCopyCommandList();

    ComPtr<ID3D12Resource2> uploadBuffer;
    ComPtr<ID3D12Resource2> index_uploadBuffer;

    vertexCount = cpu_mesh.vertices.size();
    indexCount = cpu_mesh.indices.size();

    const UINT vertexBufferSize = (positionsOnly ? sizeof(vertex::position) : sizeof(vertex)) * vertexCount;
    const UINT indexBufferSize = sizeof(uint32_t) * indexCount;

    const D3D12_HEAP_PROPERTIES def_props{
        .Type = D3D12_HEAP_TYPE_DEFAULT,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        .CreationNodeMask = 1,
        .VisibleNodeMask = 1,
    };

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
        .Width = vertexBufferSize,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE,
    };
    D3D12_RESOURCE_DESC index_upload_desc = upload_desc;
    index_upload_desc.Width = indexBufferSize;

    // vertices
    ThrowIfFailed(
        d3d_ctx.m_d3dDevice->CreateCommittedResource(
            &upload_props, 
            D3D12_HEAP_FLAG_NONE, 
            &upload_desc, 
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&uploadBuffer)
        )
    );
    ThrowIfFailed(
        d3d_ctx.m_d3dDevice->CreateCommittedResource(
            &def_props, 
            D3D12_HEAP_FLAG_NONE, 
            &upload_desc,
            D3D12_RESOURCE_STATE_COMMON, 
            nullptr, 
            IID_PPV_ARGS(&vertexBuffer)
        )
    );
    // indices
    ThrowIfFailed(
        d3d_ctx.m_d3dDevice->CreateCommittedResource(
            &upload_props, 
            D3D12_HEAP_FLAG_NONE, 
            &index_upload_desc, 
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&index_uploadBuffer)
        )
    );
    ThrowIfFailed(
        d3d_ctx.m_d3dDevice->CreateCommittedResource(
            &def_props, 
            D3D12_HEAP_FLAG_NONE, 
            &index_upload_desc,
            D3D12_RESOURCE_STATE_COMMON, 
            nullptr, 
            IID_PPV_ARGS(&indexBuffer)
        )
    );

    // Copy the triangle data to the vertex buffer.
    void* pVertexDataBegin;
    ThrowIfFailed(uploadBuffer->Map(0, nullptr, &pVertexDataBegin));
    if (positionsOnly) {
        std::vector<fvec3> tmp_positions;
        static_assert(sizeof(vertex::position) == sizeof(fvec3) && sizeof(fvec3) == sizeof(fvec4),
            "vertex::position should be the same as fvec3 AND fvec4");
        tmp_positions.reserve(vertexCount);
        std::transform(cpu_mesh.vertices.begin(), cpu_mesh.vertices.end(), std::back_inserter(tmp_positions),
            [](const vertex& v) { return v.position; });
        memcpy(pVertexDataBegin, tmp_positions.data(), vertexBufferSize);
    } else {
        memcpy(pVertexDataBegin, cpu_mesh.vertices.data(), vertexBufferSize);
    }
    uploadBuffer->Unmap(0, nullptr);

    // Copy indices data to the index buffer.
    void* pIndexDataBegin;
    ThrowIfFailed(index_uploadBuffer->Map(0, nullptr, &pIndexDataBegin));
    memcpy(pIndexDataBegin, cpu_mesh.indices.data(), indexBufferSize);
    index_uploadBuffer->Unmap(0, nullptr);

    // Copy call
    d3d_ctx.m_CopyCommandList->CopyBufferRegion(vertexBuffer.Get(), 0, uploadBuffer.Get(), 0, vertexBufferSize);
    d3d_ctx.m_CopyCommandList->CopyBufferRegion(indexBuffer.Get(), 0, index_uploadBuffer.Get(), 0, indexBufferSize);

    d3d_ctx.DispatchCopyCommandList();
    d3d_ctx.WaitForPendingCopy();
}

void GPU_mesh::transition_from_copy_to_usage() {
    D3DContext& d3d_ctx = D3DContext::Get();

    const D3D12_RESOURCE_BARRIER barriers[] = {
        D3D12_RESOURCE_BARRIER{
            .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
            .Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
            .Transition =
                D3D12_RESOURCE_TRANSITION_BARRIER{
                    .pResource = vertexBuffer.Get(),
                    .Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
                    .StateBefore = D3D12_RESOURCE_STATE_COPY_DEST,
                    //.StateAfter = D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER,
                    .StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                },
        },
        D3D12_RESOURCE_BARRIER{
            .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
            .Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
            .Transition =
                D3D12_RESOURCE_TRANSITION_BARRIER{
                    .pResource = indexBuffer.Get(),
                    .Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
                    .StateBefore = D3D12_RESOURCE_STATE_COPY_DEST,
                    //.StateAfter = D3D12_RESOURCE_STATE_INDEX_BUFFER,
                    .StateAfter = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
                },
        },
    };
    // or D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE ? or D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE ?
    d3d_ctx.m_CommandList->ResourceBarrier(_countof(barriers), barriers);
}

D3D12_GPU_VIRTUAL_ADDRESS GPU_mesh::get_blas_gpu_va() const { return blasBuffer ? blasBuffer->GetGPUVirtualAddress() : 0; }

UINT GPU_mesh::get_vertex_count() const { return vertexCount; }
UINT GPU_mesh::get_index_count() const { return indexCount; }

void GPU_mesh::create_bottom_level_AS() {
    D3DContext& d3d_ctx = D3DContext::Get();

    D3D12_RAYTRACING_GEOMETRY_DESC geomDesc = {
        .Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES,
        .Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_NONE,
        .Triangles = {}
    };

    geomDesc.Triangles.VertexBuffer.StartAddress = vertexBuffer->GetGPUVirtualAddress();
    geomDesc.Triangles.VertexBuffer.StrideInBytes = positionsOnly ? sizeof(vertex::position) : sizeof(vertex);
    geomDesc.Triangles.VertexCount = vertexCount;
    geomDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;

    geomDesc.Triangles.IndexBuffer = indexBuffer->GetGPUVirtualAddress();
    geomDesc.Triangles.IndexCount = indexCount;
    geomDesc.Triangles.IndexFormat = DXGI_FORMAT_R32_UINT;

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs;
    inputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
    inputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    inputs.NumDescs = 1;
    inputs.pGeometryDescs = &geomDesc;
    inputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO info = {};

    d3d_ctx.m_d3dDevice->GetRaytracingAccelerationStructurePrebuildInfo(&inputs, &info);

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC buildDesc;

    const D3D12_HEAP_PROPERTIES def_props{
        .Type = D3D12_HEAP_TYPE_DEFAULT,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        .CreationNodeMask = 1,
        .VisibleNodeMask = 1,
    };

    const D3D12_RESOURCE_DESC scratch_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = info.ScratchDataSizeInBytes,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
    };

    const D3D12_RESOURCE_DESC blas_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = info.ResultDataMaxSizeInBytes,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
    };

    ThrowIfFailed(
        d3d_ctx.m_d3dDevice->CreateCommittedResource(
            &def_props, 
            D3D12_HEAP_FLAG_NONE, 
            &scratch_desc,
            D3D12_RESOURCE_STATE_COMMON, 
            nullptr, 
            IID_PPV_ARGS(&scratchBuffer)
        )
    );
    ThrowIfFailed(
        d3d_ctx.m_d3dDevice->CreateCommittedResource(
            &def_props, 
            D3D12_HEAP_FLAG_NONE, 
            &blas_desc,
            D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, 
            nullptr, 
            IID_PPV_ARGS(&blasBuffer)
        )
    );

    buildDesc.Inputs = inputs;
    buildDesc.SourceAccelerationStructureData = 0;
    buildDesc.ScratchAccelerationStructureData = scratchBuffer->GetGPUVirtualAddress();
    buildDesc.DestAccelerationStructureData = blasBuffer->GetGPUVirtualAddress();

    // Build the AS
    d3d_ctx.m_CommandList->BuildRaytracingAccelerationStructure(&buildDesc, 0, nullptr);

    D3D12_RESOURCE_BARRIER uavBarrier;
    uavBarrier.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    uavBarrier.UAV.pResource = blasBuffer.Get();
    uavBarrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    d3d_ctx.m_CommandList->ResourceBarrier(1, &uavBarrier);
}

GPU_mesh::~GPU_mesh() { release_gpu_resource(); }

void GPU_mesh::release_gpu_resource() {
    D3DContext& d3d_ctx = D3DContext::Get();
    if (d3d_ctx.m_CommandQueue != nullptr) {
        d3d_ctx.WaitForPendingOperations();
        d3d_ctx.WaitForPendingCopy();
    }
    vertexBuffer.Reset();
    indexBuffer.Reset();
    scratchBuffer.Reset();
    blasBuffer.Reset();
};

// GPU_mesh

GPU_Material::GPU_Material(const Material& mat, const std::vector<int>& texture_id_conversion_table, int default_texture) {
    int size = texture_id_conversion_table.size();
    const auto lambda = [&texture_id_conversion_table, size, default_texture](int i) { 
        return (i >= 0 && i < size) ? texture_id_conversion_table[i] : default_texture;
    };
    const auto lambda_normal_map = [&texture_id_conversion_table, size](int i) { 
        return (i >= 0 && i < size) ? texture_id_conversion_table[i] : -1;
    };
    baseColorFactor = mat.baseColorFactor;
    emissiveFactor = xyz1(mat.emissiveFactor);
    attenuationFactor = xyz1(mat.attenuationFactor);
    metallicFactor = mat.metallicFactor;
    roughnessFactor = mat.roughnessFactor;
    baseColorTextureIndex = lambda(mat.baseColorTextureIndex);
    metallicRoughnessTextureIndex = lambda(mat.metallicRoughnessTextureIndex);
    normalTextureIndex = lambda_normal_map(mat.normalTextureIndex);
    ior = mat.ior;
    dielectric_f0 = mat.dielectric_f0;
    transmisionFactor = mat.transmisionFactor;
    transmissionTextureIndex = lambda(mat.transmissionTextureIndex);
    emissiveTextureIndex = lambda(mat.emissiveTextureIndex);
    emissiveStrength = mat.emissiveStrength;
    alphaCutoff = mat.alphaCutoff;

    doubleSided = mat.doubleSided;
    hasVolume = mat.hasVolume;
    alphaBlending = mat.alphaBlending;

    AOStrength = mat.AOStrength;
    aoTextureIndex = lambda(mat.aoTextureIndex);
    
    thicknessTextureIndex = lambda(mat.thicknessTextureIndex);
    thicknessFactor = mat.thicknessFactor;
    padding = 0;
}

GPU_model::GPU_model(const Model& cpu_model, bool raytracing_support) {
    if (cpu_model.meshes.size() == 0) {
        return;
    }
    meshes_.reserve(cpu_model.meshes.size());
    for (const auto& mesh : cpu_model.meshes) {
        meshes_.emplace_back(mesh);
    }
    for (auto& mesh : meshes_) {
        mesh.transition_from_copy_to_usage();
    }
    if (raytracing_support) {
        for (auto& mesh : meshes_) {
            mesh.create_bottom_level_AS();
        }
        create_top_level_AS(cpu_model);
    }
    prepare_combined_vertex_index_buffers(cpu_model);
    prepare_textures_array_buffer(cpu_model);
    prepare_materials_array_buffer(cpu_model);
    objects = cpu_model.objects;
    isEmpty_ = false;
}

void GPU_model::prepare_textures_array_buffer(const Model& cpu_model) {
    int images_size = cpu_model.images.size();
    std::vector<bool> useSRGB;
    std::vector<bool> isNormalMap;
    useSRGB.resize(images_size, false);
    isNormalMap.resize(images_size, false);
    for (const auto& mat : cpu_model.materials) {
        int albedo_id = mat.baseColorTextureIndex;
        int emissive_id = mat.emissiveTextureIndex;
        int normal_id = mat.normalTextureIndex;
        if (albedo_id >= 0 && albedo_id < images_size) {
            useSRGB[albedo_id] = true;
        }
        if (emissive_id >= 0 && emissive_id < images_size) {
            useSRGB[emissive_id] = true;
        }
        if (normal_id >= 0 && normal_id < images_size) {
            isNormalMap[normal_id] = true;
        }
    }  // use SRGB for textures which are used as albedo or emmisive

    textures.reserve(images_size);
    for (int i = 0; i < images_size; ++i) {
        const auto& img = cpu_model.images[i];
        bool srgb = useSRGB[i];
        bool is_normal_map = isNormalMap[i];
        textures.emplace_back(img, srgb, true, is_normal_map);
    }
}

GPU_model::~GPU_model() { release_gpu_resource(); }

void GPU_model::release_gpu_resource() {
    D3DContext& d3d_ctx = D3DContext::Get();
    if (d3d_ctx.m_CommandQueue != nullptr) {
        d3d_ctx.WaitForPendingOperations();
    }
    tlasScratchBuffer.Reset();
    tlasBuffer.Reset();
    instancesUploadBuffer.Reset();
    //instancesBuffer.Reset();
    MaterialsArray.Reset();

    if (isEmpty_) return;

    d3d_ctx.m_SrvDescHeapAlloc.Free(combined_mesh_indices.cpuHandle, combined_mesh_indices.gpuHandle);
    d3d_ctx.m_SrvDescHeapAlloc.Free(combined_mesh_vertices.cpuHandle, combined_mesh_vertices.gpuHandle);
    d3d_ctx.m_SrvDescHeapAlloc.Free(combined_mesh_offsets.cpuHandle, combined_mesh_offsets.gpuHandle);
    d3d_ctx.m_SrvDescHeapAlloc.Free(materials_array.cpuHandle, materials_array.gpuHandle);
};

bool GPU_model::isEmpty() const { return isEmpty_; }

void GPU_model::create_top_level_AS(const Model& cpu_model) {
    D3DContext& d3d_ctx = D3DContext::Get();

    const auto& objects = cpu_model.objects;
    const size_t instanceCount = objects.size();
    if (instanceCount == 0) return;

    // Prepare CPU-side instance descriptions
    instances.resize(instanceCount);

    for (size_t i = 0; i < instanceCount; ++i) {
        const auto& obj = objects[i];
        D3D12_RAYTRACING_INSTANCE_DESC& inst = instances[i];

        // zero-init
        ZeroMemory(&inst, sizeof(inst));

        // Fill transform: D3D expects row-major 3x4 matrix. glm::mat4 is column-major,
        // so transpose when copying: inst.Transform[row][col] = mat[col][row]
        const auto& Mat = obj.ModelMatrix;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                inst.Transform[r][c] = Mat[c][r];
            }
        }
        const auto& cpu_mesh = cpu_model.meshes[obj.meshIndex];
        const auto& material = cpu_model.materials[cpu_mesh.materialIndex];
        bool doubleSided = material.doubleSided || material.hasVolume;
        bool alphaBlending = material.alphaBlending;
        bool alphaTest = (material.alphaCutoff >= 0.0f);

        inst.InstanceID = objects[i].meshIndex;  // can be used to identify object in shaders
        inst.InstanceMask = 0xFF;
        inst.InstanceContributionToHitGroupIndex = 0;  // shader binding table offsets if used
        inst.Flags = 0;
        if (doubleSided) inst.Flags |= D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE;
        if (!alphaTest) inst.Flags |= D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_OPAQUE;

        // Set BLAS GPU address for the referenced mesh
        uint32_t meshIndex = obj.meshIndex;
        if (meshIndex < meshes_.size()) {
            inst.AccelerationStructure = meshes_[meshIndex].get_blas_gpu_va();
        } else {
            inst.AccelerationStructure = 0;
        }
    }

    // Create an upload buffer for instance descriptions
    const UINT64 instancesSizeBytes = sizeof(D3D12_RAYTRACING_INSTANCE_DESC) * instanceCount;

    const D3D12_HEAP_PROPERTIES upload_props{
        .Type = D3D12_HEAP_TYPE_UPLOAD,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        .CreationNodeMask = 1,
        .VisibleNodeMask = 1,
    };

    const D3D12_RESOURCE_DESC instances_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = instancesSizeBytes,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE,
    };

    ThrowIfFailed(d3d_ctx.m_d3dDevice->CreateCommittedResource(&upload_props, D3D12_HEAP_FLAG_NONE, &instances_desc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&instancesUploadBuffer)));

    // Copy instance descs to upload buffer
    void* pInstancesData = nullptr;
    ThrowIfFailed(instancesUploadBuffer->Map(0, nullptr, &pInstancesData));
    memcpy(pInstancesData, instances.data(), instancesSizeBytes);
    instancesUploadBuffer->Unmap(0, nullptr);

    // Build TLAS inputs
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs = {};
    inputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    inputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    inputs.NumDescs = static_cast<UINT>(instanceCount);
    inputs.InstanceDescs = instancesUploadBuffer->GetGPUVirtualAddress();
    inputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE |
                   D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_UPDATE;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO info = {};
    d3d_ctx.m_d3dDevice->GetRaytracingAccelerationStructurePrebuildInfo(&inputs, &info);

    // Create scratch and result buffers for TLAS
    const D3D12_HEAP_PROPERTIES def_props{
        .Type = D3D12_HEAP_TYPE_DEFAULT,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        .CreationNodeMask = 1,
        .VisibleNodeMask = 1,
    };

    const D3D12_RESOURCE_DESC scratch_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = info.ScratchDataSizeInBytes,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
    };

    const D3D12_RESOURCE_DESC tlas_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = info.ResultDataMaxSizeInBytes,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
    };

    ThrowIfFailed(d3d_ctx.m_d3dDevice->CreateCommittedResource(
        &def_props, D3D12_HEAP_FLAG_NONE, &scratch_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&tlasScratchBuffer)));

    ThrowIfFailed(d3d_ctx.m_d3dDevice->CreateCommittedResource(&def_props, D3D12_HEAP_FLAG_NONE, &tlas_desc,
        D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, nullptr, IID_PPV_ARGS(&tlasBuffer)));

    // Describe build
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC buildDesc = {};
    buildDesc.Inputs = inputs;
    buildDesc.ScratchAccelerationStructureData = tlasScratchBuffer->GetGPUVirtualAddress();
    buildDesc.DestAccelerationStructureData = tlasBuffer->GetGPUVirtualAddress();
    buildDesc.SourceAccelerationStructureData = 0;

    // Build TLAS
    d3d_ctx.m_CommandList->BuildRaytracingAccelerationStructure(&buildDesc, 0, nullptr);

    // Copy Instances Info into separate GPU buffer for use in shaders (if I want to do instanced rendering?)
    //ThrowIfFailed(d3d_ctx.m_d3dDevice->CreateCommittedResource(
    //    &def_props, D3D12_HEAP_FLAG_NONE, &instances_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&instancesBuffer)));
    //d3d_ctx.m_CommandList->CopyBufferRegion(instancesBuffer.Get(), 0, instancesUploadBuffer.Get(), 0, instancesSizeBytes);

    // UAV barrier
    D3D12_RESOURCE_BARRIER uavBarrier{};
    uavBarrier.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    uavBarrier.UAV.pResource = tlasBuffer.Get();
    uavBarrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    d3d_ctx.m_CommandList->ResourceBarrier(1, &uavBarrier);
}

void GPU_model::prepare_combined_vertex_index_buffers(const Model& cpu_model) {
    size_t totalVertexCount = 0;
    size_t totalIndexCount = 0;
    for (const auto& mesh : cpu_model.meshes) {
        totalVertexCount += mesh.vertices.size();
        totalIndexCount += mesh.indices.size();
    }
    std::vector<vertex> combinedVertices;
    std::vector<uint32_t> combinedIndices;
    //std::vector<uint32_t> indicesOffsets;
    combinedVertices.reserve(totalVertexCount);
    combinedIndices.reserve(totalIndexCount);
    indicesOffsets.clear();
    indicesOffsets.reserve(cpu_model.meshes.size());
    indicesSizes.clear();
    indicesSizes.reserve(cpu_model.meshes.size());

    for (const auto& mesh : cpu_model.meshes) {
        indicesOffsets.push_back(combinedIndices.size());
        indicesSizes.push_back(mesh.indices.size());
        uint32_t vertexOffset = combinedVertices.size();
        combinedVertices.insert(combinedVertices.end(), mesh.vertices.begin(), mesh.vertices.end());
        std::transform(mesh.indices.begin(), mesh.indices.end(), 
            std::back_inserter(combinedIndices),
            [vertexOffset](uint32_t idx) { return idx + vertexOffset; });
    }

    // upload combinedVertices and combinedIndices to GPU buffers (via GPU_mesh constructor)
    combinedMesh =
        GPU_mesh(Mesh{.indices = std::move(combinedIndices), .vertices = std::move(combinedVertices), .materialIndex = 0}, false);
    combinedMesh.transition_from_copy_to_usage();
    // Note: we don't need BLAS for the combined mesh, so we won't call create_bottom_level_AS() on it

    // upload indicesOffsets to gpu buffer
    D3DContext& d3d_ctx = D3DContext::Get();

    d3d_ctx.InitCopyCommandList();

    ComPtr<ID3D12Resource2> offsets_uploadBuffer;

    uint32_t meshesCount = indicesOffsets.size();

    const UINT offsetsBufferSize = sizeof(uint32_t) * meshesCount;

    const D3D12_HEAP_PROPERTIES def_props{
        .Type = D3D12_HEAP_TYPE_DEFAULT,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        .CreationNodeMask = 1,
        .VisibleNodeMask = 1,
    };

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
        .Width = offsetsBufferSize,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE,
    };

    ThrowIfFailed(d3d_ctx.m_d3dDevice->CreateCommittedResource(&upload_props, D3D12_HEAP_FLAG_NONE, &upload_desc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&offsets_uploadBuffer)));
    ThrowIfFailed(d3d_ctx.m_d3dDevice->CreateCommittedResource(&def_props, D3D12_HEAP_FLAG_NONE, &upload_desc,
        D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&MeshIndicesOffsets)));

    void* pOffsetsDataBegin;
    ThrowIfFailed(offsets_uploadBuffer->Map(0, nullptr, &pOffsetsDataBegin));
    memcpy(pOffsetsDataBegin, indicesOffsets.data(), offsetsBufferSize);
    offsets_uploadBuffer->Unmap(0, nullptr);

    // Copy call
    d3d_ctx.m_CopyCommandList->CopyBufferRegion(
        MeshIndicesOffsets.Get(), 0, offsets_uploadBuffer.Get(), 0, offsetsBufferSize);

    d3d_ctx.DispatchCopyCommandList();
    d3d_ctx.WaitForPendingCopy();

    // Create SRV for 3 buffers: combined vertices, combined indices, and indices offsets
    CreateBufferSRV(d3d_ctx, combinedMesh.indexBuffer, totalIndexCount, 0, combined_mesh_indices);
    CreateBufferSRV(d3d_ctx, combinedMesh.vertexBuffer, totalVertexCount, sizeof(vertex), combined_mesh_vertices);
    CreateBufferSRV(d3d_ctx, MeshIndicesOffsets, meshesCount, 0, combined_mesh_offsets);
}

void GPU_model::prepare_materials_array_buffer(const Model& cpu_model) {
    D3DContext& d3d_ctx = D3DContext::Get();
    const auto& heap_alloc = d3d_ctx.m_SrvDescHeapAlloc;

    // convert from cpu image_id into indices within GPU SRV Heap
    texture_id_conversion_table_.reserve(textures.size());
    std::transform(textures.begin(), textures.end(), std::back_inserter(texture_id_conversion_table_),
        [&heap_alloc](const auto& el) { return heap_alloc.GetIndex(el.GetSRVHandle()); }
    );
    default_white_texture_index_ = heap_alloc.GetIndex(default_white_texture_.GetSRVHandle());

    uint32_t matCount = cpu_model.meshes.size();
    const UINT BufferSize = sizeof(GPU_Material) * matCount;

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
        &def_props, D3D12_HEAP_FLAG_NONE, &upload_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&MaterialsArray)));
    CreateBufferSRV(d3d_ctx, MaterialsArray, matCount, sizeof(GPU_Material), materials_array);

    update_materials_array_buffer(cpu_model);
}

void GPU_model::update_materials_array_buffer(const Model& cpu_model) {
    // I pack materials differently than in CPU for ease of use in shader.
    // Here, I index material by meshID, that might lead to some redundant data
    // Todo: pack the same as CPU without redundancy
    MaterialsCPUArray.clear();
    assert(sizeof(GPU_Material) == 128);
    MaterialsCPUArray.reserve(cpu_model.meshes.size());
    for (const auto& mesh : cpu_model.meshes) {
        const auto& mat = cpu_model.materials[mesh.materialIndex];
        MaterialsCPUArray.emplace_back(mat, texture_id_conversion_table_, default_white_texture_index_);
    }

    D3DContext& d3d_ctx = D3DContext::Get();
    d3d_ctx.InitCopyCommandList();

    ComPtr<ID3D12Resource2> mat_uploadBuffer;

    uint32_t matCount = MaterialsCPUArray.size();
    const UINT BufferSize = sizeof(GPU_Material) * matCount;

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
        .Width = BufferSize,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE,
    };

    ThrowIfFailed(d3d_ctx.m_d3dDevice->CreateCommittedResource(&upload_props, D3D12_HEAP_FLAG_NONE, &upload_desc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&mat_uploadBuffer)));

    void* pDataBegin;
    ThrowIfFailed(mat_uploadBuffer->Map(0, nullptr, &pDataBegin));
    memcpy(pDataBegin, MaterialsCPUArray.data(), BufferSize);
    mat_uploadBuffer->Unmap(0, nullptr);

    d3d_ctx.m_CopyCommandList->CopyBufferRegion(MaterialsArray.Get(), 0, mat_uploadBuffer.Get(), 0, BufferSize);

    d3d_ctx.DispatchCopyCommandList();
    d3d_ctx.WaitForPendingCopy();
}

const std::vector<GPU_Material>& GPU_model::get_materials_cpu_array() const { return MaterialsCPUArray; }

void GPU_model::update_transforms(const Model& model, bool updateTLAS) {
    objects.assign(model.objects.begin(), model.objects.end());
    if (!updateTLAS) return;

    D3DContext& d3d_ctx = D3DContext::Get();
    d3d_ctx.InitDXRCommandList();

    assert(instances.size() == model.objects.size());
    if (instances.size() != model.objects.size()) {
        return;
    }

    const size_t instanceCount = objects.size();
    if (instanceCount == 0) return;

    for (size_t i = 0; i < instanceCount; ++i) {
        const auto& obj = objects[i];
        D3D12_RAYTRACING_INSTANCE_DESC& inst = instances[i];

        // Fill transform: D3D expects row-major 3x4 matrix. glm::mat4 is column-major,
        // so transpose when copying: inst.Transform[row][col] = mat[col][row]
        const auto& Mat = obj.ModelMatrix;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                inst.Transform[r][c] = Mat[c][r];
            }
        }
    }

    void* pInstancesData = nullptr;
    const UINT64 instancesSizeBytes = sizeof(D3D12_RAYTRACING_INSTANCE_DESC) * instances.size();
    ThrowIfFailed(instancesUploadBuffer->Map(0, nullptr, &pInstancesData));
    memcpy(pInstancesData, instances.data(), instancesSizeBytes);
    instancesUploadBuffer->Unmap(0, nullptr);

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC updateDesc = {};

     // Build TLAS inputs
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs = {};
    inputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    inputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    inputs.NumDescs = static_cast<UINT>(instanceCount);
    inputs.InstanceDescs = instancesUploadBuffer->GetGPUVirtualAddress();
    inputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE |
                   D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_UPDATE;

    // Describe build
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC buildDesc = {};
    buildDesc.Inputs = inputs;
    buildDesc.ScratchAccelerationStructureData = tlasScratchBuffer->GetGPUVirtualAddress();
    buildDesc.DestAccelerationStructureData = tlasBuffer->GetGPUVirtualAddress();  // in-place update
    buildDesc.SourceAccelerationStructureData = 0;

    // Build TLAS
    d3d_ctx.m_DXRCommandList->BuildRaytracingAccelerationStructure(&buildDesc, 0, nullptr);

    // UAV barrier
    D3D12_RESOURCE_BARRIER uavBarrier{};
    uavBarrier.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    uavBarrier.UAV.pResource = tlasBuffer.Get();
    uavBarrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    d3d_ctx.m_DXRCommandList->ResourceBarrier(1, &uavBarrier);

    d3d_ctx.DispatchDXRCommandList();
    d3d_ctx.WaitForPendingDXR();
}

const GPU_mesh& GPU_model::get_combined_mesh() const { return combinedMesh; }

const std::vector<GPU_mesh>& GPU_model::get_meshes() const { return meshes_; }

std::vector<GPU_texture>& GPU_model::get_textures() { return textures; }

D3D12_GPU_VIRTUAL_ADDRESS GPU_model::GetGPUVirtualAddress() const { return tlasBuffer->GetGPUVirtualAddress(); }

// GPU_texture

GPU_texture::GPU_texture(UINT64 width, UINT height, TEXTURE_TRAITS texture_options, DXGI_FORMAT format)
    : texture_options(texture_options) {
    assert((texture_options & TEXTURE_TRAITS::Cubemap) == TEXTURE_TRAITS::None || width == height);
    if (format == DXGI_FORMAT_UNKNOWN) {
        format = choose_format();
    }
    format_ = format;
    create_texture_resource(width, height, format);
}

GPU_texture::GPU_texture(const CPUTexture<hdr_pixel>& cpu_texture, bool allocate_mips, bool is_normal_map) {
    texture_options = TEXTURE_TRAITS::HDR;
    if (allocate_mips) texture_options |= TEXTURE_TRAITS::AllocateMips;
    if (is_normal_map) texture_options |= TEXTURE_TRAITS::NormalMap;
    format_ = DXGI_FORMAT_R32G32B32A32_FLOAT;
    create_texture_resource(cpu_texture.width(), cpu_texture.height(), DXGI_FORMAT_R32G32B32A32_FLOAT);
    upload_texture_to_gpu(
        cpu_texture.width(), cpu_texture.height(), cpu_texture.data(), sizeof(hdr_pixel), DXGI_FORMAT_R32G32B32A32_FLOAT);
}

GPU_texture::GPU_texture(const CPUTexture<sdr_pixel>& cpu_texture, bool srgb_, bool allocate_mips, bool is_normal_map) {
    if (srgb_) texture_options |= TEXTURE_TRAITS::sRGB;
    if (allocate_mips) texture_options |= TEXTURE_TRAITS::AllocateMips;
    if (is_normal_map) texture_options |= TEXTURE_TRAITS::NormalMap;
    format_ = srgb_ ? DXGI_FORMAT_R8G8B8A8_UNORM_SRGB : DXGI_FORMAT_R8G8B8A8_UNORM;
    create_texture_resource(cpu_texture.width(), cpu_texture.height(), format_);
    upload_texture_to_gpu(cpu_texture.width(), cpu_texture.height(), cpu_texture.data(), sizeof(sdr_pixel), format_);
}

DXGI_FORMAT GPU_texture::choose_format() const {
    if (isDepth(texture_options)) return DXGI_FORMAT_D32_FLOAT;
    if (isHDR(texture_options)) return DXGI_FORMAT_R32G32B32A32_FLOAT;
    if (isSRGB(texture_options)) return DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    return DXGI_FORMAT_R8G8B8A8_UNORM;
}

GPU_texture::~GPU_texture() { release_gpu_resource(); }

void GPU_texture::create_texture_resource(UINT64 width, UINT height, DXGI_FORMAT format) {
    D3DContext& d3d_ctx = D3DContext::Get();
    bool isRenderTarget = ::isRenderTarget(texture_options);
    bool mips = AllocateMips(texture_options);
    bool isCubemap = ::isCubemap(texture_options);
    bool isDepth = ::isDepth(texture_options);
    bool isDepthWithSRV = ::isDepthWithSRV(texture_options);
    bool isUAV = ::isUAV(texture_options);
    bool isSRGB = ::isSRGB(texture_options);
    mipLevels = mips ? CalculateMipCount(width, height) : 1;
    if (pTexture == nullptr) {
        auto flags = D3D12_RESOURCE_FLAG_NONE;
        if (isRenderTarget) flags |= D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
        if (isDepth) flags |= D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
        if (isUAV) flags |= D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
        const D3D12_RESOURCE_DESC tex_desc{
            .Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
            .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
            .Width = width,
            .Height = height,
            .DepthOrArraySize = isCubemap? 6u : 1u,
            .MipLevels = mipLevels,
            .Format = isDepthWithSRV ? DXGI_FORMAT_R32_TYPELESS : format,
            .SampleDesc = {1, 0},
            .Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
            .Flags = flags
        };
        const D3D12_HEAP_PROPERTIES def_props{
            .Type = D3D12_HEAP_TYPE_DEFAULT,
            .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
            .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        };

        D3D12_RESOURCE_STATES initial_state = D3D12_RESOURCE_STATE_COMMON;

        if (!isDepth || isDepthWithSRV) {
            d3d_ctx.m_SrvDescHeapAlloc.Alloc(&srv_handle);
        }
        if (isRenderTarget) {
            d3d_ctx.m_RtvDescHeapAlloc.Alloc(&rtv_handle);
            initial_state = D3D12_RESOURCE_STATE_RENDER_TARGET;
        }
        if (isDepth) {
            d3d_ctx.m_DsvDescHeapAlloc.Alloc(&dsv_handle);
            initial_state = D3D12_RESOURCE_STATE_DEPTH_WRITE;
        }
        if (isUAV) {
            d3d_ctx.m_SrvDescHeapAlloc.Alloc(&uav_handle);
            initial_state = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        }

        bool use_clear_value = isRenderTarget || isDepth;
        D3D12_CLEAR_VALUE clear_value = {.Format = format, .Color = {0, 0, 0, 1}};
        if (isDepth) clear_value.DepthStencil = {1.0f, 0};
        const auto* clear_value_ptr = use_clear_value ? &clear_value : nullptr;

        d3d_ctx.m_d3dDevice->CreateCommittedResource(
            &def_props, D3D12_HEAP_FLAG_NONE, &tex_desc, initial_state, clear_value_ptr, IID_PPV_ARGS(&pTexture));

        if (!isDepth || isDepthWithSRV) {
            D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc{
                .Format = isDepthWithSRV ? DXGI_FORMAT_R32_FLOAT : format,
                .ViewDimension = isCubemap ? D3D12_SRV_DIMENSION_TEXTURECUBE : D3D12_SRV_DIMENSION_TEXTURE2D,
                .Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
                .Texture2D =
                    D3D12_TEX2D_SRV{
                        .MostDetailedMip = 0,
                        .MipLevels = mipLevels,
                    },
            };
            if (isCubemap) {
                srvDesc.TextureCube = D3D12_TEXCUBE_SRV{
                    .MostDetailedMip = 0,
                    .MipLevels = mipLevels,
                    .ResourceMinLODClamp = 0.0f,
                };
            }
            d3d_ctx.m_d3dDevice->CreateShaderResourceView(pTexture.Get(), &srvDesc, srv_handle.cpuHandle);
        }

        if (isRenderTarget) {
            D3D12_RENDER_TARGET_VIEW_DESC rtvDesc{
                .Format = format,
                .ViewDimension = isCubemap ? D3D12_RTV_DIMENSION_TEXTURE2DARRAY : D3D12_RTV_DIMENSION_TEXTURE2D,
                .Texture2D =
                    D3D12_TEX2D_RTV{
                        .MipSlice = 0,
                        .PlaneSlice = 0,
                    },
            };
            if (isCubemap) {
                rtvDesc.Texture2DArray = D3D12_TEX2D_ARRAY_RTV {
                    .MipSlice = 0,
                    .FirstArraySlice = 0,
                    .ArraySize = 6,
                    .PlaneSlice = 0,
                };
            }
            d3d_ctx.m_d3dDevice->CreateRenderTargetView(pTexture.Get(), &rtvDesc, rtv_handle.cpuHandle);
        }

        if (isDepth) {
            D3D12_DEPTH_STENCIL_VIEW_DESC dsvDesc{
                .Format = format,
                .ViewDimension = isCubemap ? D3D12_DSV_DIMENSION_TEXTURE2DARRAY : D3D12_DSV_DIMENSION_TEXTURE2D,
                .Texture2D =
                    D3D12_TEX2D_DSV{
                        .MipSlice = 0,
                    },
            };
            if (isCubemap) {
                dsvDesc.Texture2DArray = D3D12_TEX2D_ARRAY_DSV{
                    .MipSlice = 0,
                    .FirstArraySlice = 0,
                    .ArraySize = 6,
                };
            }
            d3d_ctx.m_d3dDevice->CreateDepthStencilView(pTexture.Get(), &dsvDesc, dsv_handle.cpuHandle);
        }

        if (isUAV) {
            // Create a unordered access view for the texture
            D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc{
                .Format = isSRGB ? DXGI_FORMAT_R8G8B8A8_UNORM : format,
                .ViewDimension = isCubemap ? D3D12_UAV_DIMENSION_TEXTURE2DARRAY : D3D12_UAV_DIMENSION_TEXTURE2D,
                .Texture2D =
                    D3D12_TEX2D_UAV{
                        .MipSlice = 0,
                        .PlaneSlice = 0,
                    },
            };
            if (isCubemap) {
                uavDesc.Texture2DArray = D3D12_TEX2D_ARRAY_UAV{
                    .MipSlice = 0,
                    .FirstArraySlice = 0,
                    .ArraySize = 6,
                    .PlaneSlice = 0,
                };
            }
            d3d_ctx.m_d3dDevice->CreateUnorderedAccessView(pTexture.Get(), nullptr, &uavDesc, uav_handle.cpuHandle);
        }
    }
}

void GPU_texture::upload_texture_to_gpu(int width_, int height_, const auto& data_, size_t sizeofpixel, DXGI_FORMAT format) {
    ComPtr<ID3D12Resource> uploadBuffer;
    UINT uploadPitch;
    UINT uploadSize;
    void* mapped = nullptr;

    D3DContext& d3d_ctx = D3DContext::Get();

    HRESULT hr;

    // Create a temporary upload resource to move the data in
    uploadPitch = (width_ * sizeofpixel + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1u) & ~(D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1u);
    uploadSize = height_ * uploadPitch;

    const D3D12_RESOURCE_DESC upload_desc{
        .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
        .Width = uploadSize,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE,
    };
    const D3D12_HEAP_PROPERTIES upload_props{
        .Type = D3D12_HEAP_TYPE_UPLOAD,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
    };

    hr = d3d_ctx.m_d3dDevice->CreateCommittedResource(&upload_props, D3D12_HEAP_FLAG_NONE, &upload_desc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&uploadBuffer));
    assert(SUCCEEDED(hr));

    // Write pixels into the upload resource
    if (mapped == nullptr) {
        D3D12_RANGE range = {0, uploadSize};
        hr = uploadBuffer->Map(0, nullptr, &mapped);
        assert(SUCCEEDED(hr));
    }
    for (int y = 0; y < height_; y++)
        memcpy((void*)((uintptr_t)mapped + y * uploadPitch), data_.data() + y * width_, width_ * sizeofpixel);

    // Copy the upload resource content into the real resource
    const D3D12_TEXTURE_COPY_LOCATION srcLocation = {
        .pResource = uploadBuffer.Get(),
        .Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT,
        .PlacedFootprint = D3D12_PLACED_SUBRESOURCE_FOOTPRINT {
            .Footprint = D3D12_SUBRESOURCE_FOOTPRINT {
                .Format = format,
                .Width = static_cast<UINT>(width_),
                .Height = static_cast<UINT>(height_),
                .Depth = 1,
                .RowPitch = uploadPitch,
            }
        }
    };

    const D3D12_TEXTURE_COPY_LOCATION dstLocation = {
        .pResource = pTexture.Get(),
        .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX,
        .SubresourceIndex = 0,
    };

    d3d_ctx.InitCopyCommandList();

    d3d_ctx.m_CopyCommandList->CopyTextureRegion(&dstLocation, 0, 0, 0, &srcLocation, nullptr);

    d3d_ctx.DispatchCopyCommandList();
    d3d_ctx.WaitForPendingCopy();
}

ComPtr<ID3D12Resource> GPU_texture::get_gpu_resource() { return pTexture; }

void GPU_texture::GetUAVHandleForMipLevel(uint8_t mipLevel, D3D12_CPU_DESCRIPTOR_HANDLE Handle) const {
    bool isCubemap = ::isCubemap(texture_options);
    bool isUAV = ::isUAV(texture_options);
    if (!isUAV) {
        return;
    }
    bool isSRGB = ::isSRGB(texture_options);
    D3DContext& d3d_ctx = D3DContext::Get();
    if (isUAV) {
        // Create a unordered access view for the texture
        D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc{
            .Format = isSRGB ? DXGI_FORMAT_R8G8B8A8_UNORM : format_,
            .ViewDimension = isCubemap ? D3D12_UAV_DIMENSION_TEXTURE2DARRAY : D3D12_UAV_DIMENSION_TEXTURE2D,
            .Texture2D =
                D3D12_TEX2D_UAV{
                    .MipSlice = mipLevel,
                    .PlaneSlice = 0,
                },
        };
        if (isCubemap) {
            uavDesc.Texture2DArray = D3D12_TEX2D_ARRAY_UAV{
                .MipSlice = mipLevel,
                .FirstArraySlice = 0,
                .ArraySize = 6,
                .PlaneSlice = 0,
            };
        }
        d3d_ctx.m_d3dDevice->CreateUnorderedAccessView(pTexture.Get(), nullptr, &uavDesc, Handle);
    }
}

void GPU_texture::GetSRVHandleForMipLevel(uint8_t mipLevel, D3D12_CPU_DESCRIPTOR_HANDLE Handle) const {
    bool isCubemap = ::isCubemap(texture_options);
    bool isSRGB = ::isSRGB(texture_options);
    D3DContext& d3d_ctx = D3DContext::Get();

    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc{
        .Format = format_,
        .ViewDimension = isCubemap ? D3D12_SRV_DIMENSION_TEXTURECUBE : D3D12_SRV_DIMENSION_TEXTURE2D,
        .Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
        .Texture2D =
            D3D12_TEX2D_SRV{
                .MostDetailedMip = mipLevel,
                .MipLevels = 1,
            },
    };
    if (isCubemap) {
        srvDesc.TextureCube = D3D12_TEXCUBE_SRV{
            .MostDetailedMip = mipLevel,
            .MipLevels = 1,
            .ResourceMinLODClamp = 0.0f,
        };
    }
    d3d_ctx.m_d3dDevice->CreateShaderResourceView(pTexture.Get(), &srvDesc, Handle);
}

void GPU_texture::release_gpu_resource() {
    if (pTexture) {
        D3DContext& d3d_ctx = D3DContext::Get();
        if (!isDepth(texture_options) || isDepthWithSRV(texture_options)) {
            d3d_ctx.m_SrvDescHeapAlloc.Free(srv_handle);
        }
        if (isRenderTarget(texture_options)) {
            d3d_ctx.m_RtvDescHeapAlloc.Free(rtv_handle);
        }
        if (isDepth(texture_options)) {
            d3d_ctx.m_DsvDescHeapAlloc.Free(dsv_handle);
        }
        if (isUAV(texture_options)) {
            d3d_ctx.m_SrvDescHeapAlloc.Free(uav_handle);
        }

        pTexture.Reset();
    }
}

void GPU_texture::copy_texture(ComPtr<ID3D12Resource> gpuDst, ComPtr<ID3D12Resource> gpuSrc, 
    D3D12_RESOURCE_STATES dst_state, D3D12_RESOURCE_STATES src_state, ComPtr<ID3D12GraphicsCommandList4> commandList) 
{
    D3D12_RESOURCE_BARRIER to_barriers[2] = {
        CD3DX12_RESOURCE_BARRIER::Transition(gpuDst.Get(), dst_state, D3D12_RESOURCE_STATE_COMMON),
        CD3DX12_RESOURCE_BARRIER::Transition(gpuSrc.Get(), src_state, D3D12_RESOURCE_STATE_COPY_SOURCE),
    };
    commandList->ResourceBarrier(2, to_barriers);

    commandList->CopyResource(gpuDst.Get(), gpuSrc.Get());

    D3D12_RESOURCE_BARRIER from_barriers[2] = {
        CD3DX12_RESOURCE_BARRIER::Transition(gpuDst.Get(), D3D12_RESOURCE_STATE_COPY_DEST, dst_state),
        CD3DX12_RESOURCE_BARRIER::Transition(gpuSrc.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, src_state),
    };
    commandList->ResourceBarrier(2, from_barriers);
}

void GPU_texture::copy_texture(GPU_texture& dst, GPU_texture& src, 
    D3D12_RESOURCE_STATES dst_state, D3D12_RESOURCE_STATES src_state, 
    ComPtr<ID3D12GraphicsCommandList4>& commandList) 
{
    const auto& gpuDst = dst.get_gpu_resource();
    const auto& gpuSrc = src.get_gpu_resource();
    copy_texture(gpuDst, gpuSrc, dst_state, src_state, commandList);
}
void GPU_texture::copy_texture_mip0_only(GPU_texture& dst, GPU_texture& src, 
    D3D12_RESOURCE_STATES dst_state, D3D12_RESOURCE_STATES src_state, 
    ComPtr<ID3D12GraphicsCommandList4>& commandList) 
{
    const auto& gpuDst = dst.get_gpu_resource();
    const auto& gpuSrc = src.get_gpu_resource();

    D3D12_RESOURCE_BARRIER to_barriers[2] = {
        CD3DX12_RESOURCE_BARRIER::Transition(gpuDst.Get(), dst_state, D3D12_RESOURCE_STATE_COMMON),
        CD3DX12_RESOURCE_BARRIER::Transition(gpuSrc.Get(), src_state, D3D12_RESOURCE_STATE_COPY_SOURCE),
    };
    commandList->ResourceBarrier(2, to_barriers);

    CD3DX12_TEXTURE_COPY_LOCATION dstLocation = {gpuDst.Get(), 0};
    CD3DX12_TEXTURE_COPY_LOCATION srcLocation = {gpuSrc.Get(), 0};
    commandList->CopyTextureRegion(&dstLocation, 0, 0, 0, &srcLocation, nullptr);

    D3D12_RESOURCE_BARRIER from_barriers[2] = {
        CD3DX12_RESOURCE_BARRIER::Transition(gpuDst.Get(), D3D12_RESOURCE_STATE_COPY_DEST, dst_state),
        CD3DX12_RESOURCE_BARRIER::Transition(gpuSrc.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, src_state),
    };
    commandList->ResourceBarrier(2, from_barriers);
}

D3D12_GPU_DESCRIPTOR_HANDLE GPU_texture::GetSRVHandle() const { return srv_handle.gpuHandle; }

D3D12_GPU_DESCRIPTOR_HANDLE GPU_texture::GetUAVHandle() const { return uav_handle.gpuHandle; }

D3D12_CPU_DESCRIPTOR_HANDLE GPU_texture::GetRTVHandle() const { return rtv_handle.cpuHandle; }

D3D12_CPU_DESCRIPTOR_HANDLE GPU_texture::GetDSVHandle() const { return dsv_handle.cpuHandle; }

}  // namespace app