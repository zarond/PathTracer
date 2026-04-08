#include "GPU_model.h"

namespace {

using namespace app;
// Create SRV description for a buffer.
D3D12_SHADER_RESOURCE_VIEW_DESC CreatSRVDescription(UINT numElements, UINT elementSize) {
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

void CreateBufferSRV(
    D3DContext& d3d_ctx, const ComPtr<ID3D12Resource>& buffer, UINT numElements, UINT elementSize, 
    GPU_model::D3D_Handle_Pair& handles) 
{
    d3d_ctx.g_pd3dSrvDescHeapAlloc.Alloc(&handles.cpuDescriptorHandle, &handles.gpuDescriptorHandle);
    const auto srvDesc = CreatSRVDescription(numElements, elementSize);
    d3d_ctx.g_pd3dDevice->CreateShaderResourceView(buffer.Get(), &srvDesc, handles.cpuDescriptorHandle);
}

}  // namespace

namespace app {

inline void ThrowIfFailed(HRESULT hr) {
    if (FAILED(hr)) {
        throw std::exception();
    }
}

GPU_mesh::GPU_mesh(const Mesh& cpu_mesh, bool positions_only) : positions_only_(positions_only) { 
    D3DContext& d3d_ctx = D3DContext::Get();

    d3d_ctx.InitCopyCommandList();

    ComPtr<ID3D12Resource2> uploadBuffer;
    ComPtr<ID3D12Resource2> index_uploadBuffer;

    vertexCount = cpu_mesh.vertices.size();
    indexCount = cpu_mesh.indices.size();

    const UINT vertexBufferSize = (positions_only_ ? sizeof(vertex::position) : sizeof(vertex)) * vertexCount;
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
        d3d_ctx.g_pd3dDevice->CreateCommittedResource(
            &upload_props, 
            D3D12_HEAP_FLAG_NONE, 
            &upload_desc, 
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&uploadBuffer)
        )
    );
    ThrowIfFailed(
        d3d_ctx.g_pd3dDevice->CreateCommittedResource(
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
        d3d_ctx.g_pd3dDevice->CreateCommittedResource(
            &upload_props, 
            D3D12_HEAP_FLAG_NONE, 
            &index_upload_desc, 
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&index_uploadBuffer)
        )
    );
    ThrowIfFailed(
        d3d_ctx.g_pd3dDevice->CreateCommittedResource(
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
    if (positions_only_) {
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
    d3d_ctx.g_pd3dCopyCommandList->CopyBufferRegion(vertexBuffer.Get(), 0, uploadBuffer.Get(), 0, vertexBufferSize);
    d3d_ctx.g_pd3dCopyCommandList->CopyBufferRegion(indexBuffer.Get(), 0, index_uploadBuffer.Get(), 0, indexBufferSize);

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
    d3d_ctx.g_pd3dCommandList->ResourceBarrier(_countof(barriers), barriers);
}

D3D12_GPU_VIRTUAL_ADDRESS GPU_mesh::get_blas_gpu_va() const 
{ return blasBuffer ? blasBuffer->GetGPUVirtualAddress() : 0; }

void GPU_mesh::create_bottom_level_AS() {
    D3DContext& d3d_ctx = D3DContext::Get();

    D3D12_RAYTRACING_GEOMETRY_DESC geomDesc = { 
        .Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES,
        .Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_NONE,
        .Triangles = {}
    };

    geomDesc.Triangles.VertexBuffer.StartAddress = vertexBuffer->GetGPUVirtualAddress();
    geomDesc.Triangles.VertexBuffer.StrideInBytes = positions_only_ ? sizeof(vertex::position) : sizeof(vertex);
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

    d3d_ctx.g_pd3dDevice->GetRaytracingAccelerationStructurePrebuildInfo(&inputs, &info);

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
        d3d_ctx.g_pd3dDevice->CreateCommittedResource(
            &def_props, 
            D3D12_HEAP_FLAG_NONE, 
            &scratch_desc,
            D3D12_RESOURCE_STATE_COMMON, 
            nullptr, 
            IID_PPV_ARGS(&scratchBuffer)
        )
    );
    ThrowIfFailed(
        d3d_ctx.g_pd3dDevice->CreateCommittedResource(
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
    d3d_ctx.g_pd3dCommandList->BuildRaytracingAccelerationStructure(&buildDesc, 0, nullptr);

    D3D12_RESOURCE_BARRIER uavBarrier;
    uavBarrier.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    uavBarrier.UAV.pResource = blasBuffer.Get();
    uavBarrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    d3d_ctx.g_pd3dCommandList->ResourceBarrier(1, &uavBarrier);
}

GPU_mesh::~GPU_mesh() { release_gpu_resource(); }

void GPU_mesh::release_gpu_resource() {
    D3DContext& d3d_ctx = D3DContext::Get();
    if (d3d_ctx.g_pd3dCommandQueue != nullptr) {
        d3d_ctx.WaitForPendingOperations();
        d3d_ctx.WaitForPendingCopy();
    }
    vertexBuffer.Reset(); 
    indexBuffer.Reset();
    scratchBuffer.Reset();
    blasBuffer.Reset();
};

// GPU_mesh

GPU_Material::GPU_Material(
    const Material& mat, const std::vector<int>& texture_id_conversion_table, int default_texture) {
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
    alpha_cutoff = mat.alpha_cutoff;

    doubleSided = mat.doubleSided;
    hasVolume = mat.hasVolume;
    alphaBlending = mat.alphaBlending;
    padding0 = 0;
}

GPU_model::GPU_model(const Model& cpu_model) {
    if (cpu_model.meshes_.size() == 0) {
        return;
    }
    meshes_.reserve(cpu_model.meshes_.size());
    for (const auto& mesh : cpu_model.meshes_) {
        meshes_.emplace_back(mesh);
    }
    for (auto& mesh : meshes_) {
        mesh.transition_from_copy_to_usage();
    }
    for (auto& mesh : meshes_) {
        mesh.create_bottom_level_AS();
    }
    create_top_level_AS(cpu_model);
    prepare_combined_vertex_index_buffers(cpu_model);
    prepare_textures_array_buffer(cpu_model);
    prepare_materials_array_buffer(cpu_model);
    isEmpty_ = false;
}

void GPU_model::prepare_textures_array_buffer(const Model& cpu_model) {
    int images_size = cpu_model.images_.size();
    std::vector<bool> useSRGB;
    useSRGB.resize(images_size, false);
    for (const auto& mat : cpu_model.materials_) {
        int albedo_id = mat.baseColorTextureIndex;
        int emissive_id = mat.emissiveTextureIndex;
        if (albedo_id >= 0 && albedo_id < images_size) {
            useSRGB[albedo_id] = true;
        }
        if (emissive_id >= 0 && emissive_id < images_size) {
            useSRGB[emissive_id] = true;
        }
    }  // use SRGB for textures which are used as albedo or emmisive

    textures.reserve(images_size);
    for (int i = 0; i < images_size; ++i) {
        const auto& img = cpu_model.images_[i];
        bool srgb = useSRGB[i];
        textures.emplace_back(img, srgb);
    }
}

GPU_model::~GPU_model() { release_gpu_resource(); }

void GPU_model::release_gpu_resource() {
    D3DContext& d3d_ctx = D3DContext::Get();\
    if (d3d_ctx.g_pd3dCommandQueue != nullptr) {
        d3d_ctx.WaitForPendingOperations();
    }
    tlasScratchBuffer.Reset();
    tlasBuffer.Reset();
    instancesUploadBuffer.Reset();
    MaterialsArray.Reset();

    if (isEmpty_) return;

    d3d_ctx.g_pd3dSrvDescHeapAlloc.Free(combined_mesh_indices.cpuDescriptorHandle, combined_mesh_indices.gpuDescriptorHandle);
    d3d_ctx.g_pd3dSrvDescHeapAlloc.Free(combined_mesh_vertices.cpuDescriptorHandle, combined_mesh_vertices.gpuDescriptorHandle);
    d3d_ctx.g_pd3dSrvDescHeapAlloc.Free(combined_mesh_offsets.cpuDescriptorHandle, combined_mesh_offsets.gpuDescriptorHandle);
    d3d_ctx.g_pd3dSrvDescHeapAlloc.Free(materials_array.cpuDescriptorHandle, materials_array.gpuDescriptorHandle);
};

bool GPU_model::isEmpty() const { return isEmpty_; }

void GPU_model::create_top_level_AS(const Model& cpu_model) {
    D3DContext& d3d_ctx = D3DContext::Get();

    const auto& objects = cpu_model.objects_;
    const size_t instanceCount = objects.size();
    if (instanceCount == 0) return;

    // Prepare CPU-side instance descriptions
    std::vector<D3D12_RAYTRACING_INSTANCE_DESC> instances;
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
        const auto& cpu_mesh = cpu_model.meshes_[obj.meshIndex];
        const auto& material = cpu_model.materials_[cpu_mesh.materialIndex];
        bool doubleSided = material.doubleSided || material.hasVolume;
        bool alphaBlending = material.alphaBlending;
        bool alphaTest = (material.alpha_cutoff >= 0.0f);

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

    ThrowIfFailed(d3d_ctx.g_pd3dDevice->CreateCommittedResource(&upload_props, D3D12_HEAP_FLAG_NONE, &instances_desc,
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
    inputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO info = {};
    d3d_ctx.g_pd3dDevice->GetRaytracingAccelerationStructurePrebuildInfo(&inputs, &info);

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

    ThrowIfFailed(d3d_ctx.g_pd3dDevice->CreateCommittedResource(
        &def_props, D3D12_HEAP_FLAG_NONE, &scratch_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&tlasScratchBuffer)));

    ThrowIfFailed(d3d_ctx.g_pd3dDevice->CreateCommittedResource(&def_props, D3D12_HEAP_FLAG_NONE, &tlas_desc,
        D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, nullptr, IID_PPV_ARGS(&tlasBuffer)));

    // Describe build
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC buildDesc = {};
    buildDesc.Inputs = inputs;
    buildDesc.ScratchAccelerationStructureData = tlasScratchBuffer->GetGPUVirtualAddress();
    buildDesc.DestAccelerationStructureData = tlasBuffer->GetGPUVirtualAddress();
    buildDesc.SourceAccelerationStructureData = 0;

    // Build TLAS
    d3d_ctx.g_pd3dCommandList->BuildRaytracingAccelerationStructure(&buildDesc, 0, nullptr);

    // UAV barrier
    D3D12_RESOURCE_BARRIER uavBarrier{};
    uavBarrier.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
    uavBarrier.UAV.pResource = tlasBuffer.Get();
    uavBarrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    d3d_ctx.g_pd3dCommandList->ResourceBarrier(1, &uavBarrier);
}

void GPU_model::prepare_combined_vertex_index_buffers(const Model& cpu_model) {
    size_t totalVertexCount = 0;
    size_t totalIndexCount = 0;
    for (const auto& mesh : cpu_model.meshes_) {
        totalVertexCount += mesh.vertices.size();
        totalIndexCount += mesh.indices.size();
    }
    std::vector<vertex> combinedVertices;
    std::vector<uint32_t> combinedIndices;
    std::vector<uint32_t> indicesOffsets;
    combinedVertices.reserve(totalVertexCount);
    combinedIndices.reserve(totalIndexCount);
    indicesOffsets.reserve(cpu_model.meshes_.size());

    for (const auto& mesh : cpu_model.meshes_) {
        indicesOffsets.push_back(combinedIndices.size());
        uint32_t vertexOffset = combinedVertices.size();
        combinedVertices.insert(combinedVertices.end(), mesh.vertices.begin(), mesh.vertices.end());
        std::transform(mesh.indices.begin(), mesh.indices.end(), 
            std::back_inserter(combinedIndices),
            [vertexOffset](uint32_t idx) { return idx + vertexOffset; });
    }

    // upload combinedVertices and combinedIndices to GPU buffers (via GPU_mesh constructor)
    combinedMesh = GPU_mesh(Mesh{.indices = std::move(combinedIndices), .vertices = std::move(combinedVertices), .materialIndex = 0}, false);
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

    ThrowIfFailed(d3d_ctx.g_pd3dDevice->CreateCommittedResource(&upload_props, D3D12_HEAP_FLAG_NONE, &upload_desc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&offsets_uploadBuffer)));
    ThrowIfFailed(d3d_ctx.g_pd3dDevice->CreateCommittedResource(&def_props, D3D12_HEAP_FLAG_NONE, &upload_desc,
        D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&MeshIndicesOffsets)));

    void* pOffsetsDataBegin;
    ThrowIfFailed(offsets_uploadBuffer->Map(0, nullptr, &pOffsetsDataBegin));
    memcpy(pOffsetsDataBegin, indicesOffsets.data(), offsetsBufferSize);
    offsets_uploadBuffer->Unmap(0, nullptr);

    // Copy call
    d3d_ctx.g_pd3dCopyCommandList->CopyBufferRegion(MeshIndicesOffsets.Get(), 0, offsets_uploadBuffer.Get(), 0, offsetsBufferSize);

    d3d_ctx.DispatchCopyCommandList();
    d3d_ctx.WaitForPendingCopy();

    // Create SRV for 3 buffers: combined vertices, combined indices, and indices offsets
    CreateBufferSRV(d3d_ctx, combinedMesh.indexBuffer, totalIndexCount, 0, combined_mesh_indices);
    CreateBufferSRV(d3d_ctx, combinedMesh.vertexBuffer, totalVertexCount, sizeof(vertex), combined_mesh_vertices);
    CreateBufferSRV(d3d_ctx, MeshIndicesOffsets, meshesCount, 0, combined_mesh_offsets);
}

void GPU_model::prepare_materials_array_buffer(const Model& cpu_model) {
    D3DContext& d3d_ctx = D3DContext::Get();
    const auto& heap_alloc = d3d_ctx.g_pd3dSrvDescHeapAlloc;

    std::vector<GPU_Material> data;

    // convert from cpu image_id into indices within GPU SRV Heap
    std::vector<int> texture_id_conversion_table;
    texture_id_conversion_table.reserve(textures.size());
    std::transform(textures.begin(), textures.end(), std::back_inserter(texture_id_conversion_table), 
        [&heap_alloc](const auto& el) { return heap_alloc.GetIndex(el.GetSRVHandle()); }
    );
    int default_white_texture_index = heap_alloc.GetIndex(default_white_texture.GetSRVHandle());

    // I pack materials differently than in CPU for ease of use in shader. 
    // Here, I index material by meshID, that might lead to some redundant data
    // Todo: pack the same as CPU without redundancy
    assert(sizeof(GPU_Material) == 112);
    data.reserve(cpu_model.meshes_.size());
    for (const auto& mesh : cpu_model.meshes_) {
        const auto& mat = cpu_model.materials_[mesh.materialIndex];
        data.emplace_back(mat, texture_id_conversion_table, default_white_texture_index);
    }

    d3d_ctx.InitCopyCommandList();

    ComPtr<ID3D12Resource2> mat_uploadBuffer;

    uint32_t matCount = data.size();

    const UINT BufferSize = sizeof(GPU_Material) * matCount;

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
        .Width = BufferSize,
        .Height = 1,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = DXGI_FORMAT_UNKNOWN,
        .SampleDesc = {1, 0},
        .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
        .Flags = D3D12_RESOURCE_FLAG_NONE,
    };

    ThrowIfFailed(d3d_ctx.g_pd3dDevice->CreateCommittedResource(&upload_props, D3D12_HEAP_FLAG_NONE, &upload_desc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&mat_uploadBuffer)));
    ThrowIfFailed(d3d_ctx.g_pd3dDevice->CreateCommittedResource(
        &def_props, D3D12_HEAP_FLAG_NONE, &upload_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&MaterialsArray)));

    void* pDataBegin;
    ThrowIfFailed(mat_uploadBuffer->Map(0, nullptr, &pDataBegin));
    memcpy(pDataBegin, data.data(), BufferSize);
    mat_uploadBuffer->Unmap(0, nullptr);

    d3d_ctx.g_pd3dCopyCommandList->CopyBufferRegion(MaterialsArray.Get(), 0, mat_uploadBuffer.Get(), 0, BufferSize);

    d3d_ctx.DispatchCopyCommandList();
    d3d_ctx.WaitForPendingCopy();

    CreateBufferSRV(d3d_ctx, MaterialsArray, matCount, sizeof(GPU_Material), materials_array);
}

const std::vector<GPU_mesh>& GPU_model::get_meshes() const { return meshes_; }

D3D12_GPU_VIRTUAL_ADDRESS GPU_model::GetGPUVirtualAddress() const {
    return tlasBuffer->GetGPUVirtualAddress();
}

// GPU_texture

GPU_texture::GPU_texture(const CPUTexture<hdr_pixel>& cpu_texture) : HDR(true) { 
    create_texture_resource(cpu_texture.width(), cpu_texture.height(), DXGI_FORMAT_R32G32B32A32_FLOAT);
    upload_texture_to_gpu(
        cpu_texture.width(), cpu_texture.height(), cpu_texture.data(), sizeof(hdr_pixel), DXGI_FORMAT_R32G32B32A32_FLOAT);
}

GPU_texture::GPU_texture(const CPUTexture<sdr_pixel>& cpu_texture, bool srgb_) : HDR(false), srgb(srgb_) {
    DXGI_FORMAT format = srgb ? DXGI_FORMAT_R8G8B8A8_UNORM_SRGB : DXGI_FORMAT_R8G8B8A8_UNORM;
    create_texture_resource(cpu_texture.width(), cpu_texture.height(), format);
    upload_texture_to_gpu(cpu_texture.width(), cpu_texture.height(), cpu_texture.data(), sizeof(sdr_pixel), format);
}

GPU_texture::~GPU_texture() { 
    release_gpu_resource();
}

void GPU_texture::create_texture_resource(UINT64 width, UINT height, DXGI_FORMAT format) {
    D3DContext& d3d_ctx = D3DContext::Get(); 
    if (pTexture == nullptr) {
        const D3D12_RESOURCE_DESC tex_desc{
            .Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
            .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
            .Width = width,
            .Height = height,
            .DepthOrArraySize = 1,
            .MipLevels = 1,
            .Format = format,
            .SampleDesc = {1, 0},
            .Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
            .Flags = D3D12_RESOURCE_FLAG_NONE
        };
        const D3D12_HEAP_PROPERTIES def_props{
            .Type = D3D12_HEAP_TYPE_DEFAULT,
            .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
            .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        };

        d3d_ctx.g_pd3dSrvDescHeapAlloc.Alloc(&srv_cpu_handle, &srv_gpu_handle);

        d3d_ctx.g_pd3dDevice->CreateCommittedResource(
            &def_props, D3D12_HEAP_FLAG_NONE, &tex_desc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&pTexture));

        const D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc{
            .Format = format,
            .ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D,
            .Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
            .Texture2D =
                D3D12_TEX2D_SRV{
                    .MostDetailedMip = 0,
                    .MipLevels = 1,
                },
        };
        d3d_ctx.g_pd3dDevice->CreateShaderResourceView(pTexture.Get(), &srvDesc, srv_cpu_handle);
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

    hr = d3d_ctx.g_pd3dDevice->CreateCommittedResource(&upload_props, D3D12_HEAP_FLAG_NONE, &upload_desc,
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

    d3d_ctx.g_pd3dCopyCommandList->CopyTextureRegion(&dstLocation, 0, 0, 0, &srcLocation, nullptr);

    d3d_ctx.DispatchCopyCommandList();
    d3d_ctx.WaitForPendingCopy();
}

void GPU_texture::release_gpu_resource() {
    if (pTexture) {
        D3DContext::Get().g_pd3dSrvDescHeapAlloc.Free(srv_cpu_handle, srv_gpu_handle);
        pTexture.Reset();
    }
}

D3D12_GPU_DESCRIPTOR_HANDLE GPU_texture::GetSRVHandle() const { return srv_gpu_handle; }

}  // namespace app