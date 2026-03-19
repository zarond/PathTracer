#include "GPU_model.h"

namespace app {

inline void ThrowIfFailed(HRESULT hr) {
    if (FAILED(hr)) {
        throw std::exception();
    }
}

GPU_mesh::GPU_mesh(const Mesh& cpu_mesh) { 
    D3DContext& d3d_ctx = D3DContext::Get();

    //d3d_ctx.WaitForPendingOperations();
    d3d_ctx.WaitForPending—opy();
    /*
    if (d3d_ctx.g_fenceLastSignaledValue) { // ???
        d3d_ctx.g_pd3dCopyQueue->Wait(d3d_ctx.g_fence.Get(), d3d_ctx.g_fenceLastSignaledValue);
    }

    // Wait for previous copy to complete
    const UINT64 lastSignaled = d3d_ctx.copy_fenceLastSignaledValue;
    if (d3d_ctx.copy_fence->GetCompletedValue() < lastSignaled) {
        HRESULT hr = d3d_ctx.copy_fence->SetEventOnCompletion(lastSignaled, d3d_ctx.copy_fenceEvent);
        if (SUCCEEDED(hr)) {
            ::WaitForSingleObject(d3d_ctx.copy_fenceEvent, INFINITE);
        }
    }
    */
    d3d_ctx.g_pd3dCopyAllocator->Reset();
    d3d_ctx.g_pd3dCopyCommandList->Reset(d3d_ctx.g_pd3dCopyAllocator.Get(), nullptr);

    ComPtr<ID3D12Resource2> uploadBuffer;
    ComPtr<ID3D12Resource2> index_uploadBuffer;

    vertexCount = cpu_mesh.vertices.size();
    indexCount = cpu_mesh.indices.size();

    const UINT vertexBufferSize = sizeof(vertex) * vertexCount;
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
    memcpy(pVertexDataBegin, cpu_mesh.vertices.data(), vertexBufferSize);
    uploadBuffer->Unmap(0, nullptr);

    // Copy indices data to the index buffer.
    void* pIndexDataBegin;
    ThrowIfFailed(index_uploadBuffer->Map(0, nullptr, &pIndexDataBegin));
    memcpy(pIndexDataBegin, cpu_mesh.indices.data(), indexBufferSize);
    index_uploadBuffer->Unmap(0, nullptr);

    // Copy call
    d3d_ctx.g_pd3dCopyCommandList->CopyBufferRegion(vertexBuffer.Get(), 0, uploadBuffer.Get(), 0, vertexBufferSize);
    d3d_ctx.g_pd3dCopyCommandList->CopyBufferRegion(indexBuffer.Get(), 0, index_uploadBuffer.Get(), 0, indexBufferSize);

    HRESULT hr;

    hr = d3d_ctx.g_pd3dCopyCommandList->Close();
    assert(SUCCEEDED(hr));

    // Execute the copy
    ID3D12CommandList* lists[] = {d3d_ctx.g_pd3dCopyCommandList.Get()};
    d3d_ctx.g_pd3dCopyQueue->ExecuteCommandLists(1, lists);

    //hr = d3d_ctx.g_pd3dCopyQueue->Signal(d3d_ctx.copy_fence.Get(), ++d3d_ctx.copy_fenceLastSignaledValue);
    //assert(SUCCEEDED(hr));
    //d3d_ctx.g_pd3dCommandQueue->Wait(d3d_ctx.copy_fence.Get(), d3d_ctx.copy_fenceLastSignaledValue);

    // Initialize the vertex buffer view.
    vertexBufferView.BufferLocation = vertexBuffer->GetGPUVirtualAddress();
    vertexBufferView.StrideInBytes = sizeof(vertex);
    vertexBufferView.SizeInBytes = vertexBufferSize;

    // Initialize the index buffer view.
    indexBufferView.BufferLocation = indexBuffer->GetGPUVirtualAddress();
    indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    indexBufferView.SizeInBytes = sizeof(uint32_t) * cpu_mesh.indices.size();

    d3d_ctx.WaitForPending—opy();
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

    // d3d_ctx.WaitForPendingOperations();
    // d3d_ctx.WaitForPending—opy();
}

D3D12_GPU_VIRTUAL_ADDRESS GPU_mesh::get_blas_gpu_va() const 
{ return blasBuffer ? blasBuffer->GetGPUVirtualAddress() : 0; }

void GPU_mesh::create_bottom_level_AS() {
    D3DContext& d3d_ctx = D3DContext::Get();

    d3d_ctx.WaitForPendingOperations();

    D3D12_RAYTRACING_GEOMETRY_DESC geomDesc = { 
        .Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES,
        .Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE, // ???
        .Triangles = {}
    };

    geomDesc.Triangles.VertexBuffer.StartAddress = vertexBuffer->GetGPUVirtualAddress();
    geomDesc.Triangles.VertexBuffer.StrideInBytes = sizeof(vertex);
    geomDesc.Triangles.VertexCount = vertexCount;
    geomDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;

    geomDesc.Triangles.IndexBuffer = indexBufferView.BufferLocation;
    geomDesc.Triangles.IndexCount = indexCount;
    geomDesc.Triangles.IndexFormat = indexBufferView.Format;

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS inputs;
    inputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
    inputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    inputs.NumDescs = 1; // ???
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

    d3d_ctx.WaitForPendingOperations();
}

GPU_mesh::~GPU_mesh() { release_gpu_resource(); }

void GPU_mesh::release_gpu_resource() {
    D3DContext& d3d_ctx = D3DContext::Get();
    if (d3d_ctx.g_pd3dCommandQueue != nullptr) {
        d3d_ctx.WaitForPendingOperations();
        d3d_ctx.WaitForPending—opy();
    }
    vertexBuffer.Reset(); 
    indexBuffer.Reset();
    scratchBuffer.Reset();
    blasBuffer.Reset();
};

GPU_model::GPU_model(const Model& cpu_model) {
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
}

GPU_model::~GPU_model() { release_gpu_resource(); }

void GPU_model::release_gpu_resource() {
    D3DContext& d3d_ctx = D3DContext::Get();\
    if (d3d_ctx.g_pd3dCommandQueue != nullptr) {
        d3d_ctx.WaitForPendingOperations();
    }
    //d3d_ctx.WaitForPending—opy();
    tlasScratchBuffer.Reset();
    tlasBuffer.Reset();
    instancesUploadBuffer.Reset();
};

void GPU_model::create_top_level_AS(const Model& cpu_model) {
    D3DContext& d3d_ctx = D3DContext::Get();

    d3d_ctx.WaitForPendingOperations();

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
        const auto& mat = obj.ModelMatrix;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                inst.Transform[r][c] = mat[c][r];
            }
        }

        inst.InstanceID = static_cast<UINT>(i);  // can be used to identify object in shaders
        inst.InstanceMask = 0xFF;
        inst.InstanceContributionToHitGroupIndex = 0;  // shader binding table offsets if used
        inst.Flags = D3D12_RAYTRACING_INSTANCE_FLAG_NONE;

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

    d3d_ctx.WaitForPendingOperations();
}

const std::vector<GPU_mesh>& GPU_model::get_meshes() const {
    return meshes_; }

D3D12_GPU_VIRTUAL_ADDRESS GPU_model::GetGPUVirtualAddress() const {
    return tlasBuffer->GetGPUVirtualAddress();
}

}  // namespace app