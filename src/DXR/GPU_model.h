#pragma once
#include "../model_loader.h"
#include "../d3d_context.h"

#include <d3d12.h>

#define NOMINMAX
#include <wrl.h>

namespace app {

using Microsoft::WRL::ComPtr;

class GPU_mesh {
  public:
    GPU_mesh() = default;  // empty mesh, useful for combined mesh in GPU_model
    explicit GPU_mesh(const Mesh& cpu_mesh, bool positions_only = true);
    ~GPU_mesh();

    GPU_mesh(const GPU_mesh&) = delete;
    GPU_mesh& operator=(const GPU_mesh&) = delete;

    GPU_mesh(GPU_mesh&&) = default;
    GPU_mesh& operator=(GPU_mesh&&) = default;

    void transition_from_copy_to_usage();
    void create_bottom_level_AS();

    // Expose BLAS GPU address for TLAS construction
    D3D12_GPU_VIRTUAL_ADDRESS get_blas_gpu_va() const;

    ComPtr<ID3D12Resource2> vertexBuffer;
    ComPtr<ID3D12Resource2> indexBuffer;

    bool positions_only_;
        
private:
    ComPtr<ID3D12Resource> scratchBuffer;
    ComPtr<ID3D12Resource> blasBuffer;

    UINT vertexCount;
    UINT indexCount;

    void release_gpu_resource();
};

class GPU_model {
  public:
    explicit GPU_model(const Model& cpu_model);
    ~GPU_model();

    GPU_model(const GPU_model&) = delete;
    GPU_model& operator=(const GPU_model&) = delete;

    GPU_model(GPU_model&&) = default;
    GPU_model& operator=(GPU_model&&) = default;

    const std::vector<GPU_mesh>& get_meshes() const;

    D3D12_GPU_VIRTUAL_ADDRESS GetGPUVirtualAddress() const;

    struct D3D_Handle_Pair {
        D3D12_CPU_DESCRIPTOR_HANDLE cpuDescriptorHandle;
        D3D12_GPU_DESCRIPTOR_HANDLE gpuDescriptorHandle;
    };

    D3D_Handle_Pair combined_mesh_indices;
    D3D_Handle_Pair combined_mesh_vertices;
    D3D_Handle_Pair combined_mesh_offsets;

  private:
    std::vector<GPU_mesh> meshes_;

    void create_top_level_AS(const Model& cpu_model);

    // TLAS resources
    ComPtr<ID3D12Resource> tlasScratchBuffer;
    ComPtr<ID3D12Resource> tlasBuffer;
    ComPtr<ID3D12Resource> instancesUploadBuffer;

    // Combine all meshes in two buffers
    GPU_mesh combinedMesh;
    ComPtr<ID3D12Resource> MeshIndicesOffsets;

    void prepare_combined_vertex_index_buffers(const Model& cpu_model);
    void release_gpu_resource();
};

}  // namespace app