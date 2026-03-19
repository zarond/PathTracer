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
    explicit GPU_mesh(const Mesh& cpu_mesh);
    ~GPU_mesh();

    void transition_from_copy_to_usage();
    void create_bottom_level_AS();

    // Expose BLAS GPU address for TLAS construction
    D3D12_GPU_VIRTUAL_ADDRESS get_blas_gpu_va() const;

  private:
    ComPtr<ID3D12Resource2> vertexBuffer;
    D3D12_VERTEX_BUFFER_VIEW vertexBufferView;

    ComPtr<ID3D12Resource2> indexBuffer;
    D3D12_INDEX_BUFFER_VIEW indexBufferView;
        
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
    const std::vector<GPU_mesh>& get_meshes() const;

    D3D12_GPU_VIRTUAL_ADDRESS GetGPUVirtualAddress() const;

  private:
    std::vector<GPU_mesh> meshes_;

    void create_top_level_AS(const Model& cpu_model);

    // TLAS resources
    ComPtr<ID3D12Resource> tlasScratchBuffer;
    ComPtr<ID3D12Resource> tlasBuffer;
    ComPtr<ID3D12Resource> instancesUploadBuffer;

    void release_gpu_resource();
};

}  // namespace app