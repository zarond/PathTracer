#pragma once

#define NOMINMAX
#include <d3d12.h>
#include <wrl.h>

#include <glm/fwd.hpp>
#include <vector>

#include "../cpu_framebuffer.h"
#include "../model_loader.h"

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

    bool positions_only_ = false;

  private:
    ComPtr<ID3D12Resource> scratchBuffer;
    ComPtr<ID3D12Resource> blasBuffer;

    UINT vertexCount = 0;
    UINT indexCount = 0;

    void release_gpu_resource();
};

struct GPU_Material {  // enforce packing rules on CPU Material data
    fvec4 baseColorFactor;
    fvec4 emissiveFactor;
    fvec4 attenuationFactor;
    float metallicFactor;
    float roughnessFactor;
    int baseColorTextureIndex;
    int metallicRoughnessTextureIndex;
    int normalTextureIndex;
    float ior;
    float dielectric_f0;
    float transmisionFactor;
    int transmissionTextureIndex;
    int emissiveTextureIndex;
    float emissiveStrength;
    float alpha_cutoff;

    int doubleSided;
    int hasVolume;
    int alphaBlending;
    int padding0;

    GPU_Material(const Material& mat, const std::vector<int>& texture_id_conversion_table, int default_texture);
};

class GPU_texture {
  public:
    GPU_texture() = default;
    explicit GPU_texture(const CPUTexture<hdr_pixel>& cpu_texture);
    explicit GPU_texture(const CPUTexture<sdr_pixel>& cpu_texture, bool srgb_ = false);
    ~GPU_texture();

    GPU_texture(const GPU_texture&) = delete;
    GPU_texture& operator=(const GPU_texture&) = delete;

    GPU_texture(GPU_texture&&) = default;
    GPU_texture& operator=(GPU_texture&&) = default;

    D3D12_GPU_VIRTUAL_ADDRESS GetGPUVirtualAddress() const;
    D3D12_GPU_DESCRIPTOR_HANDLE GetSRVHandle() const;

    bool HDR = false;
    bool srgb = false;  // whether to apply sRGB to linear conversion when sampling; only relevant for SDR textures

  private:
    void create_texture_resource(UINT64 width, UINT height, DXGI_FORMAT format);
    void upload_texture_to_gpu(int width_, int height_, const auto& data_, size_t sizeofpixel, DXGI_FORMAT format);

    ComPtr<ID3D12Resource> pTexture;
    D3D12_CPU_DESCRIPTOR_HANDLE srv_cpu_handle;
    D3D12_GPU_DESCRIPTOR_HANDLE srv_gpu_handle;
    // ComPtr<ID3D12Resource> uploadBuffer;
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

    D3D_Handle_Pair materials_array;

    std::vector<GPU_texture> textures;

    bool isEmpty() const;

    void update_materials_array_buffer(const Model& cpu_model);

  private:
    std::vector<GPU_mesh> meshes_;

    std::vector<int> texture_id_conversion_table;
    GPU_texture default_white_texture = GPU_texture(CPUTexture<sdr_pixel>::create_white_texture(), false);
    int default_white_texture_index;

    void create_top_level_AS(const Model& cpu_model);

    // TLAS resources
    ComPtr<ID3D12Resource> tlasScratchBuffer;
    ComPtr<ID3D12Resource> tlasBuffer;
    ComPtr<ID3D12Resource> instancesUploadBuffer;

    // Combine all meshes in two buffers
    GPU_mesh combinedMesh;
    ComPtr<ID3D12Resource> MeshIndicesOffsets;
    ComPtr<ID3D12Resource> MaterialsArray;

    bool isEmpty_ = true;

    void prepare_combined_vertex_index_buffers(const Model& cpu_model);
    void prepare_materials_array_buffer(const Model& cpu_model);
    void prepare_textures_array_buffer(const Model& cpu_model);
    void release_gpu_resource();
};

}  // namespace app