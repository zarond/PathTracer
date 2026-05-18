#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <wrl.h>

#include <glm/fwd.hpp>
#include <vector>

#include "../cpu_framebuffer.h"
#include "../model_loader.h"
#include "../d3d_context.h"

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

    bool positionsOnly = false;

    UINT get_vertex_count() const;
    UINT get_index_count() const;

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
    float alphaCutoff;

    int doubleSided;
    int hasVolume;
    int alphaBlending;
   
    int aoTextureIndex;
    float AOStrength;

    float padding[3];

    GPU_Material(const Material& mat, const std::vector<int>& texture_id_conversion_table, int default_texture);
};

enum class TEXTURE_TRAITS : uint8_t {
    None            = 0,
    HDR             = 1 << 0,
    sRGB            = 1 << 1,
    Cubemap         = 1 << 2,
    RenderTarget    = 1 << 3,
    Depth           = 1 << 4,
    UAV             = 1 << 5,
    AllocateMips    = 1 << 6,
    NormalMap       = 1 << 7,
};
DEFINE_ENUM_FLAG_OPERATORS(TEXTURE_TRAITS)

class GPU_texture {
  public:
    GPU_texture() = default;
    explicit GPU_texture(UINT64 width, UINT height, TEXTURE_TRAITS texture_options);
    explicit GPU_texture(const CPUTexture<hdr_pixel>& cpu_texture, bool allocate_mips = false, bool is_normal_map = false);
    explicit GPU_texture(
        const CPUTexture<sdr_pixel>& cpu_texture, bool srgb_ = false, bool allocate_mips = false, bool is_normal_map = false);
    ~GPU_texture();

    GPU_texture(const GPU_texture&) = delete;
    GPU_texture& operator=(const GPU_texture&) = delete;

    GPU_texture(GPU_texture&&) = default;
    GPU_texture& operator=(GPU_texture&&) = default;

    D3D12_GPU_DESCRIPTOR_HANDLE GetSRVHandle() const;
    D3D12_GPU_DESCRIPTOR_HANDLE GetUAVHandle() const;
    D3D12_CPU_DESCRIPTOR_HANDLE GetRTVHandle() const;
    D3D12_CPU_DESCRIPTOR_HANDLE GetDSVHandle() const;

    ComPtr<ID3D12Resource> get_gpu_resource();

    TEXTURE_TRAITS texture_options = TEXTURE_TRAITS::None;
    uint8_t mipLevels = 1;

    void GetUAVHandleForMipLevel(uint8_t mipLevel, D3D12_CPU_DESCRIPTOR_HANDLE Handle) const;

    static constexpr uint32_t CalculateMipCount(uint32_t width, uint32_t height) { return std::bit_width(std::max(width, height)); }
    static constexpr uint32_t GetMipDimension(uint32_t baseSize, uint32_t mipLevel) { return std::max(1u, baseSize >> mipLevel); }

    void release_gpu_resource();

    static void copy_texture_from_uav(GPU_texture& dst, GPU_texture& src, ComPtr<ID3D12GraphicsCommandList4>& commandList);
    static void copy_texture_to_uav(GPU_texture& dst, GPU_texture& src, ComPtr<ID3D12GraphicsCommandList4>& commandList);

  private:
    void create_texture_resource(UINT64 width, UINT height, DXGI_FORMAT format);
    void upload_texture_to_gpu(int width_, int height_, const auto& data_, size_t sizeofpixel, DXGI_FORMAT format);

    DXGI_FORMAT choose_format() const;

    ComPtr<ID3D12Resource> pTexture;
    D3D_Handle_Pair srv_handle{};
    D3D_Handle_Pair uav_handle{};
    D3D_Handle_Pair rtv_handle{};
    D3D_Handle_Pair dsv_handle{};
    // ComPtr<ID3D12Resource> uploadBuffer;
};

class GPU_model {
  public:
    explicit GPU_model(const Model& cpu_model, bool raytracing_support = true);
    ~GPU_model();

    GPU_model(const GPU_model&) = delete;
    GPU_model& operator=(const GPU_model&) = delete;

    GPU_model(GPU_model&&) = default;
    GPU_model& operator=(GPU_model&&) = default;

    const std::vector<GPU_mesh>& get_meshes() const;
    std::vector<GPU_texture>& get_textures();

    D3D12_GPU_VIRTUAL_ADDRESS GetGPUVirtualAddress() const;

    D3D_Handle_Pair combined_mesh_indices{};
    D3D_Handle_Pair combined_mesh_vertices{};
    D3D_Handle_Pair combined_mesh_offsets{};

    D3D_Handle_Pair materials_array{};

    std::vector<uint32_t> indicesOffsets;
    std::vector<uint32_t> indicesSizes;
    
    std::vector<Object> objects;

    std::vector<GPU_texture> textures;

    bool isEmpty() const;

    void update_materials_array_buffer(const Model& cpu_model);
    const std::vector<GPU_Material>& get_materials_cpu_array() const;

    const GPU_mesh& get_combined_mesh() const;

  private:
    std::vector<GPU_mesh> meshes_;

    std::vector<int> texture_id_conversion_table_;
    GPU_texture default_white_texture_{CPUTexture<sdr_pixel>::create_white_texture(), false};
    int default_white_texture_index_ = 0;

    void create_top_level_AS(const Model& cpu_model);

    // TLAS resources
    ComPtr<ID3D12Resource> tlasScratchBuffer;
    ComPtr<ID3D12Resource> tlasBuffer;
    ComPtr<ID3D12Resource> instancesUploadBuffer;
    //ComPtr<ID3D12Resource> instancesBuffer; // ???

    // Combine all meshes in two buffers
    GPU_mesh combinedMesh;
    ComPtr<ID3D12Resource> MeshIndicesOffsets;
    ComPtr<ID3D12Resource> MaterialsArray;
    std::vector<GPU_Material> MaterialsCPUArray;
    //std::vector<D3D12_RAYTRACING_INSTANCE_DESC> instances;  // ??? 

    bool isEmpty_ = true;

    void prepare_combined_vertex_index_buffers(const Model& cpu_model);
    void prepare_materials_array_buffer(const Model& cpu_model);
    void prepare_textures_array_buffer(const Model& cpu_model);
    void release_gpu_resource();
};

}  // namespace app