#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "../acceleration_structure.h"

#include "GPU_model.h"
#include "PSO_helper.h"
#include "Drawables.h"

namespace app {
using Microsoft::WRL::ComPtr;

struct ShadowMappingInfo {
    fmat4x4 viewProjection;
    fvec4 origin;
    fvec4 direction;
    glm::ivec2 shadowMapTextureIndex_type;
    fvec2 texel_size_padding;
};

class Shadow_mapping_helper {
  public:
    Shadow_mapping_helper();
    ~Shadow_mapping_helper();

    void CreateShadowMaps(const GPU_model& gpu_model, UINT size);

    const std::vector<ShadowMappingInfo>& GetShadowMappingInfo() const;
    D3D12_GPU_DESCRIPTOR_HANDLE GetShadowMappingInfoGPUHandle() const;
    
    static void Reload();

    static constexpr int ShadowMapSizeDefault = 1024;

    float BiasMultiplier = 0.0f;

  private:
    void CreateShadowMap(const GPU_model& gpu_model, const ShadowMappingInfo& shadow,
        const GPU_texture& shadowMap, UINT size, float far_plane);
    
    static void CreateRootSignature();
    static void CreatePipelineStateObject();

    static ComPtr<ID3D12RootSignature> m_rootSignature;
    static PSO_helper m_PSO_helper;

    static constexpr const wchar_t* c_vs_file_name = L"VS_Shadows.dxil";
    static constexpr const wchar_t* c_ps_file_name = L"PS_Shadows.dxil";

    std::vector<DrawableSortingInfo> m_sortedDrawables;  // reusable vector for sorting drawables every frame
    GPU_texture m_depthTexture;

    // shadow maps textures
    std::vector<GPU_texture> m_shadowMaps;

    // shadow mapping info for main render pass
    std::vector<ShadowMappingInfo> m_shadowMappingInfo;
    ComPtr<ID3D12Resource> shadowMappingInfoArray;
    ComPtr<ID3D12Resource2> shadow_uploadBuffer;
    D3D_Handle_Pair shadow_mapping_info_array{};

    UINT ShadowSize = 0;

    void resize_shadow_maps(int new_size, int new_lights_count);

    void prepare_shadow_mapping_info_array_buffer();
    void update_shadow_mapping_info_array_buffer();

    void release_gpu_resources();
};

}  // namespace app