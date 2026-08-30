#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

struct TonemapSettings {
    enum TonemappingType : int {
        Clamp = 0,
        Reinhard,
        Hable,
        ACES_Narkowicz,
        ACES_Filmic,
        Khronos_PBR_Neutral,

        Count,
    };

    bool enabled = false;
    TonemappingType type = TonemappingType::Clamp;
    float exposure = 1.0f;
};

class Tonemapping_helper {
  public:
    Tonemapping_helper() = default;
    ~Tonemapping_helper();

    void Apply(const GPU_texture& dst_uav_texture, const GPU_texture& src_srv_texture, ComPtr<ID3D12GraphicsCommandList4> commandList);

    static void Reload();

    TonemapSettings settings{};

  private:
    static void CreateRootSignature();
    static void CreatePipelineStateObject();

    static ComPtr<ID3D12RootSignature> m_rootSignature;
    static ComPtr<ID3D12PipelineState> m_PipelineState;

    static constexpr const wchar_t* c_cs_file_name = L"CS_Tonemapping.dxil";

    void release_gpu_resources();
};

}  // namespace app