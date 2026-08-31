#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>
#include <array>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

struct TonemapSettings {
    enum TonemappingType : int {
        Clamp = 0,
        Reinhard,
        Hable_original,
        Hable_alternative,
        ACES_Narkowicz,
        ACES_Filmic,
        Khronos_PBR_Neutral,

        Count,
    };

    inline static constexpr std::array<const char* const, static_cast<size_t>(TonemappingType::Count)> TypeNames = {
        "Clamp", 
        "Reinhard", 
        "Hable original", 
        "Hable alternative", 
        "ACES Narkowicz", 
        "ACES Filmic", 
        "Khronos PBR Neutral"
    };

    bool has_white_point_controls();

    bool enabled = false;
    TonemappingType type = TonemappingType::Clamp;
    float exposure = 1.0f;
    float whitePoint = 11.2f;
    bool useWhitePoint = false;
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