#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include <span>
#include <map>
#include <utility>

namespace app {
using Microsoft::WRL::ComPtr;

struct PSO_variant_key {
    unsigned int original_PSO_index = 0;
    bool backface_culling = true;
    bool depth_clipping = true;
    bool reverse_winding = false;
    bool alpha_blending = false;

    auto operator<=>(const PSO_variant_key&) const = default;
};

// Helper class for managing graphical PSO variants based on different render settings (e.g. depth clipping, alpha blending, etc.)
class PSO_helper {
  public:
    bool init(
        unsigned int Number_of_original_PSOs, 
        std::span<const wchar_t*> vs_file_names, 
        std::span<const wchar_t*> ps_file_names, 
        ComPtr<ID3D12RootSignature> rootSignature,
        std::span<const D3D12_INPUT_ELEMENT_DESC> inputElementDescs, 
        std::span<const bool> use_input_layout,
        std::span<const UINT8> rt_write_mask,
        std::span<const D3D12_DEPTH_WRITE_MASK> depth_write_mask,
        std::span<const int> rt_num,
        bool use_float32_rt = false
    );

    void set_pipeline_state(ComPtr<ID3D12GraphicsCommandList4> commandList, const PSO_variant_key key);

    void preallocate_pipeline_state(const PSO_variant_key key);

    bool is_initialized() const;

    // Force next set_pipeline_state call to issue commandlist->SetPipelineState()
    void invalidate_last_used_key();

    void release_gpu_resources();

  private:
    unsigned int m_Number_of_original_PSOs = 0;

    std::map<PSO_variant_key, ComPtr<ID3D12PipelineState>> m_PSO_variants; // TODO: maybe use std::flat_map (C++23)

    std::vector<D3D12_INPUT_ELEMENT_DESC> m_inputElementDescs;
    std::vector<D3D12_GRAPHICS_PIPELINE_STATE_DESC> m_starting_psoDesc;
    std::vector<std::pair<ComPtr<ID3DBlob>, D3D12_SHADER_BYTECODE>> m_vs_bytecode;
    std::vector<std::pair<ComPtr<ID3DBlob>, D3D12_SHADER_BYTECODE>> m_ps_bytecode;

    PSO_variant_key last_used_key{~0u, false, false, false, false};

    void create_PSO_variant(const PSO_variant_key key, ComPtr<ID3D12PipelineState>& pipelineState);

    D3D12_GRAPHICS_PIPELINE_STATE_DESC create_starting_psoDesc(
        ComPtr<ID3D12RootSignature> rootSignature,
        bool use_input_layout, 
        UINT8 rt_write_mask,
        D3D12_DEPTH_WRITE_MASK depth_write_mask,
        int rt_num,
        bool use_float32_rt
    );
};

}
