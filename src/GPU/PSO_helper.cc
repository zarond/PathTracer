#include "PSO_helper.h"

#include "../d3d_context.h"
#include "Common_helpers.h"

namespace app {

bool PSO_helper::init(
    unsigned int Number_of_original_PSOs, 
    std::span<const wchar_t*> vs_file_names, 
    std::span<const wchar_t*> ps_file_names, 
    ComPtr<ID3D12RootSignature> rootSignature,
    std::span<const D3D12_INPUT_ELEMENT_DESC> inputElementDescs,
    std::span<const bool> use_input_layout,
    std::span<const UINT8> rt_write_mask, 
    std::span<const D3D12_DEPTH_WRITE_MASK> depth_write_mask,
    std::span<const int> rt_num
) {
    if (Number_of_original_PSOs == 0 ||
        vs_file_names.size() != Number_of_original_PSOs || 
        ps_file_names.size() != Number_of_original_PSOs ||
        use_input_layout.size() != Number_of_original_PSOs || 
        rt_write_mask.size() != Number_of_original_PSOs || 
        depth_write_mask.size() != Number_of_original_PSOs || 
        rt_num.size() != Number_of_original_PSOs) 
    {
        return false;
    }
    if (is_initialized()) {
        release_gpu_resources();
    }
    m_vs_bytecode.reserve(Number_of_original_PSOs);
    m_ps_bytecode.reserve(Number_of_original_PSOs);
    m_inputElementDescs.reserve(inputElementDescs.size());
    m_starting_psoDesc.reserve(Number_of_original_PSOs);

    for (unsigned int i = 0; i < inputElementDescs.size(); ++i) {
        m_inputElementDescs.emplace_back(inputElementDescs[i]);
    }

    for (unsigned int i = 0; i < Number_of_original_PSOs; ++i) {
        // Cache shaderBlob and bytecode to avoid reloading them later when creating PSO variants
        auto [vs_shaderBlob, vs_bytecode] = LoadShader(vs_file_names[i]);
        auto [ps_shaderBlob, ps_bytecode] = LoadShader(ps_file_names[i]);
        bool use_input = use_input_layout[i];
        auto rt_write = rt_write_mask[i];
        auto depth_write = depth_write_mask[i];
        auto rt_count = rt_num[i];

        m_vs_bytecode.emplace_back(vs_shaderBlob, vs_bytecode);
        m_ps_bytecode.emplace_back(ps_shaderBlob, ps_bytecode);
        m_starting_psoDesc.emplace_back(
            create_starting_psoDesc(rootSignature, use_input, rt_write, depth_write, rt_count));
    }

    m_Number_of_original_PSOs = Number_of_original_PSOs;
    return true;
}

D3D12_GRAPHICS_PIPELINE_STATE_DESC PSO_helper::create_starting_psoDesc(
    ComPtr<ID3D12RootSignature> rootSignature, 
    bool use_input_layout, 
    UINT8 rt_write_mask,
    D3D12_DEPTH_WRITE_MASK depth_write_mask, 
    int rt_num) 
{
    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};

    if (use_input_layout) {
        psoDesc.InputLayout = {m_inputElementDescs.data(), static_cast<UINT>(m_inputElementDescs.size())};
    } else {
        psoDesc.InputLayout = {nullptr, 0};
    }
    psoDesc.pRootSignature = rootSignature.Get();
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.RasterizerState.FrontCounterClockwise = TRUE;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.BlendState.RenderTarget[0].RenderTargetWriteMask = rt_write_mask;
    psoDesc.DepthStencilState.DepthEnable = TRUE;
    psoDesc.DepthStencilState.DepthWriteMask = depth_write_mask;
    psoDesc.DepthStencilState.StencilEnable = FALSE;
    psoDesc.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = rt_num;
    // WARNING: The following formats are hardcoded and may need to be adjusted based on the specific render targets
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    if (rt_num >= 2) { // second render target for Velocity Buffer
        psoDesc.RTVFormats[1] = DXGI_FORMAT_R16G16B16A16_FLOAT;
    }
    if (rt_num >= 3) { // third for material ID
        psoDesc.RTVFormats[2] = DXGI_FORMAT_R8_UINT;
    }
    psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    psoDesc.SampleDesc.Count = 1;

    return psoDesc;
}

bool PSO_helper::is_initialized() const { return m_Number_of_original_PSOs > 0; }

void PSO_helper::invalidate_last_used_key() { last_used_key.original_PSO_index = ~0u; }

void PSO_helper::release_gpu_resources() {
    m_PSO_variants.clear();
    m_vs_bytecode.clear();
    m_ps_bytecode.clear();
    m_inputElementDescs.clear();
    m_Number_of_original_PSOs = 0;
}

void PSO_helper::set_pipeline_state(ComPtr<ID3D12GraphicsCommandList4> commandList, const PSO_variant_key key) {
    if (key.original_PSO_index >= m_Number_of_original_PSOs) {
        return;
    }
    if (key == last_used_key) {
        return;
    }
    const auto it = m_PSO_variants.find(key);
    if (it != m_PSO_variants.end()) {
        commandList->SetPipelineState(it->second.Get());
        last_used_key = key;
        return;
    }

    ComPtr<ID3D12PipelineState> pipelineState;
    create_PSO_variant(key, pipelineState);

    auto inserted = m_PSO_variants.insert({key, pipelineState});
    commandList->SetPipelineState(inserted.first->second.Get());
    last_used_key = key;
}

void PSO_helper::preallocate_pipeline_state(const PSO_variant_key key) {
    if (key.original_PSO_index >= m_Number_of_original_PSOs) {
        return;
    }
    ComPtr<ID3D12PipelineState> pipelineState;
    create_PSO_variant(key, pipelineState);
    m_PSO_variants.insert({key, pipelineState});
}

void PSO_helper::create_PSO_variant(const PSO_variant_key key, ComPtr<ID3D12PipelineState>& pipelineState) {
    D3DContext& d3d_ctx = D3DContext::Get();
    auto device = d3d_ctx.m_d3dDevice;

    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = m_starting_psoDesc[key.original_PSO_index];

    psoDesc.VS = m_vs_bytecode[key.original_PSO_index].second;
    psoDesc.PS = m_ps_bytecode[key.original_PSO_index].second;
    psoDesc.RasterizerState.CullMode = key.backface_culling ? D3D12_CULL_MODE_BACK : D3D12_CULL_MODE_NONE;
    psoDesc.RasterizerState.FrontCounterClockwise = key.reverse_winding ? FALSE : TRUE;
    psoDesc.RasterizerState.DepthClipEnable = key.depth_clipping ? TRUE : FALSE;
    if (key.alpha_blending) {
        psoDesc.DepthStencilState.DepthWriteMask =
            D3D12_DEPTH_WRITE_MASK_ZERO;  // Disable depth writes when alpha blending is enabled
        auto& rtBlend = psoDesc.BlendState.RenderTarget[0];
        rtBlend.BlendEnable = TRUE;
        rtBlend.SrcBlend = D3D12_BLEND_SRC_ALPHA;
        rtBlend.DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
        rtBlend.BlendOp = D3D12_BLEND_OP_ADD;
        rtBlend.SrcBlendAlpha = D3D12_BLEND_ZERO;
        rtBlend.DestBlendAlpha = D3D12_BLEND_ONE;
        rtBlend.BlendOpAlpha = D3D12_BLEND_OP_ADD;
    }

    ThrowIfFailed(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&pipelineState)));
}

}  // namespace app
