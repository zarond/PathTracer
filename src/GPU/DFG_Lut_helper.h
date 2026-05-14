#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <wrl.h>
#include <directx/d3dx12.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

class DFG_Lut_helper {
  public:
    DFG_Lut_helper();
    ~DFG_Lut_helper();
    void CreateDFG_Lut();
    GPU_texture&& GetDFG_Lut();

  private:
    void CreateRootSignature();
    void CreatePipelineStateObject();
    ComPtr<ID3D12RootSignature> m_rootSignature;
    ComPtr<ID3D12PipelineState> m_pipelineState;

    const wchar_t* c_cs_file_name = L"CS_DFG_Lut.dxil";

    static constexpr int DFG_size = 256;

    GPU_texture DFG_lut{};

    void release_gpu_resources();
};

}