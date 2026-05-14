#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

class EnvCube_helper {
  public:
    EnvCube_helper();
    ~EnvCube_helper();
    void CreateDiffuseEnvmapCube(const GPU_texture& envmap);
    GPU_texture&& GetDiffuseEnvmapCube();

  private:
    void CreateRootSignature();
    void CreatePipelineStateObject();
    ComPtr<ID3D12RootSignature> m_rootSignature;
    ComPtr<ID3D12PipelineState> m_pipelineState;

    const wchar_t* c_cs_file_name = L"CS_Diffuse_Lut.dxil";

    static constexpr int Diffuse_size = 32;

    GPU_texture Diffuse_lut{};

    void release_gpu_resources();
};

}  // namespace app