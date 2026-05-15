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
    void CreateSpecularEnvmapCube(const GPU_texture& envmap);
    GPU_texture&& GetSpecularEnvmapCube();

  private:
    void CreateRootSignature();
    void CreatePipelineStateObject();
    ComPtr<ID3D12RootSignature> m_rootSignature;
    ComPtr<ID3D12PipelineState> m_DiffusePipelineState;
    ComPtr<ID3D12PipelineState> m_SpecularPipelineState;

    const wchar_t* c_cs_diffuse_file_name = L"CS_Diffuse_Lut.dxil";
    const wchar_t* c_cs_specular_file_name = L"CS_Specular_Lut.dxil";

    static constexpr int Diffuse_size = 32;
    static constexpr int Specular_size = 512;

    GPU_texture Diffuse_lut{};
    GPU_texture Specular_lut{};

    void release_gpu_resources();
};

}  // namespace app