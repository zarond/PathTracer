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

    static constexpr int Diffuse_size = 32;
    static constexpr int Specular_size = 1024;
    static constexpr int SpecularMips = GPU_texture::CalculateMipCount(Specular_size, Specular_size) - 1; // so that max mip is 2x2 texture

    static GPU_texture GetBlankSRVDiffuseTexture();
    static GPU_texture GetBlankSRVSpecularTexture();

    void ReleaseTemporaryGPUResources();

  private:
    void CreateRootSignature();
    void CreatePipelineStateObject();
    ComPtr<ID3D12RootSignature> m_rootSignature;
    ComPtr<ID3D12PipelineState> m_DiffusePipelineState;
    ComPtr<ID3D12PipelineState> m_SpecularPipelineState;

    const wchar_t* c_cs_diffuse_file_name = L"CS_Diffuse_Lut.dxil";
    const wchar_t* c_cs_specular_file_name = L"CS_Specular_Lut.dxil";

    GPU_texture Diffuse_lut{};
    GPU_texture Specular_lut{};

    std::vector<D3D_Handle_Pair> m_mip_uav_handles; // store temporary UAV views to mips for compute shader dispatches

    void release_gpu_resources();
};

}  // namespace app