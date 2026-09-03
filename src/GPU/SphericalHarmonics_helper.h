#pragma once

#define NOMINMAX
#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <wrl.h>

#include "GPU_model.h"

namespace app {
using Microsoft::WRL::ComPtr;

class SphericalHarmonics_helper {
  public:
    SphericalHarmonics_helper();
    ~SphericalHarmonics_helper();

    void Init();

    void Compute(const GPU_texture& envmap);

    std::vector<glm::fvec4> download_result_from_gpu() const;

    static void Reload();

  private:
    static void CreateRootSignature();
    static void CreatePipelineStateObject();
    void CreateCounterBuffer();

    static ComPtr<ID3D12RootSignature> m_rootSignature;
    static ComPtr<ID3D12PipelineState> m_PipelineState;

    static constexpr const wchar_t* c_cs_file_name = L"CS_SphericalHarmonicsIrradiance.dxil";

    ComPtr<ID3D12Resource> GroupCounters;
    ComPtr<ID3D12Resource2> zero_uploadBuffer;

    ComPtr<ID3D12Resource> tmp_buffer;
    int tmpWidth = 0;
    int tmpHeight = 0;
    void resize_tmp_buffer(int new_width, int new_height);

    void release_gpu_resources();
};

}  // namespace app
