#pragma once

#include <filesystem>
#include <vector>

#ifdef WINDOWS_SPECIFIC
#define NOMINMAX
#include <directx/d3d12.h>
#include <wrl.h>
#include "GPU/GPU_model.h"
#endif

#include "cpu_texture.h"

namespace app {

using glm::fvec2;
using glm::fvec3;
using glm::fvec4;

#ifdef WINDOWS_SPECIFIC
using Microsoft::WRL::ComPtr;
#endif

class CPUFrameBuffer : private CPUTexture<hdr_pixel> {
  private:
    using CPUTexture::channels_;
    using CPUTexture::data_;
    using CPUTexture::height_;
    using CPUTexture::width_;

  public:
    CPUFrameBuffer();
    CPUFrameBuffer(int width, int height);

#ifdef WINDOWS_SPECIFIC
    ~CPUFrameBuffer();
#endif

    void resize(int width, int height);
    void clear(const hdr_pixel clearColor = hdr_pixel{0.0f, 0.0f, 0.0f, 1.0f});
    hdr_pixel& at(int x, int y);
    const hdr_pixel& at(int x, int y) const;

    using CPUTexture::height;
    using CPUTexture::sample_nearest;
    using CPUTexture::width;

    void save_to_file(const std::filesystem::path& filePath, bool from_GPU_texture) const;

#ifdef WINDOWS_SPECIFIC
    void create_texture_resource();
    void upload_to_gpu();
    void release_gpu_resource();
    const GPU_texture& get_texture_resource() const;
    void transition_from_copy_to_srv() const;
    void transition_from_srv_to_copy() const;
    void transition_from_srv_to_uav() const;
    void transition_from_uav_to_srv() const;
    bool nearest_filtering = true;

    ComPtr<ID3D12Resource> get_gpu_resource() const;
    ComPtr<ID3D12Resource> get_gpu_upload_resource() const;

  private:
    GPU_texture gpuTexture;
    ComPtr<ID3D12Resource> uploadBuffer;
    UINT uploadPitch = 0;
    UINT uploadSize = 0;
    void* mapped = nullptr;

    std::vector<hdr_pixel> download_from_gpu() const;
#endif
};

}  // namespace app
