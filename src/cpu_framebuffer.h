#pragma once

#include <filesystem>
#include <vector>

#include <fastgltf/core.hpp>
#include <fastgltf/types.hpp>
#include <fastgltf/tools.hpp>

namespace app {

using namespace fastgltf::math;

using pixel = fastgltf::math::fvec4;

class CPUTexture {
public:
    explicit CPUTexture(const fastgltf::Image& image);
    explicit CPUTexture(const std::filesystem::path& path);
    CPUTexture();
    
    int width() const;
    int height() const;

    pixel sample_nearest(fvec2 uv) const;

    static CPUTexture create_white_texture();

protected:
    std::vector<pixel> data_;
    int width_ = 1;
    int height_ = 1;
    int channels_ = 4; // Number of channels in the image originally (e.g., 3 for RGB, 4 for RGBA); we always store as RGBA
};

class CPUFrameBuffer: private CPUTexture {
private:
    using CPUTexture::width_;
    using CPUTexture::height_;
    using CPUTexture::channels_;
    using CPUTexture::data_;
public:
    CPUFrameBuffer();
    CPUFrameBuffer(int width, int height);
    
    void clear(const pixel clearColor = pixel{});
    pixel& at(int x, int y);
    const pixel& at(int x, int y) const;
    
    using CPUTexture::width;
    using CPUTexture::height;
    using CPUTexture::sample_nearest;

    void save_to_file(const std::filesystem::path& filePath) const;
};

}
